/**
 * mcp2515.c — MCP2515 CAN controller driver
 *
 * SPI0 wiring
 *   SCK  → GP18   MOSI → GP19   MISO → GP16 (3.3 V divider)   CS → GP17
 *
 * CAN: 500 kbps, 8 MHz crystal
 * TX buffer: TXB0, CAN ID 0x636 (SIDH=0xC6, SIDL=0xC0), DLC=8
 */

#include "mcp2515.h"

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

/* =========================================================================
 * CS helpers
 * ========================================================================= */

static inline void cs_low(void)  { gpio_put(MCP_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(MCP_PIN_CS, 1); }

/* =========================================================================
 * Low-level register access
 * ========================================================================= */

void mcp_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t buf[3] = { MCP_CMD_WRITE, addr, val };
    cs_low();
    spi_write_blocking(MCP_SPI, buf, 3);
    cs_high();
}

/*
 * Full-duplex read: the MCP2515 clocks out the data byte simultaneously
 * with the third SPI byte, so we must use spi_write_read_blocking.
 */
uint8_t mcp_read_reg(uint8_t addr)
{
    uint8_t tx[3] = { MCP_CMD_READ, addr, 0x00 };
    uint8_t rx[3] = { 0x00, 0x00, 0x00 };
    cs_low();
    spi_write_read_blocking(MCP_SPI, tx, rx, 3);
    cs_high();
    return rx[2];
}

void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    uint8_t buf[4] = { MCP_CMD_BIT_MODIFY, addr, mask, data };
    cs_low();
    spi_write_blocking(MCP_SPI, buf, 4);
    cs_high();
}

/* =========================================================================
 * Mode control
 * ========================================================================= */

static bool mcp_set_mode(uint8_t mode)
{
    mcp_bit_modify(MCP_REG_CANCTRL, MCP_MODE_MASK, mode);
    sleep_ms(5);
    uint8_t stat = mcp_read_reg(MCP_REG_CANSTAT);
    printf("[MCP] mcp_set_mode(0x%02X): CANSTAT=0x%02X\n", mode, stat);
    return (stat & MCP_MODE_MASK) == mode;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

bool mcp2515_init(void)
{
    /* SPI peripheral + GPIO */
    spi_init(MCP_SPI, MCP_SPI_BAUD);
    spi_set_format(MCP_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(MCP_PIN_CS);
    gpio_set_dir(MCP_PIN_CS, GPIO_OUT);
    cs_high();

    /* Hardware reset */
    uint8_t rst = MCP_CMD_RESET;
    cs_low();
    spi_write_blocking(MCP_SPI, &rst, 1);
    cs_high();
    sleep_ms(10);

    /* Enter Config mode (required before writing CNF registers) */
    if (!mcp_set_mode(MCP_MODE_CONFIG)) {
        printf("[MCP] ERROR: config mode failed (CANSTAT=0x%02X)\n",
               mcp_read_reg(MCP_REG_CANSTAT));
        return false;
    }

    /* Bit-timing: 500 kbps @ 8 MHz */
    mcp_write_reg(MCP_REG_CNF1, MCP_CNF1);
    mcp_write_reg(MCP_REG_CNF2, MCP_CNF2);
    mcp_write_reg(MCP_REG_CNF3, MCP_CNF3);

    /* Disable all interrupts, clear flags, clear TX buffer control */
    mcp_write_reg(MCP_REG_CANINTE,  0x00);
    mcp_write_reg(MCP_REG_CANINTF,  0x00);
    mcp_write_reg(MCP_REG_TXB0CTRL, 0x00);

    /* Enter Normal mode */
    if (!mcp_set_mode(MCP_MODE_NORMAL)) {
        printf("[MCP] ERROR: normal mode failed (CANSTAT=0x%02X)\n",
               mcp_read_reg(MCP_REG_CANSTAT));
        return false;
    }

    printf("[MCP] Init OK — 500 kbps, 8 MHz crystal\n");
    return true;
}

bool can_send(uint8_t signal_value)
{
    /*
     * Clear TXB0CTRL first so TXREQ is not left asserted from a previous
     * timed-out send. Skipping this causes the MCP2515 to ignore the new
     * load-then-RTS sequence.
     */
    mcp_write_reg(MCP_REG_TXB0CTRL, 0x00);

    /* Load ID + DLC into TXB0 registers in one sequential write */
    uint8_t hdr[7] = {
        MCP_CMD_WRITE, MCP_REG_TXB0SIDH,
        CAN_SIDH, CAN_SIDL,
        0x00, 0x00,     /* EID8, EID0 — standard frame, unused */
        CAN_DLC
    };
    cs_low();
    spi_write_blocking(MCP_SPI, hdr, 7);
    cs_high();

    /* Load 8 data bytes; byte 0 = signal, bytes 1-7 = 0x00 */
    uint8_t payload[8];
    memset(payload, 0x00, 8);
    payload[0] = signal_value;

    uint8_t data_cmd[2] = { MCP_CMD_WRITE, MCP_REG_TXB0D0 };
    cs_low();
    spi_write_blocking(MCP_SPI, data_cmd, 2);
    spi_write_blocking(MCP_SPI, payload, 8);
    cs_high();

    /* Request-to-send TXB0 */
    uint8_t rts = MCP_CMD_RTS_TXB0;
    cs_low();
    spi_write_blocking(MCP_SPI, &rts, 1);
    cs_high();

    /*
     * Poll TXB0CTRL for up to 50 ms:
     *   bit3 (TXREQ) cleared → frame transmitted
     *   bit4 (TXERR) set     → bus error, abort
     */
    for (int i = 0; i < 50; i++) {
        uint8_t ctrl = mcp_read_reg(MCP_REG_TXB0CTRL);
        if (!(ctrl & 0x08)) {
            if (signal_value != 0u) /* suppress heartbeat spam */
                printf("[CAN] TX signal=%u  SIDH=0x%02X\n",
                       signal_value, CAN_SIDH);
            return true;
        }
        if (ctrl & 0x10) {
            printf("[CAN ERR] TXB0CTRL=0x%02X  signal=%u\n", ctrl, signal_value);
            mcp_write_reg(MCP_REG_TXB0CTRL, 0x00);
            return false;
        }
        sleep_ms(1);
    }
    printf("[CAN TIMEOUT] signal=%u\n", signal_value);
    return false;
}
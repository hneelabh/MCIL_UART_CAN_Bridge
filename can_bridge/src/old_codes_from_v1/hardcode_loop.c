#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// ─── SPI / MCP2515 ────────────────────────────────────────────────────────────
#define SPI_PORT        spi0
#define PIN_MISO        16
#define PIN_CS          17
#define PIN_SCK         18
#define PIN_MOSI        19

#define MCP_RESET       0xC0
#define MCP_READ        0x03
#define MCP_WRITE       0x02
#define MCP_RTS_TXB0    0x81
#define REG_CANCTRL     0x0F
#define REG_CANSTAT     0x0E
#define REG_CNF1        0x2A
#define REG_CNF2        0x29
#define REG_CNF3        0x28
#define REG_TXB0CTRL    0x30
#define REG_TXB0SIDH    0x31
#define REG_TXB0SIDL    0x32
#define REG_TXB0EID8    0x33
#define REG_TXB0EID0    0x34
#define REG_TXB0DLC     0x35
#define REG_TXB0D0      0x36

#define CAN_DLC         8
#define CAN_SIDH        0xC6    // CAN ID 0x636
#define CAN_SIDL        0xC0

// ─── CS helpers ───────────────────────────────────────────────────────────────
static inline void cs_select(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}
static inline void cs_deselect(void) {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

// ─── MCP2515 ──────────────────────────────────────────────────────────────────
static void mcp_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[3] = {MCP_WRITE, reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 3);
    cs_deselect();
}

static uint8_t mcp_read_reg(uint8_t reg) {
    uint8_t tx[3] = {MCP_READ, reg, 0x00};
    uint8_t rx[3] = {0};
    cs_select();
    spi_write_read_blocking(SPI_PORT, tx, rx, 3);
    cs_deselect();
    return rx[2];
}

static void mcp_reset(void) {
    uint8_t cmd = MCP_RESET;
    cs_select();
    spi_write_blocking(SPI_PORT, &cmd, 1);
    cs_deselect();
    sleep_ms(10);
}

static bool mcp2515_init(void) {
    mcp_reset();
    // 500 kbps configuration for an 8 MHz crystal
    mcp_write_reg(REG_CNF1, 0x00);
    mcp_write_reg(REG_CNF2, 0x90); 
    mcp_write_reg(REG_CNF3, 0x02);
    mcp_write_reg(REG_CANCTRL, 0x00);
    sleep_ms(10);
    uint8_t stat = mcp_read_reg(REG_CANSTAT);
    printf("[MCP] CANSTAT=0x%02X\n", stat);
    return ((stat & 0xE0) == 0x00);
}

static bool can_send_value(uint8_t value) {
    uint8_t frame[CAN_DLC] = {value, 0, 0, 0, 0, 0, 0, 0};

    mcp_write_reg(REG_TXB0CTRL, 0x00);
    mcp_write_reg(REG_TXB0SIDH, CAN_SIDH);
    mcp_write_reg(REG_TXB0SIDL, CAN_SIDL);
    mcp_write_reg(REG_TXB0EID8, 0x00);
    mcp_write_reg(REG_TXB0EID0, 0x00);
    mcp_write_reg(REG_TXB0DLC,  CAN_DLC);
    for (int i = 0; i < CAN_DLC; i++)
        mcp_write_reg(REG_TXB0D0 + i, frame[i]);

    uint8_t rts = MCP_RTS_TXB0;
    cs_select();
    spi_write_blocking(SPI_PORT, &rts, 1);
    cs_deselect();

    for (int i = 0; i < 50; i++) {
        uint8_t ctrl = mcp_read_reg(REG_TXB0CTRL);
        if (!(ctrl & 0x08)) return true;
        if (  ctrl & 0x10) {
            mcp_write_reg(REG_TXB0CTRL, 0x00);
            return false;
        }
        sleep_ms(1);
    }
    return false;
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("[DEMO] Starting CAN demo loop...\n");

    // ─── LED ──────────────────────────────────────────────────────────────────
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    // ─── SPI / MCP2515 ────────────────────────────────────────────────────────
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    if (mcp2515_init())
        printf("[DEMO] MCP2515 Normal Mode. Ready.\n\n");
    else
        printf("[DEMO] WARNING: MCP2515 not in Normal Mode\n\n");

    printf("[DEMO] CAN ID=0x636  DLC=8  250kbps\n");
    printf("[DEMO] Cycling values 1-10, 10 seconds each\n\n");

    uint8_t  current_value = 1;
    uint32_t slot_start    = to_ms_since_boot(get_absolute_time());

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // ── Advance to next value every 10 seconds ────────────────────────────
        if (now - slot_start >= 10000) {
            current_value++;
            if (current_value > 10)
                current_value = 1;
            slot_start = now;
            printf("[DEMO] Switching to value %u\n", current_value);
        }

        // ── Send current value ────────────────────────────────────────────────
        bool ok = can_send_value(current_value);
        if (ok) {
            printf("[TX] value=%u\n", current_value);
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(50);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        } else {
            printf("[TX FAIL] value=%u\n", current_value);
        }

        // ── Send once per second ──────────────────────────────────────────────
        sleep_ms(10);
    }

    return 0;
}
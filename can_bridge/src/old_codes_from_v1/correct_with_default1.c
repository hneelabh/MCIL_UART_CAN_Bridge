#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"

// ─── UART ─────────────────────────────────────────────────────────────────────
#define BT_UART         uart0
#define BT_UART_RX_PIN  1
#define BT_UART_TX_PIN  0
#define BT_BAUD_RATE    2000000

// ─── SPI / MCP2515 ────────────────────────────────────────────────────────────
#define SPI_PORT        spi0
#define PIN_MISO        16
#define PIN_CS          17
#define PIN_SCK         18
#define PIN_MOSI        19
#define PIN_INT         20

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

// ─── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_DLC             8
#define SYNC_BYTE           0xAA
#define HEADER_BYTES        6
#define CAN_SIDH            0xC6
#define CAN_SIDL            0xC0

// ─── Heartbeat ────────────────────────────────────────────────────────────────
#define HEARTBEAT_VALUE    0       // default CAN value when no UART
#define HEARTBEAT_PERIOD_MS 10      // 10ms periodicity
#define UART_SILENCE_MS     100     // declare UART silent after 100ms no data

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
            printf("[CAN ERR] TXB0CTRL=0x%02X\n", ctrl);
            mcp_write_reg(REG_TXB0CTRL, 0x00);
            return false;
        }
        sleep_ms(1);
    }
    printf("[CAN TIMEOUT]\n");
    return false;
}

// ─── Frame parser state machine ───────────────────────────────────────────────
typedef enum {
    STATE_HUNT_SYNC,
    STATE_COLLECT,
    STATE_DISCARD,
} frame_state_t;

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("[UART->CAN] Starting up...\n");

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    uart_init(BT_UART, BT_BAUD_RATE);
    gpio_set_function(BT_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BT_UART_TX_PIN, GPIO_FUNC_UART);
    uart_set_format(BT_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BT_UART, true);
    printf("[UART->CAN] UART0 RX on GP1 @ %d baud\n", BT_BAUD_RATE);

    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    if (mcp2515_init())
        printf("[UART->CAN] MCP2515 Normal Mode. Ready.\n\n");
    else
        printf("[UART->CAN] WARNING: MCP2515 not in Normal Mode\n\n");

    printf("[UART->CAN] CAN ID=0x636  DLC=8  250kbps\n");
    printf("[UART->CAN] Heartbeat=01 @ 10ms when UART silent\n\n");

    frame_state_t  state          = STATE_HUNT_SYNC;
    uint8_t        header[HEADER_BYTES];
    uint8_t        header_idx     = 0;
    uint16_t       discard_rem    = 0;
    uint32_t       frames_sent    = 0;
    uint32_t       frames_failed  = 0;

    uint32_t       last_uart_rx   = 0;   // timestamp of last UART byte
    uint32_t       last_heartbeat = 0;   // timestamp of last heartbeat TX
    bool           uart_active    = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // ── Drain all available UART bytes ────────────────────────────────────
        while (uart_is_readable(BT_UART)) {
            uint8_t b = uart_getc(BT_UART);
            last_uart_rx  = now;
            uart_active   = true;

            switch (state) {

            case STATE_HUNT_SYNC:
                if (b == SYNC_BYTE) {
                    header[0]  = b;
                    header_idx = 1;
                    state      = STATE_COLLECT;
                }
                break;

            case STATE_COLLECT:
                header[header_idx++] = b;
                if (header_idx == HEADER_BYTES) {
                    uint16_t length16 = (uint16_t)header[2] |
                                        ((uint16_t)header[3] << 8);
                    discard_rem = (length16 >= 2) ? (length16 - 2) : 0;

                    bool ok = can_send_value(header[4]); // send opcode_lo as value
                    if (ok) {
                        frames_sent++;
                        printf("[TX UART #%u] opcode=%02X%02X val=%02X\n",
                               (unsigned)frames_sent,
                               header[5], header[4], header[4]);
                        gpio_put(PICO_DEFAULT_LED_PIN, 1);
                        sleep_ms(50);
                        gpio_put(PICO_DEFAULT_LED_PIN, 0);
                    } else {
                        frames_failed++;
                        printf("[TX FAIL #%u]\n", (unsigned)frames_failed);
                    }

                    header_idx = 0;
                    state = (discard_rem > 0) ? STATE_DISCARD : STATE_HUNT_SYNC;
                }
                break;

            case STATE_DISCARD:
                discard_rem--;
                if (discard_rem == 0)
                    state = STATE_HUNT_SYNC;
                break;
            }
        }

        // ── Detect UART silence ───────────────────────────────────────────────
        if (uart_active && (now - last_uart_rx >= UART_SILENCE_MS)) {
            uart_active = false;
            printf("[HEARTBEAT] UART silent, switching to heartbeat 01\n");
        }

        // ── Heartbeat: send 01 every 10ms when UART is silent ─────────────────
        if (!uart_active && (now - last_heartbeat >= HEARTBEAT_PERIOD_MS)) {
            can_send_value(HEARTBEAT_VALUE);
            last_heartbeat = now;
        }
    }

    return 0;
}
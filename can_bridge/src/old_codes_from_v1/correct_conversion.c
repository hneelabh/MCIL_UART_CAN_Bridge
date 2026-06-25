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
#define CAN_DLC         8
#define SYNC_BYTE       0xAA
#define HEADER_BYTES    6       // sync(1) + seq(1) + length(2) + opcode(2)
#define CAN_SIDH        0xC6
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

// ─── MCP2515 low-level ────────────────────────────────────────────────────────
static void mcp_write_reg(uint8_t reg, uint8_t data) {
    uint8_t buf[3] = {MCP_WRITE, reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 3);
    cs_deselect();
}

// FIXED: single atomic 3-byte SPI transaction
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
    mcp_write_reg(REG_CNF1, 0x00);
    mcp_write_reg(REG_CNF2, 0xBA);
    mcp_write_reg(REG_CNF3, 0x03);
    mcp_write_reg(REG_CANCTRL, 0x00);
    sleep_ms(10);
    uint8_t stat = mcp_read_reg(REG_CANSTAT);
    printf("[MCP] CANSTAT=0x%02X (expect 0x00 for Normal)\n", stat);
    return ((stat & 0xE0) == 0x00);
}

static bool can_send(const uint8_t *data, uint8_t len) {
    uint8_t frame[CAN_DLC] = {0};
    memcpy(frame, data, len > CAN_DLC ? CAN_DLC : len);

    // Clear any previous TX error before loading frame
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
    printf("[CAN TIMEOUT] TXB0CTRL=0x%02X\n", mcp_read_reg(REG_TXB0CTRL));
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

    if (mcp2515_init()) {
        printf("[UART->CAN] MCP2515 Normal Mode. Ready.\n\n");
    } else {
        printf("[UART->CAN] WARNING: MCP2515 not in Normal Mode\n\n");
    }

    printf("[UART->CAN] Listening on GP1 @ 115200...\n");
    printf("[UART->CAN] CAN ID=0x636  DLC=8  250kbps\n\n");

    frame_state_t  state         = STATE_HUNT_SYNC;
    uint8_t        header[HEADER_BYTES];
    uint8_t        header_idx    = 0;
    uint16_t       discard_rem   = 0;
    uint32_t       frames_sent   = 0;
    uint32_t       frames_failed = 0;
    uint32_t       uart_bytes_rx = 0;
    uint32_t       sync_misses   = 0;
    uint32_t       last_report   = 0;

    while (true) {
        while (uart_is_readable(BT_UART)) {
            uint8_t b = uart_getc(BT_UART);
            uart_bytes_rx++;

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
                    /*
                     * little-endian length field:
                     *   header[2] = len_lo, header[3] = len_hi
                     *   length = total bytes after length field
                     *          = opcode(2) + params(N) + checksum(1)
                     *   discard = length - 2  (opcode already consumed)
                     *           = params(N) + checksum(1)
                     */
                    uint16_t length16 = (uint16_t)header[2] |
                                        ((uint16_t)header[3] << 8);
                    discard_rem = (length16 >= 2) ? (length16 - 2) : 0;

                    bool ok = can_send(header, HEADER_BYTES);
                    if (ok) {
                        frames_sent++;
                        printf("[TX OK  #%u] %02X %02X %02X %02X %02X %02X 00 00\n",
                               (unsigned)frames_sent,
                               header[0], header[1], header[2],
                               header[3], header[4], header[5]);
                    } else {
                        frames_failed++;
                        printf("[TX FAIL #%u] opcode=%02X %02X\n",
                               (unsigned)frames_failed, header[4], header[5]);
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
    }

    return 0;
}
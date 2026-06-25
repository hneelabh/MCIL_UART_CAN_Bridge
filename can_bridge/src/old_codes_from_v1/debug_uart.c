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
#define REG_TXB0CTRL   0x30
#define REG_TXB0SIDH   0x31
#define REG_TXB0SIDL   0x32
#define REG_TXB0EID8   0x33
#define REG_TXB0EID0   0x34
#define REG_TXB0DLC    0x35
#define REG_TXB0D0     0x36

// ─── CAN ──────────────────────────────────────────────────────────────────────
#define CAN_DLC         8
#define CAN_SIDH        0xC6
#define CAN_SIDL        0xC0

// ─── ACI Protocol ─────────────────────────────────────────────────────────────
#define SYNC_BYTE       0xAA
#define HEADER_BYTES    6
#define MAX_PACKET_SIZE 128

// ─── ACTUAL opcodes from your BT module ───────────────────────────────────────
//  (discovered via raw packet logging — NOT from ACI documentation)
#define OPC_BT_STATUS          0x0413   // BT link + profile status
#define OPC_DISCOVERY          0x0021   // service search / discovery
#define OPC_PLAYER_STATUS      0x000B   // media player state
#define OPC_CALL_STATUS        0x000A   // HFP call state
#define OPC_VOL_CHANGE         0x0020   // audio volume change
#define OPC_PROFILE_STATUS     0x0003   // profile connect (keep as fallback)

// ─── CAN Signal Values ────────────────────────────────────────────────────────
#define SIG_FEATURE_RGB_LED   0x01

#define VAL_BT_CONNECTED      0x01
#define VAL_BT_DISCONNECTED   0x02
#define VAL_INCOMING_CALL     0x03
#define VAL_CALL_REJECTED     0x04
#define VAL_VOLUME_UP         0x05
#define VAL_VOLUME_DOWN       0x06
#define VAL_MEDIA_CHANGED     0x07
#define VAL_MUTE              0x08
#define VAL_BRIGHTNESS        0x00

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
    mcp_write_reg(REG_CNF1, 0x00);
    mcp_write_reg(REG_CNF2, 0xBA);
    mcp_write_reg(REG_CNF3, 0x03);
    mcp_write_reg(REG_CANCTRL, 0x00);
    sleep_ms(10);
    uint8_t stat = mcp_read_reg(REG_CANSTAT);
    printf("[MCP] CANSTAT=0x%02X\n", stat);
    return ((stat & 0xE0) == 0x00);
}

static bool can_send_raw(const uint8_t *data, uint8_t len) {
    uint8_t frame[CAN_DLC] = {0};
    memcpy(frame, data, len > CAN_DLC ? CAN_DLC : len);

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
        busy_wait_us(100);
    }
    return false;
}

// ─── CAN signal queue ─────────────────────────────────────────────────────────
#define CAN_QUEUE_DEPTH  4

typedef struct { uint8_t value; uint8_t sub; } can_sig_t;

static can_sig_t can_q[CAN_QUEUE_DEPTH];
static volatile uint8_t can_q_head = 0;
static volatile uint8_t can_q_tail = 0;

static void queue_can(uint8_t value, uint8_t sub) {
    uint8_t next = (can_q_head + 1) % CAN_QUEUE_DEPTH;
    if (next == can_q_tail)
        can_q_tail = (can_q_tail + 1) % CAN_QUEUE_DEPTH;
    can_q[can_q_head].value = value;
    can_q[can_q_head].sub   = sub;
    can_q_head = next;
}

static bool can_q_empty(void) { return (can_q_head == can_q_tail); }

// ─── State tracking ───────────────────────────────────────────────────────────
static uint8_t  prev_bt_profiles    = 0xFF;   // 0x0413 params[6]
static uint8_t  prev_player_status  = 0xFF;   // 0x000B params[6]
static uint8_t  prev_vol_param      = 0xFF;   // 0x0020 params[0]

// ─── Counters ─────────────────────────────────────────────────────────────────
static uint32_t cnt_0413 = 0, cnt_0021 = 0, cnt_000B = 0;
static uint32_t cnt_000A = 0, cnt_0020 = 0, cnt_other = 0;
static uint32_t can_sent = 0, can_fail = 0;

// ─── Packet decoder (non-blocking) ────────────────────────────────────────────
static void decode_packet(const uint8_t *pkt, uint16_t total_len)
{
    uint16_t length    = (uint16_t)pkt[2] | ((uint16_t)pkt[3] << 8);
    uint16_t opcode    = (uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8);
    uint16_t params_len = (length >= 2) ? (length - 2) : 0;
    const uint8_t *params = &pkt[6];

    if (6 + params_len + 1 > total_len || length < 2) return;

    switch (opcode) {

    /* ═══════════════════════════════════════════════════════════════════════
       0x0413  BT LINK + PROFILE STATUS  (discovered — replaces 0x0014)
       
       Payload:  bd_addr(6)  profile_count(1)  [profile_list(N)]
       
       profile_count > 0  →  BT Connected (sub = number of profiles)
       profile_count = 0  →  BT Disconnected
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x0413: {
        cnt_0413++;
        if (params_len < 7) break;

        uint8_t profile_count = params[6];

        // Only act on CHANGE
        if (profile_count == prev_bt_profiles) break;
        prev_bt_profiles = profile_count;

        if (profile_count > 0) {
            queue_can(VAL_BT_CONNECTED, profile_count);
            printf("[CAN>] BT_CONNECTED  profiles=%u\n", profile_count);
        } else {
            queue_can(VAL_BT_DISCONNECTED, 0);
            printf("[CAN>] BT_DISCONNECTED\n");
        }
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       0x000B  PLAYER STATUS  (confirmed working)
       
       Payload:  reserved(6)  status(1)  reserved(1)
       status: 0=Stop  1=Playing  2=Paused
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x000B: {
        cnt_000B++;
        if (params_len < 8) break;

        uint8_t status = params[6];

        if (status == prev_player_status) break;
        prev_player_status = status;

        queue_can(VAL_MEDIA_CHANGED, status);
        printf("[CAN>] MEDIA_CHANGED  %s\n",
               status == 0 ? "Stop" : status == 1 ? "Play" : "Pause");
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       0x000A  HFP CALL STATUS
       
       Payload:  bd_addr(6)  prev_status(1)  curr_status(1)
       
       curr=0x02 → Incoming call
       prev=0x02 && curr=0x00 → Call rejected/ended
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x000A: {
        cnt_000A++;
        if (params_len < 8) break;

        uint8_t prev_s = params[6];
        uint8_t curr_s = params[7];

        if (curr_s == 0x02) {
            queue_can(VAL_INCOMING_CALL, 0);
            printf("[CAN>] INCOMING_CALL\n");
        } else if (prev_s == 0x02 && curr_s == 0x00) {
            queue_can(VAL_CALL_REJECTED, 0);
            printf("[CAN>] CALL_REJECTED\n");
        } else {
            printf("[DBG ] Call prev=%u curr=%u\n", prev_s, curr_s);
        }
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       0x0020  AUDIO VOL CHANGE
       
       Observed payload:  single byte, always 0x00 so far.
       Logging all changes for reverse-engineering.
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x0020: {
        cnt_0020++;
        if (params_len < 1) break;

        uint8_t vol = params[0];

        // Log every change for reverse-engineering
        if (vol != prev_vol_param) {
            printf("[DBG ] VOL_CHANGE param: %u -> %u\n", prev_vol_param, vol);
            prev_vol_param = vol;

            // Attempted decode (adjust once payload format is known):
            // Volume level 0-15 or 0-255:
            //   - Level increasing → VOL_UP
            //   - Level decreasing → VOL_DOWN
            //   - Level == 0      → MUTE
            //
            // For now, since param is always 0x00, we can't determine direction.
            // Sending as BRIGHTNESS placeholder (value 0 = brightness, sub = level):
            queue_can(VAL_BRIGHTNESS, vol);
            printf("[CAN>] BRIGHTNESS(?) level=%u\n", vol);
        }
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       0x0021  DISCOVERY / SERVICE SEARCH  —  ignore, just count
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x0021: {
        cnt_0021++;
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       0x0003  PROFILE STATUS (fallback — may not appear on this module)
       ═══════════════════════════════════════════════════════════════════════ */
    case 0x0003: {
        // Silently count
        break;
    }

    /* ═══════════════════════════════════════════════════════════════════════
       UNKNOWN  —  log first occurrence only
       ═══════════════════════════════════════════════════════════════════════ */
    default: {
        cnt_other++;
        static uint32_t unk_seen[256] = {0};
        uint8_t idx = opcode & 0xFF;
        unk_seen[idx]++;
        if (unk_seen[idx] == 1) {
            printf("[NEW ] opcode=0x%04X params(%u):", opcode, params_len);
            for (uint16_t i = 0; i < params_len && i < 16; i++)
                printf(" %02X", params[i]);
            printf("\n");
        }
        break;
    }
    }
}

// ─── Parser state machine ─────────────────────────────────────────────────────
typedef enum { HUNT, HEADER, PAYLOAD } pstate_t;

static pstate_t  p_state = HUNT;
static uint8_t   pkt_buf[MAX_PACKET_SIZE];
static uint16_t  pkt_idx = 0;
static uint16_t  pay_rem = 0;

static void parse_byte(uint8_t b)
{
    switch (p_state) {
    case HUNT:
        if (b == SYNC_BYTE) {
            pkt_buf[0] = b;
            pkt_idx = 1;
            p_state = HEADER;
        }
        break;
    case HEADER:
        pkt_buf[pkt_idx++] = b;
        if (pkt_idx == HEADER_BYTES) {
            uint16_t len = (uint16_t)pkt_buf[2] | ((uint16_t)pkt_buf[3] << 8);
            if (len < 2 || (5 + len) > MAX_PACKET_SIZE) {
                p_state = HUNT;
                break;
            }
            pay_rem = len - 1;
            p_state = PAYLOAD;
        }
        break;
    case PAYLOAD:
        pkt_buf[pkt_idx++] = b;
        if (--pay_rem == 0) {
            decode_packet(pkt_buf, pkt_idx);
            p_state = HUNT;
        }
        break;
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);
    printf("\n[UART->CAN] v4 Starting...\n");

    uart_init(BT_UART, BT_BAUD_RATE);
    gpio_set_function(BT_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BT_UART_TX_PIN, GPIO_FUNC_UART);
    uart_set_format(BT_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BT_UART, true);
    printf("[INIT] UART @ %d baud\n", BT_BAUD_RATE);

    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    if (mcp2515_init())
        printf("[INIT] MCP2515 OK\n");
    else
        printf("[INIT] MCP2515 FAIL\n");

    printf("[INIT] CAN 0x636 DLC=8 250kbps\n");
    printf("[INIT] Signals: 1=BtConn 2=BtDisc 3=InCall 4=CallRej");
    printf(" 5=VolUp 6=VolDn 7=Media 8=Mute 0=Brightness\n");
    printf("[INIT] BT status on opcode 0x0413 (auto-detected)\n\n");

    uint32_t last_stats = to_ms_since_boot(get_absolute_time());

    while (true) {

        // Step 1: Drain UART
        while (uart_is_readable(BT_UART))
            parse_byte(uart_getc(BT_UART));

        // Step 2: Send queued CAN signals
        while (!can_q_empty()) {
            can_sig_t sig = can_q[can_q_tail];
            can_q_tail = (can_q_tail + 1) % CAN_QUEUE_DEPTH;

            uint8_t frame[CAN_DLC] = {
                SIG_FEATURE_RGB_LED, sig.value, sig.sub, 0, 0, 0, 0, 0
            };
            bool ok = can_send_raw(frame, CAN_DLC);
            if (ok) can_sent++; else can_fail++;
            printf("[CAN%s] %02X %02X %02X 00 00 00 00 00\n",
                   ok ? " OK" : " FAIL", frame[0], frame[1], frame[2]);

            // Re-drain UART after CAN send
            while (uart_is_readable(BT_UART))
                parse_byte(uart_getc(BT_UART));
        }

        // Step 3: Stats every 10s
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_stats >= 10000) {
            printf("[STATS] 0x0413=%lu 0x0021=%lu 0x000B=%lu 0x000A=%lu"
                   " 0x0020=%lu other=%lu  CAN_ok=%lu fail=%lu\n",
                   (unsigned long)cnt_0413, (unsigned long)cnt_0021,
                   (unsigned long)cnt_000B, (unsigned long)cnt_000A,
                   (unsigned long)cnt_0020, (unsigned long)cnt_other,
                   (unsigned long)can_sent, (unsigned long)can_fail);
            last_stats = now;
        }
    }

    return 0;
}

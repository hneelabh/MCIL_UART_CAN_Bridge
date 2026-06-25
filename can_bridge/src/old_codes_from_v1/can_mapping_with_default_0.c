/**
 * main.c — RTL8763ESE ACI UART → MCP2515 CAN Bridge
 * ===================================================
 * Hardware connections
 * --------------------
 * RTL8763ESE M3_1 (ACI TX) → Pico GP1  (UART0 RX, Pin 2)
 * RTL8763ESE GND            → Pico GND  (Pin 38)
 *
 * MCP2515 SCK  → GP18 (SPI0 SCK,  Pin 24)
 * MCP2515 SI   → GP19 (SPI0 MOSI, Pin 25)
 * MCP2515 SO   → GP16 (SPI0 MISO, Pin 21) + 1kΩ series + 2kΩ to GND divider
 * MCP2515 CS   → GP17 (Pin 22)
 * MCP2515 GND  → Pin 38
 * MCP2515 VCC  → VBUS / 3.3V per module
 *
 * CAN DB9
 * MCP2515 CAN-H → DB9 Pin 7
 * MCP2515 CAN-L → DB9 Pin 2
 * Pico GND      → DB9 Pin 3
 *
 * CAN: 500 kbps, 8 MHz crystal, ID=0x636 (SIDH=0xC6 SIDL=0xC0), DLC=8
 * UART: 2 000 000 baud, 8N1, RX only
 *
 * Signal Feature_RGB_LED_CAN_In (byte 0 of CAN frame, bytes 1-7 = 0x00):
 *   0  = Heartbeat (idle/no event)
 *   1  = BT Connected       6  = Volume Down
 *   2  = BT Disconnected    7  = Media Changed (next/prev track only)
 *   3  = Incoming Call      8  = Mute
 *   4  = Call Rejected      9  = Play/Pause
 *   5  = Volume Up          10 = Outgoing Call
 *
 * Heartbeat behaviour:
 *   After UART_SILENCE_MS of no UART bytes, and once COOLDOWN_PERIOD_MS
 *   has elapsed since the last UART byte, the bridge sends CAN value 0x00
 *   every HEARTBEAT_PERIOD_MS. This keeps the Traveo 2.0 CAN timeout happy.
 *   Any new UART byte immediately resets the cooldown and stops the heartbeat.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

/* =========================================================================
 * CONFIG
 * ========================================================================= */

#define CAN_SIDH            0xC6u       /* (0x636 >> 3) — Traveo 2.0 filter */
#define CAN_SIDL            0xC0u       /* (0x636 & 7) << 5 */
#define CAN_DLC             8u

/* BT event signal values */
#define SIG_HEARTBEAT       0u
#define SIG_BT_CONNECTED    1u
#define SIG_BT_DISCONNECTED 2u
#define SIG_INCOMING_CALL   3u
#define SIG_CALL_REJECTED   4u
#define SIG_VOLUME_UP       5u
#define SIG_VOLUME_DOWN     6u
#define SIG_MEDIA_CHANGED   7u
#define SIG_MUTE            8u
#define SIG_PLAY_PAUSE      9u
#define SIG_OUTGOING_CALL   10u

/* Heartbeat timing */
#define HEARTBEAT_PERIOD_MS 10u     /* send 0x00 every 10 ms when idle */
#define UART_SILENCE_MS     100u    /* declare UART silent after 100 ms no data */
#define COOLDOWN_PERIOD_MS  4000u   /* wait 4 s after last UART byte before heartbeat */
#define LED_ON_MS           50u     /* LED stays lit for 50 ms after last UART byte */

#define MCP_SPI             spi0
#define MCP_PIN_SCK         18
#define MCP_PIN_MOSI        19
#define MCP_PIN_MISO        16
#define MCP_PIN_CS          17
#define MCP_SPI_BAUD        1000000u    /* 1 MHz — conservative, matches file 1 */

/* 500 kbps @ 8 MHz crystal */
#define MCP_CNF1            0x00u
#define MCP_CNF2            0x90u   /* matches working file 1 */
#define MCP_CNF3            0x02u   /* matches working file 1 */

#define BT_UART             uart0
#define BT_UART_RX_PIN      1
#define BT_UART_TX_PIN      0
#define BT_UART_BAUD        2000000u

/* =========================================================================
 * MCP2515
 * ========================================================================= */

#define MCP_CMD_RESET       0xC0u
#define MCP_CMD_READ        0x03u
#define MCP_CMD_WRITE       0x02u
#define MCP_CMD_RTS_TXB0    0x81u
#define MCP_CMD_BIT_MODIFY  0x05u

#define MCP_REG_CANSTAT     0x0Eu
#define MCP_REG_CANCTRL     0x0Fu
#define MCP_REG_CNF3        0x28u
#define MCP_REG_CNF2        0x29u
#define MCP_REG_CNF1        0x2Au
#define MCP_REG_CANINTE     0x2Bu
#define MCP_REG_CANINTF     0x2Cu
#define MCP_REG_TXB0CTRL    0x30u
#define MCP_REG_TXB0SIDH    0x31u
#define MCP_REG_TXB0SIDL    0x32u
#define MCP_REG_TXB0DLC     0x35u
#define MCP_REG_TXB0D0      0x36u

#define MCP_MODE_NORMAL     0x00u
#define MCP_MODE_CONFIG     0x80u
#define MCP_MODE_MASK       0xE0u

static inline void cs_low(void)  { gpio_put(MCP_PIN_CS, 0); }
static inline void cs_high(void) { gpio_put(MCP_PIN_CS, 1); }

static void mcp_write_reg(uint8_t addr, uint8_t val)
{
    uint8_t buf[3] = { MCP_CMD_WRITE, addr, val };
    cs_low(); spi_write_blocking(MCP_SPI, buf, 3); cs_high();
}

/*
 * FIX: Use a single full-duplex spi_write_read_blocking call.
 * The MCP2515 clocks out the response byte simultaneously with the
 * command/address phase — splitting into write then read misses it.
 */
static uint8_t mcp_read_reg(uint8_t addr)
{
    uint8_t tx[3] = { MCP_CMD_READ, addr, 0x00 };
    uint8_t rx[3] = { 0x00, 0x00, 0x00 };
    cs_low();
    spi_write_read_blocking(MCP_SPI, tx, rx, 3);
    cs_high();
    return rx[2];
}

static void mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data)
{
    uint8_t buf[4] = { MCP_CMD_BIT_MODIFY, addr, mask, data };
    cs_low(); spi_write_blocking(MCP_SPI, buf, 4); cs_high();
}

static bool mcp_set_mode(uint8_t mode)
{
    mcp_bit_modify(MCP_REG_CANCTRL, MCP_MODE_MASK, mode);
    sleep_ms(5);
    uint8_t stat = mcp_read_reg(MCP_REG_CANSTAT);
    printf("[MCP] mcp_set_mode(0x%02X): CANSTAT=0x%02X\n", mode, stat);
    return (stat & MCP_MODE_MASK) == mode;
}

static bool mcp2515_init(void)
{
    spi_init(MCP_SPI, MCP_SPI_BAUD);
    spi_set_format(MCP_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(MCP_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(MCP_PIN_MISO, GPIO_FUNC_SPI);
    gpio_init(MCP_PIN_CS);
    gpio_set_dir(MCP_PIN_CS, GPIO_OUT);
    cs_high();

    uint8_t rst = MCP_CMD_RESET;
    cs_low(); spi_write_blocking(MCP_SPI, &rst, 1); cs_high();
    sleep_ms(10);

    if (!mcp_set_mode(MCP_MODE_CONFIG)) {
        printf("[MCP] ERROR: config mode failed (CANSTAT=0x%02X)\n",
               mcp_read_reg(MCP_REG_CANSTAT));
        return false;
    }

    mcp_write_reg(MCP_REG_CNF1, MCP_CNF1);
    mcp_write_reg(MCP_REG_CNF2, MCP_CNF2);
    mcp_write_reg(MCP_REG_CNF3, MCP_CNF3);
    mcp_write_reg(MCP_REG_CANINTE, 0x00);
    mcp_write_reg(MCP_REG_CANINTF, 0x00);
    mcp_write_reg(MCP_REG_TXB0CTRL, 0x00);

    if (!mcp_set_mode(MCP_MODE_NORMAL)) {
        printf("[MCP] ERROR: normal mode failed (CANSTAT=0x%02X)\n",
               mcp_read_reg(MCP_REG_CANSTAT));
        return false;
    }

    printf("[MCP] Init OK — 500 kbps, 8 MHz crystal\n");
    return true;
}

static bool can_send(uint8_t signal_value)
{
    /*
     * FIX: Clear TXB0CTRL before loading the frame to ensure TXREQ
     * is not asserted from a previous (possibly timed-out) send.
     */
    mcp_write_reg(MCP_REG_TXB0CTRL, 0x00);

    /* Write ID + DLC via sequential register write */
    uint8_t hdr[7] = { MCP_CMD_WRITE, MCP_REG_TXB0SIDH,
                       CAN_SIDH, CAN_SIDL, 0x00, 0x00, CAN_DLC };
    cs_low(); spi_write_blocking(MCP_SPI, hdr, 7); cs_high();

    /* Write data bytes */
    uint8_t payload[8];
    memset(payload, 0x00, 8);
    payload[0] = signal_value;

    uint8_t data_cmd[2] = { MCP_CMD_WRITE, MCP_REG_TXB0D0 };
    cs_low();
    spi_write_blocking(MCP_SPI, data_cmd, 2);
    spi_write_blocking(MCP_SPI, payload, 8);
    cs_high();

    /* Request-to-send */
    uint8_t rts = MCP_CMD_RTS_TXB0;
    cs_low(); spi_write_blocking(MCP_SPI, &rts, 1); cs_high();

    /*
     * Poll TXB0CTRL for up to 50 ms:
     *   bit3 (TXREQ) clear = transmitted OK
     *   bit4 (TXERR) set   = bus error, abort
     */
    for (int i = 0; i < 50; i++) {
        uint8_t ctrl = mcp_read_reg(MCP_REG_TXB0CTRL);
        if (!(ctrl & 0x08)) {
            if (signal_value != SIG_HEARTBEAT)
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

/* =========================================================================
 * ACI PARSER
 *
 * Frame: AA [SeqN] [LenL LenH] [OpcL OpcH] [params...] [CheckSum]
 * Len   = 2(opcode) + N(params)
 * ChkSum = (0x00 - sum(SeqN..last_param)) & 0xFF
 * ========================================================================= */

#define ACI_SYNC            0xAAu
#define ACI_MAX_PARAMS      64u

#define OPC_BR_LINK_STATUS  0x0014u
#define OPC_CALL_STATUS     0x000Au
#define OPC_PLAYER_STATUS   0x000Bu
#define OPC_VOLUME_SYNC     0x0013u
#define OPC_AUDIO_VOL       0x0020u
#define OPC_TRACK_CHANGED   0x0021u

typedef enum {
    ST_SYNC, ST_SEQN, ST_LEN_L, ST_LEN_H,
    ST_OPC_L, ST_OPC_H, ST_PARAMS, ST_CHK
} parse_state_t;

/*
 * Call state
 *
 * CALL_IDLE     : no active call
 * CALL_INCOMING : phone is ringing, not yet answered
 * CALL_OUTGOING : outgoing call initiated, not yet answered
 * CALL_ACTIVE   : call answered and in progress
 */
typedef enum {
    CALL_IDLE,
    CALL_INCOMING,
    CALL_OUTGOING,
    CALL_ACTIVE
} call_state_t;

static parse_state_t  g_state      = ST_SYNC;
static uint8_t        g_seqn;
static uint16_t       g_length;
static uint16_t       g_opcode;
static uint8_t        g_params[ACI_MAX_PARAMS];
static uint16_t       g_param_idx;
static uint16_t       g_params_rem;
static uint16_t       g_chk_acc;

static int16_t       g_last_vol      = -1;
static bool          g_vol_baselined = false;
static bool          g_bt_connected  = false;
static call_state_t  g_call_state    = CALL_IDLE;

static void parser_reset(void)
{
    g_state      = ST_SYNC;
    g_param_idx  = 0;
    g_params_rem = 0;
    g_chk_acc    = 0;
}

static void on_packet(uint16_t opcode, const uint8_t *params, uint8_t plen)
{
    switch (opcode) {

    /* ── BT ACL link status ──────────────────────────────────────────────── */
    case OPC_BR_LINK_STATUS:
        if (plen < 1) break;
        if (params[0] == 0x01) {
            printf("[EVT] BT Connected\n");
            g_bt_connected  = true;
            g_last_vol      = -1;
            g_vol_baselined = false;
            g_call_state    = CALL_IDLE;
            can_send(SIG_BT_CONNECTED);
        } else {
            printf("[EVT] BT Disconnected (state=0x%02X)\n", params[0]);
            g_bt_connected  = false;
            g_last_vol      = -1;
            g_vol_baselined = false;
            g_call_state    = CALL_IDLE;
            can_send(SIG_BT_DISCONNECTED);
        }
        break;

    /* ── HFP call status ─────────────────────────────────────────────────── */
    case OPC_CALL_STATUS:
        printf("[CALL_RAW] plen=%u: ", plen);
        for (uint8_t i = 0; i < plen && i < 16; i++)
            printf("%02X ", params[i]);
        printf("\n");

        if (plen < 2) {
            printf("[CALL] plen too short (%u), skipping\n", plen);
            break;
        }
        {
            uint8_t prev = params[0];
            uint8_t curr = params[1];
            printf("[EVT] Call status prev=0x%02X curr=0x%02X\n", prev, curr);

            if (curr == 0x02) {
                g_call_state = CALL_INCOMING;
                can_send(SIG_INCOMING_CALL);
            } else if (curr == 0x03) {
                g_call_state = CALL_OUTGOING;
                can_send(SIG_OUTGOING_CALL);
            } else if (curr == 0x04) {
                g_call_state = CALL_ACTIVE;
            } else if (curr == 0x00) {
                if (g_call_state == CALL_INCOMING) {
                    can_send(SIG_CALL_REJECTED);
                }
                g_call_state    = CALL_IDLE;
                g_vol_baselined = false;
            }
        }
        break;

    /* ── AVRCP player status ─────────────────────────────────────────────── */
    case OPC_PLAYER_STATUS:
        if (plen < 7) break;
        {
            uint8_t status = params[6];
            printf("[EVT] Player status=0x%02X\n", status);
            if (status == 0x01 || status == 0x02) {
                can_send(SIG_PLAY_PAUSE);
            }
        }
        break;

    /* ── Volume sync (HFP) ───────────────────────────────────────────────── */
    case OPC_VOLUME_SYNC:
        if (plen < 7) break;
        printf("[EVT] Volume sync gain=0x%02X (ignored for CAN)\n", params[6]);
        break;

    /* ── AVRCP absolute volume change ────────────────────────────────────── */
    case OPC_AUDIO_VOL:
        if (plen < 1) break;
        {
            uint8_t v = params[0];
            printf("[EVT] Audio vol=0x%02X last=0x%02X baselined=%d call=%d\n",
                   v, (uint8_t)g_last_vol, g_vol_baselined, g_call_state);

            if (g_call_state != CALL_IDLE) {
                printf("[EVT] Vol event suppressed during call\n");
                g_last_vol = v;
                break;
            }

            if (!g_vol_baselined) {
                printf("[EVT] Vol baseline set to 0x%02X (suppressed)\n", v);
                g_last_vol      = v;
                g_vol_baselined = true;
            } else if (v == 0x00 && (uint8_t)g_last_vol > 0x00) {
                printf("[EVT] Mute\n");
                g_last_vol = 0;
                can_send(SIG_MUTE);
            } else if (v > (uint8_t)g_last_vol) {
                printf("[EVT] Volume Up (0x%02X->0x%02X)\n",
                       (uint8_t)g_last_vol, v);
                g_last_vol = v;
                can_send(SIG_VOLUME_UP);
            } else if (v < (uint8_t)g_last_vol) {
                printf("[EVT] Volume Down (0x%02X->0x%02X)\n",
                       (uint8_t)g_last_vol, v);
                g_last_vol = v;
                can_send(SIG_VOLUME_DOWN);
            }
        }
        break;

    /* ── Track changed ───────────────────────────────────────────────────── */
    case OPC_TRACK_CHANGED:
        printf("[EVT] Media Changed (track)\n");
        can_send(SIG_MEDIA_CHANGED);
        break;

    default:
        printf("[ACI] opcode=0x%04X plen=%u: ", opcode, plen);
        for (uint8_t i = 0; i < plen && i < 16; i++)
            printf("%02X ", params[i]);
        printf("\n");
        break;
    }
}

static void parser_feed(uint8_t byte)
{
    switch (g_state) {
    case ST_SYNC:
        if (byte == ACI_SYNC) { parser_reset(); g_state = ST_SEQN; }
        break;
    case ST_SEQN:
        g_seqn    = byte;
        g_chk_acc = byte;
        g_state   = ST_LEN_L;
        break;
    case ST_LEN_L:
        g_length   = byte;
        g_chk_acc += byte;
        g_state    = ST_LEN_H;
        break;
    case ST_LEN_H:
        g_length  |= ((uint16_t)byte << 8);
        g_chk_acc += byte;
        if (g_length < 2) { parser_reset(); break; }
        g_params_rem = g_length - 2;
        g_state = ST_OPC_L;
        break;
    case ST_OPC_L:
        g_opcode   = byte;
        g_chk_acc += byte;
        g_state    = ST_OPC_H;
        break;
    case ST_OPC_H:
        g_opcode  |= ((uint16_t)byte << 8);
        g_chk_acc += byte;
        g_param_idx = 0;
        g_state = (g_params_rem == 0) ? ST_CHK : ST_PARAMS;
        break;
    case ST_PARAMS:
        if (g_param_idx < ACI_MAX_PARAMS) g_params[g_param_idx] = byte;
        g_param_idx++;
        g_chk_acc += byte;
        if (--g_params_rem == 0) g_state = ST_CHK;
        break;
    case ST_CHK: {
        uint8_t expected = (uint8_t)(0x00u - (g_chk_acc & 0xFFu));
        if (byte == expected) {
            uint8_t plen = (g_param_idx < ACI_MAX_PARAMS)
                           ? (uint8_t)g_param_idx : ACI_MAX_PARAMS;
            on_packet(g_opcode, g_params, plen);
        } else {
            printf("[ACI] Chksum fail opc=0x%04X got=0x%02X exp=0x%02X\n",
                   g_opcode, byte, expected);
        }
        parser_reset();
        break;
    }
    default:
        parser_reset();
        break;
    }
}

/* =========================================================================
 * MAIN
 * ========================================================================= */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);

    printf("=== BT UART -> CAN Bridge ===\n");
    printf("CAN SIDH: 0x%02X  SIDL: 0x%02X  DLC: %u  Speed: 500kbps\n",
           CAN_SIDH, CAN_SIDL, CAN_DLC);
    printf("UART: %u baud on GP%d\n", BT_UART_BAUD, BT_UART_RX_PIN);
    printf("Heartbeat: 0x00 @ %ums after %ums silence + %ums cooldown\n",
           HEARTBEAT_PERIOD_MS, UART_SILENCE_MS, COOLDOWN_PERIOD_MS);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    if (!mcp2515_init()) {
        /* Fast blink = MCP init failed */
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
        }
    }

    uart_init(BT_UART, BT_UART_BAUD);
    gpio_set_function(BT_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BT_UART_TX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(BT_UART, false, false);
    uart_set_format(BT_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BT_UART, true);

    printf("[MAIN] Running...\n");

    /* ── Heartbeat state ─────────────────────────────────────────────────── */
    uint32_t last_uart_rx      = 0;
    uint32_t last_heartbeat    = 0;
    uint32_t silence_start     = 0;
    uint32_t led_last_rx       = 0;
    bool     uart_active       = false;
    bool     in_cooldown       = false;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        /* ── Drain all available UART bytes ────────────────────────────── */
        while (uart_is_readable(BT_UART)) {
            uint8_t b = uart_getc(BT_UART);

            last_uart_rx = now;
            led_last_rx  = now;
            gpio_put(PICO_DEFAULT_LED_PIN, 1);

            if (!uart_active) {
                uart_active = true;
                printf("[UART] Activity resumed\n");
            }
            in_cooldown = false;

            parser_feed(b);
        }

        /* ── Detect UART going silent ───────────────────────────────────── */
        if (uart_active && (now - last_uart_rx >= UART_SILENCE_MS)) {
            uart_active   = false;
            in_cooldown   = true;
            silence_start = now;
            printf("[UART] Silent — starting %u ms cooldown\n", COOLDOWN_PERIOD_MS);
        }

        /* ── Expire cooldown ────────────────────────────────────────────── */
        if (in_cooldown && (now - silence_start >= COOLDOWN_PERIOD_MS)) {
            in_cooldown    = false;
            last_heartbeat = now;
            printf("[HEARTBEAT] Cooldown expired — starting 0x00 @ %u ms\n",
                   HEARTBEAT_PERIOD_MS);
        }

        /* ── Heartbeat TX ───────────────────────────────────────────────── */
        if (!uart_active && !in_cooldown &&
            (now - last_heartbeat >= HEARTBEAT_PERIOD_MS)) {
            can_send(SIG_HEARTBEAT);
            last_heartbeat = now;
        }

        /* ── LED off after LED_ON_MS of no UART bytes ───────────────────── */
        if (gpio_get(PICO_DEFAULT_LED_PIN) &&
            (now - led_last_rx >= LED_ON_MS)) {
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
        }
    }

    return 0;
}
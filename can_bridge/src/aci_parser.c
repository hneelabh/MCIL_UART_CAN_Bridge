/**
 * aci_parser.c — RTL8763ESE ACI protocol parser
 *
 * Consumes raw UART bytes via parser_feed().  When a complete, valid frame
 * arrives, on_packet() maps the ACI opcode + params to a CAN signal value
 * and calls can_send().
 */

#include "aci_parser.h"
#include "can_signals.h"
#include "mcp2515.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/* =========================================================================
 * Parser state machine types
 * ========================================================================= */

typedef enum {
    ST_SYNC,
    ST_SEQN,
    ST_LEN_L,
    ST_LEN_H,
    ST_OPC_L,
    ST_OPC_H,
    ST_PARAMS,
    ST_CHK
} parse_state_t;

/*
 * Call state
 *   CALL_IDLE     : no active call
 *   CALL_INCOMING : ringing, not yet answered
 *   CALL_OUTGOING : dialling, not yet answered
 *   CALL_ACTIVE   : answered, in progress
 */
typedef enum {
    CALL_IDLE,
    CALL_INCOMING,
    CALL_OUTGOING,
    CALL_ACTIVE
} call_state_t;

/* =========================================================================
 * Parser state (file-scope)
 * ========================================================================= */

static parse_state_t  g_state      = ST_SYNC;
static uint8_t        g_seqn;
static uint16_t       g_length;
static uint16_t       g_opcode;
static uint8_t        g_params[ACI_MAX_PARAMS];
static uint16_t       g_param_idx;
static uint16_t       g_params_rem;
static uint16_t       g_chk_acc;

/* =========================================================================
 * Application state (file-scope)
 * ========================================================================= */

static int16_t      g_last_vol      = -1;
static bool         g_vol_baselined = false;
static bool         g_bt_connected  = false;
static call_state_t g_call_state    = CALL_IDLE;

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

static void parser_reset(void)
{
    g_state      = ST_SYNC;
    g_param_idx  = 0;
    g_params_rem = 0;
    g_chk_acc    = 0;
}

/* =========================================================================
 * on_packet — ACI opcode dispatcher
 * ========================================================================= */

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
                if (g_call_state == CALL_INCOMING)
                    can_send(SIG_CALL_REJECTED);
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
            if (status == 0x01 || status == 0x02)
                can_send(SIG_PLAY_PAUSE);
        }
        break;

    /* ── Volume sync (HFP) — no CAN action, log only ────────────────────── */
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
                /* First volume event after connect — treat as baseline, no CAN */
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

/* =========================================================================
 * Public API
 * ========================================================================= */

void parser_feed(uint8_t byte)
{
    switch (g_state) {

    case ST_SYNC:
        if (byte == ACI_SYNC) {
            parser_reset();
            g_state = ST_SEQN;
        }
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
        g_params_rem = g_length - 2;    /* subtract 2 opcode bytes */
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
        if (g_param_idx < ACI_MAX_PARAMS)
            g_params[g_param_idx] = byte;
        g_param_idx++;
        g_chk_acc += byte;
        if (--g_params_rem == 0)
            g_state = ST_CHK;
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
#ifndef ACI_PARSER_H
#define ACI_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * ACI frame format
 *
 *   AA  [SeqN]  [LenL LenH]  [OpcL OpcH]  [params…]  [CheckSum]
 *
 *   Len    = 2 (opcode bytes) + N (param bytes)
 *   ChkSum = (0x00 - sum(SeqN .. last_param)) & 0xFF
 * ========================================================================= */

#define ACI_SYNC            0xAAu
#define ACI_MAX_PARAMS      64u

/* Opcodes handled by on_packet() */
#define OPC_BR_LINK_STATUS  0x0014u
#define OPC_CALL_STATUS     0x000Au
#define OPC_PLAYER_STATUS   0x000Bu
#define OPC_VOLUME_SYNC     0x0013u
#define OPC_AUDIO_VOL       0x0020u
#define OPC_TRACK_CHANGED   0x0021u

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * Feed one byte from the UART stream into the ACI parser.
 * When a complete, checksum-valid frame is assembled, on_packet() fires
 * internally and the appropriate can_send() call is made.
 */
void parser_feed(uint8_t byte);

#endif /* ACI_PARSER_H */
#ifndef CAN_SIGNALS_H
#define CAN_SIGNALS_H

#include <stdint.h>

/* =========================================================================
 * Feature_RGB_LED_CAN_In signal values  (CAN frame byte 0)
 *
 * CAN ID 0x636, DLC 8.  Bytes 1-7 are always 0x00.
 * ========================================================================= */

#define SIG_HEARTBEAT       0u   /* idle / no event        */
#define SIG_BT_CONNECTED    1u
#define SIG_BT_DISCONNECTED 2u
#define SIG_INCOMING_CALL   3u
#define SIG_CALL_REJECTED   4u
#define SIG_VOLUME_UP       5u
#define SIG_VOLUME_DOWN     6u
#define SIG_MEDIA_CHANGED   7u   /* next / prev track only */
#define SIG_MUTE            8u
#define SIG_PLAY_PAUSE      9u
#define SIG_OUTGOING_CALL   10u

#endif /* CAN_SIGNALS_H */
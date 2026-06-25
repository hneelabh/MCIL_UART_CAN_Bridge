#include "can_signals.h"
#include <stdint.h>

const char *signal_name(uint8_t sig)
{
    switch (sig) {
    case SIG_HEARTBEAT:       return "Heartbeat";
    case SIG_BT_CONNECTED:    return "BT Connected";
    case SIG_BT_DISCONNECTED: return "BT Disconnected";
    case SIG_INCOMING_CALL:   return "Incoming Call";
    case SIG_CALL_REJECTED:   return "Call Rejected";
    case SIG_VOLUME_UP:       return "Volume Up";
    case SIG_VOLUME_DOWN:     return "Volume Down";
    case SIG_MEDIA_CHANGED:   return "Media Changed";
    case SIG_MUTE:            return "Mute";
    case SIG_PLAY_PAUSE:      return "Play/Pause";
    case SIG_OUTGOING_CALL:   return "Outgoing Call";
    default:                  return "Unknown";
    }
}
# MSIL UART–CAN Bridge

Firmware for a Raspberry Pi Pico that bridges a **Realtek RTL8763ESE Bluetooth EVK** to an **Infineon Traveo 2 (CYT2CL)** CAN bus, translating ACI UART events into CAN frames for automotive cluster HMI.

---

## Overview

The RTL8763ESE outputs Bluetooth events over a proprietary serial protocol called ACI (Application Control Interface) at 2 Mbaud. The Traveo 2 cluster expects CAN frames on ID `0x636` to drive the `Feature_RGB_LED_CAN_In` signal. The Pico sits in the middle, parsing the ACI byte stream and transmitting the appropriate CAN value via an MCP2515 CAN controller.

```
RTL8763ESE              Raspberry Pi Pico             Traveo 2 (CYT2CL)
(BT EVK)                                              (Cluster / HMI)

  ACI TX ──── UART0 RX ──► ACI Parser
  (2 Mbaud)               Can Signal Map ──► MCP2515 ──► CAN Bus (500 kbps)
                                             (SPI0)       ID 0x636
```

---

## Hardware

### Components

| Part | Description |
|---|---|
| Raspberry Pi Pico | RP2040 microcontroller — bridge MCU |
| Realtek RTL8763ESE EVK | Bluetooth module, ACI UART output |
| MCP2515 module (blue) | CAN controller with integrated TJA1050 transceiver |
| CAN DB9 connector | Physical CAN bus connection to Traveo 2 |

### Wiring

#### UART (BT Module → Pico)

| RTL8763ESE | Pico Pin | GPIO |
|---|---|---|
| M3_1 (ACI TX) | Pin 2 | GP1 (UART0 RX) |
| GND | Pin 38 | GND |

#### SPI (Pico → MCP2515)

| MCP2515 | Pico Pin | GPIO | Note |
|---|---|---|---|
| SCK | Pin 24 | GP18 | SPI0 SCK |
| SI (MOSI) | Pin 25 | GP19 | SPI0 MOSI |
| SO (MISO) | Pin 21 | GP16 | SPI0 MISO — 1kΩ series + 2kΩ to GND (5V→3.3V divider) |
| CS | Pin 22 | GP17 | Chip select |
| VCC | VBUS / 3.3V | — | Per module variant |
| GND | Pin 38 | GND | |

#### CAN Bus (MCP2515 → DB9 → Traveo 2)

| MCP2515 | DB9 Pin |
|---|---|
| CAN-H | Pin 7 |
| CAN-L | Pin 2 |
| GND | Pin 3 |

> **Note:** The MCP2515 blue module has an onboard TJA1050 transceiver — no external transceiver needed.

---

## Signal Mapping

CAN frame: ID `0x636`, DLC 8. Byte 0 carries the signal value; bytes 1–7 are `0x00`.

| Byte 0 Value | BT Event |
|:---:|---|
| `0x00` | Heartbeat (idle) |
| `0x01` | BT Connected |
| `0x02` | BT Disconnected |
| `0x03` | Incoming Call |
| `0x04` | Call Rejected |
| `0x05` | Volume Up |
| `0x06` | Volume Down |
| `0x07` | Media Changed (next/prev track) |
| `0x08` | Mute |
| `0x09` | Play / Pause |
| `0x0A` | Outgoing Call |

### Heartbeat

When no UART activity is detected for `100 ms`, a cooldown timer starts. After `4000 ms` of continued silence, the bridge begins sending `0x00` every `10 ms` to prevent the Traveo 2 CAN timeout from firing. Any new UART byte immediately cancels the cooldown and stops the heartbeat.

### Volume Handling

The first volume event after a BT connection is treated as a baseline and suppressed (no CAN TX). Subsequent events are compared against the baseline to determine Up / Down / Mute. Volume events during an active call are also suppressed.

---

## CAN Configuration

| Parameter | Value |
|---|---|
| Bit rate | 500 kbps |
| Crystal | 8 MHz |
| CAN ID | `0x636` (standard, 11-bit) |
| SIDH | `0xC6` |
| SIDL | `0xC0` |
| DLC | 8 |
| MCP2515 SPI speed | 1 MHz |
| CNF1 / CNF2 / CNF3 | `0x00` / `0x90` / `0x02` |

---

## Project Structure

```
MSIL_UART_CAN_Bridge/
├── can_bridge/
│   ├── CMakeLists.txt
│   ├── pico_sdk_import.cmake
│   └── src/
│       ├── main.c              # Init, UART drain loop, heartbeat state machine
│       ├── mcp2515.h/.c        # SPI driver, MCP2515 init, can_send()
│       ├── aci_parser.h/.c     # ACI byte-stream parser, BT event dispatcher
│       ├── can_signals.h/.c    # Signal value defines (SIG_*)
│       └── old_codes_from_v1/  # Historical reference — single-file iterations
└── pico-sdk/                   # Pico SDK (not tracked in git)
```

---

## Building

### Prerequisites

```bash
sudo apt install gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential cmake
```

### Build

```bash
cd can_bridge
mkdir build && cd build
cmake ..
make -j4
```

The build produces `can_bridge_v2.uf2` in the `build/` directory.

### Flash

Hold **BOOTSEL** on the Pico, connect USB, then copy the `.uf2`:

```bash
cp build/can_bridge_v2.uf2 /media/$USER/RPI-RP2/
```

---

## Debug Output

USB CDC serial (`115200` baud) prints live event logs:

```
=== BT UART -> CAN Bridge (v2) ===
[MCP] Init OK — 500 kbps, 8 MHz crystal
[MAIN] Running...
[EVT] BT Connected
[CAN] TX signal=1  SIDH=0xC6
[EVT] Volume Up (0x30->0x40)
[CAN] TX signal=5  SIDH=0xC6
[UART] Silent — starting 4000 ms cooldown
[HEARTBEAT] Cooldown expired — 0x00 @ 10 ms
```

The on-board LED lights while UART bytes are arriving and extinguishes 50 ms after the last byte. Fast blinking at startup indicates MCP2515 initialisation failure (check SPI wiring).

---

## ACI Protocol (RTL8763ESE)

Frames follow the structure:

```
AA  [SeqN]  [LenL LenH]  [OpcL OpcH]  [params...]  [CheckSum]
```

- **Len** = 2 (opcode bytes) + N (param bytes)
- **Checksum** = `(0x00 - sum(SeqN .. last_param)) & 0xFF`

Opcodes handled:

| Opcode | Event |
|---|---|
| `0x0014` | BR Link Status (connect / disconnect) |
| `0x000A` | HFP Call Status |
| `0x000B` | AVRCP Player Status (play/pause) |
| `0x0013` | Volume Sync (HFP, logged only) |
| `0x0020` | AVRCP Absolute Volume Change |
| `0x0021` | Track Changed |

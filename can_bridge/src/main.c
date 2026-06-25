/**
 * main.c — RTL8763ESE ACI UART → MCP2515 CAN Bridge  (can_bridge_v2)
 * ===================================================================
 *
 * Hardware connections
 * --------------------
 * RTL8763ESE M3_1 (ACI TX) → Pico GP1  (UART0 RX, Pin 2)
 * RTL8763ESE GND            → Pico GND  (Pin 38)
 *
 * MCP2515 SCK  → GP18 (SPI0 SCK,  Pin 24)
 * MCP2515 SI   → GP19 (SPI0 MOSI, Pin 25)
 * MCP2515 SO   → GP16 (SPI0 MISO, Pin 21)  + 1kΩ/2kΩ divider
 * MCP2515 CS   → GP17 (Pin 22)
 * MCP2515 VCC  → VBUS / 3.3 V per module
 * MCP2515 GND  → Pin 38
 *
 * CAN DB9
 *   CAN-H → DB9 Pin 7     CAN-L → DB9 Pin 2     GND → DB9 Pin 3
 *
 * CAN : 500 kbps, 8 MHz crystal, ID 0x636, DLC 8
 * UART: 2 000 000 baud, 8N1, RX only (GP1)
 *
 * Heartbeat behaviour
 * -------------------
 * After UART_SILENCE_MS of no UART bytes a cooldown timer starts.
 * Once COOLDOWN_PERIOD_MS elapses, the bridge sends CAN value 0x00
 * every HEARTBEAT_PERIOD_MS to keep the Traveo 2 CAN timeout happy.
 * Any new UART byte immediately resets the cooldown and stops the
 * heartbeat.
 *
 * LED behaviour
 * -------------
 * On-board LED lights while UART bytes are arriving; extinguishes
 * LED_ON_MS after the last byte.  Fast blink = MCP2515 init failure.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "mcp2515.h"
#include "aci_parser.h"
#include "can_signals.h"

/* =========================================================================
 * UART config
 * ========================================================================= */
#define BT_UART             uart0
#define BT_UART_RX_PIN      1
#define BT_UART_TX_PIN      0       /* not used — pin reserved by hardware_uart */
#define BT_UART_BAUD        2000000u

/* =========================================================================
 * Heartbeat / LED timing
 * ========================================================================= */
#define HEARTBEAT_PERIOD_MS 10u     /* CAN 0x00 interval when idle   */
#define UART_SILENCE_MS     100u    /* declare silent after 100 ms   */
#define COOLDOWN_PERIOD_MS  4000u   /* wait 4 s before heartbeating  */
#define LED_ON_MS           50u     /* LED hold time after last byte */

/* =========================================================================
 * main
 * ========================================================================= */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);     /* wait for USB CDC enumeration on host */

    printf("=== BT UART -> CAN Bridge (v2) ===\n");
    printf("CAN  SIDH: 0x%02X  SIDL: 0x%02X  DLC: %u  Speed: 500 kbps\n",
           CAN_SIDH, CAN_SIDL, CAN_DLC);
    printf("UART %u baud on GP%d\n", BT_UART_BAUD, BT_UART_RX_PIN);
    printf("Heartbeat 0x00 @ %u ms after %u ms silence + %u ms cooldown\n",
           HEARTBEAT_PERIOD_MS, UART_SILENCE_MS, COOLDOWN_PERIOD_MS);

    /* On-board LED */
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    /* MCP2515 */
    if (!mcp2515_init()) {
        /* Fast blink indicates hardware/SPI fault */
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1); sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0); sleep_ms(100);
        }
    }

    /* BT UART — RX only */
    uart_init(BT_UART, BT_UART_BAUD);
    gpio_set_function(BT_UART_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(BT_UART_TX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(BT_UART, false, false);
    uart_set_format(BT_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(BT_UART, true);

    printf("[MAIN] Running...\n");

    /* ── Heartbeat / LED state ───────────────────────────────────────────── */
    uint32_t last_uart_rx   = 0;
    uint32_t last_heartbeat = 0;
    uint32_t silence_start  = 0;
    uint32_t led_last_rx    = 0;
    bool     uart_active    = false;
    bool     in_cooldown    = false;

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
            printf("[UART] Silent — starting %u ms cooldown\n",
                   COOLDOWN_PERIOD_MS);
        }

        /* ── Expire cooldown ────────────────────────────────────────────── */
        if (in_cooldown && (now - silence_start >= COOLDOWN_PERIOD_MS)) {
            in_cooldown    = false;
            last_heartbeat = now;
            printf("[HEARTBEAT] Cooldown expired — 0x00 @ %u ms\n",
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
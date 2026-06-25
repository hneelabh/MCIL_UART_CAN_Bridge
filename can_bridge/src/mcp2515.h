#ifndef MCP2515_H
#define MCP2515_H

#include <stdint.h>
#include <stdbool.h>

/* =========================================================================
 * Hardware pin / SPI config
 * ========================================================================= */
#define MCP_SPI             spi0
#define MCP_PIN_SCK         18
#define MCP_PIN_MOSI        19
#define MCP_PIN_MISO        16
#define MCP_PIN_CS          17
#define MCP_SPI_BAUD        1000000u    /* 1 MHz */

/* 500 kbps @ 8 MHz crystal */
#define MCP_CNF1            0x00u
#define MCP_CNF2            0x90u
#define MCP_CNF3            0x02u

/* =========================================================================
 * MCP2515 SPI commands
 * ========================================================================= */
#define MCP_CMD_RESET       0xC0u
#define MCP_CMD_READ        0x03u
#define MCP_CMD_WRITE       0x02u
#define MCP_CMD_RTS_TXB0    0x81u
#define MCP_CMD_BIT_MODIFY  0x05u

/* =========================================================================
 * MCP2515 register addresses
 * ========================================================================= */
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

/* =========================================================================
 * Mode constants
 * ========================================================================= */
#define MCP_MODE_NORMAL     0x00u
#define MCP_MODE_CONFIG     0x80u
#define MCP_MODE_MASK       0xE0u

/* =========================================================================
 * CAN frame config (Traveo 2 filter: ID 0x636)
 * ========================================================================= */
#define CAN_SIDH            0xC6u   /* (0x636 >> 3) */
#define CAN_SIDL            0xC0u   /* (0x636 & 7) << 5 */
#define CAN_DLC             8u

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * Initialise SPI and MCP2515. Sets 500 kbps, enters Normal mode.
 * Returns true on success.
 */
bool mcp2515_init(void);

/**
 * Transmit a single 8-byte CAN frame on ID 0x636.
 * Byte 0 = signal_value, bytes 1-7 = 0x00.
 * Returns true if the MCP2515 confirms TX within ~50 ms.
 */
bool can_send(uint8_t signal_value);

/* Low-level helpers (exposed so aci_parser / main can read status if needed) */
void    mcp_write_reg(uint8_t addr, uint8_t val);
uint8_t mcp_read_reg(uint8_t addr);
void    mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t data);

#endif /* MCP2515_H */
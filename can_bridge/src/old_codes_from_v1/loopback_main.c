THIS IS THE OLD CODE WHICH WAS USED FOR THE LOOPBACK TEST

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// Pin Definitions based on your wiring
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19
#define PIN_INT  20

// MCP2515 SPI Commands & Registers
#define SPI_RESET       0xC0
#define SPI_READ        0x03
#define SPI_WRITE       0x02
#define CANCTRL_REG     0x0F
#define CANSTAT_REG     0x0E
#define TXB0CTRL_REG    0x30
#define RXB0CTRL_REG    0x60
#define SPI_RTS_TXB0    0x81

// Create a buffer to hold up to 8 characters (max CAN payload)
uint8_t tx_buffer[8];
uint8_t tx_index = 0;

// Helper function to assert Chip Select
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0);
    asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

// Write to an MCP2515 Register
void mcp2515_write_register(uint8_t reg, uint8_t data) {
    uint8_t buffer[3] = {SPI_WRITE, reg, data};
    cs_select();
    spi_write_blocking(SPI_PORT, buffer, 3);
    cs_deselect();
}

// Read from an MCP2515 Register
uint8_t mcp2515_read_register(uint8_t reg) {
    uint8_t tx[2] = {SPI_READ, reg};
    uint8_t rx[3]; // 2 bytes for command + 1 byte for data
    cs_select();
    spi_write_read_blocking(SPI_PORT, tx, rx, 2); 
    spi_read_blocking(SPI_PORT, 0, &rx[2], 1); // Read the actual data byte
    cs_deselect();
    return rx[2];
}

void mcp2515_reset() {
    uint8_t reset_cmd = SPI_RESET;
    cs_select();
    spi_write_blocking(SPI_PORT, &reset_cmd, 1);
    cs_deselect();
    sleep_ms(10);
}

int main() {
    stdio_init_all();
    printf("Starting Buffered MCP2515 Loopback Test...\n");

    // Initialize SPI
    spi_init(SPI_PORT, 1000 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); 

    mcp2515_reset();

    // ---> NEW: Disable Receive Filters for Buffer 0 <---
    // 0x60 = Receive any valid message, regardless of ID
    mcp2515_write_register(0x60, 0x60); // RXB0CTRL

    // Set to Loopback Mode
    mcp2515_write_register(CANCTRL_REG, 0x40);
    
    if((mcp2515_read_register(CANSTAT_REG) & 0xE0) == 0x40) {
        printf("MCP2515 in Loopback Mode. Ready for typing.\n\n");
    }

    uint8_t tx_buffer[8];
    uint8_t tx_index = 0;


    while (true) {
        int uart_char = getchar_timeout_us(0); 
        
        if (uart_char != PICO_ERROR_TIMEOUT) {
            
            // 1. If it is a normal letter, add it to our buffer and print it
            if (uart_char != '\r' && uart_char != '\n') {
                putchar((char)uart_char); // Echo to screen
                tx_buffer[tx_index++] = (uint8_t)uart_char;
            }
            
            // 2. Trigger send IF the user presses Enter OR the buffer hits exactly 8 bytes
            if (uart_char == '\r' || uart_char == '\n' || tx_index == 8) {
                
                // Only send if there is actual data
                if (tx_index > 0) {
                    printf("\n[Sending %d bytes via CAN...]\n", tx_index);
                    
                    // Set Standard CAN ID (0x123)
                    mcp2515_write_register(0x31, 0x24); // TXB0SIDH
                    mcp2515_write_register(0x32, 0x60); // TXB0SIDL
                    
                    // Set Data Length Code (DLC)
                    mcp2515_write_register(0x35, tx_index); 
                    
                    // Load the bytes safely (tx_index can NEVER be > 8 here)
                    for (int i = 0; i < tx_index; i++) {
                        mcp2515_write_register(0x36 + i, tx_buffer[i]); 
                    }
                    
                    // Request to Send (RTS)
                    uint8_t rts_cmd = SPI_RTS_TXB0;
                    cs_select();
                    spi_write_blocking(SPI_PORT, &rts_cmd, 1);
                    cs_deselect();
                    
                    // Poll for RX Interrupt Flag
                    bool message_received = false;
                    for (int wait = 0; wait < 20; wait++) {
                        if (mcp2515_read_register(0x2C) & 0x01) { 
                            message_received = true;
                            break;
                        }
                        sleep_ms(2);
                    }

                    // Print loopback results
                    if (message_received) {
                        uint8_t rx_dlc = mcp2515_read_register(0x65) & 0x0F;
                        printf("CAN RX -> UART: ");
                        for (int i = 0; i < rx_dlc; i++) {
                            putchar((char)mcp2515_read_register(0x66 + i));
                        }
                        printf("\n\n");
                        mcp2515_write_register(0x2C, 0x00); // Clear locks
                    } else {
                        printf("ERROR: Frame lost in CAN controller!\n\n");
                    }
                    
                    // Reset our buffer index for the next payload
                    tx_index = 0; 
                }
            } 
        }
        sleep_ms(1);
    }
    return 0;
}
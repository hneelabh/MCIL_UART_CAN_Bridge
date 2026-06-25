cmake_minimum_required(VERSION 3.13)

# Initialize the SDK (same pattern as your working can_bridge project)
include(pico_sdk_import.cmake)

project(can_bridge_v2)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable — all sources listed explicitly
add_executable(can_bridge_v2
    src/main.c
    src/mcp2515.c
    src/aci_parser.c
    src/can_signals.c
)

# So #include "mcp2515.h" etc. resolve without path prefixes
target_include_directories(can_bridge_v2 PRIVATE src)

# Link required libraries
# hardware_uart and hardware_gpio are pulled in transitively by pico_stdlib,
# but listed explicitly for clarity
target_link_libraries(can_bridge_v2
    pico_stdlib
    hardware_spi
    hardware_uart
    hardware_gpio
)

# UART0 is reserved for the BT module RX — keep stdio off it
pico_enable_stdio_uart(can_bridge_v2 0)
pico_enable_stdio_usb(can_bridge_v2 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(can_bridge_v2)
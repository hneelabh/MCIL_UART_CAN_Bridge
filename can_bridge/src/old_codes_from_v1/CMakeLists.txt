cmake_minimum_required(VERSION 3.13)

# Initialize the SDK
include(pico_sdk_import.cmake)

project(can_bridge_project)

# Initialize the Pico SDK
pico_sdk_init()

# Add your executable
add_executable(can_bridge main.c)

# Link the required libraries (stdlib and hardware_spi)
target_link_libraries(can_bridge pico_stdlib hardware_spi hardware_uart)

# Enable UART for standard I/O (printf)
pico_enable_stdio_uart(can_bridge 0)
pico_enable_stdio_usb(can_bridge 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(can_bridge)

# CMakeLists.txt for PIO Clock and Side-set Test

cmake_minimum_required(VERSION 3.13)

# Set board type for PGA2350
set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Board type")
set(PICO_PLATFORM "rp2350-arm-s")

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(test_clock_sideset C CXX ASM)

# Initialize the SDK
pico_sdk_init()

# Create executable
add_executable(test_clock_sideset
    test_clock_sideset.c
)

# Generate PIO header from .pio file
pico_generate_pio_header(test_clock_sideset
    ${CMAKE_CURRENT_LIST_DIR}/test_clock_sideset.pio
)

# Link required libraries
target_link_libraries(test_clock_sideset
    pico_stdlib
    pico_stdio
    pico_stdio_uart
    hardware_uart
    hardware_pio
    hardware_gpio
    hardware_clocks
    pico_util
)

# Enable both USB and UART output for debugging
pico_enable_stdio_usb(test_clock_sideset 1)
pico_enable_stdio_uart(test_clock_sideset 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(test_clock_sideset)

# Print build info
message(STATUS "Building PIO Clock and Side-set Test for PGA2350")
message(STATUS "  CLOCK_5_PIN: 29")
message(STATUS "  CLOCK_15B_PIN: 30")
message(STATUS "  DEN_PIN: 26")
message(STATUS "  IO_M_PIN: 27")
message(STATUS "  LED_PIN: 28 (avoiding board LED on pin 25)")
message(STATUS "  UART0 TX: pin 46")
message(STATUS "  UART0 RX: pin 45")
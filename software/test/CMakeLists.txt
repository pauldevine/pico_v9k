# Test directory CMakeLists.txt

# Test DMA Hardware executable
add_executable(test_dma_hardware
    test_dma_hardware.c
    ${CMAKE_SOURCE_DIR}/pico_victor/dma.c
)

# Include directories
target_include_directories(test_dma_hardware PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
    ${CMAKE_SOURCE_DIR}/pico_fujinet
)

# Generate PIO headers
pico_generate_pio_header(test_dma_hardware ${CMAKE_SOURCE_DIR}/pico_victor/dma_read_write.pio)

# Link libraries
target_link_libraries(test_dma_hardware
    pico_stdlib
    hardware_pio
    hardware_dma
    pico_multicore
)

# Enable USB or UART output
pico_enable_stdio_usb(test_dma_hardware 0)
pico_enable_stdio_uart(test_dma_hardware 1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(test_dma_hardware)

# Run from RAM (no flash)
pico_set_binary_type(test_dma_hardware no_flash)

# Test DMA Diagnostic executable
add_executable(test_dma_diagnostic
    test_dma_diagnostic.c
    test_signal_simulator.c
    ${CMAKE_SOURCE_DIR}/pico_victor/dma.c
)

# Include directories
target_include_directories(test_dma_diagnostic PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
    ${CMAKE_SOURCE_DIR}/pico_fujinet
)

# Generate PIO headers
pico_generate_pio_header(test_dma_diagnostic ${CMAKE_SOURCE_DIR}/pico_victor/dma_read_write.pio)

# Link libraries
target_link_libraries(test_dma_diagnostic
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_pwm
    hardware_timer
    pico_multicore
)

# Enable USB or UART output
pico_enable_stdio_usb(test_dma_diagnostic 0)
pico_enable_stdio_uart(test_dma_diagnostic 1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(test_dma_diagnostic)

# Run from RAM (no flash)
pico_set_binary_type(test_dma_diagnostic no_flash)

# Clock check test
add_executable(test_clock_check
    test_clock_check.c
)

target_include_directories(test_clock_check PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
)

target_link_libraries(test_clock_check
    pico_stdlib
    hardware_gpio
    hardware_pio
)

pico_enable_stdio_usb(test_clock_check 0)
pico_enable_stdio_uart(test_clock_check 1)
pico_add_extra_outputs(test_clock_check)
pico_set_binary_type(test_clock_check no_flash)

# Side-set validation test
add_executable(test_sideset_validation
    test_sideset_validation.c
)

target_include_directories(test_sideset_validation PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
)

target_link_libraries(test_sideset_validation
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_clocks
)

pico_enable_stdio_usb(test_sideset_validation 0)
pico_enable_stdio_uart(test_sideset_validation 1)
pico_add_extra_outputs(test_sideset_validation)
pico_set_binary_type(test_sideset_validation no_flash)

# Side-set explicit direction test
add_executable(test_sideset_explicit
    test_sideset_explicit.c
)

target_include_directories(test_sideset_explicit PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
)

target_link_libraries(test_sideset_explicit
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_clocks
    hardware_uart
)

pico_enable_stdio_usb(test_sideset_explicit 0)
pico_enable_stdio_uart(test_sideset_explicit 1)
pico_add_extra_outputs(test_sideset_explicit)
pico_set_binary_type(test_sideset_explicit no_flash)

# PIO HOLD control test
add_executable(test_pio_hold_control
    test_pio_hold_control.c
)

target_include_directories(test_pio_hold_control PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
)

target_link_libraries(test_pio_hold_control
    pico_stdlib
    hardware_gpio
    hardware_pio
    hardware_clocks
    hardware_uart
)

pico_enable_stdio_usb(test_pio_hold_control 0)
pico_enable_stdio_uart(test_pio_hold_control 1)
pico_add_extra_outputs(test_pio_hold_control)
pico_set_binary_type(test_pio_hold_control no_flash)
# Test directory CMakeLists.txt

# Clock check test - Simple diagnostic for verifying clock signals
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

# Clean DMA test - Current working test for DMA read/write operations
add_executable(test_dma_cleaned
    test_dma_cleaned.c
    ${CMAKE_SOURCE_DIR}/pico_victor/dma.c
)

# Include directories
target_include_directories(test_dma_cleaned PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/pico_victor
    ${CMAKE_SOURCE_DIR}/pico_fujinet
)

# Generate PIO headers
pico_generate_pio_header(test_dma_cleaned ${CMAKE_SOURCE_DIR}/pico_victor/dma_read_write.pio)

# Link libraries
target_link_libraries(test_dma_cleaned
    pico_stdlib
    hardware_pio
    hardware_dma
    hardware_gpio
    hardware_clocks
)

# Enable USB or UART output
pico_enable_stdio_usb(test_dma_cleaned 0)
pico_enable_stdio_uart(test_dma_cleaned 1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(test_dma_cleaned)

# Run from RAM (no flash)
pico_set_binary_type(test_dma_cleaned no_flash)

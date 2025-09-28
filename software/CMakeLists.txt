# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Board type")
set(PICO_PLATFORM "rp2350-arm-s")
set(PICO_NUM_GPIOS 48)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(dma_board C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(dma_board dma_board.c )
pico_set_program_name(dma_board "dma_board")
pico_set_program_version(dma_board "0.1")

target_sources(dma_board PRIVATE  
    logging.c 
    sasi.c
    pico_victor/dma.c
    pico_victor/dma_fast.c
    pico_victor/dma_ultra_fast.c
    pico_fujinet/spi.c)

# default means the target is to run from flash
pico_set_binary_type(dma_board default)

# Generate PIO headers
pico_generate_pio_header(dma_board ${CMAKE_CURRENT_LIST_DIR}/pico_victor/board_registers.pio)
pico_generate_pio_header(dma_board ${CMAKE_CURRENT_LIST_DIR}/pico_victor/dma_read_write.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(dma_board 1)
pico_enable_stdio_usb(dma_board 0)

# Add the standard library to the build
target_link_libraries(dma_board
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(dma_board PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/pico_victor
        ${CMAKE_CURRENT_LIST_DIR}/pico_fujinet
)

# Add any user requested libraries
target_link_libraries(dma_board
        pico_stdlib
        pico_multicore
        pico_util
        hardware_pio
        hardware_spi
        hardware_dma
        )

pico_add_extra_outputs(dma_board)

# On-device test executable (UNIT_TEST mode, no PIO bus activity)
add_executable(dma_board_tests
    test/on_device_runner.c
    logging.c
    sasi.c
    ${CMAKE_CURRENT_LIST_DIR}/pico_victor/dma.c
    ${CMAKE_CURRENT_LIST_DIR}/pico_fujinet/spi.c)

target_compile_definitions(dma_board_tests PRIVATE UNIT_TEST=1)

pico_set_program_name(dma_board_tests "dma_board_tests")
pico_set_program_version(dma_board_tests "0.1")

pico_enable_stdio_uart(dma_board_tests 1)
pico_enable_stdio_usb(dma_board_tests 0)

target_include_directories(dma_board_tests PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/pico_victor
    ${CMAKE_CURRENT_LIST_DIR}/pico_fujinet
)

target_link_libraries(dma_board_tests
    pico_stdlib
    pico_multicore
    pico_util
    hardware_pio
    hardware_spi
    hardware_dma
)

pico_add_extra_outputs(dma_board_tests)

# IRQ Handler Benchmark executable
add_executable(benchmark_irq
    test/benchmark_irq.c
    pico_victor/dma.c
    pico_victor/dma_fast.c
    pico_victor/dma_ultra_fast.c
)

target_compile_definitions(benchmark_irq PRIVATE BENCHMARK_MODE=1)

pico_set_program_name(benchmark_irq "benchmark_irq")
pico_set_program_version(benchmark_irq "0.1")

pico_enable_stdio_uart(benchmark_irq 1)
pico_enable_stdio_usb(benchmark_irq 0)

target_include_directories(benchmark_irq PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/pico_victor
)

target_link_libraries(benchmark_irq
    pico_stdlib
    hardware_pio
    hardware_dma
)

pico_add_extra_outputs(benchmark_irq)

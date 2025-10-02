/*
 * Test program to validate DMA read/write hardware setup
 * This program tests the dma_read_write.pio code by:
 * 1. Writing a 100-byte test pattern to Victor RAM
 * 2. Reading the same 100 bytes back
 * 3. Verifying the data matches
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "dma_read_write.pio.h"
#include "pico_victor/dma.h"

#define TEST_SIZE 5
#define TEST_ADDRESS 0x10000  // Test address in Victor RAM (segment 0x1000:0x0000)
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Simple core1 stub - just sleeps since we don't need register handling for this test
void core1_test_stub() {
    while (1) {
        sleep_ms(100);
    }
}

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

// Initialize the 74LVC245 direction control pins
void initialize_bus_direction() {
    // Initialize direction control pins
    gpio_init(LOW_ADDR_DIR);
    gpio_init(BUS_CNTRL_DIR);

    // Set as outputs
    gpio_set_dir(LOW_ADDR_DIR, GPIO_OUT);
    gpio_set_dir(BUS_CNTRL_DIR, GPIO_OUT);

    // Set initial direction to 8088->Pico (read mode)
    gpio_put(LOW_ADDR_DIR, DIR_8088_TO_PICO);
    gpio_put(BUS_CNTRL_DIR, DIR_8088_TO_PICO);

    printf("Bus direction pins initialized:\n");
    printf("  LOW_ADDR_DIR (pin %d) = %d\n", LOW_ADDR_DIR, gpio_get(LOW_ADDR_DIR));
    printf("  BUS_CNTRL_DIR (pin %d) = %d\n", BUS_CNTRL_DIR, gpio_get(BUS_CNTRL_DIR));
}

// Generate test pattern
void generate_test_pattern(uint8_t *buffer, size_t size) {
    // Generate a recognizable pattern
    for (size_t i = 0; i < size; i++) {
        // Mix of sequential and inverted bytes for easy verification
        if (i % 2 == 0) {
            buffer[i] = (uint8_t)(i & 0xFF);  // Sequential
        } else {
            buffer[i] = (uint8_t)(~i & 0xFF);  // Inverted
        }
    }
}

// Print hex dump of buffer
void print_hex_dump(const char *label, uint8_t *data, size_t size) {
    printf("%s:\n", label);
    for (size_t i = 0; i < size; i++) {
        if (i % 16 == 0) printf("%04zx: ", i);
        printf("%02x ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    if (size % 16 != 0) printf("\n");
}

// Compare buffers and report differences
int compare_buffers(uint8_t *expected, uint8_t *actual, size_t size) {
    int errors = 0;
    for (size_t i = 0; i < size; i++) {
        if (expected[i] != actual[i]) {
            if (errors < 10) {  // Report first 10 errors
                printf("ERROR at offset %04zx: expected %02x, got %02x\n",
                       i, expected[i], actual[i]);
            }
            errors++;
        }
    }
    return errors;
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(200000, true);
    initialize_uart();

    printf("\n=== DMA Hardware Test Program ===\n");
    printf("Testing dma_read_write.pio with %d bytes\n\n", TEST_SIZE);

    // Sleep briefly to ensure serial output is visible
    sleep_ms(2000);

    // Initialize bus direction control
    initialize_bus_direction();

    // Launch stub core1 (not doing register handling for this test)
    multicore_launch_core1(core1_test_stub);

    // Configure DMA PIO state machines
    PIO dma_pio = PIO_DMA;
    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int read_sm = pio_claim_unused_sm(dma_pio, true);
    int write_sm = pio_claim_unused_sm(dma_pio, true);

    printf("Initializing PIO state machines:\n");
    printf("  PIO: %d\n", pio_get_index(dma_pio));
    printf("  Read SM: %d\n", read_sm);
    printf("  Write SM: %d\n", write_sm);
    printf("  Program offset: 0x%02x\n", dma_read_write_program_offset);

    // Initialize state machines
    dma_read_write_program_init(dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN, DMA_READ);
    dma_read_write_program_init(dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN, DMA_WRITE);

    // Prime the state machines with their operation mode
    printf("\nPriming state machines with operation modes...\n");
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ);
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ_T2_PINDIRS);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE_T2_PINDIRS);

    // Prepare test data
    uint8_t write_buffer[TEST_SIZE];
    uint8_t read_buffer[TEST_SIZE];
    memset(read_buffer, 0xAA, TEST_SIZE);  // Fill with pattern to ensure it gets overwritten

    generate_test_pattern(write_buffer, TEST_SIZE);

    printf("\n=== Test 1: Write to Victor RAM ===\n");
    print_hex_dump("Data to write", write_buffer, TEST_SIZE > 32 ? 32 : TEST_SIZE);
    printf("  ... (showing first 32 bytes)\n\n");

    // Enable write state machine and disable read
    pio_sm_set_enabled(dma_pio, read_sm, false);
    pio_sm_set_enabled(dma_pio, write_sm, true);

    printf("Writing %d bytes to Victor RAM at address 0x%05X...\n", TEST_SIZE, TEST_ADDRESS);
    dma_write_to_victor_ram(dma_pio, write_sm, write_buffer, TEST_SIZE, TEST_ADDRESS);

    // Wait for writes to complete
    sleep_ms(100);

    // Disable write state machine
    pio_sm_set_enabled(dma_pio, write_sm, false);

    printf("Write complete!\n\n");

    printf("=== Test 2: Read from Victor RAM ===\n");
    printf("Reading %d bytes from Victor RAM at address 0x%05X...\n", TEST_SIZE, TEST_ADDRESS);

    // Enable read state machine
    pio_sm_set_enabled(dma_pio, read_sm, true);

    dma_read_from_victor_ram(dma_pio, read_sm, read_buffer, TEST_SIZE, TEST_ADDRESS);

    // Wait for reads to complete
    sleep_ms(100);

    // Disable read state machine
    pio_sm_set_enabled(dma_pio, read_sm, false);

    printf("Read complete!\n\n");

    print_hex_dump("Data read back", read_buffer, TEST_SIZE > 32 ? 32 : TEST_SIZE);
    printf("  ... (showing first 32 bytes)\n\n");

    printf("=== Test Results ===\n");
    int errors = compare_buffers(write_buffer, read_buffer, TEST_SIZE);

    if (errors == 0) {
        printf("SUCCESS: All %d bytes match! Hardware is working correctly.\n", TEST_SIZE);
    } else {
        printf("FAILURE: %d/%d bytes mismatch.\n", errors, TEST_SIZE);
        printf("\nPossible causes:\n");
        printf("  1. Wiring issue on data or address lines\n");
        printf("  2. Timing issue with Victor 9000 bus\n");
        printf("  3. Bus contention (check HOLD/HLDA signals)\n");
        printf("  4. 74LVC245 direction control issue\n");
    }

    printf("\n=== Test 3: Different Address Test ===\n");
    printf("Testing at different address 0x20000...\n");

    // Clear read buffer
    memset(read_buffer, 0xFF, TEST_SIZE);

    // Write to different address
    pio_sm_set_enabled(dma_pio, write_sm, true);
    dma_write_to_victor_ram(dma_pio, write_sm, write_buffer, TEST_SIZE, 0x20000);
    sleep_ms(100);
    pio_sm_set_enabled(dma_pio, write_sm, false);

    // Read back
    pio_sm_set_enabled(dma_pio, read_sm, true);
    dma_read_from_victor_ram(dma_pio, read_sm, read_buffer, TEST_SIZE, 0x20000);
    sleep_ms(100);
    pio_sm_set_enabled(dma_pio, read_sm, false);

    errors = compare_buffers(write_buffer, read_buffer, TEST_SIZE);
    if (errors == 0) {
        printf("SUCCESS: Address 0x20000 test passed!\n");
    } else {
        printf("FAILURE: Address 0x20000 test failed with %d errors.\n", errors);
    }

    printf("\n=== Hardware Validation Complete ===\n");

    // Keep running and report PIO state periodically
    printf("\nContinuing to monitor PIO state (Ctrl+C to exit)...\n");
    while (1) {
        sleep_ms(5000);
        printf("PIO status - Read SM: PC=0x%x, stalled=%d | Write SM: PC=0x%x, stalled=%d\n",
               pio_sm_get_pc(dma_pio, read_sm),
               pio_sm_is_exec_stalled(dma_pio, read_sm),
               pio_sm_get_pc(dma_pio, write_sm),
               pio_sm_is_exec_stalled(dma_pio, write_sm));
    }

    return 0;
}
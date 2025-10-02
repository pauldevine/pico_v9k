/**
 * test_dma_safe.c
 *
 * Safer DMA test that avoids crashing the 8088
 * - Provides abort mechanism
 * - Step-by-step execution with delays
 * - No ARM GPIO manipulation after PIO init
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "dma_read_write.pio.h"
#include "pico_victor/dma.h"

#define TEST_SIZE 1
#define TEST_ADDRESS 0x10000
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Simple core1 safety monitor
volatile bool abort_test = false;
volatile bool test_running = false;

void core1_safety_monitor() {
    while (1) {
        if (test_running) {
            // Monitor for 5 seconds max per test
            for (int i = 0; i < 50; i++) {
                sleep_ms(100);
                if (!test_running) break;
            }
            if (test_running) {
                printf("\n!!! SAFETY: Test timeout - aborting !!!\n");
                abort_test = true;
                test_running = false;
            }
        }
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

void wait_for_keypress(const char* message) {
    printf("\n%s", message);
    printf("\nPress ENTER to continue, 'q' to quit: ");

    int c = getchar();
    if (c == 'q' || c == 'Q') {
        printf("\nAborting test.\n");
        abort_test = true;
    }
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(200000, true);
    initialize_uart();

    sleep_ms(3000);

    printf("\n=== Safe DMA Test ===\n");
    printf("This test will proceed step-by-step with your confirmation.\n");
    printf("The 8088 should be running normally.\n");

    // Start safety monitor on core1
    multicore_launch_core1(core1_safety_monitor);
    sleep_ms(100);

    // Get PIO and state machines ready
    PIO dma_pio = pio0;
    uint read_sm = 0;
    uint write_sm = 1;

    wait_for_keypress("Ready to load PIO program?");
    if (abort_test) return 0;

    // Load program
    uint dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    printf("\nPIO program loaded at offset: 0x%02x\n", dma_read_write_program_offset);

    wait_for_keypress("Ready to initialize state machines?");
    if (abort_test) return 0;

    // Initialize but DON'T enable yet
    printf("\nInitializing state machines...\n");
    dma_read_write_program_init(dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN, DMA_WRITE);
    printf("Write SM initialized (not enabled)\n");

    dma_read_write_program_init(dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN, DMA_READ);
    printf("Read SM initialized (not enabled)\n");

    // Check initial state
    printf("\nInitial state:\n");
    printf("  Write SM: PC=0x%02x, TX FIFO empty=%d\n",
           pio_sm_get_pc(dma_pio, write_sm),
           pio_sm_is_tx_fifo_empty(dma_pio, write_sm));
    printf("  Read SM:  PC=0x%02x, TX FIFO empty=%d\n",
           pio_sm_get_pc(dma_pio, read_sm),
           pio_sm_is_tx_fifo_empty(dma_pio, read_sm));

    wait_for_keypress("Ready to load FIFO for SINGLE BYTE write to 0x10000?");
    if (abort_test) return 0;

    // Load minimal data - just one byte write
    printf("\nLoading Write SM FIFO:\n");
    printf("  1. Direction: DMA_WRITE (0x%08x)\n", DMA_WRITE);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE);

    printf("  2. Pin dirs: DMA_WRITE_T2_PINDIRS (0x%08x)\n", DMA_WRITE_T2_PINDIRS);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE_T2_PINDIRS);

    uint32_t addr = TEST_ADDRESS & 0xFFFFF;
    uint32_t data_word = (addr & 0xFFF00) | (0x42 & 0xFF);  // Write 0x42
    printf("  3. Address: 0x%05x\n", addr);
    pio_sm_put_blocking(dma_pio, write_sm, addr);

    printf("  4. Data: 0x42 (combined word: 0x%08x)\n", data_word);
    pio_sm_put_blocking(dma_pio, write_sm, data_word);

    printf("\nFIFO loaded. TX FIFO level: %d/8\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm));

    wait_for_keypress("Ready to ENABLE write SM? (This will attempt DMA)");
    if (abort_test) return 0;

    printf("\n!!! ENABLING WRITE SM - WATCH 8088 !!!\n");
    test_running = true;

    // Enable for just a brief moment
    pio_sm_set_enabled(dma_pio, write_sm, true);

    // Monitor quickly
    for (int i = 0; i < 10; i++) {
        printf("  %dms: PC=0x%02x, TX=%d/8\n",
               i * 10,
               pio_sm_get_pc(dma_pio, write_sm),
               pio_sm_get_tx_fifo_level(dma_pio, write_sm));
        sleep_ms(10);

        if (abort_test) {
            pio_sm_set_enabled(dma_pio, write_sm, false);
            printf("ABORTED - SM disabled\n");
            break;
        }
    }

    test_running = false;

    // Disable SM
    pio_sm_set_enabled(dma_pio, write_sm, false);
    printf("\nWrite SM disabled\n");

    // Final state
    printf("\nFinal state:\n");
    printf("  PC=0x%02x\n", pio_sm_get_pc(dma_pio, write_sm));
    printf("  TX FIFO: %d/8\n", pio_sm_get_tx_fifo_level(dma_pio, write_sm));
    printf("  RX FIFO: %d/8\n", pio_sm_get_rx_fifo_level(dma_pio, write_sm));

    // Check for PIO errors
    uint32_t fdebug = pio0_hw->fdebug;
    if (fdebug != 0) {
        printf("\n!!! PIO FDEBUG: 0x%08x !!!\n", fdebug);
    }

    wait_for_keypress("\nTest complete. Check if 8088 is still running.");

    printf("\n\n=== Test Summary ===\n");
    if (abort_test) {
        printf("Test was aborted.\n");
    } else {
        printf("Test completed.\n");
        printf("If the 8088 crashed, the PIO is affecting the bus.\n");
        printf("If the 8088 is fine but no DMA occurred, check HOLD/HLDA.\n");
    }

    return 0;
}
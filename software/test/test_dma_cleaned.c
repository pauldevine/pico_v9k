/**
 * test_dma_cleaned.c - Minimal, clean DMA test focusing on proper HOLD/HLDA handling
 *
 * This test removes the cruft from test_dma_diagnostic and focuses on the essential
 * DMA write operation with proper bus arbitration learned from test_hold_hlda.c
 *
 * Key points:
 * - Minimal ARM-side initialization to avoid interference
 * - Proper open-drain HOLD handling via pindirs
 * - Single DMA write test only
 * - Clear separation of concerns
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/iobank0.h"
#include "dma_read_write.pio.h"
#include "pico_victor/dma.h"

// Test parameters - start simple
#define TEST_ADDRESS    0x10000  // Test address in Victor RAM (segment 0x1000:0x0000)
#define TEST_DATA       0xAA     // Test data pattern

// Custom UART configuration matching your hardware
#define UART_ID         uart0
#define BAUD_RATE       115200
#define UART_TX_PIN     46
#define UART_RX_PIN     45

// Debug pin for oscilloscope measurements
#define DEBUG_PIN       44

/**
 * Initialize custom UART configuration
 */
static void initialize_uart(void) {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

/**
 * Quick check of critical bus signals
 */
static void check_critical_signals(void) {
    printf("\n=== Critical Signal Check ===\n");

    // Check HLDA (should be low when bus is not granted)
    printf("HLDA = %d (expect 0 when idle)\n", gpio_get(HLDA_PIN));

    // Check READY (should be high for normal operation)
    printf("READY = %d (expect 1 for normal)\n", gpio_get(READY_PIN));

    // Quick clock activity check
    int clk5_changes = 0;
    int last_clk5 = gpio_get(CLOCK_5_PIN);

    for (int i = 0; i < 1000; i++) {
        int current = gpio_get(CLOCK_5_PIN);
        if (current != last_clk5) {
            clk5_changes++;
            last_clk5 = current;
        }
        busy_wait_us(1);
    }

    printf("CLK5 transitions in 1ms: %d\n", clk5_changes);

    if (clk5_changes == 0) {
        printf("WARNING: No CLK5 activity detected!\n");
    } else {
        printf("CLK5 is active ✓\n");
    }
}

/**
 * Monitor PIO state machine progress
 */
static void monitor_pio_state(PIO pio, uint sm, uint offset, const char* label) {
    uint32_t pc = pio_sm_get_pc(pio, sm);
    bool stalled = pio_sm_is_exec_stalled(pio, sm);
    uint32_t tx_level = pio_sm_get_tx_fifo_level(pio, sm);

    printf("%s: PC=0x%02x (offset 0x%02x), stalled=%d, TX=%u/4\n",
           label, pc, pc - offset, stalled, tx_level);

    // If stalled, try to determine where
    if (stalled) {
        uint32_t instr_offset = pc - offset;
        switch (instr_offset) {
            case 4:  // wait for HLDA
                printf("  -> Waiting for HLDA (current=%d)\n", gpio_get(HLDA_PIN));
                break;
            case 8:  // wait CLK5 high/low in T1
            case 13: // wait CLK5 in T2
            case 14:
            case 16: // wait CLK5 in T3
            case 17:
            case 20: // wait CLK5 in T4
                printf("  -> Waiting for CLK5 (current=%d)\n", gpio_get(CLOCK_5_PIN));
                break;
            case 15: // wait READY
                printf("  -> Waiting for READY (current=%d)\n", gpio_get(READY_PIN));
                break;
            case 24: // wait HLDA low
                printf("  -> Waiting for HLDA to release (current=%d)\n", gpio_get(HLDA_PIN));
                break;
            default:
                printf("  -> Unknown wait state\n");
        }
    }
}

/**
 * Verify HOLD pin control works correctly
 * Based on test_hold_hlda.c approach
 */
static void verify_hold_control(PIO pio, uint sm) {
    printf("\n=== Verifying HOLD Pin Control ===\n");

    // CRITICAL: RP2350 requires executing a pindirs instruction to unlock side-set pindirs
    printf("Executing pindirs unlock sequence...\n");
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));  // All inputs first

    // Check we can control HOLD via pindirs
    printf("Testing HOLD control:\n");

    // 1. HOLD as output (drives low due to pin value = 0)
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, (1u << HOLD_PIN)));
    sleep_us(10);
    printf("  HOLD as output: pin=%d (expect 0)\n", gpio_get(HOLD_PIN));

    // 2. HOLD as input (floats high via pullup)
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0));
    sleep_us(10);
    printf("  HOLD as input:  pin=%d (expect 1)\n", gpio_get(HOLD_PIN));

    // Verify side-set will control the correct pin
    uint32_t pinctrl = pio->sm[sm].pinctrl;
    uint32_t sideset_base = (pinctrl >> 10) & 0x1F;
    printf("Side-set base pin: %u (expect %d)\n", sideset_base, HOLD_PIN);

    if (sideset_base != HOLD_PIN) {
        printf("ERROR: Side-set misconfigured!\n");
    } else {
        printf("Side-set configuration correct ✓\n");
    }
}

/**
 * Main test - single DMA write operation
 */
int main() {
    // System initialization
    set_sys_clock_khz(200000, true);
    initialize_uart();

    // Wait for serial connection
    sleep_ms(2000);

    printf("\n\n=== Clean DMA Write Test ===\n");
    printf("Minimal test focusing on HOLD/HLDA handling\n\n");

    // Initialize debug pin
    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    gpio_put(DEBUG_PIN, 0);

    // Basic signal check
    check_critical_signals();

    // === PIO Setup ===
    printf("\n=== PIO Configuration ===\n");

    PIO pio = pio0;
    uint sm = 0;  // Use state machine 0

    // Add the PIO program
    uint offset = pio_add_program(pio, &dma_read_write_program);
    printf("Program loaded at offset: 0x%02x\n", offset);
    printf("Program length: %d instructions\n", dma_read_write_program.length);

    // Initialize the PIO state machine
    printf("Initializing PIO state machine...\n");
    dma_read_write_program_init(pio, sm, offset, BD0_PIN);

    // Verify HOLD control before enabling SM
    verify_hold_control(pio, sm);

    // Check initial state
    printf("\n=== Initial State (before enable) ===\n");
    monitor_pio_state(pio, sm, offset, "Initial");

    // Enable the state machine
    printf("\n=== Enabling State Machine ===\n");
    pio_sm_set_enabled(pio, sm, true);

    // Brief delay to let SM initialize
    sleep_ms(1);

    // Check state after enable
    monitor_pio_state(pio, sm, offset, "After enable");

    // === Perform DMA Write ===
    printf("\n=== DMA Write Operation ===\n");
    printf("Target: Address 0x%05X, Data 0x%02X\n", TEST_ADDRESS, TEST_DATA);

    // Format the write command using the macro from dma_read_write.pio
    uint32_t write_cmd = DMA_FORMAT_WRITE(TEST_ADDRESS, TEST_DATA);
    printf("Formatted command: 0x%08X\n", write_cmd);
    printf("  R/W bit (LSB): %d (1=write)\n", write_cmd & 1);
    printf("  Address bits: 0x%05X\n", (write_cmd >> 1) & 0xFFFFF);
    printf("  Data byte: 0x%02X\n", (write_cmd >> 21) & 0xFF);

    // Send the write command
    printf("\nSending write command to PIO...\n");
    gpio_put(DEBUG_PIN, 1);  // Mark start on scope
    pio_sm_put_blocking(pio, sm, write_cmd);
    gpio_put(DEBUG_PIN, 0);

    // Monitor progress at various intervals
    const uint32_t check_points_us[] = {10, 50, 100, 500, 1000, 5000};
    const size_t num_checks = sizeof(check_points_us) / sizeof(check_points_us[0]);

    printf("\nMonitoring DMA operation progress:\n");
    uint32_t start_time = to_us_since_boot(get_absolute_time());

    for (size_t i = 0; i < num_checks; i++) {
        // Wait until checkpoint
        while (to_us_since_boot(get_absolute_time()) - start_time < check_points_us[i]) {
            tight_loop_contents();
        }

        char label[32];
        snprintf(label, sizeof(label), "+%u us", check_points_us[i]);
        monitor_pio_state(pio, sm, offset, label);

        // Check if operation completed (TX FIFO empty and not stalled)
        if (pio_sm_is_tx_fifo_empty(pio, sm) && !pio_sm_is_exec_stalled(pio, sm)) {
            printf("DMA operation appears complete!\n");
            break;
        }
    }

    // Final state check
    printf("\n=== Final State ===\n");
    monitor_pio_state(pio, sm, offset, "Final");

    // Check HOLD is released
    int hold_state = gpio_get(HOLD_PIN);
    int hlda_state = gpio_get(HLDA_PIN);
    printf("HOLD = %d (expect 1 for released)\n", hold_state);
    printf("HLDA = %d (expect 0 for idle)\n", hlda_state);

    if (hold_state == 1 && hlda_state == 0) {
        printf("✓ Bus properly released\n");
    } else {
        printf("⚠ Bus state unexpected!\n");
    }

    // === Cleanup ===
    printf("\n=== Test Cleanup ===\n");

    // Disable the state machine
    pio_sm_set_enabled(pio, sm, false);
    printf("State machine disabled\n");

    // Ensure HOLD is released (take ARM control and set high)
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 1);
    printf("HOLD pin released via ARM\n");

    printf("\n=== Test Complete ===\n");
    printf("Check oscilloscope/logic analyzer for timing details\n");
    printf("Verify Victor 9000 memory at 0x%05X for value 0x%02X\n",
           TEST_ADDRESS, TEST_DATA);

    return 0;
}
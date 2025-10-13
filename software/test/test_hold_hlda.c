/**
 * test_hold_hlda.c - Minimal SAFE test for HOLD/HLDA bus arbitration
 *
 * SAFETY CRITICAL: This test ONLY uses:
 *   - Pin 25 (HOLD) as OUTPUT - PIO controlled (active LOW)
 *   - Pin 28 (HLDA) as INPUT (read-only) - PIO reads this (active HIGH)
 *   - Pin 46 (UART TX) for debug output
 *
 * NO OTHER PINS ARE TOUCHED to protect vintage 8088 hardware
 *
 * Bus arbitration logic:
 *   - HOLD is active LOW (pull down to request bus)
 *   - HLDA is active HIGH (goes high when bus is granted)
 *
 * This test program runs a simple PIO state machine that:
 * 1. Asserts HOLD (pulls low) to request the bus
 * 2. Waits for HLDA to go HIGH (bus granted acknowledgment)
 * 3. Releases HOLD (sets high) to release the bus
 * 4. Delays with NOPs
 * 5. Repeats
 */

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "test_hold_hlda.pio.h"

// Pin definitions (matching the PIO file)
// CRITICAL: Only these pins are used!
#define HOLD_PIN  25  // OUTPUT - PIO controls this to request bus
#define HLDA_PIN  28  // INPUT - PIO reads this for acknowledgment

// UART for safe debug output (not on 8088 bus)
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

void initialize_uart() {
    // Only initialize UART TX pin for output
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

int main() {
    // Set system clock first
    set_sys_clock_khz(200000, true);

    // Initialize UART for debug output
    initialize_uart();

    // Give time for serial connection
    sleep_ms(2000);

    printf("\n\n=== SAFE HOLD/HLDA Bus Arbitration Test ===\n");
    printf("SAFETY: Only using pins 25 (HOLD out) and 28 (HLDA in)\n");
    printf("Testing minimal PIO program for bus handshaking\n\n");

    // Display pin configuration
    printf("Pin Configuration:\n");
    printf("  HOLD output: GPIO %d (PIO controlled, active LOW)\n", HOLD_PIN);
    printf("  HLDA input:  GPIO %d (PIO reads only, active HIGH)\n", HLDA_PIN);
    printf("  NO OTHER PINS WILL BE TOUCHED\n");
    printf("\n");

    // Before starting PIO, make sure HOLD is high (bus released)
    printf("Setting HOLD high before PIO initialization...\n");
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 1);  // HIGH = bus released
    sleep_ms(100);  // Let the bus settle

    // Choose PIO instance and state machine
    PIO pio = pio0;
    uint sm = 0;

    printf("Loading PIO program...\n");

    // Load the PIO program
    uint offset = pio_add_program(pio, &test_hold_hlda_program);
    printf("  Program loaded at offset: 0x%02x\n", offset);

    // Initialize the program (this will take over HOLD pin)
    printf("Initializing PIO state machine...\n");
    test_hold_hlda_program_init(pio, sm, offset);

    printf("  State machine %d initialized and running\n", sm);
    printf("  PIO now controls HOLD pin\n");
    printf("\n");

    // Monitor the bus arbitration with better sampling
    printf("Monitoring bus arbitration (press any key to stop)...\n");
    printf("NOTE: PIO cycles VERY fast - sampling and reporting periodically\n\n");

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_report_time = 0;
    uint32_t sample_count = 0;
    uint32_t hold_low_count = 0;
    uint32_t hlda_high_count = 0;
    uint32_t last_pc = 0;
    uint32_t pc_changes = 0;
    uint32_t report_number = 0;

    // Main monitoring loop - sample rapidly and report periodically
    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
        // Sample the pins rapidly for 10ms without printing
        uint32_t sample_start = to_us_since_boot(get_absolute_time());
        while (to_us_since_boot(get_absolute_time()) - sample_start < 10000) {  // 10ms of sampling
            uint32_t hold_state = gpio_get(HOLD_PIN);
            uint32_t hlda_state = gpio_get(HLDA_PIN);
            uint32_t current_pc = pio_sm_get_pc(pio, sm) - offset;

            sample_count++;
            if (hold_state == 0) hold_low_count++;
            if (hlda_state == 1) hlda_high_count++;
            if (current_pc != last_pc) {
                pc_changes++;
                last_pc = current_pc;
            }

            // Tight loop - no delays during sampling
        }

        uint32_t current_time = to_ms_since_boot(get_absolute_time()) - start_time;

        // Report every 100ms
        if (current_time - last_report_time >= 100) {
            report_number++;

            // Get current snapshot
            uint32_t hold_now = gpio_get(HOLD_PIN);
            uint32_t hlda_now = gpio_get(HLDA_PIN);
            uint32_t sm_addr = pio_sm_get_pc(pio, sm) - offset;
            uint32_t sm_stalled = !!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)));

            printf("=== Report #%u at %lu ms ===\n", report_number, current_time);
            printf("Current state: HOLD=%s, HLDA=%s, PC=0x%02x, Status=%s\n",
                   hold_now ? "HIGH" : "LOW",
                   hlda_now ? "HIGH" : "LOW",
                   sm_addr,
                   sm_stalled ? "STALLED" : "RUNNING");

            if (sample_count > 0) {
                // Calculate percentages
                uint32_t hold_low_pct = (hold_low_count * 100) / sample_count;
                uint32_t hlda_high_pct = (hlda_high_count * 100) / sample_count;

                printf("Statistics over %u samples:\n", sample_count);
                printf("  - HOLD was LOW %u%% of the time (bus requested)\n", hold_low_pct);
                printf("  - HLDA was HIGH %u%% of the time (bus granted)\n", hlda_high_pct);
                printf("  - Program counter changed %u times\n", pc_changes);

                // Estimate cycles based on PC changes
                // The PIO program has 6 instructions (set, wait, set, nop, nop, nop)
                // So roughly 6 PC changes per complete cycle
                uint32_t estimated_cycles = pc_changes / 6;
                uint32_t cycles_per_second = estimated_cycles * 10;  // Since we report every 100ms

                printf("  - Estimated ~%u complete HOLD/HLDA cycles\n", estimated_cycles);
                printf("  - Rate: ~%u cycles/second\n", cycles_per_second);

                // Check for success
                if (pc_changes > 0 && hold_low_count > 0 && hlda_high_count > 0) {
                    printf("  ✓ SUCCESS: Bus arbitration is working!\n");
                } else if (pc_changes == 0) {
                    printf("  ⚠ WARNING: PIO might be stalled\n");
                } else if (hold_low_count == 0) {
                    printf("  ⚠ WARNING: HOLD never went low\n");
                } else if (hlda_high_count == 0) {
                    printf("  ⚠ WARNING: HLDA never went high (no acknowledgment)\n");
                }
            }

            printf("\n");

            // Reset counters for next period
            sample_count = 0;
            hold_low_count = 0;
            hlda_high_count = 0;
            pc_changes = 0;
            last_report_time = current_time;
        }

        // Small delay before next sampling burst
        sleep_us(100);
    }

    printf("\n\nTest stopped after %u reports.\n", report_number);

    // CRITICAL: Safely shut down and release the bus
    printf("Safely shutting down...\n");

    // Disable the state machine
    pio_sm_set_enabled(pio, sm, false);
    printf("  State machine disabled.\n");

    // Take back control of HOLD pin and set it high (release bus)
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 1);  // HIGH = bus released
    printf("  HOLD pin set HIGH (bus released).\n");
    printf("  System is safe.\n");

    printf("\nTest complete.\n");

    return 0;
}
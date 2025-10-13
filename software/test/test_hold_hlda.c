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
    printf("  HOLD output: GPIO %d (PIO controlled)\n", HOLD_PIN);
    printf("  HLDA input:  GPIO %d (PIO reads only)\n", HLDA_PIN);
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

    // Monitor the bus arbitration
    printf("Monitoring bus arbitration (press any key to stop)...\n");
    printf("Time(ms)  HOLD  HLDA  SM_ADDR  STATUS\n");
    printf("--------  ----  ----  -------  -------\n");

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_hold_state = 1;  // HOLD starts high (idle)
    uint32_t last_hlda_state = gpio_get(HLDA_PIN);
    uint32_t cycle_count = 0;
    bool hold_was_low = false;

    // Main monitoring loop
    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT) {
        uint32_t current_time = to_ms_since_boot(get_absolute_time()) - start_time;

        // Read current pin states (read-only, no modification)
        uint32_t hold_state = gpio_get(HOLD_PIN);
        uint32_t hlda_state = gpio_get(HLDA_PIN);

        // Get PIO state machine status
        uint32_t sm_addr = pio_sm_get_pc(pio, sm) - offset;
        uint32_t sm_stalled = !!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)));

        // Detect state changes and report them
        if (hold_state != last_hold_state) {
            if (hold_state == 0) {
                printf("%8lu  LOW   %s     0x%02x     %s  <- HOLD asserted\n",
                       current_time, hlda_state ? "HIGH" : "LOW ", sm_addr,
                       sm_stalled ? "STALLED" : "RUNNING");
                hold_was_low = true;
            } else {
                printf("%8lu  HIGH  %s     0x%02x     %s  <- HOLD released\n",
                       current_time, hlda_state ? "HIGH" : "LOW ", sm_addr,
                       sm_stalled ? "STALLED" : "RUNNING");
                if (hold_was_low) {
                    cycle_count++;
                    printf("         [Cycle %d completed]\n", cycle_count);
                    hold_was_low = false;
                }
            }
            last_hold_state = hold_state;
        }

        if (hlda_state != last_hlda_state) {
            if (hlda_state == 0) {
                printf("%8lu  %s   LOW      0x%02x     %s  <- HLDA acknowledged\n",
                       current_time, hold_state ? "HIGH" : "LOW ", sm_addr,
                       sm_stalled ? "STALLED" : "RUNNING");
            } else {
                printf("%8lu  %s   HIGH     0x%02x     %s  <- HLDA deasserted\n",
                       current_time, hold_state ? "HIGH" : "LOW ", sm_addr,
                       sm_stalled ? "STALLED" : "RUNNING");
            }
            last_hlda_state = hlda_state;
        }

        sleep_ms(1);  // Small delay to not flood output
    }

    printf("\n\nTest stopped. Total cycles: %d\n", cycle_count);

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
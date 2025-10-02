/**
 * test_sideset_validation.c
 *
 * Minimal test to verify PIO side-set operations on HOLD pin
 * This tests whether the PIO can successfully control GPIO 24 (HOLD)
 * through side-set instructions.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"

// UART configuration for this hardware
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Pin definitions - matching dma_read_write.pio
#define HOLD_PIN 24
#define HLDA_PIN 25
#define CLK5_PIN 26
#define CLK15B_PIN 27

// Simple PIO program to test side-set on HOLD pin
const uint16_t sideset_test_program_instructions[] = {
    // .side_set 4 opt  (matching dma_read_write.pio configuration)
    0xe047,  // 0: set pins, 7 [no side-set] - Set control pins high initially
    0xe000,  // 1: set x, 0 - Clear X for delay counter
    0xf407,  // 2: set pins, 7 side 0b0100 - Should set HOLD=0 via side-set
    0xa0c3,  // 3: mov x, !x - Invert X for max delay
    0x0044,  // 4: jmp x--, 4 - Delay loop
    0xfc07,  // 5: set pins, 7 side 0b1100 - Should set HOLD=1 via side-set
    0xa0c3,  // 6: mov x, !x - Invert X for max delay
    0x0047,  // 7: jmp x--, 7 - Delay loop then wrap
};

const pio_program_t sideset_test_program = {
    .instructions = sideset_test_program_instructions,
    .length = 8,
    .origin = -1,
};

void setup_sideset_test_pio(PIO pio, uint sm) {
    // Load the test program
    uint offset = pio_add_program(pio, &sideset_test_program);

    printf("PIO program loaded at offset %d\n", offset);

    // Configure the state machine
    pio_sm_config c = pio_get_default_sm_config();

    // Configure side-set (4 pins starting at HOLD_PIN, optional)
    sm_config_set_sideset(&c, 4, true, false);  // 4 bits, optional, no pindirs
    sm_config_set_sideset_pins(&c, HOLD_PIN);

    // Configure set pins (just dummy pins for the SET instruction)
    sm_config_set_set_pins(&c, 0, 3);  // Use GPIO 0-2 as dummy set pins

    // Initialize HOLD pin for PIO control
    pio_gpio_init(pio, HOLD_PIN);

    // Initialize other pins for input (to match dma_read_write.pio)
    pio_gpio_init(pio, HLDA_PIN);
    pio_gpio_init(pio, CLK5_PIN);
    pio_gpio_init(pio, CLK15B_PIN);

    // Set pin directions - HOLD starts as input
    pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, false);  // HOLD as input initially
    pio_sm_set_consecutive_pindirs(pio, sm, HLDA_PIN, 1, false);  // HLDA as input
    pio_sm_set_consecutive_pindirs(pio, sm, CLK5_PIN, 1, false);  // CLK5 as input
    pio_sm_set_consecutive_pindirs(pio, sm, CLK15B_PIN, 1, false); // CLK15B as input

    // Initialize dummy set pins
    for (int i = 0; i < 3; i++) {
        pio_gpio_init(pio, i);
        pio_sm_set_consecutive_pindirs(pio, sm, i, 1, true);  // Output
    }

    // Initialize the state machine with our config
    pio_sm_init(pio, sm, offset, &c);

    printf("State machine configured\n");
}

void monitor_hold_pin() {
    printf("\nMonitoring HOLD pin for 1 second...\n");
    printf("Time (ms) | HOLD State | Transitions\n");
    printf("----------|------------|-------------\n");

    int last_hold = gpio_get(HOLD_PIN);
    int transitions = 0;
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t last_print = start_time;

    while (absolute_time_diff_us(start_time, get_absolute_time()) < 1000000) {
        int current_hold = gpio_get(HOLD_PIN);

        if (current_hold != last_hold) {
            transitions++;
            last_hold = current_hold;
        }

        // Print status every 100ms
        if (absolute_time_diff_us(last_print, get_absolute_time()) >= 100000) {
            uint32_t elapsed_ms = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;
            printf("%9lu |     %d      | %d\n", elapsed_ms, current_hold, transitions);
            last_print = get_absolute_time();
        }
    }

    printf("\nTotal transitions detected: %d\n", transitions);
    if (transitions == 0) {
        printf("WARNING: No transitions detected - side-set may not be working!\n");
    } else {
        printf("Side-set appears to be working - HOLD pin is toggling\n");
    }
}

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

int main() {
    // Initialize stdio first
    stdio_init_all();

    // Set system clock to 200MHz to match main project
    set_sys_clock_khz(200000, true);

    // Now initialize UART with custom pin configuration
    initialize_uart();

    // Give user time to connect serial console
    sleep_ms(3000);

    printf("\n=== PIO Side-Set Validation Test ===\n");
    printf("Testing side-set control of HOLD pin (GPIO %d)\n\n", HOLD_PIN);
    printf("System clock set to 200MHz\n");

    // Choose PIO instance and state machine
    PIO pio = pio0;
    uint sm = 0;

    // Setup the test PIO program
    setup_sideset_test_pio(pio, sm);

    // Check initial state
    printf("\nInitial HOLD pin state: %d\n", gpio_get(HOLD_PIN));
    printf("Initial pin direction: %s\n",
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    // Enable the state machine
    printf("\nEnabling PIO state machine...\n");
    pio_sm_set_enabled(pio, sm, true);

    // Brief delay to let PIO start
    sleep_ms(10);

    // Check if pin direction changed (side-set should make it output)
    printf("HOLD pin direction after PIO start: %s\n",
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    // Monitor the HOLD pin
    monitor_hold_pin();

    // Check final state machine status
    printf("\n=== Final State Machine Status ===\n");
    printf("State machine PC: 0x%x\n", pio_sm_get_pc(pio, sm));
    printf("TX FIFO level: %d/8\n", pio_sm_get_tx_fifo_level(pio, sm));
    printf("RX FIFO level: %d/8\n", pio_sm_get_rx_fifo_level(pio, sm));

    // Disable state machine
    pio_sm_set_enabled(pio, sm, false);

    printf("\n=== Test Complete ===\n");
    printf("If transitions were detected, side-set is working correctly.\n");
    printf("If no transitions, there may be an issue with side-set configuration.\n");

    return 0;
}
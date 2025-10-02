/**
 * test_sideset_explicit.c
 *
 * Test PIO side-set with explicit pin direction control
 * This version explicitly sets HOLD as OUTPUT to test if side-set
 * can control it when the direction is already set.
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

// Test 1: Simple PIO program with side-set that controls pin direction
const uint16_t sideset_with_pindirs_instructions[] = {
    // .side_set 4 opt pindirs  (4 bits, optional, controls pin direction)
    0xe047,  // 0: set pins, 7 [no side-set]
    0xe000,  // 1: set x, 0
    0xf417,  // 2: set pins, 7 side 0b0101 pindirs - HOLD becomes OUTPUT and LOW
    0xa0c3,  // 3: mov x, !x
    0x0044,  // 4: jmp x--, 4
    0xfc17,  // 5: set pins, 7 side 0b1101 pindirs - HOLD stays OUTPUT but HIGH
    0xa0c3,  // 6: mov x, !x
    0x0047,  // 7: jmp x--, 7
};

const pio_program_t sideset_with_pindirs = {
    .instructions = sideset_with_pindirs_instructions,
    .length = 8,
    .origin = -1,
};

// Test 2: PIO program with explicit pin direction setting first
const uint16_t sideset_explicit_dir_instructions[] = {
    // .side_set 4 opt  (4 bits, optional, no pindirs)
    0xe081,  // 0: set pindirs, 1  - Set HOLD pin as OUTPUT explicitly
    0xe047,  // 1: set pins, 7 [no side-set]
    0xe000,  // 2: set x, 0
    0xf407,  // 3: set pins, 7 side 0b0100 - HOLD=0
    0xa0c3,  // 4: mov x, !x
    0x0045,  // 5: jmp x--, 5
    0xfc07,  // 6: set pins, 7 side 0b1100 - HOLD=1
    0x0043,  // 7: jmp 3  - Loop back to instruction 3
};

const pio_program_t sideset_explicit_dir = {
    .instructions = sideset_explicit_dir_instructions,
    .length = 8,
    .origin = -1,
};

void setup_test_pio(PIO pio, uint sm, const pio_program_t *program, bool use_pindirs) {
    // Load the test program
    uint offset = pio_add_program(pio, program);

    printf("PIO program loaded at offset %d\n", offset);

    // Configure the state machine
    pio_sm_config c = pio_get_default_sm_config();

    // Configure side-set (4 pins starting at HOLD_PIN)
    sm_config_set_sideset(&c, 4, true, use_pindirs);  // 4 bits, optional, pindirs flag
    sm_config_set_sideset_pins(&c, HOLD_PIN);

    // Configure set pins for both SET and SET PINDIRS instructions
    sm_config_set_set_pins(&c, HOLD_PIN, 4);  // Use HOLD_PIN for SET operations

    // Initialize pins for PIO control
    pio_gpio_init(pio, HOLD_PIN);
    pio_gpio_init(pio, HLDA_PIN);
    pio_gpio_init(pio, CLK5_PIN);
    pio_gpio_init(pio, CLK15B_PIN);

    // Set initial pin directions
    if (!use_pindirs) {
        // For non-pindirs test, start with HOLD as INPUT (will be changed by SET PINDIRS)
        pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, false);
    } else {
        // For pindirs test, start with HOLD as INPUT (will be changed by side-set)
        pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, false);
    }

    pio_sm_set_consecutive_pindirs(pio, sm, HLDA_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CLK5_PIN, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, CLK15B_PIN, 1, false);

    // Initialize the state machine with our config
    pio_sm_init(pio, sm, offset, &c);

    printf("State machine configured (pindirs=%s)\n", use_pindirs ? "true" : "false");
}

void monitor_hold_pin(const char *test_name) {
    printf("\n=== %s ===\n", test_name);
    printf("Monitoring HOLD pin for 500ms...\n");
    printf("Time (ms) | HOLD State | Direction | Transitions\n");
    printf("----------|------------|-----------|-------------\n");

    int last_hold = gpio_get(HOLD_PIN);
    int transitions = 0;
    absolute_time_t start_time = get_absolute_time();
    absolute_time_t last_print = start_time;

    while (absolute_time_diff_us(start_time, get_absolute_time()) < 500000) {
        int current_hold = gpio_get(HOLD_PIN);

        if (current_hold != last_hold) {
            transitions++;
            last_hold = current_hold;
        }

        // Print status every 50ms
        if (absolute_time_diff_us(last_print, get_absolute_time()) >= 50000) {
            uint32_t elapsed_ms = absolute_time_diff_us(start_time, get_absolute_time()) / 1000;
            const char *dir = gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT";
            printf("%9lu |     %d      | %-9s | %d\n", elapsed_ms, current_hold, dir, transitions);
            last_print = get_absolute_time();
        }
    }

    printf("\nTotal transitions: %d\n", transitions);
    if (transitions == 0) {
        printf("FAILED: No transitions detected!\n");
    } else {
        printf("SUCCESS: HOLD pin is toggling (%d transitions)\n", transitions);
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

    printf("\n=== PIO Side-Set Direction Control Test ===\n");
    printf("Testing different approaches to control HOLD pin (GPIO %d)\n\n", HOLD_PIN);

    PIO pio = pio0;
    uint sm = 0;

    // Test 1: Side-set with pindirs flag
    printf("\n--- Test 1: Side-set with pindirs flag ---\n");
    printf("This test uses side-set with pindirs to control pin direction\n");

    setup_test_pio(pio, sm, &sideset_with_pindirs, true);

    printf("Initial HOLD state: %d, direction: %s\n",
           gpio_get(HOLD_PIN),
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    pio_sm_set_enabled(pio, sm, true);
    sleep_ms(10);

    monitor_hold_pin("Test 1: Side-set with pindirs");

    pio_sm_set_enabled(pio, sm, false);
    pio_remove_program(pio, &sideset_with_pindirs, pio_sm_get_pc(pio, sm) - 7);

    sleep_ms(100);

    // Test 2: Explicit SET PINDIRS then side-set
    printf("\n--- Test 2: Explicit SET PINDIRS then side-set ---\n");
    printf("This test first sets HOLD as OUTPUT, then uses side-set\n");

    setup_test_pio(pio, sm, &sideset_explicit_dir, false);

    printf("Initial HOLD state: %d, direction: %s\n",
           gpio_get(HOLD_PIN),
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    pio_sm_set_enabled(pio, sm, true);
    sleep_ms(10);

    monitor_hold_pin("Test 2: Explicit SET PINDIRS");

    pio_sm_set_enabled(pio, sm, false);

    // Test 3: Manual GPIO control (sanity check)
    printf("\n--- Test 3: Manual GPIO Control (Sanity Check) ---\n");
    printf("Manually toggling HOLD pin to verify hardware\n");

    // Take control away from PIO
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);

    printf("Starting manual toggle...\n");
    int manual_transitions = 0;
    absolute_time_t start = get_absolute_time();

    while (absolute_time_diff_us(start, get_absolute_time()) < 100000) { // 100ms
        gpio_put(HOLD_PIN, manual_transitions % 2);
        manual_transitions++;
        sleep_us(1000); // Toggle every 1ms
    }

    printf("Manual control: Toggled %d times\n", manual_transitions);
    printf("Final HOLD state: %d\n", gpio_get(HOLD_PIN));

    printf("\n=== Test Summary ===\n");
    printf("If Test 1 or Test 2 shows transitions, side-set is working.\n");
    printf("If only Test 3 shows changes, there's a PIO configuration issue.\n");
    printf("If no tests show changes, check hardware connections.\n");

    return 0;
}
/**
 * test_pio_hold_control.c
 *
 * Direct test of PIO control of HOLD pin
 * Focus on actual pin state changes, not direction reporting
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

// Simplest possible PIO program - just toggle HOLD pin directly
const uint16_t simple_toggle_instructions[] = {
    0xe001,  // 0: set pins, 1     - Set HOLD high
    0xe020,  // 1: set x, 0 [2]    - Clear X, 2 cycle delay
    0xa0c2,  // 2: mov x, !x [2]   - Invert X for max delay, 2 cycle delay
    0x0043,  // 3: jmp x--, 3      - Delay loop
    0xe000,  // 4: set pins, 0     - Set HOLD low
    0xe020,  // 5: set x, 0 [2]    - Clear X, 2 cycle delay
    0xa0c2,  // 6: mov x, !x [2]   - Invert X for max delay, 2 cycle delay
    0x0047,  // 7: jmp x--, 7      - Delay loop, then wrap to 0
};

const pio_program_t simple_toggle = {
    .instructions = simple_toggle_instructions,
    .length = 8,
    .origin = -1,
};

// Even simpler - just set output and toggle
const uint16_t ultra_simple_instructions[] = {
    0xe081,  // 0: set pindirs, 1  - Make HOLD output
    0xe001,  // 1: set pins, 1     - HOLD = 1
    0xe000,  // 2: set pins, 0     - HOLD = 0
    0x0001,  // 3: jmp 1           - Loop back
};

const pio_program_t ultra_simple = {
    .instructions = ultra_simple_instructions,
    .length = 4,
    .origin = -1,
};

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

void test_simple_toggle(PIO pio, uint sm) {
    printf("\n=== Test: Simple PIO Toggle (SET pins only) ===\n");

    // Load program
    uint offset = pio_add_program(pio, &simple_toggle);
    printf("Program loaded at offset %d\n", offset);

    // Configure state machine
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, HOLD_PIN, 1);  // Single pin for SET

    // Initialize pin for PIO
    pio_gpio_init(pio, HOLD_PIN);

    // Set as output
    pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, true);

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &c);

    // Sample pin state before enabling
    printf("Before enable: HOLD = %d\n", gpio_get(HOLD_PIN));

    // Enable and monitor
    pio_sm_set_enabled(pio, sm, true);

    printf("Monitoring for 100ms (should see ~390 transitions at 200MHz with delay loops):\n");
    int last_state = gpio_get(HOLD_PIN);
    int transitions = 0;
    absolute_time_t start = get_absolute_time();

    while (absolute_time_diff_us(start, get_absolute_time()) < 100000) {
        int current = gpio_get(HOLD_PIN);
        if (current != last_state) {
            transitions++;
            last_state = current;
            if (transitions <= 10) {
                printf("  Transition %d: HOLD = %d at %lld us\n",
                       transitions, current,
                       absolute_time_diff_us(start, get_absolute_time()));
            }
        }
    }

    pio_sm_set_enabled(pio, sm, false);
    printf("Total transitions: %d\n", transitions);
    printf("Final PC: 0x%x\n", pio_sm_get_pc(pio, sm));

    pio_remove_program(pio, &simple_toggle, offset);
}

void test_ultra_simple(PIO pio, uint sm) {
    printf("\n=== Test: Ultra Simple (SET PINDIRS + rapid toggle) ===\n");

    // Load program
    uint offset = pio_add_program(pio, &ultra_simple);
    printf("Program loaded at offset %d\n", offset);

    // Configure state machine
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, HOLD_PIN, 1);  // Single pin for SET

    // Initialize pin for PIO
    pio_gpio_init(pio, HOLD_PIN);

    // Start as input (PIO will change it)
    pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, false);

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &c);

    // Sample pin state before enabling
    printf("Before enable: HOLD = %d, dir = %s\n",
           gpio_get(HOLD_PIN),
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    // Enable and monitor
    pio_sm_set_enabled(pio, sm, true);

    // Quick check after PIO starts
    sleep_us(10);
    printf("After enable: HOLD = %d, dir = %s\n",
           gpio_get(HOLD_PIN),
           gpio_get_dir(HOLD_PIN) ? "OUTPUT" : "INPUT");

    printf("Monitoring for 10ms (should see MANY transitions at 200MHz):\n");
    int last_state = gpio_get(HOLD_PIN);
    int transitions = 0;
    absolute_time_t start = get_absolute_time();

    while (absolute_time_diff_us(start, get_absolute_time()) < 10000) {
        int current = gpio_get(HOLD_PIN);
        if (current != last_state) {
            transitions++;
            last_state = current;
        }
    }

    pio_sm_set_enabled(pio, sm, false);
    printf("Total transitions in 10ms: %d\n", transitions);
    printf("Final PC: 0x%x\n", pio_sm_get_pc(pio, sm));
    printf("Expected ~16.6M transitions if running at full speed\n");
    printf("Actual indicates PIO clock divider or execution speed\n");

    pio_remove_program(pio, &ultra_simple, offset);
}

void test_with_side_set(PIO pio, uint sm) {
    printf("\n=== Test: With Optional Side-set (like dma_read_write.pio) ===\n");

    // This matches the configuration in dma_read_write.pio more closely
    const uint16_t sideset_program[] = {
        0xe081,  // 0: set pindirs, 1 - Make HOLD output via SET
        0xf401,  // 1: set pins, 1 side 0b0100 - HOLD=0 via side-set (bit 2)
        0xe020,  // 2: set x, 0 [2]
        0xa0c2,  // 3: mov x, !x [2]
        0x0044,  // 4: jmp x--, 4
        0xfc01,  // 5: set pins, 1 side 0b1100 - HOLD=1 via side-set (bit 2)
        0xe020,  // 6: set x, 0 [2]
        0x0041,  // 7: jmp 1
    };

    const pio_program_t sideset_prog = {
        .instructions = sideset_program,
        .length = 8,
        .origin = -1,
    };

    // Load program
    uint offset = pio_add_program(pio, &sideset_prog);
    printf("Program loaded at offset %d\n", offset);

    // Configure state machine with side-set
    pio_sm_config c = pio_get_default_sm_config();

    // Side-set configuration: 4 bits, optional, no pindirs
    sm_config_set_sideset(&c, 4, true, false);
    sm_config_set_sideset_pins(&c, HOLD_PIN);

    // Also configure SET pins for the SET PINDIRS instruction
    sm_config_set_set_pins(&c, HOLD_PIN, 1);

    // Initialize pins for PIO
    pio_gpio_init(pio, HOLD_PIN);
    pio_gpio_init(pio, HLDA_PIN);
    pio_gpio_init(pio, CLK5_PIN);
    pio_gpio_init(pio, CLK15B_PIN);

    // All start as inputs
    pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 4, false);

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &c);

    // Sample pin state before enabling
    printf("Before enable: HOLD = %d\n", gpio_get(HOLD_PIN));

    // Enable and monitor
    pio_sm_set_enabled(pio, sm, true);

    printf("Monitoring for 100ms:\n");
    int last_state = gpio_get(HOLD_PIN);
    int transitions = 0;
    absolute_time_t start = get_absolute_time();

    while (absolute_time_diff_us(start, get_absolute_time()) < 100000) {
        int current = gpio_get(HOLD_PIN);
        if (current != last_state) {
            transitions++;
            last_state = current;
            if (transitions <= 10) {
                printf("  Transition %d: HOLD = %d\n", transitions, current);
            }
        }
    }

    pio_sm_set_enabled(pio, sm, false);
    printf("Total transitions: %d\n", transitions);
    printf("Final PC: 0x%x\n", pio_sm_get_pc(pio, sm));

    pio_remove_program(pio, &sideset_prog, offset);
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

    printf("\n=== PIO HOLD Pin Control Test ===\n");
    printf("Testing different PIO approaches to control HOLD (GPIO %d)\n", HOLD_PIN);
    printf("Watch logic analyzer on HOLD pin\n");

    PIO pio = pio0;
    uint sm = 0;

    // Test 1: Simple SET pins toggle
    test_simple_toggle(pio, sm);
    sleep_ms(100);

    // Test 2: Ultra simple with SET PINDIRS
    test_ultra_simple(pio, sm);
    sleep_ms(100);

    // Test 3: With side-set like dma_read_write.pio
    test_with_side_set(pio, sm);

    printf("\n=== All Tests Complete ===\n");
    printf("Check logic analyzer for actual pin transitions\n");
    printf("If transitions seen on analyzer but not in software,\n");
    printf("the PIO is working but gpio_get() may not reflect PIO-driven changes\n");

    return 0;
}
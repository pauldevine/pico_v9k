/*
 * Simple test to verify PIO can execute basic instructions
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico_victor/dma.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46

// Simple PIO program that just pulls data and toggles pins
const uint16_t simple_test_program[] = {
    0x80a0, // 0: pull block
    0xf407, // 1: set pins, 7
    0xf400, // 2: set pins, 0
    0x0001, // 3: jmp 1
};

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(200000, true);
    initialize_uart();

    printf("\n=== Simple PIO Test ===\n");
    sleep_ms(2000);

    PIO pio = pio0;
    int sm = 0;

    // Load program
    uint offset = pio_add_program_at_offset(pio, simple_test_program, 4, 0);
    printf("Program loaded at offset: %d\n", offset);

    // Configure SM
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_set_pins(&c, HOLD_PIN, 1);

    // Initialize pin
    pio_gpio_init(pio, HOLD_PIN);
    pio_sm_set_consecutive_pindirs(pio, sm, HOLD_PIN, 1, true); // Output

    // Initialize SM
    pio_sm_init(pio, sm, offset, &c);

    printf("Initial state: PC=0x%x, enabled=%d\n",
           pio_sm_get_pc(pio, sm),
           (pio0_hw->ctrl >> sm) & 1);

    // Put data and enable
    pio_sm_put_blocking(pio, sm, 0x12345678);
    printf("Data loaded, FIFO level: %d/8\n",
           pio_sm_get_tx_fifo_level(pio, sm));

    pio_sm_set_enabled(pio, sm, true);
    printf("SM enabled\n");

    // Monitor
    for (int i = 0; i < 10; i++) {
        sleep_ms(100);
        printf("PC=0x%x, FIFO=%d/8, HOLD=%d, stalled=%d\n",
               pio_sm_get_pc(pio, sm),
               pio_sm_get_tx_fifo_level(pio, sm),
               gpio_get(HOLD_PIN),
               pio_sm_is_exec_stalled(pio, sm));
    }

    return 0;
}
/*
 * Simple test to check if clock signals are being received
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "pico_victor/dma.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46

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

    printf("\n=== Clock Signal Test ===\n");
    sleep_ms(2000);

    printf("\nTest 1: Reading clocks as normal GPIO\n");
    gpio_init(CLOCK_5_PIN);
    gpio_init(CLOCK_15B_PIN);
    gpio_init(HLDA_PIN);
    gpio_init(Ready_PIN);

    gpio_set_dir(CLOCK_5_PIN, GPIO_IN);
    gpio_set_dir(CLOCK_15B_PIN, GPIO_IN);
    gpio_set_dir(HLDA_PIN, GPIO_IN);
    gpio_set_dir(Ready_PIN, GPIO_IN);

    // Sample for transitions
    int clk5_transitions = 0;
    int clk15_transitions = 0;
    int last_clk5 = gpio_get(CLOCK_5_PIN);
    int last_clk15 = gpio_get(CLOCK_15B_PIN);

    for (int i = 0; i < 10000; i++) {
        int current_clk5 = gpio_get(CLOCK_5_PIN);
        int current_clk15 = gpio_get(CLOCK_15B_PIN);

        if (current_clk5 != last_clk5) clk5_transitions++;
        if (current_clk15 != last_clk15) clk15_transitions++;

        last_clk5 = current_clk5;
        last_clk15 = current_clk15;

        busy_wait_us(1);
    }

    printf("GPIO mode - CLK5 transitions: %d, CLK15B transitions: %d\n",
           clk5_transitions, clk15_transitions);
    printf("READY pin: %d, HLDA pin: %d\n", gpio_get(Ready_PIN), gpio_get(HLDA_PIN));

    printf("\nTest 2: Reading clocks after PIO init\n");

    // Initialize for PIO
    PIO pio = pio0;
    for (int i = 28; i <= 31; i++) {
        pio_gpio_init(pio, i);
        gpio_set_dir(i, GPIO_IN);  // Ensure they're inputs
    }

    // Sample again
    clk5_transitions = 0;
    clk15_transitions = 0;
    last_clk5 = gpio_get(CLOCK_5_PIN);
    last_clk15 = gpio_get(CLOCK_15B_PIN);

    for (int i = 0; i < 10000; i++) {
        int current_clk5 = gpio_get(CLOCK_5_PIN);
        int current_clk15 = gpio_get(CLOCK_15B_PIN);

        if (current_clk5 != last_clk5) clk5_transitions++;
        if (current_clk15 != last_clk15) clk15_transitions++;

        last_clk5 = current_clk5;
        last_clk15 = current_clk15;

        busy_wait_us(1);
    }

    printf("After PIO init - CLK5 transitions: %d, CLK15B transitions: %d\n",
           clk5_transitions, clk15_transitions);
    printf("READY pin: %d, HLDA pin: %d\n", gpio_get(Ready_PIN), gpio_get(HLDA_PIN));

    // Continuous monitoring
    printf("\nContinuous monitoring (shows every 100ms):\n");
    while (1) {
        int samples = 0;
        int clk5_high = 0;
        int clk15_high = 0;

        for (int i = 0; i < 1000; i++) {
            clk5_high += gpio_get(CLOCK_5_PIN);
            clk15_high += gpio_get(CLOCK_15B_PIN);
            samples++;
            busy_wait_us(10);
        }

        printf("CLK5: %d/%d high, CLK15B: %d/%d high, READY: %d, HLDA: %d\r",
               clk5_high, samples, clk15_high, samples,
               gpio_get(Ready_PIN), gpio_get(HLDA_PIN));
        fflush(stdout);
    }

    return 0;
}
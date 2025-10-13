/*
 * Clock signal simulator for testing without Victor 9000 hardware
 * Generates CLK5 (5 MHz) and CLK15B (1.5 MHz) signals
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico_victor/dma.h"

static uint clock5_slice = 0;
static uint clock15b_slice = 0;
static bool clocks_running = false;

void clock_simulator_init(void) {
    if (clocks_running) {
        printf("Clock simulator already running\n");
        return;
    }

    printf("Initializing clock simulator for testing...\n");

    // Configure CLK5 (5 MHz) on pin 29
    gpio_set_function(CLOCK_5_PIN, GPIO_FUNC_PWM);
    clock5_slice = pwm_gpio_to_slice_num(CLOCK_5_PIN);

    // Set up PWM for 5 MHz
    // System clock is 200 MHz, so we need divider of 40 for 5 MHz
    // PWM period of 1 gives us 200MHz / 40 / 2 = 2.5MHz toggle rate = 5MHz square wave
    pwm_set_clkdiv(clock5_slice, 20.0f);  // 200MHz / 20 = 10MHz PWM base
    pwm_set_wrap(clock5_slice, 1);  // Count 0-1 (2 counts) = 5MHz output
    pwm_set_gpio_level(CLOCK_5_PIN, 1);  // 50% duty cycle

    // Configure CLK15B (1.5 MHz) on pin 30
    gpio_set_function(CLOCK_15B_PIN, GPIO_FUNC_PWM);
    clock15b_slice = pwm_gpio_to_slice_num(CLOCK_15B_PIN);

    // Set up PWM for 1.5 MHz
    // 200MHz / 66.67 / 2 = 1.5MHz
    pwm_set_clkdiv(clock15b_slice, 66.67f);
    pwm_set_wrap(clock15b_slice, 1);  // Count 0-1 (2 counts)
    pwm_set_gpio_level(CLOCK_15B_PIN, 1);  // 50% duty cycle

    // Enable both PWM slices
    pwm_set_enabled(clock5_slice, true);
    pwm_set_enabled(clock15b_slice, true);

    clocks_running = true;
    printf("Clock simulator started:\n");
    printf("  CLK5 (pin %d): ~5 MHz\n", CLOCK_5_PIN);
    printf("  CLK15B (pin %d): ~1.5 MHz\n", CLOCK_15B_PIN);
}

void clock_simulator_stop(void) {
    if (!clocks_running) {
        return;
    }

    printf("Stopping clock simulator...\n");

    // Disable PWM slices
    pwm_set_enabled(clock5_slice, false);
    pwm_set_enabled(clock15b_slice, false);

    // Return pins to GPIO function
    gpio_init(CLOCK_5_PIN);
    gpio_set_dir(CLOCK_5_PIN, GPIO_IN);
    gpio_init(CLOCK_15B_PIN);
    gpio_set_dir(CLOCK_15B_PIN, GPIO_IN);

    clocks_running = false;
    printf("Clock simulator stopped\n");
}

void clock_simulator_test(void) {
    printf("Testing clock simulator output...\n");

    // Sample the clock pins to verify they're toggling
    int samples = 1000;
    int clk5_changes = 0;
    int clk15_changes = 0;
    int last_clk5 = gpio_get(CLOCK_5_PIN);
    int last_clk15 = gpio_get(CLOCK_15B_PIN);

    for (int i = 0; i < samples; i++) {
        int curr_clk5 = gpio_get(CLOCK_5_PIN);
        int curr_clk15 = gpio_get(CLOCK_15B_PIN);

        if (curr_clk5 != last_clk5) {
            clk5_changes++;
            last_clk5 = curr_clk5;
        }
        if (curr_clk15 != last_clk15) {
            clk15_changes++;
            last_clk15 = curr_clk15;
        }

        busy_wait_us(10);
    }

    printf("Clock test results (over %d us):\n", samples * 10);
    printf("  CLK5 transitions: %d (expected ~100 for 5MHz)\n", clk5_changes);
    printf("  CLK15B transitions: %d (expected ~30 for 1.5MHz)\n", clk15_changes);
}
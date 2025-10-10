/*
 * Signal simulator for testing DMA PIO without actual Victor 9000 hardware
 * Simulates CLK5, CLK15B, HLDA, and READY signals
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "pico_victor/dma.h"

// Signal generation state
static volatile bool simulator_running = false;
static volatile bool hold_detected = false;
static alarm_id_t hlda_alarm = -1;

// HLDA response callback - simulates 8088 responding to HOLD request
int64_t hlda_response_callback(alarm_id_t id, void *user_data) {
    if (gpio_get(HOLD_PIN) == 0) {
        // HOLD is asserted (low), respond with HLDA high after 400ns
        gpio_put(HLDA_PIN, 1);
    } else {
        // HOLD is deasserted (high), respond with HLDA low
        gpio_put(HLDA_PIN, 0);
    }
    return 0; // Don't repeat
}

// Monitor HOLD pin for changes
void hold_monitor_callback(uint gpio, uint32_t events) {
    if (gpio == HOLD_PIN) {
        // Schedule HLDA response after 400ns (simulating 8088 bus cycle completion)
        // Note: minimum alarm resolution is 1us, so we use that
        if (hlda_alarm >= 0) {
            cancel_alarm(hlda_alarm);
        }
        hlda_alarm = add_alarm_in_us(1, hlda_response_callback, NULL, false);
    }
}

// Initialize PWM for clock generation
void setup_clock_pwm(uint gpio, uint freq_hz) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate divider and wrap for desired frequency
    // Get system clock frequency
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    float divider = (float)sys_clk_hz / (freq_hz * 65536);

    if (divider < 1.0f) {
        divider = 1.0f;
    }

    uint16_t wrap = (uint16_t)((float)sys_clk_hz / (freq_hz * divider) - 1);

    pwm_set_clkdiv(slice_num, divider);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_gpio_level(gpio, wrap / 2); // 50% duty cycle
    pwm_set_enabled(slice_num, true);

    printf("Clock on GPIO %d: freq=%d Hz, div=%.2f, wrap=%d\n",
           gpio, freq_hz, divider, wrap);
}

// Initialize signal simulator
void signal_simulator_init() {
    printf("\n=== Initializing Signal Simulator ===\n");

    // 1. Set READY signal high (normal operation)
    gpio_init(READY_PIN);
    gpio_set_dir(READY_PIN, GPIO_OUT);
    gpio_put(READY_PIN, 1);
    printf("READY signal set HIGH (normal operation)\n");

    // 2. Initialize HLDA as output, initially low
    gpio_init(HLDA_PIN);
    gpio_set_dir(HLDA_PIN, GPIO_OUT);
    gpio_put(HLDA_PIN, 0);
    printf("HLDA initialized LOW (no bus grant)\n");

    // 3. Set up interrupt on HOLD pin to simulate 8088 response
    gpio_set_irq_enabled_with_callback(HOLD_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &hold_monitor_callback);
    printf("HOLD monitor installed (will respond with HLDA)\n");

    // 4. Generate CLK5 (5MHz) and CLK15B (15MHz) using PWM
    setup_clock_pwm(CLOCK_5_PIN, 5000000);   // 5 MHz
    setup_clock_pwm(CLOCK_15B_PIN, 15000000); // 15 MHz

    simulator_running = true;
    printf("Signal simulator started\n");
}

// Stop signal simulator
void signal_simulator_stop() {
    if (!simulator_running) return;

    printf("\n=== Stopping Signal Simulator ===\n");

    // Disable PWM on clock pins
    uint slice5 = pwm_gpio_to_slice_num(CLOCK_5_PIN);
    uint slice15 = pwm_gpio_to_slice_num(CLOCK_15B_PIN);
    pwm_set_enabled(slice5, false);
    pwm_set_enabled(slice15, false);

    // Disable HOLD interrupt
    gpio_set_irq_enabled(HOLD_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Cancel any pending HLDA alarm
    if (hlda_alarm >= 0) {
        cancel_alarm(hlda_alarm);
        hlda_alarm = -1;
    }

    // Reset pins to inputs
    gpio_set_dir(READY_PIN, GPIO_IN);
    gpio_set_dir(HLDA_PIN, GPIO_IN);
    gpio_set_dir(CLOCK_5_PIN, GPIO_IN);
    gpio_set_dir(CLOCK_15B_PIN, GPIO_IN);

    simulator_running = false;
    printf("Signal simulator stopped\n");
}

// Check if simulator is running
bool signal_simulator_is_running() {
    return simulator_running;
}

// Test the simulator signals
void signal_simulator_test() {
    printf("\n=== Testing Signal Simulator ===\n");

    // Check READY
    printf("READY = %d (should be 1)\n", gpio_get(READY_PIN));

    // Check clocks are toggling
    int clk5_samples = 0;
    int clk15_samples = 0;
    int clk5_last = gpio_get(CLOCK_5_PIN);
    int clk15_last = gpio_get(CLOCK_15B_PIN);
    int clk5_transitions = 0;
    int clk15_transitions = 0;

    for (int i = 0; i < 10000; i++) {
        int clk5 = gpio_get(CLOCK_5_PIN);
        int clk15 = gpio_get(CLOCK_15B_PIN);

        if (clk5 != clk5_last) clk5_transitions++;
        if (clk15 != clk15_last) clk15_transitions++;

        clk5_last = clk5;
        clk15_last = clk15;

        busy_wait_us(1);
    }

    printf("CLK5 transitions in 10ms: %d (expect ~10000)\n", clk5_transitions);
    printf("CLK15B transitions in 10ms: %d (expect ~30000)\n", clk15_transitions);

    // Test HOLD/HLDA handshake
    printf("\nTesting HOLD/HLDA handshake:\n");
    printf("Initial HLDA = %d\n", gpio_get(HLDA_PIN));

    // Simulate HOLD request (PIO would do this)
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 0); // Assert HOLD (active low)

    sleep_ms(1); // Wait for response
    printf("After HOLD=0: HLDA = %d (should be 1)\n", gpio_get(HLDA_PIN));

    gpio_put(HOLD_PIN, 1); // Deassert HOLD
    sleep_ms(1); // Wait for response
    printf("After HOLD=1: HLDA = %d (should be 0)\n", gpio_get(HLDA_PIN));

    // Return HOLD to input for PIO
    gpio_set_dir(HOLD_PIN, GPIO_IN);
}
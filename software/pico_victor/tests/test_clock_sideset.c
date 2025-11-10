#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/stdio_uart.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "test_clock_sideset.pio.h"


// Pin definitions - matching the PIO file
#define CLOCK_5_PIN  29
#define CLOCK_15B_PIN 30
#define DEN_PIN 25      // First side-set pin (must be consecutive for PIO)
#define EXTIO_PIN 27     // Second side-set pin

#define UART_ID uart0   // Changed back to uart0 like working programs
#define BAUD_RATE 115200
#define UART_TX_PIN 0   // Using same pins as main project
#define UART_RX_PIN -1

// LED for status indication - PGA2350 has LED on pin 25
// Using a different pin for status LED to avoid conflict
#define LED_PIN 28      // Using pin 28 for status LED

void initialize_uart() {
    // Initialize UART TX pin
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    // Initialize UART (matching working programs - TX only)
    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}


void print_pin_states(void) {
    printf("Pin States - CLOCK_5: %d, CLOCK_15B: %d, DEN: %d, IO_M: %d\n",
           gpio_get(CLOCK_5_PIN),
           gpio_get(CLOCK_15B_PIN),
           gpio_get(DEN_PIN),
           gpio_get(EXTIO_PIN));
}

void setup_clock_simulation(void) {
    // Optional: Set up GPIO pins to simulate clock signals for testing
    // You can comment this out if you have actual clock signals
    printf("\nSetting up clock simulation pins (optional - remove if using real clocks)\n");

    // These would be outputs if we're simulating the clocks
    // gpio_init(CLOCK_5_PIN);
    // gpio_init(CLOCK_15B_PIN);
    // gpio_set_dir(CLOCK_5_PIN, GPIO_OUT);
    // gpio_set_dir(CLOCK_15B_PIN, GPIO_OUT);
    // gpio_put(CLOCK_5_PIN, 0);
    // gpio_put(CLOCK_15B_PIN, 0);
}

void toggle_test_clocks(void) {
    // This function can be used to manually toggle clocks for testing
    // if you don't have actual clock signals
    static bool clock_5_state = false;
    static bool clock_15b_state = false;

    clock_5_state = !clock_5_state;
    gpio_put(CLOCK_5_PIN, clock_5_state);

    // Toggle CLOCK_15B at different rate
    if (clock_5_state) {
        clock_15b_state = !clock_15b_state;
        gpio_put(CLOCK_15B_PIN, clock_15b_state);
    }
}

int main() {
    stdio_init_all();

    set_sys_clock_khz(200000, true);

    initialize_uart();

    // Wait for USB serial connection
    sleep_ms(2000);

    printf("\n=== PIO Clock Wait and Side-Set Test ===\n");
    printf("This test verifies:\n");
    printf("1. CLOCK_5 wait functionality (pin %d)\n", CLOCK_5_PIN);
    printf("2. CLOCK_15B wait functionality (pin %d)\n", CLOCK_15B_PIN);
    printf("3. Side-set output on DEN (pin %d) and IO_M (pin %d)\n", DEN_PIN, EXTIO_PIN);
    printf("\n");

    // Initialize LED for status
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // Turn on to indicate program is running

    // Initialize monitoring pins (to read their state)
    gpio_init(DEN_PIN);
    gpio_init(EXTIO_PIN);

    // Before PIO takes over, set them as inputs to read initial state
    gpio_set_dir(DEN_PIN, GPIO_IN);
    gpio_set_dir(EXTIO_PIN, GPIO_IN);

    printf("Initial pin states:\n");
    print_pin_states();

    // Choose PIO instance and state machine
    PIO pio = pio0;
    uint sm = 0;

    // Load the PIO program
    uint offset = pio_add_program(pio, &test_clock_sideset_program);
    printf("\nPIO program loaded at offset %d\n", offset);

    // Initialize the PIO program
    test_clock_sideset_program_init(pio, sm, offset);

    printf("\n=== Test Started ===\n");
    printf("The PIO program is now running and will:\n");
    printf("1. Wait for CLOCK_5 low->high transition\n");
    printf("2. Wait for CLOCK_15B low->high transition\n");
    printf("3. Set DEN and IO_M high via side-set\n");
    printf("4. Cycle through different side-set combinations\n");
    printf("5. Loop continuously\n\n");

    // Optional: Uncomment to simulate clocks if you don't have real signals
    // setup_clock_simulation();

    // Main monitoring loop
    uint32_t loop_count = 0;
    uint32_t last_den = 2, last_iom = 2;  // Invalid initial values to detect changes

    while (true) {
        // Read current pin states
        uint32_t den_state = gpio_get(DEN_PIN);
        uint32_t iom_state = gpio_get(EXTIO_PIN);

        // Print when side-set pins change
        if (den_state != last_den || iom_state != last_iom) {
            printf("[%lu] Side-set change detected: DEN=%d, IO_M=%d",
                   loop_count, den_state, iom_state);

            // Decode what this means
            if (den_state && iom_state) {
                printf(" (Both HIGH - side 0b11)\n");
            } else if (!den_state && iom_state) {
                printf(" (DEN=0, IO_M=1 - side 0b01)\n");
            } else if (den_state && !iom_state) {
                printf(" (DEN=1, IO_M=0 - side 0b10)\n");
            } else {
                printf(" (Both LOW - side 0b00)\n");
            }

            last_den = den_state;
            last_iom = iom_state;
        }

        // Every 1000 iterations, print full status
        if (loop_count % 1000 == 0) {
            // Blink LED to show program is alive
            gpio_put(LED_PIN, loop_count & 0x200 ? 1 : 0);

            // Periodically print all pin states
            if (loop_count % 10000 == 0) {
                printf("[%lu] ", loop_count);
                print_pin_states();
            }
        }

        // Optional: Toggle test clocks if simulating
        // if (loop_count % 100 == 0) {
        //     toggle_test_clocks();
        // }

        loop_count++;
        sleep_us(10);  // Small delay to avoid overwhelming the output
    }

    return 0;
}
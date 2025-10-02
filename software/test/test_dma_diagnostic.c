/*
 * Diagnostic test program to identify DMA hardware issues
 * This program performs step-by-step testing with extensive logging
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "dma_read_write.pio.h"
#include "pico_victor/dma.h"

#define TEST_SIZE 2  // Start with just 2 bytes for debugging
#define TEST_ADDRESS 0x10000  // Test address in Victor RAM (segment 0x1000:0x0000)
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Simple core1 stub
void core1_test_stub() {
    while (1) {
        sleep_ms(100);
    }
}

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

void print_gpio_states() {
    printf("\n=== GPIO Pin States ===\n");

    // NOTE: We do NOT initialize pins here anymore as it interferes with PIO
    // We just read the current state of pins

    // Data pins
    printf("Data pins (BD0-BD7): ");
    for (int i = BD0_PIN; i < BD0_PIN + 8; i++) {
        printf("%d:%d ", i, gpio_get(i));
    }
    printf("\n");

    // Address pins
    printf("Address pins (A0-A19): ");
    for (int i = 0; i < 20; i++) {
        printf("%d:%d ", i, gpio_get(i));
    }
    printf("\n");

    // Control pins
    printf("Control pins:\n");
    printf("  RD (pin %d) = %d\n", RD_PIN, gpio_get(RD_PIN));
    printf("  WR (pin %d) = %d\n", WR_PIN, gpio_get(WR_PIN));
    printf("  HOLD (pin %d) = %d\n", HOLD_PIN, gpio_get(HOLD_PIN));
    printf("  ALE (pin %d) = %d\n", ALE_PIN, gpio_get(ALE_PIN));
    printf("  HLDA (pin %d) = %d\n", HLDA_PIN, gpio_get(HLDA_PIN));
    printf("  READY (pin %d) = %d\n", Ready_PIN, gpio_get(Ready_PIN));
    printf("  CLOCK_5 (pin %d) = %d\n", CLOCK_5_PIN, gpio_get(CLOCK_5_PIN));
    printf("  CLOCK_15B (pin %d) = %d\n", CLOCK_15B_PIN, gpio_get(CLOCK_15B_PIN));
    printf("  LOW_ADDR_DIR (pin %d) = %d\n", LOW_ADDR_DIR, gpio_get(LOW_ADDR_DIR));
    printf("  BUS_CNTRL_DIR (pin %d) = %d\n", BUS_CNTRL_DIR, gpio_get(BUS_CNTRL_DIR));
}

void check_bus_signals() {
    printf("\n=== Bus Signal Check ===\n");

    // Check clock signals WITHOUT taking them from PIO control
    // Just read their current state
    printf("Clock signals (reading without init):\n");
    int clk5_samples = 0;
    int clk15_samples = 0;

    // Sample clocks without initializing them
    for (int i = 0; i < 1000; i++) {
        clk5_samples += gpio_get(CLOCK_5_PIN);
        clk15_samples += gpio_get(CLOCK_15B_PIN);
        busy_wait_us(1);
    }
    printf("  CLK5 activity (out of 1000): %d\n", clk5_samples);
    printf("  CLK15B activity (out of 1000): %d\n", clk15_samples);

    // Check for transitions
    int clk5_transitions = 0;
    int clk15_transitions = 0;
    int last_clk5 = gpio_get(CLOCK_5_PIN);
    int last_clk15 = gpio_get(CLOCK_15B_PIN);

    for (int i = 0; i < 1000; i++) {
        int current_clk5 = gpio_get(CLOCK_5_PIN);
        int current_clk15 = gpio_get(CLOCK_15B_PIN);

        if (current_clk5 != last_clk5) clk5_transitions++;
        if (current_clk15 != last_clk15) clk15_transitions++;

        last_clk5 = current_clk5;
        last_clk15 = current_clk15;

        busy_wait_us(1);
    }
    printf("  CLK5 transitions detected: %d\n", clk5_transitions);
    printf("  CLK15B transitions detected: %d\n", clk15_transitions);

    printf("\nWARNING: Skipping HOLD/HLDA test to avoid interfering with PIO\n");
    printf("The PIO will test HOLD/HLDA when it runs\n");

    printf("READY signal = %d (should be 1 for normal operation)\n", gpio_get(Ready_PIN));
}

void test_pio_single_write(PIO pio, uint sm, uint32_t address, uint8_t data) {
    printf("\n=== Single Write Test ===\n");
    printf("Writing 0x%02X to address 0x%05X\n", data, address);

    // Check PIO state before
    printf("Before: SM%d PC=0x%x, stalled=%d, TX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_tx_fifo_empty(pio, sm));

    // Prepare the data for PIO
    uint32_t addr_only = address & 0xFFFFF;  // 20-bit address
    uint32_t addr_with_data = (addr_only & 0xFFF00) | (data & 0xFF);  // Address with data in low byte

    printf("Sending to PIO: addr=0x%05X, addr_with_data=0x%05X\n", addr_only, addr_with_data);

    // Send to PIO
    pio_sm_put_blocking(pio, sm, addr_only);
    pio_sm_put_blocking(pio, sm, addr_with_data);

    // Wait for operation
    sleep_ms(10);

    // Check PIO state after
    printf("After: SM%d PC=0x%x, stalled=%d, TX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_tx_fifo_empty(pio, sm));
}

void test_pio_single_read(PIO pio, uint sm, uint32_t address) {
    printf("\n=== Single Read Test ===\n");
    printf("Reading from address 0x%05X\n", address);

    // Check PIO state before
    printf("Before: SM%d PC=0x%x, stalled=%d, RX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_rx_fifo_empty(pio, sm));

    // Send address to PIO
    uint32_t addr_only = address & 0xFFFFF;
    printf("Sending to PIO: addr=0x%05X\n", addr_only);
    pio_sm_put_blocking(pio, sm, addr_only);

    // Wait and try to read
    sleep_ms(10);

    uint32_t data = 0xFF;  // Default value
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        data = pio_sm_get(pio, sm);
        printf("Read data: 0x%02X\n", data & 0xFF);
    } else {
        printf("No data in RX FIFO!\n");
    }

    // Check PIO state after
    printf("After: SM%d PC=0x%x, stalled=%d, RX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_rx_fifo_empty(pio, sm));
}

// Forward declarations for signal simulator
void signal_simulator_init();
void signal_simulator_stop();
void signal_simulator_test();

void initialize_bus_direction() {
    printf("\n=== Initializing Bus Direction ===\n");

    // Initialize direction control pins
    gpio_init(LOW_ADDR_DIR);
    gpio_init(BUS_CNTRL_DIR);

    // Set as outputs
    gpio_set_dir(LOW_ADDR_DIR, GPIO_OUT);
    gpio_set_dir(BUS_CNTRL_DIR, GPIO_OUT);

    // Set initial direction to 8088->Pico (read mode)
    gpio_put(LOW_ADDR_DIR, DIR_8088_TO_PICO);
    gpio_put(BUS_CNTRL_DIR, DIR_8088_TO_PICO);

    printf("Bus direction pins set to 8088->Pico mode\n");

    // Try toggling to check they work
    printf("Testing direction pin toggle...\n");
    gpio_put(LOW_ADDR_DIR, DIR_PICO_TO_8088);
    gpio_put(BUS_CNTRL_DIR, DIR_PICO_TO_8088);
    sleep_ms(1);
    printf("  After toggle to Pico->8088: LOW_ADDR=%d, BUS_CNTRL=%d\n",
           gpio_get(LOW_ADDR_DIR), gpio_get(BUS_CNTRL_DIR));

    gpio_put(LOW_ADDR_DIR, DIR_8088_TO_PICO);
    gpio_put(BUS_CNTRL_DIR, DIR_8088_TO_PICO);
    sleep_ms(1);
    printf("  After toggle back to 8088->Pico: LOW_ADDR=%d, BUS_CNTRL=%d\n",
           gpio_get(LOW_ADDR_DIR), gpio_get(BUS_CNTRL_DIR));
}

int main() {
    stdio_init_all();
    set_sys_clock_khz(200000, true);
    initialize_uart();

    printf("\n=== DMA Hardware Diagnostic Test ===\n");
    printf("Running comprehensive diagnostics...\n\n");

    // Sleep briefly to ensure serial output is visible
    sleep_ms(2000);

    // Initialize bus direction control
    initialize_bus_direction();

    // Toggle HOLD pin like in hello.c to wake up DMA arbitration
    printf("\n=== Toggling HOLD Pin ===\n");
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 0);
    gpio_put(HOLD_PIN, 1);
    gpio_put(HOLD_PIN, 1);
    gpio_put(HOLD_PIN, 0);
    printf("HOLD pin toggled\n");

    // Ensure clock and handshake pins are configured as inputs for real hardware
    printf("\n=== Configuring Hardware Input Pins ===\n");
    gpio_init(CLOCK_5_PIN);
    gpio_init(CLOCK_15B_PIN);
    gpio_init(Ready_PIN);
    gpio_init(HLDA_PIN);
    gpio_set_dir(CLOCK_5_PIN, GPIO_IN);
    gpio_set_dir(CLOCK_15B_PIN, GPIO_IN);
    gpio_set_dir(Ready_PIN, GPIO_IN);
    gpio_set_dir(HLDA_PIN, GPIO_IN);
    printf("Clock and handshake pins configured as inputs\n");

    // Comment out simulator since we're using real hardware
    // // Initialize signal simulator for testing without real hardware
    // printf("\n=== Starting Signal Simulator ===\n");
    // printf("This simulates CLK5, CLK15B, READY, and HLDA signals\n");
    // signal_simulator_init();
    // // Test the simulator
    // signal_simulator_test();

    // Check initial GPIO states
    print_gpio_states();

    // Check bus signals (should now show activity)
    check_bus_signals();

    // Launch stub core1
    multicore_launch_core1(core1_test_stub);

    // Configure DMA PIO state machines
    PIO dma_pio = PIO_DMA;

    // CRITICAL: Set GPIO base for PIO (from hello.c)
    int outcome = pio_set_gpio_base(dma_pio, BD0_PIN);
    printf("pio_set_gpio_base outcome: %d\n", outcome);

    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int read_sm = pio_claim_unused_sm(dma_pio, true);
    int write_sm = pio_claim_unused_sm(dma_pio, true);

    printf("\n=== PIO Configuration ===\n");
    printf("PIO: %d\n", pio_get_index(dma_pio));
    printf("Read SM: %d\n", read_sm);
    printf("Write SM: %d\n", write_sm);
    printf("Program offset: 0x%02x\n", dma_read_write_program_offset);
    printf("Program length: %d instructions\n", dma_read_write_program.length);

    // Initialize state machines
    dma_read_write_program_init(dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN, DMA_READ);
    dma_read_write_program_init(dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN, DMA_WRITE);

    // Don't prime here - we'll do it right before enabling each SM
    printf("\n=== State Machine Configuration Complete ===\n");

    // Check if FIFOs are ready
    printf("Read SM TX FIFO empty: %d\n", pio_sm_is_tx_fifo_empty(dma_pio, read_sm));
    printf("Write SM TX FIFO empty: %d\n", pio_sm_is_tx_fifo_empty(dma_pio, write_sm));

    // Check SM states after init
    printf("\nState after init:\n");
    printf("Read SM: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, read_sm),
           pio_sm_is_exec_stalled(dma_pio, read_sm));
    printf("Write SM: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, write_sm),
           pio_sm_is_exec_stalled(dma_pio, write_sm));

    // Test 1: Single write operation (TX FIFO only has 4 slots!)
    printf("\n=== Test 1: Single Write Operation ===\n");

    // Put ALL data in FIFO before enabling
    printf("Loading TX FIFO (4 slots total)...\n");

    // X and Y for init (2 slots)
    printf("  Slot 1: DMA_WRITE (0x%08x)\n", DMA_WRITE);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE);

    printf("  Slot 2: DMA_WRITE_T2_PINDIRS (0x%08x)\n", DMA_WRITE_T2_PINDIRS);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE_T2_PINDIRS);

    // First write operation (2 more slots = full!)
    uint32_t addr = TEST_ADDRESS & 0xFFFFF;
    uint32_t data_word = (addr & 0xFFF00) | (0xAA & 0xFF);

    printf("  Slot 3: Address (0x%05x)\n", addr);
    pio_sm_put_blocking(dma_pio, write_sm, addr);

    printf("  Slot 4: Data word (0x%08x) - data byte is 0xAA\n", data_word);
    pio_sm_put_blocking(dma_pio, write_sm, data_word);

    // Check FIFO before enabling
    printf("Before enabling - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm));

    printf("Enabling write SM...\n");
    pio_sm_set_enabled(dma_pio, write_sm, true);

    // Check immediately
    printf("Immediately after enabling - TX FIFO: %d/4, PC=0x%x\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm),
           pio_sm_get_pc(dma_pio, write_sm));

    // Check HOLD and HLDA states immediately
    printf("HOLD pin: %d, HLDA pin: %d\n", gpio_get(HOLD_PIN), gpio_get(HLDA_PIN));

    sleep_ms(1);

    printf("After 1ms - TX FIFO: %d/4, PC=0x%x\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm),
           pio_sm_get_pc(dma_pio, write_sm));
    printf("HOLD: %d, HLDA: %d, READY: %d, CLK5: %d, CLK15B: %d\n",
           gpio_get(HOLD_PIN), gpio_get(HLDA_PIN), gpio_get(Ready_PIN),
           gpio_get(CLOCK_5_PIN), gpio_get(CLOCK_15B_PIN));

    // Poll for a bit to see if signals change
    printf("Polling signals for 100ms...\n");
    int hlda_changes = 0;
    int last_hlda = gpio_get(HLDA_PIN);
    for (int i = 0; i < 100; i++) {
        int current_hlda = gpio_get(HLDA_PIN);
        if (current_hlda != last_hlda) {
            hlda_changes++;
            printf("  HLDA changed to %d at %dms\n", current_hlda, i);
            last_hlda = current_hlda;
        }
        sleep_ms(1);
    }
    printf("HLDA changes detected: %d\n", hlda_changes);

    // Check FIFO after 10ms total
    printf("After 10ms - TX FIFO level: %d/4, PC=0x%x\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm),
           pio_sm_get_pc(dma_pio, write_sm));

    // Check if state machine is waiting at expected point
    sleep_ms(100);
    printf("After write attempt - SM state: PC=0x%x, stalled=%d, TX FIFO: %d/4\n",
           pio_sm_get_pc(dma_pio, write_sm),
           pio_sm_is_exec_stalled(dma_pio, write_sm),
           pio_sm_get_tx_fifo_level(dma_pio, write_sm));

    // Check HOLD pin state
    printf("HOLD pin state: %d (should be 0 if PIO is requesting bus)\n", gpio_get(HOLD_PIN));

    pio_sm_set_enabled(dma_pio, write_sm, false);

    // Test 2: Single read operation
    printf("\n=== Test 2: Single Read Operation ===\n");

    // Initialize the read state machine with required values
    printf("Loading read SM FIFO with init values...\n");
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ);  // Direction
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ_T2_PINDIRS);  // Pin directions for read

    printf("Before enabling read SM - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, read_sm));

    pio_sm_set_enabled(dma_pio, read_sm, true);
    sleep_ms(10);

    test_pio_single_read(dma_pio, read_sm, TEST_ADDRESS);

    sleep_ms(100);
    printf("After read attempt - SM state: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, read_sm),
           pio_sm_is_exec_stalled(dma_pio, read_sm));

    pio_sm_set_enabled(dma_pio, read_sm, false);

    // Final GPIO state check
    printf("\n=== Final GPIO State ===\n");
    print_gpio_states();

    // Check for any PIO errors
    printf("\n=== PIO Error Check ===\n");
    uint32_t fdebug = pio0_hw->fdebug;
    printf("PIO0 FDEBUG register: 0x%08X\n", fdebug);
    if (fdebug & 0x0F) {
        printf("  TX STALL detected on SM: %d\n", fdebug & 0x0F);
    }
    if ((fdebug >> 8) & 0x0F) {
        printf("  TX OVER detected on SM: %d\n", (fdebug >> 8) & 0x0F);
    }
    if ((fdebug >> 16) & 0x0F) {
        printf("  RX UNDER detected on SM: %d\n", (fdebug >> 16) & 0x0F);
    }
    if ((fdebug >> 24) & 0x0F) {
        printf("  RX STALL detected on SM: %d\n", (fdebug >> 24) & 0x0F);
    }

    printf("\n=== Diagnostic Complete ===\n");
    printf("Key observations:\n");
    printf("1. Check if HLDA responds to HOLD signal\n");
    printf("2. Check if clocks are running\n");
    printf("3. Check PIO state machine progression\n");
    printf("4. Check for PIO errors\n");

    // Keep running for monitoring
    while (1) {
        sleep_ms(5000);
    }

    return 0;
}
/*
 * benchmark_irq.c - Benchmark different IRQ handler implementations
 */

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "pico_victor/dma.h"
#include "pico_victor/dma_fast.h"
#include "pico_victor/dma_ultra_fast.h"

// UART configuration matching dma_board.c
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Test data structure
typedef struct {
    uint32_t address;
    uint8_t data;
    bool is_read;
    const char *description;
} test_case_t;

// Test cases covering all register types
static test_case_t test_cases[] = {
    // Control register tests
    {0xEF300, 0x15, false, "Write control register"},
    {0xEF300, 0x00, true,  "Read control register"},
    
    // Data register tests  
    {0xEF310, 0x55, false, "Write data register"},
    {0xEF310, 0x00, true,  "Read data register"},
    
    // Status register tests
    {0xEF320, 0x00, true,  "Read status register"},
    
    // Address register tests (most common)
    {0xEF380, 0x00, false, "Write DMA addr low"},
    {0xEF380, 0x00, true,  "Read DMA addr low"},
    {0xEF3A0, 0x50, false, "Write DMA addr mid"},
    {0xEF3A0, 0x00, true,  "Read DMA addr mid"},
    {0xEF3C0, 0x0F, false, "Write DMA addr high"},
    {0xEF3C0, 0x00, true,  "Read DMA addr high"},
};

// Simulate PIO FIFO access
static uint32_t simulated_fifo_value;
static uint32_t simulated_tx_value;

// Mock PIO functions for benchmark mode using macros
// This avoids redefinition errors with SDK functions
#define pio_sm_get(pio, sm) (simulated_fifo_value)
#define pio_sm_get_tx_fifo_level(pio, sm) (4)

// Stub functions for external dependencies
void fast_log(const char *msg) {
    // No-op for benchmark
    (void)msg;
}

void handle_sasi_command_byte(uint8_t cmd) {
    // No-op for benchmark
    (void)cmd;
}

// Benchmark function
typedef void (*irq_handler_t)(void);

static void benchmark_handler(irq_handler_t handler, const char *name, test_case_t *test) {
    // Prepare simulated FIFO value
    simulated_fifo_value = test->address | 
                          (test->data << 20) | 
                          (test->is_read ? 0x10000000 : 0);
    
    // Configure SysTick for measurement
    systick_hw->csr = 0x5; // Enable, use processor clock
    systick_hw->rvr = 0x00FFFFFF;
    
    // Warm up caches
    for (int i = 0; i < 10; i++) {
        handler();
    }
    
    // Measure multiple runs
    uint32_t min_cycles = 0xFFFFFFFF;
    uint32_t max_cycles = 0;
    uint32_t total_cycles = 0;
    const int runs = 1000;
    
    for (int i = 0; i < runs; i++) {
        uint32_t start = systick_hw->cvr;
        handler();
        uint32_t end = systick_hw->cvr;
        
        uint32_t cycles = (start >= end) ? (start - end) : (start + (0x00FFFFFF - end));
        
        if (cycles < min_cycles) min_cycles = cycles;
        if (cycles > max_cycles) max_cycles = cycles;
        total_cycles += cycles;
    }
    
    uint32_t avg_cycles = total_cycles / runs;
    
    printf("%-20s | %-30s | Min: %3lu cyc (%5.1f ns) | Avg: %3lu cyc (%5.1f ns) | Max: %3lu cyc (%5.1f ns)\n",
           name, test->description,
           min_cycles, min_cycles * 5.0,
           avg_cycles, avg_cycles * 5.0, 
           max_cycles, max_cycles * 5.0);
}

int main() {
    // Initialize UART with custom pins matching dma_board.c
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);

    sleep_ms(2000); // Wait for UART to stabilize
    
    printf("\n=== DMA IRQ Handler Benchmark ===\n");
    printf("System clock: 200MHz (5ns per cycle)\n");
    printf("Target: <80ns (<16 cycles)\n\n");
    
    // Initialize DMA registers
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        printf("Failed to initialize DMA registers\n");
        return -1;
    }

    
    // Run benchmarks for each test case
    for (size_t i = 0; i < sizeof(test_cases)/sizeof(test_cases[0]); i++) {
        printf("\nTest case: %s\n", test_cases[i].description);
        printf("%-20s | %-30s | %-25s | %-25s | %-25s\n",
               "Implementation", "Operation", "Minimum", "Average", "Maximum");
        printf("================================================================================================================================================\n");
        
        benchmark_handler(registers_irq_handler, "Original", &test_cases[i]);
        benchmark_handler(registers_irq_handler_fast, "Fast", &test_cases[i]);
        benchmark_handler(registers_irq_handler_ultra, "Ultra", &test_cases[i]);
        benchmark_handler(registers_irq_handler_ultra_asm, "Ultra ASM", &test_cases[i]);
    }
    
    printf("\n=== Summary ===\n");
    printf("Address register operations (0x80-0xFF) should be fastest\n");
    printf("Complex registers (0x00-0x2F) will be slower due to logic\n");
    printf("Ultra ASM version should achieve <80ns for address registers\n");
    
    return 0;
}
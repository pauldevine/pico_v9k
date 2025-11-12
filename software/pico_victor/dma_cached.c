/*
 * dma_cached.c - Modified DMA implementation with cached/deferred processing
 *
 * This file contains the modified core1_main that sets up the cached IRQ handler
 * and runs the deferred processing worker.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/structs/systick.h"
#include "dma.h"
#include "dma_defer.h"
#include "logging.h"

// External handler declarations
void registers_irq_handler_cached(void);
void registers_irq_handler_cached_asm(void);
void registers_irq_handler_cached_init(void);

static inline void warmup_read_sequence(PIO pio, int sm, uint32_t address) {
    // bus_output_helper pushes PREFETCH when handling reads
    if (pio_sm_is_tx_fifo_empty(pio, sm)) {
        pio_sm_put(pio, sm, board_fifo_encode_read(address));
        bus_output_helper_irq_handler_cached_asm();
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            pio_sm_get(pio, sm);
        }
    }
}

static inline void warmup_write_sequence(PIO pio, int sm, uint32_t address, uint8_t value) {
    // board_registers pushes WRITE_VALUE when handling writes
    if (pio_sm_is_tx_fifo_empty(pio, sm)) {
        pio_sm_put(pio, sm, dma_fifo_encode_write(address, value));
        board_registers_irq_handler_cached_asm();
        if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
            pio_sm_get(pio, sm);
        }
    }
}

// Modified core1_main that uses cached handler and deferred processing
void core1_main_cached() {
    // Run IRQ processing on separate core
    // Configure the IRQs used to indicate a Register has been accessed on the 8088 bus

    PIO register_pio = PIO_REGISTERS;
    int register_sm = REGISTERS_SM;
    PIO bus_helper_pio = PIO_BUS_HELPER;
    int bus_helper_sm = BUS_HELPER_SM;

    printf("Setting up CACHED IRQ handlers\n");
    printf("  board_registers: PIO%d SM%d\n", pio_get_index(register_pio), register_sm);
    printf("  bus_output_helper: PIO%d SM%d\n", pio_get_index(bus_helper_pio), bus_helper_sm);

    // Initialize the cached/deferred system
    printf("Initializing cached/deferred processing system...\n");
    registers_irq_handler_cached_init();  // This calls dma_defer_init() internally

    dma_registers_t *dma = dma_get_registers();
    if (dma) {
        // Initialize DMA registers to known state
        dma->dma_address.full = 0;
        dma->control = 0;
        dma->status = 0;
        dma->bus_ctrl = 0;

        // Sync initial state to cached registers
        cached_registers_t *cached = defer_get_cached_registers();
        cached->values[REG_ADDR_L] = 0;
        cached->values[REG_ADDR_M] = 0;
        cached->values[REG_ADDR_H] = 0;
        cached->values[REG_STATUS] = 0;
        cached->values[0x30] = 0;  // Status alias
        cached->values[REG_DATA] = 0xFF;
        cached->values[REG_CONTROL] = 0;

        printf("Pre-warming IRQ handler caches...\n");

        // Temporarily disable IRQs while we warm up
        irq_set_enabled(PIO0_IRQ_0, false);
        irq_set_enabled(PIO1_IRQ_0, false);

        // Set the IRQ handlers for both PIOs
        irq_set_exclusive_handler(PIO0_IRQ_0, board_registers_irq_handler_cached_asm);
        irq_set_exclusive_handler(PIO1_IRQ_0, bus_output_helper_irq_handler_cached_asm);

        // Warm up board_registers handler (handles writes only)
        setup_pio_instance(register_pio, register_sm);
        for (int warm_iter = 0; warm_iter < 10; warm_iter++) {
            warmup_write_sequence(register_pio, register_sm, DMA_REGISTER_BASE + REG_CONTROL, 0x00);
            warmup_write_sequence(register_pio, register_sm, DMA_REGISTER_BASE + REG_ADDR_L, 0x00);
            warmup_write_sequence(register_pio, register_sm, DMA_REGISTER_BASE + REG_ADDR_M, 0x00);
            warmup_write_sequence(register_pio, register_sm, DMA_REGISTER_BASE + REG_ADDR_H, 0x00);
        }

        // Warm up bus_output_helper handler (handles reads and DMA reads)
        // Note: bus_output_helper actually processes reads, not board_registers
        setup_pio_instance(bus_helper_pio, bus_helper_sm);
        for (int warm_iter = 0; warm_iter < 10; warm_iter++) {
            warmup_read_sequence(bus_helper_pio, bus_helper_sm, DMA_REGISTER_BASE + REG_DATA);
            warmup_read_sequence(bus_helper_pio, bus_helper_sm, DMA_REGISTER_BASE + REG_STATUS);
            warmup_read_sequence(bus_helper_pio, bus_helper_sm, DMA_REGISTER_BASE + REG_ADDR_L);
        }

        printf("Cache pre-warming complete\n");

        // Small delay to let caches settle
        busy_wait_us(100);

        // One more round of warming after the delay
        for (int i = 0; i < 5; i++) {
            warmup_write_sequence(register_pio, register_sm, DMA_REGISTER_BASE + REG_ADDR_L, (uint8_t)i);
        }
    }

    // Clear the FIFOs so cache timing doesn't interfere with normal operation
    pio_sm_set_enabled(register_pio, register_sm, false);
    pio_sm_clear_fifos(register_pio, register_sm);
    pio_sm_set_enabled(register_pio, register_sm, true);

    pio_sm_set_enabled(bus_helper_pio, bus_helper_sm, false);
    pio_sm_clear_fifos(bus_helper_pio, bus_helper_sm);
    pio_sm_set_enabled(bus_helper_pio, bus_helper_sm, true);

    printf("\nPIO check board_registers: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(register_pio, register_sm),
                    pio_sm_get_tx_fifo_level(register_pio, register_sm),
                    pio_sm_get_pc(register_pio, register_sm),
                    pio_sm_is_exec_stalled(register_pio, register_sm));

    printf("PIO check bus_output_helper: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(bus_helper_pio, bus_helper_sm),
                    pio_sm_get_tx_fifo_level(bus_helper_pio, bus_helper_sm),
                    pio_sm_get_pc(bus_helper_pio, bus_helper_sm),
                    pio_sm_is_exec_stalled(bus_helper_pio, bus_helper_sm));

    // Enable the IRQs for both PIOs
    pio_set_irq0_source_enabled(register_pio, fifo_sources[register_sm], true);
    pio_set_irq0_source_enabled(bus_helper_pio, fifo_sources[bus_helper_sm], true);
    irq_set_enabled(PIO0_IRQ_0, true);
    irq_set_enabled(PIO1_IRQ_0, true);

    printf("Core1 started with separate CACHED handlers\n");
    printf("  board_registers handler on PIO0_IRQ_0\n");
    printf("  bus_output_helper handler on PIO1_IRQ_0\n");

    // Initialize systick for timing measurements
    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)

    // Now run the deferred processing worker
    // This will process the queue of deferred register operations
    printf("Starting deferred processing worker on Core1...\n");
    defer_worker_main();  // This never returns
}

// Alternative: Process deferred events from main loop instead of Core1
// This can be called periodically from the main loop if you prefer
void dma_process_deferred_events_cached(void) {
    static uint32_t last_process = 0;
    defer_queue_t *queue = defer_get_queue();
    defer_entry_t entry;
    dma_registers_t *dma = dma_get_registers();

    // Process up to 10 entries per call to avoid blocking too long
    int processed = 0;
    while (processed < 10 && defer_dequeue(queue, &entry)) {
        defer_process_entry(dma, &entry);
        processed++;
    }

    // Log statistics periodically (every 1000 calls)
    static uint32_t call_count = 0;
    if (++call_count >= 1000) {
        call_count = 0;
        if (queue->drops > 0 || queue->processed > last_process + 100) {
            fast_log("DEFER_STATS: processed=%u, drops=%u, queue_level=%u\n",
                     queue->processed, queue->drops,
                     (queue->head - queue->tail) & DEFER_QUEUE_MASK);
            last_process = queue->processed;
        }
    }
}

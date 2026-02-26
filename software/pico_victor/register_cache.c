/*
 * register_cache.c - Modified DMA implementation with cached/deferred processing
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
#include "hardware/sync.h"  // For __dmb()
#include "dma.h"
#include "reg_queue_processor.h"
#include "register_irq_handlers.h"
#include "dma_irq_handler.h"
#include "fifo_helpers.h"
#include "logging.h"
#include "pico_storage/storage.h"
#include "pico_storage/sd_storage.h"
#include "pico_fujinet/spi.h"
#include "sasi_log.h"

void setup_irq_handlers(void) {
    //delete any data that might be in the FIFOs from cache warming
    pio_sm_clear_fifos(PIO_REGISTERS, REG_SM_CONTROL);
    pio_sm_clear_fifos(PIO_DMA_MASTER, DMA_SM_CONTROL);

    // Register READ/WRITE path = 8088 accessing pico registers (single SM)
    // MUST have highest priority so cache is updated before any read returns stale data
    pio_set_irq0_source_enabled(PIO_REGISTERS, fifo_sources[REG_SM_CONTROL], true);
    irq_set_exclusive_handler(PIO0_IRQ_0, register_read_irq_isr);
    irq_set_priority(PIO0_IRQ_0, 0);   // highest priority - updates cache first
    irq_set_enabled(PIO0_IRQ_0, true);

    // DMA READ path = pico doing DMA read from 8088 bus
    // NOTE: DMA read IRQ is NOT enabled at startup to avoid spurious interrupts
    // during register-only operations. Enable via enable_dma_read_irq() when DMA is needed.
    // The current blocking DMA implementation (pio_sm_get_blocking) doesn't need the IRQ anyway.
    pio_set_irq1_source_enabled(PIO_DMA_MASTER, fifo_sources[DMA_SM_CONTROL], true);
    irq_set_exclusive_handler(PIO1_IRQ_1, dma_read_isr);
    irq_set_priority(PIO1_IRQ_1, 2);
    // irq_set_enabled(PIO1_IRQ_1, true);  // Disabled - enable when DMA is needed

    // DMA Write path does not generate IRQs
}

// Enable DMA read IRQ - call this before starting DMA operations
void enable_dma_read_irq(void) {
    irq_set_enabled(PIO1_IRQ_1, true);
}

// Disable DMA read IRQ - call this when DMA operations are complete
void disable_dma_read_irq(void) {
    irq_set_enabled(PIO1_IRQ_1, false);
}

void warm_caches(void) {
    PIO register_pio = PIO_REGISTERS;
    int register_control = REG_SM_CONTROL;
    PIO pio_dma_master = PIO_DMA_MASTER;
    int dma_sm_control = DMA_SM_CONTROL;

    printf("Pre-warming IRQ handler caches...\n");

    // Temporarily disable IRQs while we warm up
    irq_set_enabled(PIO0_IRQ_0, false);
    irq_set_enabled(PIO1_IRQ_0, false);
    irq_set_enabled(PIO1_IRQ_1, false);

    // Warm cache lines directly instead of invoking IRQ handlers with synthetic FIFO data.
    cached_registers_t *cached = &cached_regs;
    volatile uint8_t dummy = 0;

    for (int iter = 0; iter < 8; iter++) {
        dummy ^= cached->values[REG_CONTROL];
        dummy ^= cached->values[REG_STATUS];
        dummy ^= cached->values[0x30];
        dummy ^= cached->values[REG_DATA];
        dummy ^= cached->values[REG_ADDR_L];
        dummy ^= cached->values[REG_ADDR_M];
        dummy ^= cached->values[REG_ADDR_H];
        dummy ^= (uint8_t)PIO_REGISTERS->fstat;
        dummy ^= (uint8_t)PIO_DMA_MASTER->fstat;
    }
    (void)dummy;

    printf("Cache pre-warming complete\n");

    // Small delay to let caches settle
    busy_wait_us(100);

    // Safe restart: clears FIFOs, releases XACK/EXTIO, JMPs to T0.
    reset_register_pio_sm();

    pio_sm_set_enabled(pio_dma_master, dma_sm_control, false);
    pio_sm_clear_fifos(pio_dma_master, dma_sm_control);
    pio_sm_set_enabled(pio_dma_master, dma_sm_control, true);


    // Re-enable IRQs (except DMA read IRQ which stays disabled until DMA is needed)
    irq_set_enabled(PIO0_IRQ_0, true);
    irq_set_enabled(PIO1_IRQ_0, true);
    // irq_set_enabled(PIO1_IRQ_1, true);  // DMA read IRQ stays disabled
}

// Modified core1_main that uses cached handler and deferred processing
void core1_main() {
    // Run IRQ processing on separate core
    // Configure the IRQs used to indicate a Register has been accessed on the 8088 bus

    PIO register_pio = PIO_REGISTERS;
    int register_control = REG_SM_CONTROL;
    PIO pio_dma_master = PIO_DMA_MASTER;
    int dma_sm_control = DMA_SM_CONTROL;

    printf("Setting up CACHED IRQ handlers\n");
    printf("  registers_control: PIO%d SM%d\n", pio_get_index(register_pio), register_control);
    printf("  dma_sm_control: PIO%d SM%d\n", pio_get_index(pio_dma_master), dma_sm_control);

    // Initialize the cached/deferred system
    printf("Initializing cached/deferred processing system...\n");
    init_register_irq_handlers();  // This calls dma_defer_init() internally
    setup_irq_handlers();

    // Initialize DMA registers to known state
    dma_registers.dma_address.full = 0;
    dma_registers.control = 0;
    dma_registers.status = 0;
    dma_registers.bus_ctrl = 0;

    // Sync initial state to cached registers
    cached_regs.values[REG_ADDR_L] = 0;
    cached_regs.values[REG_ADDR_M] = 0;
    cached_regs.values[REG_ADDR_H] = 0;
    cached_regs.values[REG_STATUS] = 0;
    cached_regs.values[0x30] = 0;  // Status alias
    cached_regs.values[REG_DATA] = 0xFF;
    cached_regs.values[REG_CONTROL] = 0;

    // Pre-warm the caches for both IRQ handlers
    warm_caches();

    printf("\nPIO check board_registers: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(register_pio, register_control),
                    pio_sm_get_tx_fifo_level(register_pio, register_control),
                    pio_sm_get_pc(register_pio, register_control),
                    pio_sm_is_exec_stalled(register_pio, register_control));

    printf("PIO check dma_sm_control: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(pio_dma_master, dma_sm_control),
                    pio_sm_get_tx_fifo_level(pio_dma_master, dma_sm_control),
                    pio_sm_get_pc(pio_dma_master, dma_sm_control),
                    pio_sm_is_exec_stalled(pio_dma_master, dma_sm_control));
    printf("Core1 started with separate CACHED handlers\n");
    printf("  board_registers handler on PIO0_IRQ_0\n");
    printf("  bus_output_helper handler on PIO1_IRQ_0\n");

    // Initialize systick for timing measurements
    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)

    // --- Storage initialization runs on Core 1 ---
    // This ensures the SDIO library's DMA_IRQ_1 is registered on Core 1's
    // NVIC (irq_set_enabled enables on the calling core).  Running SD init
    // here keeps storage I/O and SASI command processing on the same core.
    //
    // IMPORTANT: After SDIO init, DMA_IRQ_1 must be set to a LOWER priority
    // than PIO0_IRQ_0 (the register ISR, priority 0).  The SDIO library
    // leaves DMA_IRQ_1 at default priority 0 (highest), which is the SAME
    // hardware priority as the register ISR.  Equal-priority interrupts
    // cannot preempt each other on ARM Cortex-M, so the register ISR would
    // be blocked while the SDIO DMA handler runs.  This causes RX FIFO
    // overflow during rapid register writes (configure_dma_mode writes 7
    // registers in ~5us), leading to missed CONTROL writes and stale cache.
    //
    // While SD init runs (~300-500ms), the register ISR is already active
    // and serves cached responses.  The defer queue accumulates entries but
    // the 8088 sends very few register accesses this early in boot.
#if USE_SD_STORAGE
    printf("Core1: Initializing SD card storage backend...\n");
    sd_storage_register();
    if (storage_init(STORAGE_BACKEND_SDCARD)) {
        int image_count = sd_storage_get_image_count();
        if (image_count > 0) {
            const char *first_image = sd_storage_get_image_name(0);
            if (first_image) {
                printf("Core1: Auto-mounting '%s'\n", first_image);
                if (!storage_mount(0, first_image, false)) {
                    printf("Core1: failed to mount '%s' on target 0\n", first_image);
                }
            }
        } else {
            printf("Core1: No images found, trying default '%s'\n", SD_DISK_IMAGE);
            if (!storage_mount(0, SD_DISK_IMAGE, false)) {
                printf("Core1: failed to mount '%s' on target 0\n", SD_DISK_IMAGE);
            }
        }
    } else {
        printf("Core1: SD init failed, falling back to FujiNet\n");
        spi_bus_init();
        if (!fujinet_config_boot(false)) {
            printf("Core1: FujiNet failed to clear boot config\n");
        }
        if (!fujinet_mount_host(0, FUJINET_DISK_ACCESS_READ)) {
            printf("Core1: FujiNet failed to mount host slot 0\n");
        }
        if (!fujinet_mount_disk_slot(0, FUJINET_DISK_ACCESS_READ)) {
            printf("Core1: FujiNet failed to mount disk slot 0\n");
        }
    }
#else
    printf("Core1: Initializing FujiNet storage backend...\n");
    spi_bus_init();
    if (!fujinet_config_boot(false)) {
        printf("Core1: FujiNet failed to clear boot config\n");
    }
    if (!fujinet_mount_host(0, FUJINET_DISK_ACCESS_READ)) {
        printf("Core1: FujiNet failed to mount host slot 0\n");
    }
    if (!fujinet_mount_disk_slot(0, FUJINET_DISK_ACCESS_READ)) {
        printf("Core1: FujiNet failed to mount disk slot 0\n");
    }
#endif

    // CRITICAL: Lower DMA_IRQ_1 priority so the register ISR (PIO0_IRQ_0,
    // priority 0) can always preempt the SDIO DMA completion handler.
    // The SDIO library leaves DMA_IRQ_1 at default priority 0 (highest),
    // which blocks the register ISR during SD card operations.
    // RP2350 Cortex-M33 has 4 priority bits (bits 7:4), so 0x80 = level 8.
    irq_set_priority(DMA_IRQ_1, 0x80);

    sasi_log_init();

    // Signal Core 0 that storage is ready for log flushing
    extern volatile bool storage_ready;
    storage_ready = true;
    __dmb();

    printf("Core1: Storage ready, entering defer worker loop\n");

    // Now run the deferred processing worker
    // This will process the queue of deferred register operations
    defer_worker_main();  // This never returns
}

// Alternative: Process deferred events from main loop instead of Core1
// This can be called periodically from the main loop if you prefer
void dma_process_deferred_events_cached(void) {
    static uint32_t last_process = 0;
    defer_queue_t *queue = &defer_queue;
    defer_entry_t entry;
    dma_registers_t *dma = &dma_registers;

    // Process up to 10 entries per call to avoid blocking too long
    int processed = 0;
    while (processed < 10 && defer_dequeue(queue, &entry)) {
        defer_process_entry(dma, &entry);
        processed++;
    }

    // Log statistics periodically (every 1000 calls)
    static uint32_t call_count = 0;
    if (++call_count >= 10000) {
        call_count = 0;
        if (queue->drops > 0 || queue->processed > last_process + 100) {
            fast_log("DEFER_STATS: processed=%u, drops=%u, queue_level=%u\n",
                     queue->processed, queue->drops,
                     (queue->head - queue->tail) & DEFER_QUEUE_MASK);
            last_process = queue->processed;
        }
    }
}

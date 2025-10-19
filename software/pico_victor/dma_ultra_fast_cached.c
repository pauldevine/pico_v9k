/*
 * dma_ultra_fast_cached.c - Ultra-optimized cached DMA register handler
 *
 * This implementation returns cached values immediately to meet the 200ns timing
 * requirement, then enqueues register operations for deferred processing.
 */

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "dma_defer.h"
#include "logging.h"

// Define DEBUG_PIN if not already defined
#ifndef DEBUG_PIN
#define DEBUG_PIN 44 
#endif

// For pins > 31, we need to use different registers
#if DEBUG_PIN >= 32
#define DEBUG_PIN_MASK (1u << (DEBUG_PIN - 32))
#define SIO_GPIO_OUT_SET_REG (SIO_BASE + SIO_GPIO_HI_OUT_SET_OFFSET)
#define SIO_GPIO_OUT_CLR_REG (SIO_BASE + SIO_GPIO_HI_OUT_CLR_OFFSET)
#else
#define DEBUG_PIN_MASK (1u << DEBUG_PIN)
#define SIO_GPIO_OUT_SET_REG (SIO_BASE + SIO_GPIO_OUT_SET_OFFSET)
#define SIO_GPIO_OUT_CLR_REG (SIO_BASE + SIO_GPIO_OUT_CLR_OFFSET)
#endif

// External cache and queue instances from dma_defer.h - place in time_critical section for fast access
extern defer_queue_t defer_queue;
extern cached_registers_t cached_regs;

// Mask register offset based on MAME-style rules
static inline uint32_t mask_offset(uint32_t offset) {
    if (offset >= 0x80) {
        return offset & ~0x1F;  // Address registers: mask to 0x80, 0xA0, 0xC0
    }
    return offset & ~0x0F;      // Other registers: mask to 0x00, 0x10, 0x20, 0x30
}

// Pre-warm caches by touching critical memory locations
void registers_irq_handler_cached_init(void) {
    // Initialize deferred processing system
    dma_defer_init();

    // Get cached registers and pre-warm
    cached_registers_t *cached = defer_get_cached_registers();
    volatile uint8_t dummy = 0;

    // Touch all cached values multiple times
    for (int iter = 0; iter < 3; iter++) {
        for (int i = 0; i < 256; i++) {
            dummy = cached->values[i];
        }
    }

    // Touch the most commonly accessed addresses
    for (int iter = 0; iter < 10; iter++) {
        dummy = cached->values[0x80];
        dummy = cached->values[0xA0];
        dummy = cached->values[0xC0];
        dummy = cached->values[0x20];
        dummy = cached->values[0x30];
    }

    // Touch critical SIO registers
    volatile uint32_t dummy32;
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG;
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG;

    // Touch PIO registers
    dummy32 = PIO_REGISTERS->rxf[REGISTERS_SM];
    dummy32 = PIO_REGISTERS->txf[REGISTERS_SM];
    dummy32 = PIO_REGISTERS->fstat;

    (void)dummy;
    (void)dummy32;
}

// Ultra-optimized cached IRQ handler - returns cached values immediately
void __time_critical_func(registers_irq_handler_cached)() {
    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from PIO FIFO
    uint32_t raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);

    // Extract offset and check bounds
    uint32_t offset = (raw_value & 0xFFFFF) - DMA_REGISTER_BASE;
    if (offset >= 0x100) {
        *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
        return;
    }

    // Get cached registers
    cached_registers_t *cached = defer_get_cached_registers();

    // Apply MAME-style masking
    uint32_t masked_offset = mask_offset(offset);
    if (masked_offset == 0x30) {
        masked_offset = REG_STATUS;  // Alias handling
    }

    // Check if this is a read (bit 28)
    if (raw_value & 0x10000000) {
        // READ path - return cached value immediately
        uint8_t data = cached->values[masked_offset];

        #ifndef BENCHMARK_MODE
        uint32_t response = (0xFF00 | (uint32_t)data);
        pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, response);
        #endif

        // Always log address register reads for debugging
        if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
            fast_log("CACHED_READ: offset=0x%02x, data=0x%02x (cached)\n", masked_offset, data);
        }
    } else {
        // WRITE path - update cached value immediately for next read
        uint8_t write_data = (raw_value >> 20) & 0xFF;

        // Special handling for address high register
        if (masked_offset == REG_ADDR_H) {
            write_data &= 0x0F;
        }

        // Update cached value immediately
        cached->values[masked_offset] = write_data;

        // Handle status register alias
        if (masked_offset == REG_STATUS) {
            cached->values[0x30] = write_data;
        }

        // Log address register writes for debugging
        if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
            fast_log("CACHED_WRITE: offset=0x%02x, data=0x%02x\n", masked_offset, write_data);
        }
    }

    // Enqueue for deferred processing - this updates actual DMA state
    defer_queue_t *queue = defer_get_queue();
    defer_enqueue_fast(queue, raw_value);

    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

// Even more optimized version with minimal overhead
void __time_critical_func(registers_irq_handler_cached_asm)() {
    uint32_t raw_value;
    static uint32_t offset;
    uint8_t data;
    
    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from PIO FIFO - this is the critical timing point
    raw_value = PIO_REGISTERS->rxf[REGISTERS_SM];

    //extract 2-bit payload type flag
    uint32_t payload_type = (raw_value >> 30) & 0x03;

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    bool enque_result = false;

    // Handle different payload types
    switch(payload_type) {
        case FIFO_PREFETCH_ADDRESS:
            // Handle prefetch address
            offset = ((raw_value >> 10) & 0xFFFFF) - DMA_REGISTER_BASE;   // extract 20-bit payload, subtract base
            offset &= 0xFF;    // guarantees the index stays inside cached->values[]
            // READ - fetch from cache and respond immediately
            data = cached->values[offset];
            PIO_REGISTERS->txf[REGISTERS_SM] = (0xFF00 | (uint32_t)data); //send cached results w/in 200ns
            //fast_log("CACHED ASM offset=0x%08x\n", offset);  // DO NOT LOG - kills performance!
            *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
            break;
        case FIFO_READ_COMMIT:
            enque_result = true;
            break;
        case FIFO_WRITE_VALUE:
            enque_result = true;
            // Update cache immediately
            offset = ((raw_value >> 2) & 0xFFFFF) - DMA_REGISTER_BASE;   // extract 20-bit payload, subtract base
            offset &= 0xFF;    // guarantees the index stays inside cached->values[]
            data = (raw_value >> 22) & 0xFF; // extract 8-bit data
            // Special handling for address high register
            if (mask_offset(offset) == REG_ADDR_H) {
                data &= 0x0F;
            }
            
             // Update cache
            cached->values[offset] = data;
            //fast_log("CACHED ASM WRITE: raw_value=0x%08x, offset=0x%02x, data=0x%02x\n", raw_value, offset, data); // DO NOT LOG - kills performance!

            // Handle status register alias
            if (offset == 0x20) {
                cached->values[0x30] = data;
            }
            break;
        default:
            fast_log("CACHED ASM: Unknown payload type: 0x%02x\n", payload_type);
            goto exit; // Unknown payload type
            break;
    }

    //fast_log("CACHED ASM before queue: raw_value=0x%08x\n", raw_value);
    if (enque_result) {
        // Enqueue for deferred processing (fast inline version)
        //fast_log("CACHED ASM ENQUEUE: raw_value=0x%08x\n", raw_value);
        defer_queue_t *queue = &defer_queue;
        uint32_t head = queue->head;
        uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

        if (next != queue->tail) {
            queue->entries[head].raw_value = raw_value;
            __asm volatile("dmb" ::: "memory");
            queue->head = next;
        }
    }

exit:
    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}
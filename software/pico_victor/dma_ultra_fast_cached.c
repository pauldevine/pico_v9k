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

// Trace tagged FIFO payloads to help diagnose sequencing issues
#define FIFO_TRACE_SIZE 256
#define FIFO_TRACE_MASK (FIFO_TRACE_SIZE - 1)

enum {
    FIFO_TRACE_FLAG_ERROR = 0x01,
    FIFO_TRACE_FLAG_WRITE = 0x02,
};

typedef struct {
    uint32_t raw_value;
    uint8_t tag;
    uint8_t pending_before;
    uint8_t pending_after;
    uint8_t flags;
    uint8_t data;
} fifo_trace_entry_t;

static fifo_trace_entry_t fifo_trace[FIFO_TRACE_SIZE];
static volatile uint32_t fifo_trace_head = 0;
static volatile uint32_t fifo_trace_tail = 0;
static volatile uint32_t fifo_pending_prefetch = 0;

static inline void fifo_trace_record(uint32_t raw_value,
                                     uint32_t tag,
                                     uint8_t pending_before,
                                     uint8_t pending_after,
                                     uint8_t flags,
                                     uint8_t data) {
    uint32_t head = fifo_trace_head;
    fifo_trace_entry_t *entry = &fifo_trace[head & FIFO_TRACE_MASK];
    entry->raw_value = raw_value;
    entry->tag = (uint8_t)tag;
    entry->pending_before = pending_before;
    entry->pending_after = pending_after;
    entry->flags = flags;
    entry->data = data;
    __asm volatile("dmb" ::: "memory");
    fifo_trace_head = head + 1;
}

void dma_fifo_trace_flush(void) {
    uint32_t tail = fifo_trace_tail;
    while (tail != fifo_trace_head) {
        fifo_trace_entry_t entry = fifo_trace[tail & FIFO_TRACE_MASK];
        fast_log("FIFO TRACE tag=%u before=%u after=%u flags=0x%02x data=0x%02x raw=0x%08x\n",
                 entry.tag,
                 entry.pending_before,
                 entry.pending_after,
                 entry.flags,
                 entry.data,
                 entry.raw_value);
        tail++;
    }
    fifo_trace_tail = tail;
}

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

    uint32_t payload_type = dma_fifo_payload_type(raw_value);
    cached_registers_t *cached = defer_get_cached_registers();
    uint8_t pending_before = (uint8_t)fifo_pending_prefetch;
    uint8_t trace_flags = 0;
    uint8_t trace_data = 0;

    switch (payload_type) {
        case FIFO_PREFETCH_ADDRESS: {
            uint32_t address = dma_fifo_prefetch_address(raw_value);
            uint32_t offset = address - DMA_REGISTER_BASE;
            if (offset >= 0x100) {
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                break;
            }

            uint32_t masked_offset = mask_offset(offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            fifo_pending_prefetch++;
            uint8_t data = cached->values[masked_offset];
            trace_data = data;

            #ifndef BENCHMARK_MODE
            uint32_t response = (0xFF00 | (uint32_t)data);
            pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, response);
            #endif

            if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
                fast_log("CACHED_READ: offset=0x%02x, data=0x%02x (cached)\n", masked_offset, data);
            }
            break;
        }

        case FIFO_READ_COMMIT: {
            if (fifo_pending_prefetch == 0) {
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                fast_log("FIFO WARN: Commit without prefetch raw=0x%08x\n", raw_value);
            } else {
                fifo_pending_prefetch--;
            }
            defer_queue_t *queue = defer_get_queue();
            defer_enqueue_fast(queue, raw_value);
            break;
        }

        case FIFO_WRITE_VALUE: {
            uint32_t address = dma_fifo_write_address(raw_value);
            uint32_t offset = address - DMA_REGISTER_BASE;
            if (offset >= 0x100) {
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                break;
            }

            uint32_t masked_offset = mask_offset(offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            uint8_t write_data = dma_fifo_write_data(raw_value);
            if (masked_offset == REG_ADDR_H) {
                write_data &= 0x0F;
            }

            cached->values[masked_offset] = write_data;
            if (masked_offset == REG_STATUS) {
                cached->values[0x30] = write_data;
            }
            trace_data = write_data;
            if (fifo_pending_prefetch > 0) {
                fifo_pending_prefetch--;
            }
            trace_flags |= FIFO_TRACE_FLAG_WRITE;

            if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
                fast_log("CACHED_WRITE: offset=0x%02x, data=0x%02x\n", masked_offset, write_data);
            }

            defer_queue_t *queue = defer_get_queue();
            defer_enqueue_fast(queue, raw_value);
            break;
        }

        default:
            trace_flags |= FIFO_TRACE_FLAG_ERROR;
            fast_log("FIFO WARN: Unknown payload type %u raw=0x%08x\n", payload_type, raw_value);
            break;
    }

    uint8_t pending_after = (uint8_t)fifo_pending_prefetch;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, trace_data);

    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

// Even more optimized version with minimal overhead
void __time_critical_func(registers_irq_handler_cached_asm)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_pending_prefetch;
    uint8_t trace_data = 0;

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
        {
            uint32_t address = dma_fifo_prefetch_address(raw_value);
            masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
            masked_offset &= 0xFF;

            fifo_pending_prefetch++;
            uint8_t data_now = cached->values[masked_offset];
            trace_data = data_now;
            //PIO_REGISTERS->txf[REGISTERS_SM] = (0xFF00 | (uint32_t)data_now); //TODO: return the real data when I'm done debugging
            PIO_REGISTERS->txf[REGISTERS_SM] = (uint32_t)0xFFFFFFFF; 
            fast_log("PREFETCH BD0 func=%d\n", gpio_get_function(BD0_PIN));
            data = data_now;
            break;
        }
        case FIFO_READ_COMMIT:
            if (fifo_pending_prefetch == 0) {
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                fast_log("FIFO WARN: Commit without prefetch raw=0x%08x\n", raw_value);
            } else {
                fifo_pending_prefetch--;
            }
            enque_result = true;
            break;
        case FIFO_WRITE_VALUE:
        {
            enque_result = true;
            uint32_t address = dma_fifo_write_address(raw_value);
            masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
            masked_offset &= 0xFF;

            data = dma_fifo_write_data(raw_value);
            if (masked_offset == REG_ADDR_H) {
                data &= 0x0F;
            }

            cached->values[masked_offset] = data;
            if (masked_offset == REG_STATUS) {
                cached->values[0x30] = data;
            }
            trace_data = data;
            if (fifo_pending_prefetch > 0) {
                fifo_pending_prefetch--;
            }
            trace_flags |= FIFO_TRACE_FLAG_WRITE;
            break;
        }
        default:
            trace_flags |= FIFO_TRACE_FLAG_ERROR;
            fast_log("CACHED ASM: Unknown payload type: 0x%02x raw=0x%08x\n", payload_type, raw_value);
            break;
    }

    uint8_t pending_after = (uint8_t)fifo_pending_prefetch;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, trace_data);

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

    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

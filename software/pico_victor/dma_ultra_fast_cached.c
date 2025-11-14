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
static volatile uint32_t fifo_read_count = 0;

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

    uint32_t payload_type = fifo_payload_type(raw_value);
    cached_registers_t *cached = defer_get_cached_registers();
    uint8_t pending_before = (uint8_t)fifo_read_count;
    uint8_t trace_flags = 0;
    uint8_t data = 0;

    switch (payload_type) {
        case FIFO_REG_READ: {
            uint32_t address = board_fifo_read_address(raw_value);
            uint32_t offset = address - DMA_REGISTER_BASE;
            if (offset >= 0x100) {
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                break;
            }

            uint32_t masked_offset = mask_offset(offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            fifo_read_count++;
            data = cached->values[masked_offset];

            #ifndef BENCHMARK_MODE
            // Push data byte to bus_output_helper (not board_registers!)
            PIO helper_pio = dma_get_bus_helper_pio();
            int helper_sm = dma_get_bus_helper_sm();

            // Debug: Check FIFO state before push
            uint32_t tx_level_before = pio_sm_get_tx_fifo_level(helper_pio, helper_sm);
            pio_sm_put_blocking(helper_pio, helper_sm, (uint32_t)(data & 0xFFu));
            uint32_t tx_level_after = pio_sm_get_tx_fifo_level(helper_pio, helper_sm);

            // Log the push attempt
            fast_log("BUS_HELPER_PUSH: data=0x%02x, TX before=%d, after=%d, pio=%p, sm=%d\n",
                     data, tx_level_before, tx_level_after, helper_pio, helper_sm);
            #endif

            if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
                fast_log("CACHED_READ: offset=0x%02x, data=0x%02x (cached)\n", masked_offset, data);
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

            data = dma_fifo_write_data(raw_value);
            if (masked_offset == REG_ADDR_H) {
                data &= 0x0F;
            }

            cached->values[masked_offset] = data;
            if (masked_offset == REG_STATUS) {
                cached->values[0x30] = data;
            }
            trace_flags |= FIFO_TRACE_FLAG_WRITE;

            if (masked_offset == 0x80 || masked_offset == 0xA0 || masked_offset == 0xC0) {
                fast_log("CACHED_WRITE: offset=0x%02x, data=0x%02x\n", masked_offset, data);
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

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);

    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

// IRQ handler for board_registers PIO (handles WRITE_VALUE only)
void __time_critical_func(board_registers_irq_handler_cached_asm)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_read_count;

    // Get value from board_registers PIO FIFO
    raw_value = PIO_REGISTERS->rxf[REGISTERS_SM];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    // board_registers only pushes WRITE_VALUE payloads
    if (payload_type == FIFO_WRITE_VALUE) {
        //case 8088 is writing to a pico register
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
        trace_flags |= FIFO_TRACE_FLAG_WRITE;

        // Enqueue for deferred processing (fast inline version)
        defer_queue_t *queue = &defer_queue;
        uint32_t head = queue->head;
        uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

        if (next != queue->tail) {
            queue->entries[head].raw_value = raw_value;
            __asm volatile("dmb" ::: "memory");
            queue->head = next;
        }
    } else {
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fast_log("BOARD_REG: Unexpected payload type: 0x%02x raw=0x%08x\n", payload_type, raw_value);
        data = 0;
    }

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);
}

// IRQ handler for bus_output_helper PIO (handles PREFETCH and DMA_READ)
void __time_critical_func(bus_output_helper_irq_handler_cached_asm)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data = 0;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_read_count;

    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from bus_output_helper PIO FIFO
    raw_value = PIO_BUS_HELPER->rxf[BUS_HELPER_SM];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    bool enque_result = false;

    // Handle different payload types
    switch(payload_type) {
        case FIFO_REG_READ:
        {
            //case 8088 is reading a pico register
            // bus_output_helper pushes REG_READ when 8088 reads a register, some side effects can handled later
            uint32_t address = board_fifo_read_address(raw_value);
            masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
            masked_offset &= 0xFF;

            enque_result = true;

            fifo_read_count++;
            data = cached->values[masked_offset];

            // Debug: Check if PIO_BUS_HELPER and BUS_HELPER_SM are correct
            fast_log("BUS_HELPER_REG_READ: offset=0x%02x data=0x%02x pio=%p sm=%d\n",
                     masked_offset, data, PIO_BUS_HELPER, BUS_HELPER_SM);

            // Check TX FIFO level before pushing
            uint32_t tx_before = pio_sm_get_tx_fifo_level(PIO_BUS_HELPER, BUS_HELPER_SM);

            // Push data byte to bus_output_helper for output on BD0-BD7
            pio_sm_put_blocking(PIO_BUS_HELPER, BUS_HELPER_SM, (uint32_t)(data & 0xFFu));

            // Check TX FIFO level after pushing
            uint32_t tx_after = pio_sm_get_tx_fifo_level(PIO_BUS_HELPER, BUS_HELPER_SM);
            fast_log("BUS_HELPER_PUSH_RESULT: tx_before=%d tx_after=%d\n", tx_before, tx_after);
            break;
        }
        case FIFO_DMA_READ:
        {
            //case pico is doing a DMA read from the bus
            // bus_output_helper pushes DMA_READ data after capturing from bus
            enque_result = true;
            // Extract the data from the payload (bits 22-29 contain the data byte)
            data = (raw_value >> 22) & 0xFF;
            // Extract address for logging (bits 2-21 contain the 20-bit address)
            uint32_t dma_address = (raw_value >> 2) & 0xFFFFF;
            fast_log("BUS_HELPER_DMA_READ: addr=0x%05x data=0x%02x raw=0x%08x\n",
                     dma_address, data, raw_value);
            break;
        }
        default:
            trace_flags |= FIFO_TRACE_FLAG_ERROR;
            fast_log("BUS_HELPER: Unknown payload type: 0x%02x raw=0x%08x\n", payload_type, raw_value);
            break;
    }

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);

    if (enque_result) {
        // Enqueue for deferred processing (fast inline version)
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

// Legacy handler - kept for compatibility with older code paths
void __time_critical_func(registers_irq_handler_cached_asm)() {
    // This legacy handler is deprecated - use the new separate handlers instead
    board_registers_irq_handler_cached_asm();
}

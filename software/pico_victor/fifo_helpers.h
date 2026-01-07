#ifndef FIFO_HELPERS_H
#define FIFO_HELPERS_H

#include "logging.h"
#include <stdint.h>

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

#ifdef FIFO_HELPERS_IMPLEMENTATION
fifo_trace_entry_t fifo_trace[FIFO_TRACE_SIZE];
volatile uint32_t fifo_trace_head = 0;
volatile uint32_t fifo_trace_tail = 0;
volatile uint32_t fifo_pending_prefetch = 0;
#else
extern fifo_trace_entry_t fifo_trace[FIFO_TRACE_SIZE];
extern volatile uint32_t fifo_trace_head;
extern volatile uint32_t fifo_trace_tail;
extern volatile uint32_t fifo_pending_prefetch;
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

static inline void dma_fifo_trace_flush(void) {
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

#endif // FIFO_HELPERS_H

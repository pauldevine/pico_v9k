/*
 * dma_defer.h - Deferred processing system for DMA register operations
 *
 * This system separates the fast path (return cached values) from the slow path
 * (process state changes), allowing us to meet the 200ns timing requirement.
 */

#ifndef DMA_DEFER_H
#define DMA_DEFER_H

#include <stdint.h>
#include <stdbool.h>
#include "dma.h"

// Queue size must be power of 2 for fast masking
#define DEFER_QUEUE_SIZE 256
#define DEFER_QUEUE_MASK (DEFER_QUEUE_SIZE - 1)

// Deferred operation entry
typedef struct {
    uint32_t raw_value;     // Original PIO payload
    uint32_t timestamp;     // Optional timestamp for debugging
} defer_entry_t;

// Deferred processing queue
typedef struct {
    defer_entry_t entries[DEFER_QUEUE_SIZE];
    volatile uint32_t head;  // Producer index (IRQ handler)
    volatile uint32_t tail;  // Consumer index (Core 1 worker)

    // Statistics for debugging
    volatile uint32_t drops;     // Count of dropped entries due to full queue
    volatile uint32_t processed; // Count of processed entries
} defer_queue_t;

// Cached register values for fast reads
// Aligned to cache line for optimal performance
typedef struct {
    uint8_t values[256];
    uint32_t last_update;  // Timestamp of last update
} __attribute__((aligned(64))) cached_registers_t;

// Initialize the deferred processing system
void dma_defer_init(void);

// Fast path: enqueue operation for deferred processing (called from IRQ)
static inline bool defer_enqueue_fast(defer_queue_t *queue, uint32_t raw_value) {
    uint32_t head = queue->head;
    uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

    // Check if queue is full
    if (next == queue->tail) {
        queue->drops++;
        return false;
    }

    // Add entry
    queue->entries[head].raw_value = raw_value;

    // Memory barrier to ensure write completes before advancing head
    __asm volatile("dmb" ::: "memory");

    queue->head = next;
    return true;
}

// Slow path: dequeue and process operation (called from Core 1)
bool defer_dequeue(defer_queue_t *queue, defer_entry_t *entry);

// Process a deferred operation
void defer_process_entry(dma_registers_t *dma, const defer_entry_t *entry);

// Core 1 worker main loop
void defer_worker_main(void);

// Get the shared queue instance
defer_queue_t* defer_get_queue(void);

// Get the cached registers instance
cached_registers_t* defer_get_cached_registers(void);

// Update cached register value
static inline void defer_update_cached(cached_registers_t *cached, uint32_t offset, uint8_t value) {
    cached->values[offset & 0xFF] = value;
}

// Get cached register value
static inline uint8_t defer_get_cached(cached_registers_t *cached, uint32_t offset) {
    return cached->values[offset & 0xFF];
}

#endif // DMA_DEFER_H
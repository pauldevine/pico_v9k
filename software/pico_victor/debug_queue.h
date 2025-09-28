/*
 * debug_queue.h - Lock-free debug queue for PIO register access logging
 *
 * This provides a minimal-impact debug logging system where the IRQ handler
 * simply pushes raw PIO values to a circular buffer, and core0 handles all
 * the decoding and printing.
 */

#ifndef DEBUG_QUEUE_H
#define DEBUG_QUEUE_H

// Set to 0 to completely disable debug queue at compile time for zero overhead
#ifndef ENABLE_DEBUG_QUEUE
#define ENABLE_DEBUG_QUEUE 1
#endif

#include <stdint.h>
#include <stdbool.h>
#include "pico.h"

// Queue size - must be power of 2 for efficient modulo operation
#define DEBUG_QUEUE_SIZE 256

// Debug queue structure - aligned for cache efficiency
typedef struct {
    volatile uint32_t buffer[DEBUG_QUEUE_SIZE];
    volatile uint32_t write_idx;  // Written by core1 (IRQ handler)
    uint32_t padding[14];         // Padding to separate write/read indices (cache line)
    volatile uint32_t read_idx;   // Written by core0 (consumer)
} debug_queue_t;

// Global debug queue instance
extern debug_queue_t debug_queue;

// Global enable flag - keep in separate cache line
extern volatile uint32_t debug_queue_enabled_flag;

// Initialize the debug queue
void debug_queue_init(void);

// Ultra-fast push implementation - optimized for minimal overhead
// The enable check is designed to be as cheap as possible
static inline __attribute__((always_inline)) void debug_queue_push_fast(uint32_t value) {
#if ENABLE_DEBUG_QUEUE
    // Single load of enable flag - branch will be predicted correctly after warmup
    // Using __builtin_expect to hint that disabled is the common case
    if (__builtin_expect(debug_queue_enabled_flag == 0, 1)) {
        return;  // Fast path when disabled - likely branch
    }

    // Slow path - only taken when debugging is enabled
    uint32_t next_write = (debug_queue.write_idx + 1) & (DEBUG_QUEUE_SIZE - 1);

    // Check if queue is full
    if (next_write != debug_queue.read_idx) {
        // Store value and update write index
        debug_queue.buffer[debug_queue.write_idx] = value;
        __asm volatile ("dmb" : : : "memory");
        debug_queue.write_idx = next_write;
    }
#else
    // Completely compiled out for zero overhead
    (void)value;
#endif
}

// Pop a value from the queue (called from core0)
// Returns false if queue is empty
static inline bool debug_queue_pop(uint32_t *value) {
    // Check if queue is empty
    if (debug_queue.read_idx == debug_queue.write_idx) {
        return false;
    }

    // Read value and update read index
    *value = debug_queue.buffer[debug_queue.read_idx];
    __asm volatile ("dmb" : : : "memory");  // Ensure read completes before updating index
    debug_queue.read_idx = (debug_queue.read_idx + 1) & (DEBUG_QUEUE_SIZE - 1);

    return true;
}

// Process and print debug queue entries (called from core0 main loop)
void debug_queue_process(void);

// Enable/disable debug logging
void debug_queue_enable(bool enable);
bool debug_queue_is_enabled(void);

#endif // DEBUG_QUEUE_H
/*
 * debug_queue.c - Debug queue implementation for PIO register access logging
 */

#include "debug_queue.h"
#include "dma.h"
#include <stdio.h>
#include <string.h>

// Global debug queue instance - aligned to cache line
debug_queue_t debug_queue __attribute__((aligned(64), section(".time_critical.debug_queue")));

// Global enable flag - in separate cache line for fast checking
// Using uint32_t instead of bool for better performance
volatile uint32_t debug_queue_enabled_flag __attribute__((aligned(64), section(".time_critical.debug_flag"))) = 0;

// Initialize the debug queue
void debug_queue_init(void) {
    memset((void*)&debug_queue, 0, sizeof(debug_queue));
    debug_queue.write_idx = 0;
    debug_queue.read_idx = 0;
    debug_queue_enabled_flag = 0;
}

// Enable/disable debug logging
void debug_queue_enable(bool enable) {
    debug_queue_enabled_flag = enable ? 1 : 0;
    if (enable) {
        printf("Debug queue enabled (size=%d)\n", DEBUG_QUEUE_SIZE);
    } else {
        printf("Debug queue disabled\n");
    }
}

bool debug_queue_is_enabled(void) {
    return debug_queue_enabled_flag != 0;
}

// Decode register offset to name
static const char* get_register_name(uint32_t offset) {
    // Apply MAME-style masking based on offset range
    if (offset < 0x30) {
        offset &= ~0xf;
    } else {
        offset &= ~0x1f;
    }

    switch (offset) {
        case 0x00: return "CONTROL";
        case 0x10: return "DATA";
        case 0x20: return "STATUS";
        case 0x80: return "DMA_ADDR_LOW";
        case 0xA0: return "DMA_ADDR_MID";
        case 0xC0: return "DMA_ADDR_HIGH";
        default: return "UNKNOWN";
    }
}

// Process and print debug queue entries
void debug_queue_process(void) {
    if (!debug_queue_enabled_flag) {
        return;
    }

    uint32_t value;
    int count = 0;
    const int max_per_call = 10;  // Process max 10 entries per call to avoid blocking

    while (count < max_per_call && debug_queue_pop(&value)) {
        // Decode the raw PIO value
        uint32_t address = value & 0xFFFFF;
        uint8_t data = (value >> 20) & 0xFF;
        bool is_read = (value & 0x10000000) != 0;

        // Calculate offset from DMA register base
        uint32_t offset = address - DMA_REGISTER_BASE;

        // Only print writes (reads have race conditions as you noted)
        if (!is_read && offset < 0x100) {
            const char* reg_name = get_register_name(offset);

            // Format based on register type
            if (offset >= 0x80) {
                // Address register - show hex value
                printf("DMA: %s <- 0x%02X\n", reg_name, data);
            } else if (offset == 0x00) {
                // Control register - decode bits
                printf("DMA: CONTROL <- 0x%02X [", data);
                if (data & DMA_RESET_BIT) printf("RESET ");
                if (data & DMA_ON_LATCH_BIT) printf("ON_LATCH ");
                if (data & DMA_ON_VALUE_BIT) printf("ON_VALUE ");
                if (data & DMA_WR_MODE_BIT) printf("WR_MODE ");
                if (data & DMA_SELECT_BIT) printf("SELECT ");
                printf("]\n");
            } else {
                // Other registers - show hex
                printf("DMA: %s <- 0x%02X\n", reg_name, data);
            }
        }

        count++;
    }

    // If we processed max entries, there might be more
    if (count == max_per_call) {
        // Queue might have more entries - they'll be processed on next call
        uint32_t remaining = (debug_queue.write_idx - debug_queue.read_idx) & (DEBUG_QUEUE_SIZE - 1);
        if (remaining > 0) {
            printf("DMA: [%d more entries queued]\n", remaining);
        }
    }
}
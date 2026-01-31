/*
 * psram_trace.h - Two-tier trace buffer using PSRAM for low-overhead logging
 *
 * This provides a massive trace buffer (2MB) in PSRAM that is drained from
 * the existing debug_queue (L1 SRAM) during HLDA windows when timing is relaxed.
 *
 * Usage:
 *   1. Call psram_trace_init() during startup
 *   2. Continue using debug_queue_push_fast() in ISRs (unchanged)
 *   3. Call psram_trace_drain() during HLDA windows or idle time
 *   4. Use psram_trace_dump_uart() or psram_trace_dump_sd() for retrieval
 */

#ifndef PSRAM_TRACE_H
#define PSRAM_TRACE_H

#include <stdint.h>
#include <stdbool.h>

// PSRAM memory window on RP2350 (QMI M1)
#define PSRAM_BASE_ADDR     0x11000000

// Trace buffer configuration
#define PSRAM_TRACE_SIZE    (2 * 1024 * 1024)  // 2MB buffer
#define PSRAM_TRACE_ENTRY_SIZE 8               // 8 bytes per entry

// Trace entry structure: timestamp + payload = 8 bytes
typedef struct {
    uint32_t timestamp;   // SysTick counter value (counts down from reload)
    uint32_t payload;     // Raw FIFO value or encoded event from debug_queue
} __attribute__((packed)) psram_trace_entry_t;

#define PSRAM_TRACE_ENTRIES (PSRAM_TRACE_SIZE / sizeof(psram_trace_entry_t))
#define PSRAM_TRACE_MASK    (PSRAM_TRACE_ENTRIES - 1)  // For efficient modulo

// Control structure (lives in SRAM for fast access)
typedef struct {
    volatile uint32_t head;         // Next write position (producer)
    volatile uint32_t tail;         // Read position (consumer, for dump)
    volatile uint32_t count;        // Total entries written (may wrap)
    volatile uint32_t overflow;     // Count of dropped entries
    volatile bool paused;           // Pause flag for capture control
    bool initialized;               // PSRAM init complete
} psram_trace_ctrl_t;

// Global control structure
extern psram_trace_ctrl_t psram_trace_ctrl;

/**
 * Initialize PSRAM and trace buffer
 * Must be called after debug_queue_init()
 * Returns true if PSRAM detected and initialized
 */
bool psram_trace_init(void);

/**
 * Drain debug_queue (L1) to PSRAM buffer (L2)
 * Safe to call from HLDA window or main loop idle time
 * Non-blocking: drains all available entries
 */
void psram_trace_drain(void);

/**
 * Dump trace entries to UART
 * @param count Number of most recent entries to dump (0 = all)
 */
void psram_trace_dump_uart(uint32_t count);

/**
 * Dump trace buffer to SD card file
 * @param filename File path on SD card (e.g., "trace_001.bin")
 * @return true if successful
 */
bool psram_trace_dump_sd(const char *filename);

/**
 * Get number of entries currently in buffer
 */
uint32_t psram_trace_count(void);

/**
 * Clear trace buffer
 */
void psram_trace_clear(void);

/**
 * Print status to UART (count, overflow, etc.)
 */
void psram_trace_status(void);

/**
 * Toggle pause state
 */
void psram_trace_toggle_pause(void);

/**
 * Check if PSRAM was successfully initialized
 */
static inline bool psram_trace_is_initialized(void) {
    return psram_trace_ctrl.initialized;
}

/**
 * Check if tracing is paused
 */
static inline bool psram_trace_is_paused(void) {
    return psram_trace_ctrl.paused;
}

#endif // PSRAM_TRACE_H

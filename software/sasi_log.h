/*
 * sasi_log.h - SD card SASI interaction logger
 *
 * Captures every SASI command with timing to a text file on the SD card.
 * Uses a RAM ring buffer for fast capture; flushes to SD during bus-free
 * idle periods on Core 1.
 */
#ifndef SASI_LOG_H
#define SASI_LOG_H

#include <stdint.h>
#include <stdbool.h>
#include "pico_victor/dma.h"

// Compile-time default: set to 0 to disable logging by default
#ifndef SASI_LOG_ENABLED_DEFAULT
#define SASI_LOG_ENABLED_DEFAULT 1
#endif

// Ring buffer size (must be power of 2).
// 512 entries = 16KB -- holds a full stress-test phase so we can defer
// all SD card I/O until the bus has been idle for 2+ seconds.
#define SASI_LOG_BUFFER_SIZE 512
#define SASI_LOG_BUFFER_MASK (SASI_LOG_BUFFER_SIZE - 1)

// One entry per SASI command (32 bytes)
typedef struct {
    uint64_t timestamp_us;    // time_us_64() at command start
    uint32_t duration_us;     // elapsed microseconds for command
    uint32_t lba;             // starting LBA from CDB
    uint32_t dma_address;     // 20-bit DMA start address
    uint16_t block_count;     // sector count from CDB
    uint8_t  opcode;          // SASI opcode byte
    uint8_t  status;          // completion status (0x00=GOOD, 0xFF=aborted)
    uint8_t  target_id;       // SASI target 0-7
    uint8_t  flags;           // bit 0: reset_aborted, bit 1: storage_error
    uint8_t  reserved[2];     // pad to 32 bytes
} sasi_log_entry_t;

_Static_assert(sizeof(sasi_log_entry_t) == 32, "Log entry must be 32 bytes");

// Single-producer single-consumer ring buffer
// Both producer and consumer run on Core 1 (sequentially), so no cross-core
// synchronization is needed.
typedef struct {
    sasi_log_entry_t entries[SASI_LOG_BUFFER_SIZE];
    uint32_t write_idx;
    uint32_t read_idx;
    uint32_t total_logged;
    uint32_t total_dropped;
    uint32_t total_flushed;
} sasi_log_buffer_t;

// Push an entry into the ring buffer (called from command handlers on Core 1)
static inline bool sasi_log_push(sasi_log_buffer_t *buf, const sasi_log_entry_t *entry) {
    uint32_t next = (buf->write_idx + 1) & SASI_LOG_BUFFER_MASK;
    if (next == buf->read_idx) {
        buf->total_dropped++;
        return false;  // Buffer full
    }
    buf->entries[buf->write_idx] = *entry;
    buf->write_idx = next;
    buf->total_logged++;
    return true;
}

// Pop an entry from the ring buffer (called from flush on Core 1)
static inline bool sasi_log_pop(sasi_log_buffer_t *buf, sasi_log_entry_t *entry) {
    if (buf->read_idx == buf->write_idx) {
        return false;  // Empty
    }
    *entry = buf->entries[buf->read_idx];
    buf->read_idx = (buf->read_idx + 1) & SASI_LOG_BUFFER_MASK;
    return true;
}

// Initialize the SASI logger (call from main() before multicore_launch_core1)
void sasi_log_init(void);

// Record command start (call when command_complete() fires, before route_to_sasi_target)
void sasi_log_cmd_start(const dma_registers_t *dma, const uint8_t *cmd, int len);

// Record command completion (call from signal_command_complete and abort paths)
void sasi_log_cmd_complete(const dma_registers_t *dma, uint8_t status);

// Flush pending entries to SD card (call from defer_worker_main when bus is idle)
void sasi_log_flush_if_ready(const dma_registers_t *dma);

// Dump all buffered entries to UART (call from dma_device_reset on Core 1).
// No SD card I/O -- prints directly so we don't lose data on crash/reboot.
void sasi_log_dump_uart(void);

// Global enable flag (readable for conditional checks)
extern bool sasi_log_enabled;

#endif // SASI_LOG_H

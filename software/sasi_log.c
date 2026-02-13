/*
 * sasi_log.c - SD card SASI interaction logger
 *
 * Logs every SASI command to SASI_LOG.TXT on the SD card.
 * Binary entries are pushed to a ring buffer during command processing,
 * then formatted and flushed to SD during bus-free idle periods on Core 1.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "ff.h"
#include "sasi_log.h"

// Global state
bool sasi_log_enabled = false;
static sasi_log_buffer_t sasi_log_buffer;
static FIL sasi_log_fil;
static bool sasi_log_file_open = false;
static uint64_t bus_free_since_us = 0;
static bool was_bus_free = false;
static uint32_t entries_since_sync = 0;

// In-progress command tracking (file-scope statics, only accessed on Core 1)
static uint64_t cmd_start_us = 0;
static uint8_t  cmd_opcode = 0;
static uint32_t cmd_lba = 0;
static uint16_t cmd_blocks = 0;
static uint32_t cmd_dma_addr = 0;
static uint8_t  cmd_target = 0;

void sasi_log_init(void) {
    memset(&sasi_log_buffer, 0, sizeof(sasi_log_buffer));

    // Check marker files for runtime configuration
    FILINFO fno;
    if (f_stat("NOLOG", &fno) == FR_OK) {
        printf("SASI Log: NOLOG marker found, logging disabled\n");
        sasi_log_enabled = false;
        return;
    }
    if (f_stat("SASLOG", &fno) == FR_OK) {
        printf("SASI Log: SASLOG marker found, logging enabled\n");
        sasi_log_enabled = true;
    } else {
        sasi_log_enabled = SASI_LOG_ENABLED_DEFAULT;
        printf("SASI Log: using compile-time default (%s)\n",
               sasi_log_enabled ? "enabled" : "disabled");
    }

    if (!sasi_log_enabled) {
        return;
    }

    // Open log file in append mode
    FRESULT fr = f_open(&sasi_log_fil, "SASI_LOG.TXT",
                        FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        // Try creating it
        fr = f_open(&sasi_log_fil, "SASI_LOG.TXT",
                    FA_WRITE | FA_CREATE_ALWAYS);
    }
    if (fr != FR_OK) {
        printf("SASI Log: failed to open SASI_LOG.TXT (err %d), disabling\n", fr);
        sasi_log_enabled = false;
        return;
    }
    sasi_log_file_open = true;

    // Write boot header
    char header[80];
    uint64_t boot_ms = time_us_64() / 1000;
    int len = snprintf(header, sizeof(header),
                       "=== SASI Log boot T=%llu ===\n",
                       (unsigned long long)boot_ms);
    UINT bw;
    f_write(&sasi_log_fil, header, len, &bw);
    f_sync(&sasi_log_fil);

    printf("SASI Log: initialized, logging to SASI_LOG.TXT\n");
}

void sasi_log_cmd_start(const dma_registers_t *dma, const uint8_t *cmd, int len) {
    if (!sasi_log_enabled) return;

    cmd_start_us = time_us_64();
    cmd_opcode = cmd[0];
    cmd_lba = ((cmd[1] & 0x1F) << 16) | (cmd[2] << 8) | cmd[3];
    // Block count is only meaningful for Read(6)/Write(6)
    if (cmd[0] == 0x08 || cmd[0] == 0x0A) {
        cmd_blocks = cmd[4] ? cmd[4] : 256;
    } else {
        cmd_blocks = 0;
    }
    cmd_dma_addr = dma ? (dma->dma_address.full & 0xFFFFF) : 0;
    cmd_target = dma ? (dma->selected_target & 0x07) : 0;
}

void sasi_log_cmd_complete(const dma_registers_t *dma, uint8_t status) {
    if (!sasi_log_enabled) return;

    uint64_t now = time_us_64();
    sasi_log_entry_t entry = {
        .timestamp_us = cmd_start_us,
        .duration_us = (uint32_t)(now - cmd_start_us),
        .lba = cmd_lba,
        .dma_address = cmd_dma_addr,
        .block_count = cmd_blocks,
        .opcode = cmd_opcode,
        .status = status,
        .target_id = cmd_target,
        .flags = (status == 0xFF) ? 0x01 : 0x00,
        .reserved = {0, 0}
    };

    sasi_log_push(&sasi_log_buffer, &entry);
}

void sasi_log_dump_uart(void) {
    if (!sasi_log_enabled) return;

    uint32_t count = 0;
    uint32_t rd = sasi_log_buffer.read_idx;
    uint32_t wr = sasi_log_buffer.write_idx;

    // Count entries without consuming them
    uint32_t pending = (wr - rd) & SASI_LOG_BUFFER_MASK;

    printf("--- SASI Log dump: %lu pending, %lu logged, %lu dropped, %lu flushed ---\n",
           (unsigned long)pending,
           (unsigned long)sasi_log_buffer.total_logged,
           (unsigned long)sasi_log_buffer.total_dropped,
           (unsigned long)sasi_log_buffer.total_flushed);

    sasi_log_entry_t entry;
    while (sasi_log_pop(&sasi_log_buffer, &entry)) {
        printf("T=%llu OP=%02X LBA=%lu N=%u DMA=%05lX S=%02X D=%lu TGT=%u%s\n",
            (unsigned long long)(entry.timestamp_us / 1000),
            entry.opcode,
            (unsigned long)entry.lba,
            (unsigned)entry.block_count,
            (unsigned long)entry.dma_address,
            entry.status,
            (unsigned long)entry.duration_us,
            (unsigned)entry.target_id,
            (entry.flags & 0x01) ? " ABORT" : "");
        count++;
    }

    printf("--- SASI Log dump complete: %lu entries ---\n", (unsigned long)count);
}

// Minimum bus-free idle time before flushing entries to SD card.
// ANY f_write() blocks Core 1 for 1-100ms (SPI to SD card), which is
// far longer than the BIOS SELECT→BSY timeout (~100 us).  DOS has
// legitimate 100-450ms gaps between SASI commands (FAT computation,
// directory updates), so the delay must exceed that to avoid flushing
// during active I/O sequences.
#define SASI_LOG_FLUSH_DELAY_US  2000000  // 2 seconds
#define SASI_LOG_SYNC_DELAY_US   5000000  // 5 seconds

void sasi_log_flush_if_ready(const dma_registers_t *dma) {
    if (!sasi_log_enabled || !sasi_log_file_open) return;

    // Only flush when bus is free (no active SASI command, no FatFS disk I/O)
    if (dma->bus_ctrl != 0) {
        was_bus_free = false;
        return;
    }

    // Track how long we've been bus-free
    uint64_t now = time_us_64();
    if (!was_bus_free) {
        bus_free_since_us = now;
        was_bus_free = true;
    }

    uint64_t idle_us = now - bus_free_since_us;

    // Don't do any SD card I/O until bus has been idle for 2 seconds.
    // This keeps Core 1 responsive to new register writes during rapid
    // command sequences (boot, stress tests).  DOS has legitimate
    // 100-450ms gaps between commands, so 2s is the safe threshold.
    if (idle_us < SASI_LOG_FLUSH_DELAY_US) {
        return;
    }

    // Nothing to flush?
    if (sasi_log_buffer.read_idx == sasi_log_buffer.write_idx) {
        if (entries_since_sync > 0 && idle_us > SASI_LOG_SYNC_DELAY_US) {
            f_sync(&sasi_log_fil);
            entries_since_sync = 0;
        }
        return;
    }

    // Drain the entire ring buffer.  We pack as many formatted entries as
    // fit into a 2KB batch, issue one f_write, then refill.  SPI
    // setup/teardown per f_write dwarfs the payload transfer time, so
    // fewer larger writes is much cheaper.  Between writes we re-check
    // bus_ctrl so we can abort if a command arrives mid-flush.
    sasi_log_entry_t entry;
    char batch[2048];

    while (sasi_log_buffer.read_idx != sasi_log_buffer.write_idx) {
        int batch_len = 0;
        int flushed = 0;

        // Fill the batch buffer (~70 bytes per entry, fits ~29 entries)
        while (sasi_log_pop(&sasi_log_buffer, &entry)) {
            int remaining = (int)sizeof(batch) - batch_len;
            if (remaining < 80) break;  // not enough room for another entry
            int len = snprintf(batch + batch_len, remaining,
                "T=%llu OP=%02X LBA=%lu N=%u DMA=%05lX S=%02X D=%lu TGT=%u%s\n",
                (unsigned long long)(entry.timestamp_us / 1000),
                entry.opcode,
                (unsigned long)entry.lba,
                (unsigned)entry.block_count,
                (unsigned long)entry.dma_address,
                entry.status,
                (unsigned long)entry.duration_us,
                (unsigned)entry.target_id,
                (entry.flags & 0x01) ? " ABORT" : "");
            batch_len += len;
            flushed++;
        }

        if (batch_len > 0) {
            UINT bw;
            f_write(&sasi_log_fil, batch, batch_len, &bw);
            sasi_log_buffer.total_flushed += flushed;
            entries_since_sync += flushed;
        }

        // Between writes, bail out if the bus became active
        if (dma->bus_ctrl != 0) {
            was_bus_free = false;
            return;
        }
    }

    // Sync only after a longer idle period to avoid SD card GC stalls
    if (entries_since_sync > 0 && idle_us > SASI_LOG_SYNC_DELAY_US) {
        f_sync(&sasi_log_fil);
        entries_since_sync = 0;
    }
}

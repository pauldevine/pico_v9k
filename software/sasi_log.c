/* sasi_log.c
 * SASI command logging to SD card
 *
 * Writes a human-readable log of SASI commands to SASI_LOG.TXT on the
 * SD card.  Enabled at runtime by placing a zero-byte marker file named
 * "SASLOG" on the SD card root.
 *
 * IMPORTANT: The batch buffer used during flush is declared at file scope
 * (static), NOT on the stack.  Core 1's stack is only 2048 bytes, and a
 * stack-allocated buffer of that size caused immediate stack overflow and
 * system hangs.
 */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/sync.h"

/* FatFS */
#include "ff.h"
#include "f_util.h"

#include "pico_victor/dma.h"
#include "sasi_log.h"

/* ------------------------------------------------------------------ */
/* State                                                              */
/* ------------------------------------------------------------------ */

static bool sasi_log_enabled   = SASI_LOG_ENABLED_DEFAULT;
static bool sasi_log_file_open = false;
static FIL  sasi_log_fil;

static sasi_log_buffer_t sasi_log_buffer;

/* Idle-detection state for flush scheduling */
static bool     was_bus_free    = false;
static uint64_t bus_free_since  = 0;

/* Minimum idle time (microseconds) before we flush to SD card.
 * Writing to SD can take several milliseconds, so only flush when
 * the bus has been idle long enough that we won't miss a command. */
#define SASI_LOG_IDLE_FLUSH_US  50000   /* 50 ms */

/* ------------------------------------------------------------------ */
/* File-scope batch buffer -- MUST NOT be on Core 1 stack!            */
/* ------------------------------------------------------------------ */
static char sasi_log_batch[2048];

/* ------------------------------------------------------------------ */
/* Command name lookup                                                */
/* ------------------------------------------------------------------ */
static const char *sasi_opcode_name(uint8_t op) {
    switch (op) {
        case 0x00: return "TUR";
        case 0x01: return "REZERO";
        case 0x03: return "SENSE";
        case 0x08: return "READ";
        case 0x0A: return "WRITE";
        case 0x0C: return "INIT_DRV";
        case 0x15: return "MODE_SEL";
        case 0xE0: return "DIAG_RAM";
        case 0xE3: return "DIAG_DRV";
        case 0xE4: return "DIAG_INT";
        default:   return "???";
    }
}

/* ------------------------------------------------------------------ */
/* Initialisation                                                     */
/* ------------------------------------------------------------------ */

void sasi_log_init(void) {
    memset(&sasi_log_buffer, 0, sizeof(sasi_log_buffer));
    sasi_log_file_open = false;
    sasi_log_enabled   = SASI_LOG_ENABLED_DEFAULT;

    /* Check for SASLOG marker file to enable logging */
    FILINFO fno;
    FRESULT fr = f_stat("SASLOG", &fno);
    if (fr == FR_OK) {
        sasi_log_enabled = true;
        printf("SASI Log: enabled by SASLOG marker file\n");
    } else if (!sasi_log_enabled) {
        printf("SASI Log: disabled (no SASLOG file on SD card)\n");
        return;
    }

    /* Open (or create) the log file in append mode */
    fr = f_open(&sasi_log_fil, "SASI_LOG.TXT", FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) {
        printf("SASI Log: failed to open SASI_LOG.TXT (%d)\n", fr);
        sasi_log_enabled = false;
        return;
    }
    sasi_log_file_open = true;

    /* Write a session header */
    const char *hdr = "\n=== SASI LOG SESSION START ===\n"
                      "Event     Op    Name      Tgt  LBA       Blks  DMA_Addr  BusCtrl  Dur_ms  Status\n"
                      "--------  ----  --------  ---  --------  ----  --------  -------  ------  ------\n";
    UINT bw;
    f_write(&sasi_log_fil, hdr, strlen(hdr), &bw);
    f_sync(&sasi_log_fil);

    printf("SASI Log: ready, logging to SASI_LOG.TXT\n");
}

/* ------------------------------------------------------------------ */
/* Ring buffer push (called from Core 1 command paths)                */
/* ------------------------------------------------------------------ */

static void sasi_log_push(const sasi_log_entry_t *entry) {
    if (!sasi_log_enabled) return;

    uint32_t idx = sasi_log_buffer.write_idx & (SASI_LOG_BUFFER_SIZE - 1);
    sasi_log_buffer.entries[idx] = *entry;
    __dmb();
    sasi_log_buffer.write_idx++;
}

void sasi_log_cmd_start(uint8_t opcode, uint8_t target, uint32_t lba,
                        uint16_t blocks, uint32_t dma_addr, uint8_t bus_ctrl) {
    sasi_log_entry_t e = {0};
    e.timestamp_us = (uint32_t)(time_us_64() & 0xFFFFFFFF);
    e.opcode       = opcode;
    e.target       = target;
    e.event        = 0;  /* start */
    e.lba          = lba;
    e.block_count  = blocks;
    e.dma_address  = dma_addr;
    e.bus_ctrl     = bus_ctrl;
    sasi_log_push(&e);
}

void sasi_log_cmd_complete(uint8_t opcode, uint8_t target, uint8_t status,
                           uint32_t lba, uint16_t blocks, uint32_t dma_addr,
                           uint8_t bus_ctrl) {
    sasi_log_entry_t e = {0};
    e.timestamp_us = (uint32_t)(time_us_64() & 0xFFFFFFFF);
    e.opcode       = opcode;
    e.target       = target;
    e.status       = status;
    e.event        = 1;  /* complete */
    e.lba          = lba;
    e.block_count  = blocks;
    e.dma_address  = dma_addr;
    e.bus_ctrl     = bus_ctrl;
    sasi_log_push(&e);
}

void sasi_log_dma_error(uint8_t error_code, uint32_t dma_addr, uint8_t bus_ctrl) {
    sasi_log_entry_t e = {0};
    e.timestamp_us = (uint32_t)(time_us_64() & 0xFFFFFFFF);
    e.opcode       = error_code;
    e.event        = 2;  /* DMA error */
    e.dma_address  = dma_addr;
    e.bus_ctrl     = bus_ctrl;
    sasi_log_push(&e);
}

/* ------------------------------------------------------------------ */
/* Flush to SD card (called from Core 1 defer worker loop)            */
/* ------------------------------------------------------------------ */

void sasi_log_flush_if_ready(const dma_registers_t *dma) {
    if (!sasi_log_enabled || !sasi_log_file_open) return;

    /* Only flush when bus is idle (bus_ctrl == 0 means bus free) */
    if (dma->bus_ctrl != 0) {
        was_bus_free = false;
        return;
    }

    /* Track how long the bus has been idle */
    uint64_t now = time_us_64();
    if (!was_bus_free) {
        was_bus_free   = true;
        bus_free_since = now;
        return;
    }

    /* Wait for sufficient idle time before flushing */
    if ((now - bus_free_since) < SASI_LOG_IDLE_FLUSH_US) {
        return;
    }

    /* Nothing to flush? */
    if (sasi_log_buffer.read_idx == sasi_log_buffer.write_idx) {
        return;
    }

    /* Format entries into the STATIC batch buffer and write to SD */
    int batch_pos = 0;
    while (sasi_log_buffer.read_idx != sasi_log_buffer.write_idx) {
        uint32_t idx = sasi_log_buffer.read_idx & (SASI_LOG_BUFFER_SIZE - 1);
        sasi_log_entry_t *e = &sasi_log_buffer.entries[idx];

        const char *event_str;
        switch (e->event) {
            case 0:  event_str = "CMD_START"; break;
            case 1:  event_str = "CMD_DONE "; break;
            case 2:  event_str = "DMA_ERROR"; break;
            default: event_str = "UNKNOWN  "; break;
        }
        const char *name      = sasi_opcode_name(e->opcode);

        int n = snprintf(sasi_log_batch + batch_pos,
                         sizeof(sasi_log_batch) - batch_pos,
                         "%s  0x%02X  %-8s  %3d  %08lX  %4u  %08lX  0x%02X     %6u  0x%02X\n",
                         event_str,
                         e->opcode,
                         name,
                         e->target,
                         (unsigned long)e->lba,
                         e->block_count,
                         (unsigned long)e->dma_address,
                         e->bus_ctrl,
                         e->duration_ms,
                         e->status);

        if (n <= 0 || (batch_pos + n) >= (int)(sizeof(sasi_log_batch) - 1)) {
            break;  /* buffer full, flush what we have */
        }
        batch_pos += n;
        sasi_log_buffer.read_idx++;
    }

    if (batch_pos > 0) {
        UINT bw;
        f_write(&sasi_log_fil, sasi_log_batch, batch_pos, &bw);
        f_sync(&sasi_log_fil);
    }

    /* Reset idle tracker so we don't flush again immediately */
    bus_free_since = time_us_64();
}

bool sasi_log_is_enabled(void) {
    return sasi_log_enabled;
}

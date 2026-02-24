/* sasi_log.h
 * SASI command logging to SD card for debugging Victor 9000 DMA board
 *
 * Logs SASI commands (opcode, LBA, block count, duration, status) to
 * SASI_LOG.TXT on the SD card. Logging is opt-in: place a file named
 * "SASLOG" on the SD card root to enable it.
 */
#ifndef SASI_LOG_H
#define SASI_LOG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Note: This header requires dma.h to be included first for dma_registers_t */

/* Default: logging disabled unless SASLOG marker file exists on SD card */
#ifndef SASI_LOG_ENABLED_DEFAULT
#define SASI_LOG_ENABLED_DEFAULT 0
#endif

/* Ring buffer holds up to this many entries before oldest are overwritten */
#define SASI_LOG_BUFFER_SIZE 512

/* Per-command log entry (32 bytes, power-of-2 for fast masking) */
typedef struct {
    uint32_t timestamp_us;    /* microseconds since boot */
    uint8_t  opcode;          /* SASI command opcode */
    uint8_t  status;          /* completion status (0x00 = GOOD) */
    uint8_t  target;          /* SASI target ID */
    uint8_t  event;           /* 0 = cmd_start, 1 = cmd_complete */
    uint32_t lba;             /* logical block address */
    uint16_t block_count;     /* sector count */
    uint16_t duration_ms;     /* command duration (complete only) */
    uint32_t dma_address;     /* DMA address at event time */
    uint8_t  bus_ctrl;        /* bus control state */
    uint8_t  reserved[11];    /* pad to 32 bytes */
} sasi_log_entry_t;

_Static_assert(sizeof(sasi_log_entry_t) == 32, "sasi_log_entry_t must be 32 bytes");

/* Ring buffer */
typedef struct {
    sasi_log_entry_t entries[SASI_LOG_BUFFER_SIZE];
    volatile uint32_t write_idx;
    volatile uint32_t read_idx;
} sasi_log_buffer_t;

/* Initialize logging subsystem. Call after SD card is mounted. */
void sasi_log_init(void);

/* Record start of a SASI command */
void sasi_log_cmd_start(uint8_t opcode, uint8_t target, uint32_t lba,
                        uint16_t blocks, uint32_t dma_addr, uint8_t bus_ctrl);

/* Record completion of a SASI command */
void sasi_log_cmd_complete(uint8_t opcode, uint8_t target, uint8_t status,
                           uint32_t lba, uint16_t blocks, uint32_t dma_addr,
                           uint8_t bus_ctrl);

/* Record a DMA error event (e.g. bus master acquisition failure, PIO timeout).
 * Uses event=2 to distinguish from CMD_START(0) / CMD_DONE(1). */
void sasi_log_dma_error(uint8_t error_code, uint32_t dma_addr, uint8_t bus_ctrl);

/* Flush buffered entries to SD card when bus is idle.
 * Call from the Core 0 main loop. Safe to call frequently;
 * only writes when bus_ctrl == 0 and enough idle time has elapsed. */
void sasi_log_flush_if_ready(const dma_registers_t *dma);

/* Force a flush of buffered log entries immediately.
 * Intended for manual debug control from UART command handlers. */
void sasi_log_flush_now(void);

/* Query whether logging is currently active */
bool sasi_log_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* SASI_LOG_H */

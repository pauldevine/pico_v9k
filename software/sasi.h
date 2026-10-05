#ifndef SASI_H
#define SASI_H

#include <stdint.h>
#include <stdbool.h>

// Note: This header requires dma.h to be included first for dma_registers_t
// Forward declaration doesn't work well because we need the full struct definition

#define XEBEC_INTERNAL_DIAG 0xE4
#define XEBEC_RAM_DIAG      0xE0
#define XEBEC_DRIVE_DIAG    0xE3

// Diagnostic trace - records last N events for post-mortem debugging
#define SASI_TRACE_SIZE 256

typedef enum {
    TRACE_CMD_BYTE,      // Command byte received
    TRACE_CMD_COMPLETE,  // Full command dispatched (opcode in value)
    TRACE_STATUS_PHASE,  // Entered status phase (status byte in value)
    TRACE_MSG_PHASE,     // Entered message phase
    TRACE_BUS_FREE,      // Bus released
    TRACE_DMA_READ,      // DMA read started (sector in value)
    TRACE_DMA_WRITE,     // DMA write started (sector in value)
    TRACE_RESET,         // Device reset
    TRACE_SELECT,        // Target selected
    TRACE_DATA_READ,     // DATA register read by host (bus_ctrl in value)
    TRACE_STATUS_READ,   // STATUS register read by host
    TRACE_DATA_OUT,      // Data-out byte received (for 0x0C params, etc.)
    TRACE_DATAOUT_SETUP, // Data-out phase set up (value=expected bytes, bus_ctrl=new state)
    TRACE_DMA_RESULT,    // DMA transfer result (value: 0=ok, 1=storage_fail, 2=dma_fail)
    TRACE_DMA_SECTOR,    // Per-sector progress (value=sector&0xFF, cmd_idx=completed_blocks, bus_ctrl=addr_low)
    TRACE_DMA_ADDR,      // Full 20-bit DMA start address (value=addr_low, cmd_idx=addr_mid, bus_ctrl=addr_high)
} sasi_trace_type_t;

typedef struct {
    uint8_t type;        // sasi_trace_type_t
    uint8_t cmd_index;   // Current command byte index
    uint8_t value;       // Event-specific value
    uint8_t bus_ctrl;    // Bus control state at time of event
    uint32_t seq;        // Sequence number
} sasi_trace_entry_t;

typedef struct {
    sasi_trace_entry_t entries[SASI_TRACE_SIZE];
    uint32_t head;       // Next write position
    uint32_t seq;        // Sequence counter
} sasi_trace_t;

// Trace functions
void sasi_trace_init(void);
void sasi_trace_event(sasi_trace_type_t type, uint8_t value, uint8_t cmd_idx, uint8_t bus_ctrl);
void sasi_trace_dump(void);  // Call this to print trace to UART

#ifndef SASI_COMMAND_DELAY_US
// SASI controllers require longer execution delays than SCSI (~100μs vs ~50μs)
// per RASCSI reference implementation. This ensures timing-sensitive hosts
// have adequate time to observe phase transitions.
#define SASI_COMMAND_DELAY_US 100u
#endif

typedef enum {
    SASI_PHASE_IDLE,
    SASI_PHASE_COMMAND,
    SASI_PHASE_DATA_IN,
    SASI_PHASE_DATA_OUT,
    SASI_PHASE_STATUS,
    SASI_PHASE_MESSAGE
} sasi_phase_t;

// Command routing and helpers
void route_to_sasi_target(dma_registers_t *dma, uint8_t *cmd, int len);
void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd);
void handle_write_sectors(dma_registers_t *dma, uint8_t *cmd);
void handle_request_sense(dma_registers_t *dma, uint8_t *cmd);
void handle_mode_select(dma_registers_t *dma, uint8_t *cmd);
void handle_sasi_command_byte(dma_registers_t *dma, uint8_t cmd_byte);
bool handle_sasi_data_out_byte(dma_registers_t *dma, uint8_t data_byte);
void handle_test_unit_ready(dma_registers_t *dma);
void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type);

// Disk read (uses FujiNet when available). Returns false on fatal error.
bool read_sector_from_disk(dma_registers_t *dma, uint32_t sector, uint8_t *buffer);

// Command lifecycle
bool command_complete(uint8_t *command_buffer, int cmd_index);
void signal_command_complete(dma_registers_t *dma);

// Reset SASI command state (call on device reset)
void sasi_reset_command_state(void);
void sasi_discard_partial_command(void);

// Per-command timing diagnostics (updated by Core 1, read by UART dump)
extern volatile uint32_t sasi_last_cmd_us;
extern volatile uint32_t sasi_max_cmd_us;
extern volatile uint32_t sasi_cmd_over_1s_count;
extern volatile uint32_t sasi_cmd_over_4s_count;

// Set by Core 1 during DMA transfers so Core 0 stuck detector doesn't false-fire.
extern volatile bool sasi_in_dma_transfer;

// Per-operation timing breakdown within sector loops.
// Tracks max duration of each step so we can identify stall sources.
typedef struct {
    uint32_t max_dma_read_us;      // DMA read from Victor (WRITE(6) path)
    uint32_t max_dma_write_us;     // DMA write to Victor (READ(6) path)
    uint32_t max_sd_read_us;       // SD card read (READ(6) path)
    uint32_t max_sd_write_us;      // SD card write (WRITE(6) path)
    uint32_t max_sync_us;          // f_sync at end of WRITE(6)
    uint32_t max_verify_us;        // VERIFY_DMA_WRITES read-back
    uint32_t max_obtain_us;        // obtain_dma_master (HOLD/HLDA)
    uint32_t stall_count;          // operations exceeding STALL_THRESHOLD_US
    uint32_t stall_last_op;        // which op hit the last stall (see SASI_OP_* below)
    uint32_t stall_last_us;        // duration of last stall
    uint32_t stall_last_lba;       // LBA when last stall occurred
} sasi_op_timing_t;

// Operation identifiers for stall_last_op
#define SASI_OP_DMA_READ   1
#define SASI_OP_DMA_WRITE  2
#define SASI_OP_SD_READ    3
#define SASI_OP_SD_WRITE   4
#define SASI_OP_SYNC       5
#define SASI_OP_VERIFY     6
#define SASI_OP_OBTAIN     7

// Threshold for logging a stall (100ms)
#define SASI_STALL_THRESHOLD_US 100000

extern sasi_op_timing_t sasi_op_timing;
void sasi_op_timing_init(void);

// Record timing for one operation; updates max and stall tracking.
// Defined inline in the header so both sasi.c and dma.c can use it.
static inline void sasi_op_record(uint32_t elapsed_us, volatile uint32_t *max_field,
                                  uint32_t op_id, uint32_t lba) {
    if (elapsed_us > *max_field) *max_field = elapsed_us;
    if (elapsed_us > SASI_STALL_THRESHOLD_US) {
        sasi_op_timing.stall_count++;
        sasi_op_timing.stall_last_op = op_id;
        sasi_op_timing.stall_last_us = elapsed_us;
        sasi_op_timing.stall_last_lba = lba;
    }
}

// Per-sector CRC trace ring buffer for DMA transfer debugging.
// Records CRC-8 + first/last byte for every sector transferred via DMA
// in both directions: 'R' = disk→Victor (READ(6)), 'W' = Victor→disk (WRITE(6)).
#if VERIFY_DMA_WRITES
#define DMA_CRC_TRACE_SIZE 2048

typedef struct {
    uint32_t lba;
    uint32_t dma_addr;   // 20-bit Victor bus address
    uint8_t crc8;
    uint8_t direction;   // 'R' = READ(6) write-to-Victor, 'W' = WRITE(6) read-from-Victor
    uint8_t first_byte;
    uint8_t last_byte;
} dma_crc_entry_t;

typedef struct {
    dma_crc_entry_t entries[DMA_CRC_TRACE_SIZE];
    uint32_t head;
    uint32_t total;
} dma_crc_trace_t;

extern dma_crc_trace_t dma_crc_trace;

void dma_crc_trace_init(void);
void dma_crc_trace_record(uint32_t lba, uint32_t dma_addr,
                          const uint8_t *data, uint16_t len, uint8_t direction);
void dma_crc_trace_dump(void);
#endif // VERIFY_DMA_WRITES

#endif

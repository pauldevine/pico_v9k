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

// Disk read (uses FujiNet when available)
void read_sector_from_disk(dma_registers_t *dma, uint32_t sector, uint8_t *buffer);

// Command lifecycle
bool command_complete(uint8_t *command_buffer, int cmd_index);
void signal_command_complete(dma_registers_t *dma);

// Reset SASI command state (call on device reset)
void sasi_reset_command_state(void);

#endif

#define XEBEC_INTERNAL_DIAG 0xE4
#define XEBEC_RAM_DIAG      0xE0
#define XEBEC_DRIVE_DIAG    0xE3

typedef enum {
    SASI_PHASE_IDLE,
    SASI_PHASE_COMMAND,
    SASI_PHASE_DATA_IN,
    SASI_PHASE_DATA_OUT,
    SASI_PHASE_STATUS,
    SASI_PHASE_MESSAGE
} sasi_phase_t;

// Command routing and helpers
void route_to_scsi_target(dma_registers_t *dma, uint8_t *cmd, int len);
void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd);
void handle_scsi_command_byte(dma_registers_t *dma, uint8_t cmd_byte);
void handle_test_unit_ready(dma_registers_t *dma);
void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type);

// Disk read (uses FujiNet when available)
void read_sector_from_disk(dma_registers_t *dma, uint32_t sector, uint8_t *buffer);

// Command lifecycle
bool command_complete(uint8_t *command_buffer, int cmd_index);
void signal_command_complete(dma_registers_t *dma);

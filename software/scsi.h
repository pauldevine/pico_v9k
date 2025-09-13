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

// In your DMA register structure, add:
struct {
    sasi_phase_t current_phase;
    uint8_t command_buffer[16];
    int command_index;
    bool in_command_phase;
} sasi_state;


void route_to_sasi_target(dma_registers_t *dma, uint8_t *cmd, int len);
void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd);
void handle_sasi_command_byte(dma_registers_t *dma, uint8_t cmd_byte);
void handle_test_unit_ready(dma_registers_t *dma);
void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type);
void read_sector_from_disk(uint32_t sector, uint8_t *buffer);
bool command_complete(uint8_t *command_buffer, int cmd_index);
void signal_command_complete(dma_registers_t *dma);
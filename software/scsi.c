#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dma.h"
#include "logging.h"
#include "scsi.h"

void route_to_scsi_target(dma_registers_t *dma, uint8_t *cmd, int len) {
    switch (cmd[0]) {
        case 0x01: // Test Unit Ready
            handle_test_unit_ready(dma);
            break;
            
        case 0xE0: // Xebec Diagnostic
            handle_xebec_diagnostic(dma, cmd[1]); // Second byte is diagnostic type
            break;
            
        case 0x08: // Read Command
            handle_read_sectors(dma, cmd);
            break;
    }
}

void handle_scsi_command_byte(dma_registers_t *dma, uint8_t cmd_byte) {
    static uint8_t command_buffer[16];
    static int cmd_index = 0;
    
    command_buffer[cmd_index++] = cmd_byte;
    
    // First byte is the opcode
    if (cmd_index == 1) {
        switch (cmd_byte) {
            case 0x01: // Test Unit Ready
                printf("SCSI: Test Unit Ready\n");
                break;
            case XEBEC_RAM_DIAG: // Xebec RAM diagnostic command
                printf("SCSI: Xebec RAM Diagnostic E0\n");
                break;
            case XEBEC_DRIVE_DIAG: // Xebec drive diagnostic command
                printf("SCSI: Xebec Drive Diagnostic E3\n");
                break;
            case XEBEC_INTERNAL_DIAG: // Xebec diagnostic command
                printf("SCSI: Xebec Diagnostic E4\n");
                break;
            case 0x08: // Read sectors
                printf("SCSI: Read Command\n");
                break;
            // Add other opcodes as needed
        }
    }
    
    // When command is complete, route to appropriate handler
    if (command_complete(command_buffer, cmd_index)) {
        route_to_scsi_target(dma, command_buffer, cmd_index);
        cmd_index = 0; // Reset for next command
    }
}

void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd) {
    // Parse command to extract sector number, count, etc.
    uint32_t sector = (cmd[1] << 16) | (cmd[2] << 8) | cmd[3];
    uint8_t count = cmd[4];
    
    printf("Reading %d sectors starting at %d\n", count, sector);
    
    // Simulate reading from disk and DMA to system RAM
    for (int i = 0; i < count; i++) {
        uint8_t sector_data[512];
        read_sector_from_disk(sector + i, sector_data);
        
        // DMA transfer to system RAM
        dma_write_to_victor_ram(PIO_DMA, WRITE_SM, 
                               sector_data, 512, 
                               dma->dma_address.full);
        
        // Auto-increment DMA address (the missing piece!)
        dma->dma_address.full += 512;
    }
    
    // Signal completion to host
    signal_command_complete(dma);
}

void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type) {
    printf("SASI: Executing diagnostic 0x%02X\n", diagnostic_type);

    // Execute diagnostic (always pass for now)
    uint8_t result = 0x00;  // 0x00 = no error
    
    // Store result for when host reads REG_DATA
    dma->command = result;
    
    // Transition to status phase - controller wants to send status
    dma->bus_ctrl = SASI_BSY_BIT | SASI_REQ_BIT | SASI_INP_BIT;  // Status phase: BSY=1, REQ=1, C/D=0, I/O=1
    dma_update_interrupts(dma, true);  // Generate interrupt to notify host
    
    // Set flag so we know we're in status phase
    dma->state.non_dma_req = 1;  // Next REG_DATA read will return the status
}
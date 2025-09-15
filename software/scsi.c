#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "pico_victor/dma.h"
#include "logging.h"
#include "scsi.h"
#include "pico_fujinet/spi.h"

static void sasi_request_cmd_byte(dma_registers_t *dma) {
    // Controller requests next command byte: BSY|REQ|CTL, I/O=0 (host->dev)
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
    dma->bus_ctrl &= ~SASI_INP_BIT; // host -> controller
    dma->state.non_dma_req = 1;
    dma_update_interrupts(dma, true);
}

static void sasi_enter_status_phase(dma_registers_t *dma, uint8_t status_byte) {
    // Prepare to send status: BSY|REQ|CTL|INP
    dma->status = status_byte;
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT);
    dma->bus_ctrl &= ~SASI_MSG_BIT;
    dma->state.non_dma_req = 1;
    dma_update_interrupts(dma, true);
}

static void sasi_enter_message_phase(dma_registers_t *dma) {
    // Move to message in: BSY|REQ|CTL|INP|MSG (single 0x00 completion message)
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    dma->state.non_dma_req = 1;
    dma_update_interrupts(dma, true);
}

static void sasi_release_bus(dma_registers_t *dma) {
    // Drop REQ/BSY and clear control lines to idle
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_BSY_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT);
    dma_update_interrupts(dma, false);
}

void route_to_scsi_target(dma_registers_t *dma, uint8_t *cmd, int len) {
    switch (cmd[0]) {
        case 0x01: // Test Unit Ready
            handle_test_unit_ready(dma);
            break;
            
        case 0xE0: // Xebec Diagnostic
            handle_xebec_diagnostic(dma, cmd[1]); // Second byte is diagnostic type
            break;
            
        case 0x08: // Read(6)
        case 0x03: // Older SASI/Xebec Read
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
    } else {
        // Request next byte per log sequence (BSY|REQ|CTL)
        sasi_request_cmd_byte(dma);
    }
}

void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd) {
    // Parse Read(6)/SASI READ
    // For Read(6): cmd[1] (5 MSBs), cmd[2], cmd[3], cmd[4]=count
    // For SASI/Xebec variant in logs (0x03), bytes match 3-byte LBA + count
    uint32_t sector = ((cmd[1] & 0x1F) << 16) | (cmd[2] << 8) | cmd[3];
    uint8_t count = cmd[4] ? cmd[4] : 256; // SCSI Read(6): 0 means 256 blocks
    
    printf("Reading %d sectors starting at %d\n", count, sector);
    
    // Indicate busy during data transfer (matches log polling values)
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);

    // Simulate reading from disk and DMA to system RAM
    for (int i = 0; i < count; i++) {
        uint8_t sector_data[512];
        read_sector_from_disk(dma, sector + i, sector_data);
        
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
    
    // Transition to status phase (BSY|REQ|CTL|INP) and queue result
    sasi_enter_status_phase(dma, result);
}

bool command_complete(uint8_t *command_buffer, int cmd_index) {
    if (cmd_index == 0) return false;
    uint8_t op = command_buffer[0];
    switch (op) {
        case 0x01: // Test Unit Ready
            return cmd_index >= 1;
        case 0xE0: // Xebec RAM diag (observed as 6 bytes total in log)
        case 0xE3: // Drive diag
        case 0xE4: // Internal diag
        case 0x03: // SASI Read
        case 0x08: // Read(6)
            return cmd_index >= 6;
        default:
            // Assume 6-byte CDB by default
            return cmd_index >= 6;
    }
}

void signal_command_complete(dma_registers_t *dma) {
    // After data/command phase, send GOOD status then message-in 0x00
    sasi_enter_status_phase(dma, 0x00);
}

void handle_test_unit_ready(dma_registers_t *dma) {
    // Respond immediately with GOOD status
    sasi_enter_status_phase(dma, 0x00);
}

void read_sector_from_disk(dma_registers_t *dma, uint32_t sector, uint8_t *buffer) {
    // Map selected target to FujiNet device
    uint8_t target = dma ? (dma->selected_target & 0x07) : 0;
    uint8_t device = DEVICE_DISK_BASE + target;

    // Try FujiNet; fall back to deterministic data if not available
    if (!fujinet_read_sector(device, sector, buffer, 512)) {
        for (int i = 0; i < 512; i++) buffer[i] = (uint8_t)((sector + i) & 0xFF);
    }
}

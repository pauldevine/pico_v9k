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
#include "sasi.h"
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

static bool is_request_sense_cdb(const uint8_t *cmd, int len) {
    if (len < 6) return false;
    if (cmd[0] != 0x03) return false;
    // Heuristic: RS has reserved bytes 2/3 = 0, allocation length > 0
    return (cmd[2] == 0x00 && cmd[3] == 0x00 && cmd[4] > 0);
}

void route_to_sasi_target(dma_registers_t *dma, uint8_t *cmd, int len) {
    switch (cmd[0]) {
        case 0x01: // Test Unit Ready
            handle_test_unit_ready(dma);
            break;
            
        case 0xE0: // Xebec Diagnostic
            handle_xebec_diagnostic(dma, cmd[1]); // Second byte is diagnostic type
            break;
            
        case 0x08: // Read(6)
            handle_read_sectors(dma, cmd);
            break;
        case 0x03: // Either Request Sense or SASI Read
            if (is_request_sense_cdb(cmd, len)) {
                handle_request_sense(dma, cmd);
            } else {
                handle_read_sectors(dma, cmd); // observed in Victor logs
            }
            break;
        case 0x0A: // Write(6)
            handle_write_sectors(dma, cmd);
            break;
        case 0x15: // Mode Select(6)
            handle_mode_select(dma, cmd);
            break;
    }
}

void handle_sasi_command_byte(dma_registers_t *dma, uint8_t cmd_byte) {
    static uint8_t command_buffer[16];
    static int cmd_index = 0;
    
    command_buffer[cmd_index++] = cmd_byte;
    
    // First byte is the opcode
    if (cmd_index == 1) {
        switch (cmd_byte) {
            case 0x01: // Test Unit Ready
                printf("SASI: Test Unit Ready\n");
                break;
            case XEBEC_RAM_DIAG: // Xebec RAM diagnostic command
                printf("SASI: Xebec RAM Diagnostic E0\n");
                break;
            case XEBEC_DRIVE_DIAG: // Xebec drive diagnostic command
                printf("SASI: Xebec Drive Diagnostic E3\n");
                break;
            case XEBEC_INTERNAL_DIAG: // Xebec diagnostic command
                printf("SASI: Xebec Diagnostic E4\n");
                break;
            case 0x08: // Read sectors
                printf("SASI: Read Command\n");
                break;
            // Add other opcodes as needed
        }
    }
    
    // When command is complete, route to appropriate handler
    if (command_complete(command_buffer, cmd_index)) {
        route_to_sasi_target(dma, command_buffer, cmd_index);
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
    uint16_t blocks = cmd[4] ? cmd[4] : 256; // SASI Read(6): 0 means 256 blocks

    printf("Reading %u sectors starting at %u\n", (unsigned)blocks, (unsigned)sector);

    if (dma) {
        dma->logical_block.full = sector;
        dma->block_count.full = blocks;
    }

    // Indicate busy during data transfer (matches log polling values)
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);

    // Simulate reading from disk and DMA to system RAM
    for (uint16_t i = 0; i < blocks; i++) {
        uint8_t sector_data[512];
        read_sector_from_disk(dma, sector + i, sector_data);
        
        // DMA transfer to system RAM
        dma_write_to_victor_ram(PIO_DMA, WRITE_SM, 
                               sector_data, 512, 
                               dma->dma_address.full);
        
        // Auto-increment DMA address (the missing piece!)
        dma->dma_address.full += 512;
    }

    if (dma) {
        dma->logical_block.full = sector + blocks;
        dma->block_count.full = 0;
    }

    // Signal completion to host
    signal_command_complete(dma);
}

void handle_write_sectors(dma_registers_t *dma, uint8_t *cmd) {
    // Parse Write(6)
    uint32_t sector = ((cmd[1] & 0x1F) << 16) | (cmd[2] << 8) | cmd[3];
    uint16_t blocks = cmd[4] ? cmd[4] : 256;

    printf("Writing %u sectors starting at %u\n", (unsigned)blocks, (unsigned)sector);

    if (dma) {
        dma->logical_block.full = sector;
        dma->block_count.full = blocks;
    }

    // Indicate busy during data transfer (matching observed status polling)
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);

    for (uint16_t i = 0; i < blocks; i++) {
        uint8_t sector_data[512];
        // Read from Victor RAM into local buffer
        dma_read_from_victor_ram(PIO_DMA, READ_SM, sector_data, 512, dma->dma_address.full);
        dma->dma_address.full += 512;

        // Write to disk via FujiNet
        uint8_t target = dma ? (dma->selected_target & 0x07) : 0;
        uint8_t device = DEVICE_DISK_BASE + target;
        if (!fujinet_write_sector(device, sector + i, sector_data, 512)) {
            // For now, ignore failures and continue
            printf("Warning: write sector LBA %lu failed, continuing\n", (unsigned long)(sector + i));
        }
    }

    if (dma) {
        dma->logical_block.full = sector + blocks;
        dma->block_count.full = 0;
    }

    signal_command_complete(dma);
}

void handle_request_sense(dma_registers_t *dma, uint8_t *cmd) {
    uint8_t alloc_len = cmd[4];
    uint8_t sense_len = alloc_len ? alloc_len : 0; // if 0, return nothing
    if (sense_len > 18) sense_len = 18; // minimal fixed format sense

    uint8_t sense[18] = {0};
    // Minimal sense: all zeros indicates no error for old SASI targets

    // Data-in transfer to Victor RAM
    if (sense_len) {
        // Mirror read-phase busy style during DMA
        dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT);
        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT | SASI_INP_BIT);

        dma_write_to_victor_ram(PIO_DMA, WRITE_SM, sense, sense_len, dma->dma_address.full);
        dma->dma_address.full += sense_len;
    }

    signal_command_complete(dma);
}

void handle_mode_select(dma_registers_t *dma, uint8_t *cmd) {
    size_t param_len = cmd[4];
    if (param_len) {
        uint8_t params[256];
        if (param_len > sizeof(params)) param_len = sizeof(params);

        // Busy during host->device data transfer
        dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_INP_BIT | SASI_MSG_BIT);
        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);

        // Read parameter list from Victor RAM (ignore content for now)
        dma_read_from_victor_ram(PIO_DMA, READ_SM, params, param_len, dma->dma_address.full);
        dma->dma_address.full += param_len;
    }

    // Respond GOOD
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

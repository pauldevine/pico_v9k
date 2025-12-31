#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"  // For __dmb() memory barrier
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "pico_victor/dma.h"
#include "logging.h"
#include "reg_queue_processor.h"
#include "sasi.h"
#include "pico_fujinet/spi.h"

// SASI command state - file-scope so it can be reset on device reset
static uint8_t sasi_command_buffer[16];
static int sasi_cmd_index = 0;

void sasi_reset_command_state(void) {
    sasi_cmd_index = 0;
    // Optionally clear buffer for cleaner debugging
    for (int i = 0; i < 16; i++) {
        sasi_command_buffer[i] = 0;
    }
}

static void sasi_request_cmd_byte(dma_registers_t *dma) {
    // Controller requests next command byte: BSY|REQ|CTL, I/O=0 (host->dev)
    uint8_t before = dma->bus_ctrl;
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
    dma->bus_ctrl &= ~SASI_INP_BIT; // host -> controller
    __dmb();  // Ensure Core 0 sees bus_ctrl update before any DATA read
    dma->state.non_dma_req = 1;
    dma_update_interrupts(dma, true);
    cached_status_sync_from_bus(dma);
    fast_log("SASI: request_cmd_byte done, bus_ctrl 0x%02X->0x%02X\n", before, dma->bus_ctrl);
}

static void sasi_enter_status_phase(dma_registers_t *dma, uint8_t status_byte) {
    // Prepare to send status: BSY|REQ|CTL|INP
    fast_log("SASI: entering status phase, status=0x%02X\n", status_byte);
    dma->status = status_byte;
    cached_set_data(status_byte);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT);
    dma->bus_ctrl &= ~SASI_MSG_BIT;
    __dmb();  // Ensure Core 0 sees bus_ctrl update before any DATA read
    dma->state.non_dma_req = 1;
    dma->state.status_pending = 1;  // Mark that we're waiting for host to read status
    dma_update_interrupts(dma, true);
    cached_status_sync_from_bus(dma);
    fast_log("SASI: status phase ready, bus_ctrl=0x%02X\n", dma->bus_ctrl);
}

static void sasi_enter_message_phase(dma_registers_t *dma) {
    // Move to message in: BSY|REQ|CTL|INP|MSG (single 0x00 completion message)
    cached_set_data(0x00);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl update
    dma->state.non_dma_req = 1;
    dma_update_interrupts(dma, true);
    cached_status_sync_from_bus(dma);
}

static void sasi_release_bus(dma_registers_t *dma) {
    // Drop REQ/BSY and clear control lines to idle
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_BSY_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl cleared
    dma_update_interrupts(dma, false);
    cached_status_sync_from_bus(dma);
}

static bool is_request_sense_cdb(const uint8_t *cmd, int len) {
    if (len < 6) return false;
    if (cmd[0] != 0x03) return false;
    // Heuristic: RS has reserved bytes 2/3 = 0, allocation length > 0
    return (cmd[2] == 0x00 && cmd[3] == 0x00 && cmd[4] > 0);
}

void route_to_sasi_target(dma_registers_t *dma, uint8_t *cmd, int len) {
    switch (cmd[0]) {
        case 0x00: // Test Unit Ready
            handle_test_unit_ready(dma);
            break;

        case 0x01: // Recalibrate/Rezero Unit - treat same as Test Unit Ready for now
            printf("SASI: Recalibrate/Rezero Unit\n");
            handle_test_unit_ready(dma);  // Return GOOD status
            break;

        case XEBEC_RAM_DIAG:      // 0xE0 - Xebec RAM diagnostic
        case XEBEC_DRIVE_DIAG:    // 0xE3 - Xebec drive interface diagnostic
        case XEBEC_INTERNAL_DIAG: // 0xE4 - Xebec internal diagnostic
            handle_xebec_diagnostic(dma, cmd[1]); // Second byte is diagnostic subtype
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

        case 0x0C: // Initialize Drive Characteristics
            printf("SASI: Initialize Drive Characteristics\n");
            handle_mode_select(dma, cmd);  // Accept parameters and return GOOD
            break;

        case 0x15: // Mode Select(6)
            handle_mode_select(dma, cmd);
            break;

        default:
            // Unknown command - return GOOD status anyway to not block boot
            printf("SASI: Unknown command 0x%02X, returning GOOD status\n", cmd[0]);
            signal_command_complete(dma);
            break;
    }
}

void handle_sasi_command_byte(dma_registers_t *dma, uint8_t cmd_byte) {
    fast_log("SASI: handle_cmd_byte entry idx=%d byte=0x%02X\n", sasi_cmd_index, cmd_byte);
    printf("SASI CMD[%d] = 0x%02X\n", sasi_cmd_index, cmd_byte);
    sasi_command_buffer[sasi_cmd_index++] = cmd_byte;

    // First byte is the opcode - log recognized commands
    if (sasi_cmd_index == 1) {
        switch (cmd_byte) {
            case 0x00:
                printf("SASI: Test Unit Ready\n");
                break;
            case 0x01:
                printf("SASI: Recalibrate/Rezero Unit\n");
                break;
            case 0x03:
                printf("SASI: Request Sense or Read\n");
                break;
            case 0x08:
                printf("SASI: Read(6)\n");
                break;
            case 0x0A:
                printf("SASI: Write(6)\n");
                break;
            case 0x0C:
                printf("SASI: Initialize Drive Characteristics\n");
                break;
            case 0x15:
                printf("SASI: Mode Select(6)\n");
                break;
            case XEBEC_RAM_DIAG:
                printf("SASI: Xebec RAM Diagnostic 0xE0\n");
                break;
            case XEBEC_DRIVE_DIAG:
                printf("SASI: Xebec Drive Diagnostic 0xE3\n");
                break;
            case XEBEC_INTERNAL_DIAG:
                printf("SASI: Xebec Internal Diagnostic 0xE4\n");
                break;
            default:
                printf("SASI: Unknown opcode 0x%02X\n", cmd_byte);
                break;
        }
    }

    // When command is complete, route to appropriate handler
    if (command_complete(sasi_command_buffer, sasi_cmd_index)) {
        fast_log("SASI: command complete, routing opcode=0x%02X\n", sasi_command_buffer[0]);
        route_to_sasi_target(dma, sasi_command_buffer, sasi_cmd_index);
        sasi_cmd_index = 0; // Reset for next command
    } else {
        // Request next byte per log sequence (BSY|REQ|CTL)
        fast_log("SASI: requesting next byte, idx=%d\n", sasi_cmd_index);
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

    // Indicate data-in phase: BSY asserted, C/D low, I/O high
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_CTL_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_INP_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl update
    cached_status_sync_from_bus(dma);

    // Simulate reading from disk and DMA to system RAM
    for (uint16_t i = 0; i < blocks; i++) {
        uint8_t sector_data[512];
        read_sector_from_disk(dma, sector + i, sector_data);
        
        // DMA transfer to system RAM
        dma_write_to_victor_ram(sector_data, 512, dma->dma_address.full);

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

    // Indicate data-out phase: BSY asserted, C/D low, I/O low
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_CTL_BIT);
    dma->bus_ctrl |= SASI_BSY_BIT;
    dma->bus_ctrl &= ~SASI_INP_BIT;
    __dmb();  // Ensure Core 0 sees bus_ctrl update
    cached_status_sync_from_bus(dma);

    for (uint16_t i = 0; i < blocks; i++) {
        uint8_t sector_data[512];
        // Read from Victor RAM into local buffer
        dma_read_from_victor_ram(sector_data, 512, dma->dma_address.full);
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
        // Data-in phase for sense data: BSY asserted, I/O high, C/D low
        dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_CTL_BIT);
        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_INP_BIT);
        __dmb();  // Ensure Core 0 sees bus_ctrl update
        cached_status_sync_from_bus(dma);

        dma_write_to_victor_ram(sense, sense_len, dma->dma_address.full);
        dma->dma_address.full += sense_len;
    }

    signal_command_complete(dma);
}

void handle_mode_select(dma_registers_t *dma, uint8_t *cmd) {
    size_t param_len = cmd[4];
    if (param_len) {
        uint8_t params[256];
        if (param_len > sizeof(params)) param_len = sizeof(params);

        // Data-out phase: BSY asserted, I/O low, C/D low
        dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_CTL_BIT);
        dma->bus_ctrl |= SASI_BSY_BIT;
        dma->bus_ctrl &= ~SASI_INP_BIT;
        __dmb();  // Ensure Core 0 sees bus_ctrl update
        cached_status_sync_from_bus(dma);

        // Read parameter list from Victor RAM (ignore content for now)
        dma_read_from_victor_ram(params, param_len, dma->dma_address.full);
        dma->dma_address.full += param_len;
    }

    // Respond GOOD
    signal_command_complete(dma);
}
void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type) {
    printf("SASI: Executing diagnostic 0x%02X (opcode 0x%02X)\n", diagnostic_type, sasi_command_buffer[0]);

    // Execute diagnostic (always pass for now)
    // Diagnostics are non-data commands that complete immediately with status.
    // The host will read status via register 0x10 (DATA) when it sees REQ|CTL|INP.
    // The defer_process_read() handler will advance through status -> message -> bus free
    // as the host reads the DATA register.

    // Enter status phase with GOOD (0x00) status - host will read this from DATA register
    // The bus state machine in defer_process_read() handles the rest automatically:
    //   1. Host sees status 0x0F (REQ|CTL|INP|BSY)
    //   2. Host reads DATA register -> gets status byte (0x00)
    //   3. We transition to message phase (status becomes 0x1F)
    //   4. Host reads DATA register -> gets message byte (0x00)
    //   5. Bus is released (status becomes 0x00)
    signal_command_complete(dma);
}

bool command_complete(uint8_t *command_buffer, int cmd_index) {
    if (cmd_index == 0) return false;
    uint8_t op = command_buffer[0];

    // SASI/SCSI-1 commands are grouped by class (upper 3 bits of opcode):
    // Class 0 (0x00-0x1F): 6-byte CDB
    // Class 1 (0x20-0x3F): 10-byte CDB (not commonly used in SASI)
    // Class 7 (0xE0-0xFF): Vendor-specific (Xebec uses 6-byte)

    // All commands observed in Victor 9000 boot sequence are 6 bytes:
    // 0x00 - Test Unit Ready (6 bytes)
    // 0x01 - Recalibrate/Rezero (6 bytes)
    // 0x03 - Request Sense (6 bytes)
    // 0x08 - Read(6) (6 bytes)
    // 0x0A - Write(6) (6 bytes)
    // 0x0C - Initialize Drive Characteristics (6 bytes)
    // 0x15 - Mode Select(6) (6 bytes)
    // 0xE0 - Xebec RAM Diagnostic (6 bytes)
    // 0xE3 - Xebec Drive Diagnostic (6 bytes)
    // 0xE4 - Xebec Internal Diagnostic (6 bytes)

    return cmd_index >= 6;
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

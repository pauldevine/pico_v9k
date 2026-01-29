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
#include "pico_storage/storage.h"
#include "pico_fujinet/spi.h"

// Set to 1 to enable SASI debug printf (WARNING: slows Core 1 dramatically, causes queue overflow)
#define SASI_DEBUG_PRINTF 0
// Set to 1 to enable SASI fast_log tracing (WARNING: causes queue overflow)
#define SASI_DEBUG_FASTLOG 0

#if SASI_DEBUG_PRINTF
#define sasi_printf(...) printf(__VA_ARGS__)
#else
#define sasi_printf(...) ((void)0)
#endif

#if SASI_DEBUG_FASTLOG
#define sasi_fastlog(...) fast_log(__VA_ARGS__)
#else
#define sasi_fastlog(...) ((void)0)
#endif

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

static void sasi_apply_command_delay(dma_registers_t *dma) {
    if (!dma || SASI_COMMAND_DELAY_US == 0) {
        return;
    }

    // Hold command phase busy (BSY|CTL, no REQ) while the controller "executes".
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_INP_BIT | SASI_ACK_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl update
    cached_status_sync_from_bus(dma);

    sleep_us(SASI_COMMAND_DELAY_US);
}

static void sasi_request_cmd_byte(dma_registers_t *dma) {
    // Controller requests next command byte: BSY|REQ|CTL, I/O=0 (host->dev)
    uint8_t before = dma->bus_ctrl;
    // Drop ACK between bytes and ensure we're in command phase.
    dma->bus_ctrl &= ~(SASI_ACK_BIT | SASI_MSG_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
    dma->bus_ctrl &= ~SASI_INP_BIT; // host -> controller
    __dmb();  // Ensure bus_ctrl update is complete
    dma->state.non_dma_req = 1;
    // CRITICAL: Update cache BEFORE asserting interrupt!
    cached_status_sync_from_bus(dma);
    __dmb();  // Ensure cache write is visible to Core 0
    dma_update_interrupts(dma, true);
    sasi_fastlog("SASI: request_cmd_byte done, bus_ctrl 0x%02X->0x%02X\n", before, dma->bus_ctrl);
}

static void sasi_enter_status_phase(dma_registers_t *dma, uint8_t status_byte) {
    // Prepare to send status: BSY|REQ|CTL|INP
    sasi_fastlog("SASI: entering status phase, status=0x%02X, irq_pend=%d\n",
             status_byte, dma->state.interrupt_pending);
    dma->status = status_byte;
    cached_set_data(status_byte);
    dma->bus_ctrl &= ~SASI_ACK_BIT;
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT);
    dma->bus_ctrl &= ~SASI_MSG_BIT;
    __dmb();  // Ensure bus_ctrl update is complete
    dma->state.non_dma_req = 1;
    dma->state.status_pending = 1;  // Mark that we're waiting for host to read status
    // CRITICAL: Update cache BEFORE asserting interrupt!
    // Host polls immediately after seeing IR4 - cache must be ready.
    cached_status_sync_from_bus(dma);
    __dmb();  // Ensure cache write is visible to Core 0
    dma_update_interrupts(dma, true);
    sasi_fastlog("SASI: status phase ready, bus_ctrl=0x%02X, irq_pend=%d\n",
             dma->bus_ctrl, dma->state.interrupt_pending);
}

static void sasi_enter_message_phase(dma_registers_t *dma) {
    // Move to message in: BSY|REQ|CTL|INP|MSG (single 0x00 completion message)
    cached_set_data(0x00);
    dma->bus_ctrl &= ~SASI_ACK_BIT;
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
    __dmb();  // Ensure bus_ctrl update is complete
    dma->state.non_dma_req = 1;
    // CRITICAL: Update cache BEFORE asserting interrupt!
    cached_status_sync_from_bus(dma);
    __dmb();  // Ensure cache write is visible to Core 0
    dma_update_interrupts(dma, true);
}

static void sasi_release_bus(dma_registers_t *dma) {
    // Drop REQ/BSY and clear control lines to idle
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_BSY_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl cleared
    dma_update_interrupts(dma, false);
    cached_status_sync_from_bus(dma);
}

void route_to_sasi_target(dma_registers_t *dma, uint8_t *cmd, int len) {
    switch (cmd[0]) {
        case 0x00: // Test Unit Ready
            handle_test_unit_ready(dma);
            break;

        case 0x01: // Recalibrate/Rezero Unit - treat same as Test Unit Ready for now
            sasi_printf("SASI: Recalibrate/Rezero Unit\n");
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

        case 0x03: // Request Sense (BIOS uses 0x08 for READ, 0x03 is always SENSE)
            handle_request_sense(dma, cmd);
            break;

        case 0x0A: // Write(6)
            handle_write_sectors(dma, cmd);
            break;

        case 0x0C: // Initialize Drive Characteristics
            sasi_printf("SASI: Initialize Drive Characteristics\n");
            handle_mode_select(dma, cmd);  // Accept parameters and return GOOD
            break;

        case 0x15: // Mode Select(6)
            handle_mode_select(dma, cmd);
            break;

        default:
            // Unknown command - return GOOD status anyway to not block boot
            sasi_printf("SASI: Unknown command 0x%02X, returning GOOD status\n", cmd[0]);
            signal_command_complete(dma);
            break;
    }
}

void handle_sasi_command_byte(dma_registers_t *dma, uint8_t cmd_byte) {
    sasi_fastlog("SASI: handle_cmd_byte entry idx=%d byte=0x%02X\n", sasi_cmd_index, cmd_byte);
    sasi_printf("SASI CMD[%d] = 0x%02X\n", sasi_cmd_index, cmd_byte);
    sasi_command_buffer[sasi_cmd_index++] = cmd_byte;

    // First byte is the opcode - log recognized commands
    if (sasi_cmd_index == 1) {
        switch (cmd_byte) {
            case 0x00:
                sasi_printf("SASI: Test Unit Ready\n");
                break;
            case 0x01:
                sasi_printf("SASI: Recalibrate/Rezero Unit\n");
                break;
            case 0x03:
                sasi_printf("SASI: Request Sense\n");
                break;
            case 0x08:
                sasi_printf("SASI: Read(6)\n");
                break;
            case 0x0A:
                sasi_printf("SASI: Write(6)\n");
                break;
            case 0x0C:
                sasi_printf("SASI: Initialize Drive Characteristics\n");
                break;
            case 0x15:
                sasi_printf("SASI: Mode Select(6)\n");
                break;
            case XEBEC_RAM_DIAG:
                sasi_printf("SASI: Xebec RAM Diagnostic 0xE0\n");
                break;
            case XEBEC_DRIVE_DIAG:
                sasi_printf("SASI: Xebec Drive Diagnostic 0xE3\n");
                break;
            case XEBEC_INTERNAL_DIAG:
                sasi_printf("SASI: Xebec Internal Diagnostic 0xE4\n");
                break;
            default:
                sasi_printf("SASI: Unknown opcode 0x%02X\n", cmd_byte);
                break;
        }
    }

    // When command is complete, route to appropriate handler
    if (command_complete(sasi_command_buffer, sasi_cmd_index)) {
        sasi_fastlog("SASI: command complete, routing opcode=0x%02X\n", sasi_command_buffer[0]);
        route_to_sasi_target(dma, sasi_command_buffer, sasi_cmd_index);
        sasi_cmd_index = 0; // Reset for next command
    } else {
        // Request next byte per log sequence (BSY|REQ|CTL)
        sasi_fastlog("SASI: requesting next byte, idx=%d\n", sasi_cmd_index);
        sasi_request_cmd_byte(dma);
    }
}

void handle_read_sectors(dma_registers_t *dma, uint8_t *cmd) {
    // Parse Read(6) - command 0x08
    // cmd[1] (5 MSBs), cmd[2], cmd[3] = LBA, cmd[4] = count
    uint32_t sector = ((cmd[1] & 0x1F) << 16) | (cmd[2] << 8) | cmd[3];
    uint16_t blocks = cmd[4] ? cmd[4] : 256; // SASI Read(6): 0 means 256 blocks

    sasi_printf("RD sectors:%u starting: %u\n", (unsigned)blocks, (unsigned)sector);

    if (dma) {
        dma->logical_block.full = sector;
        dma->block_count.full = blocks;
    }

    sasi_apply_command_delay(dma);

    // During DMA transfers, keep status at BSY|CTL (command busy) while REQ stays low.
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_INP_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);
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
    cached_sync_dma_address(dma);
    signal_command_complete(dma);
}

void handle_write_sectors(dma_registers_t *dma, uint8_t *cmd) {
    // Parse Write(6)
    uint32_t sector = ((cmd[1] & 0x1F) << 16) | (cmd[2] << 8) | cmd[3];
    uint16_t blocks = cmd[4] ? cmd[4] : 256;

    sasi_printf("WR sectors:%u starting: %u\n", (unsigned)blocks, (unsigned)sector);

    if (dma) {
        dma->logical_block.full = sector;
        dma->block_count.full = blocks;
    }

    sasi_apply_command_delay(dma);

    // During DMA transfers, keep status at BSY|CTL (command busy) while REQ stays low.
    dma->bus_ctrl &= ~(SASI_REQ_BIT | SASI_MSG_BIT | SASI_INP_BIT);
    dma->bus_ctrl |= (SASI_BSY_BIT | SASI_CTL_BIT);
    __dmb();  // Ensure Core 0 sees bus_ctrl update
    cached_status_sync_from_bus(dma);

    uint8_t target = dma ? (dma->selected_target & 0x07) : 0;

    for (uint16_t i = 0; i < blocks; i++) {
        uint8_t sector_data[512];
        // Read from Victor RAM into local buffer
        dma_read_from_victor_ram(sector_data, 512, dma->dma_address.full);
        dma->dma_address.full += 512;

        // Try storage abstraction layer first (handles SD card, FujiNet, etc.)
        bool write_ok = false;
        if (storage_is_mounted(target)) {
            write_ok = storage_write_sector(target, sector + i, sector_data, 512);
        }

        // Fall back to FujiNet direct access if storage layer not available
        if (!write_ok) {
            uint8_t device = DEVICE_DISK_BASE + target;
            if (!fujinet_write_sector(device, sector + i, sector_data, 512)) {
                sasi_printf("Warning: write sector LBA %lu failed, continuing\n", (unsigned long)(sector + i));
            }
        }
    }

    if (dma) {
        dma->logical_block.full = sector + blocks;
        dma->block_count.full = 0;
    }

    cached_sync_dma_address(dma);
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
        cached_sync_dma_address(dma);
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
        cached_sync_dma_address(dma);
    }

    // Respond GOOD
    signal_command_complete(dma);
}
void handle_xebec_diagnostic(dma_registers_t *dma, uint8_t diagnostic_type) {
    sasi_printf("SASI: Executing diagnostic 0x%02X (opcode 0x%02X)\n", diagnostic_type, sasi_command_buffer[0]);

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
    uint8_t target = dma ? (dma->selected_target & 0x07) : 0;

    // Try storage abstraction layer first (handles SD card, FujiNet, etc.)
    if (storage_is_mounted(target)) {
        if (storage_read_sector(target, sector, buffer, 512)) {
            return;
        }
        sasi_printf("Warning: storage_read_sector failed for target %d, LBA %lu\n",
                    target, (unsigned long)sector);
    }

    // Fall back to FujiNet direct access if storage layer not available
    uint8_t device = DEVICE_DISK_BASE + target;
    if (!fujinet_read_sector(device, sector, buffer, 512)) {
        // Last resort: generate deterministic test data
        for (int i = 0; i < 512; i++) buffer[i] = (uint8_t)((sector + i) & 0xFF);
    }
    sasi_printf("RD sector %lu target %d\n", (unsigned long)sector, target);
}

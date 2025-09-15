#include <stdint.h>

// victor_boot_test.c
// Test for the first 4 steps of Victor BIOS boot sequence based on MAME trace
typedef uint32_t PIO; // Mock PIO type
// Mock PIO functions for testing
PIO mock_pio = (PIO)0x1000;
uint32_t mock_sm = 0;
uint32_t pis_sm0_rx_fifo_not_empty = 0;
uint32_t pis_sm1_rx_fifo_not_empty = 1;
uint32_t pis_sm2_rx_fifo_not_empty = 2;
uint32_t pis_sm3_rx_fifo_not_empty = 3;


#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include "../dma.h"
#include "../scsi.h"


// Test state tracking
typedef struct {
    scsi_phase_t current_phase;
    uint8_t command_buffer[16];
    int command_index;
    bool target_selected;
    bool diagnostic_complete;
} test_scsi_state_t;

static test_scsi_state_t test_state = {0};

// Mock SCSI target responses
void mock_scsi_response(dma_registers_t *dma, uint8_t status_bits) {
    printf("Mock SCSI: Setting bus status to 0x%02X\n", status_bits);
    dma->bus_ctrl = status_bits;
    
    // Generate interrupt if controller wants attention (REQ + CMD)
    if ((status_bits & (SASI_REQ | SASI_CTL)) == (SASI_REQ | SASI_CTL)) {
        dma_update_interrupts(dma, true);
    }
}

void handle_scsi_command_byte(dma_registers_t *dma, uint8_t cmd_byte) {
    test_state.command_buffer[test_state.command_index++] = cmd_byte;
    
    printf("SCSI Command byte[%d]: 0x%02X\n", test_state.command_index-1, cmd_byte);
    
    // First byte is opcode
    if (test_state.command_index == 1) {
        switch (cmd_byte) {
            case 0x01:
                printf("SCSI: Test Unit Ready command\n");
                break;
            case 0xE0:
                printf("SCSI: Xebec Diagnostic E0 command\n");
                break;
            default:
                printf("SCSI: Unknown command 0x%02X\n", cmd_byte);
                break;
        }
    }
    
    // Simulate controller requesting next byte (for multi-byte commands)
    if (test_state.command_index < 6) { // Assume 6-byte commands
        // Drop REQ briefly, then assert again for next byte
        mock_scsi_response(dma, SASI_BSY);  // Just busy
        mock_scsi_response(dma, SASI_BSY | SASI_REQ | SASI_CTL); // Want next command byte
    } else {
        // Command complete - handle it
        handle_command_complete(dma);
    }
}

void handle_command_complete(dma_registers_t *dma) {
    uint8_t opcode = test_state.command_buffer[0];
    
    printf("SCSI: Command complete - opcode 0x%02X\n", opcode);
    
    switch (opcode) {
        case 0x01: // Test Unit Ready
            printf("SCSI: Test Unit Ready - responding OK\n");
            // Transition to status phase
            mock_scsi_response(dma, SASI_BSY | SASI_REQ | SASI_INP); // Status phase
            break;
            
        case 0xE0: // Xebec Diagnostic
            printf("SCSI: Diagnostic E0 complete - responding OK\n");
            test_state.diagnostic_complete = true;
            // Transition to status phase  
            mock_scsi_response(dma, SASI_BSY | SASI_REQ | SASI_INP); // Status phase
            break;
    }
    
    // Reset command buffer
    test_state.command_index = 0;
    memset(test_state.command_buffer, 0, sizeof(test_state.command_buffer));
}

// Enhanced DMA write function that handles SCSI command tracking
void test_dma_write_register(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value) {
    printf("DMA Write: offset 0x%02X data 0x%02X\n", offset, value);
    
    // Call your existing function
    dma_write_register(dma, offset, value);
    
    // Additional test logic for SCSI command tracking
    if (offset == REG_DATA) {
        // Check if we're in command phase
        if (dma->bus_ctrl & SASI_CTL) {
            handle_scsi_command_byte(dma, value);
        }
    } else if (offset == REG_CONTROL) {
        // Track target selection
        if (value & SELECT_BIT) {
            printf("Test: SCSI target selection initiated\n");
            test_state.target_selected = true;
            // Simulate target responding to selection
            mock_scsi_response(dma, SASI_BSY);
        }
    }
}

// Enhanced DMA read function with test logging
uint8_t test_dma_read_register(dma_registers_t *dma, dma_reg_offsets_t offset) {
    uint8_t data = dma_read_register(dma, offset);
    printf("DMA Read: offset 0x%02X data 0x%02X\n", offset, data);
    return data;
}

// Test Step 1: Initialize DMA controller
bool test_step1_initialization() {
    printf("\n=== STEP 1: DMA INITIALIZATION ===\n");
    
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        printf("FAIL: Could not get DMA registers\n");
        return false;
    }
    
    // Reset controller (from trace: Write offset 00 data 00)
    test_dma_write_register(dma, REG_CONTROL, 0x00);
    
    // Enable DMA latch (from trace: Write offset 00 data 04)  
    test_dma_write_register(dma, REG_CONTROL, 0x04);
    
    // Verify state
    if (dma->control != 0x04) {
        printf("FAIL: Control register not set correctly\n");
        return false;
    }
    
    printf("PASS: DMA initialized\n");
    return true;
}

// Test Step 2: Configure DMA addresses
bool test_step2_dma_address_config() {
    printf("\n=== STEP 2: DMA ADDRESS CONFIGURATION ===\n");
    
    dma_registers_t *dma = dma_get_registers();
    
    // Set first address: 0x05AA55 (from trace)
    test_dma_write_register(dma, REG_ADDR_H, 0x05);
    test_dma_write_register(dma, REG_ADDR_M, 0xAA); 
    test_dma_write_register(dma, REG_ADDR_L, 0x55);
    
    // Verify by reading back
    uint8_t high = test_dma_read_register(dma, REG_ADDR_H);
    uint8_t mid = test_dma_read_register(dma, REG_ADDR_M);
    uint8_t low = test_dma_read_register(dma, REG_ADDR_L);
    
    if (high != 0x05 || mid != 0xAA || low != 0x55) {
        printf("FAIL: First DMA address not set correctly: %02X%02X%02X\n", high, mid, low);
        return false;
    }
    
    // Set second address: 0x0A55AA (from trace)
    test_dma_write_register(dma, REG_ADDR_H, 0x0A);
    test_dma_write_register(dma, REG_ADDR_M, 0x55);
    test_dma_write_register(dma, REG_ADDR_L, 0xAA);
    
    // Verify second address
    high = test_dma_read_register(dma, REG_ADDR_H);
    mid = test_dma_read_register(dma, REG_ADDR_M);
    low = test_dma_read_register(dma, REG_ADDR_L);
    
    if (high != 0x0A || mid != 0x55 || low != 0xAA) {
        printf("FAIL: Second DMA address not set correctly: %02X%02X%02X\n", high, mid, low);
        return false;
    }
    
    printf("PASS: DMA addresses configured correctly\n");
    return true;
}

// Test Step 3: SCSI Target Selection  
bool test_step3_target_selection() {
    printf("\n=== STEP 3: SCSI TARGET SELECTION ===\n");
    
    dma_registers_t *dma = dma_get_registers();
    test_state.target_selected = false;
    
    // From trace: Write offset 00 data 24 (SELECT + WMODE + LATCH)
    test_dma_write_register(dma, REG_CONTROL, 0x24);
    
    if (!test_state.target_selected) {
        printf("FAIL: Target selection not detected\n");
        return false;
    }
    
    // Clear SELECT (from trace: Write offset 00 data 04)
    test_dma_write_register(dma, REG_CONTROL, 0x04);
    
    // Check status shows target is busy (from trace: Read offset 20 data 04)
    uint8_t status = test_dma_read_register(dma, REG_STATUS);
    
    if (!(status & SASI_BSY_BIT)) {
        printf("FAIL: Target not showing as busy after selection\n");
        return false;
    }
    
    printf("PASS: SCSI target selection successful\n");
    return true;
}

// Test Step 4: Send Diagnostic Command
bool test_step4_diagnostic_command() {
    printf("\n=== STEP 4: SEND DIAGNOSTIC COMMAND ===\n");
    
    dma_registers_t *dma = dma_get_registers();
    test_state.diagnostic_complete = false;
    
    // Simulate controller wanting command (REQ + CMD + BSY)
    mock_scsi_response(dma, SASI_BSY | SASI_REQ | SASI_CTL);
    
    // Send diagnostic command sequence (from trace)
    test_dma_write_register(dma, REG_DATA, 0x01); // First command byte
    test_dma_write_register(dma, REG_DATA, 0xE0); // Diagnostic opcode  
    test_dma_write_register(dma, REG_DATA, 0x00); // Parameter 1
    test_dma_write_register(dma, REG_DATA, 0xAA); // Parameter 2
    test_dma_write_register(dma, REG_DATA, 0x00); // Parameter 3 (assumed)
    test_dma_write_register(dma, REG_DATA, 0x00); // Parameter 4 (assumed)
    
    if (!test_state.diagnostic_complete) {
        printf("FAIL: Diagnostic command not completed\n");
        return false;
    }
    
    printf("PASS: Diagnostic command sent and completed\n");
    return true;
}

// Main test function
int main() {
    printf("Victor BIOS Boot Sequence Test\n");
    printf("==============================\n");
    
    bool all_passed = true;
    
    all_passed &= test_step1_initialization();
    all_passed &= test_step2_dma_address_config();  
    all_passed &= test_step3_target_selection();
    all_passed &= test_step4_diagnostic_command();
    
    printf("\n=== TEST RESULTS ===\n");
    if (all_passed) {
        printf("ALL TESTS PASSED! ✓\n");
        printf("DMA controller ready for boot sector read\n");
        return 0;
    } else {
        printf("SOME TESTS FAILED! ✗\n");
        return 1;
    }
}
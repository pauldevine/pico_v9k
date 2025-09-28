/*
 * dma_fast.c - Optimized DMA register implementation using direct memory mapping
 * This replaces the switch-based register access with direct memory indexing
 */

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dma.h"
#include "sasi.h"
#include "logging.h"

// DMA register space spans 0x100 bytes (0xEF300 - 0xEF3FF)
#define DMA_REGISTER_SPACE_SIZE 0x100

// Function pointer types for register handlers
typedef void (*reg_write_handler_t)(dma_registers_t *dma, uint8_t value);
typedef uint8_t (*reg_read_handler_t)(dma_registers_t *dma);

// Register handler structure
typedef struct {
    reg_read_handler_t read;
    reg_write_handler_t write;
    uint8_t *direct_ptr;  // Direct pointer for simple register access
} reg_handler_t;

// Forward declarations of handler functions
static void control_write(dma_registers_t *dma, uint8_t value);
static void data_write(dma_registers_t *dma, uint8_t value);
static void status_write(dma_registers_t *dma, uint8_t value);
static void addr_l_write(dma_registers_t *dma, uint8_t value);
static void addr_m_write(dma_registers_t *dma, uint8_t value);
static void addr_h_write(dma_registers_t *dma, uint8_t value);

static uint8_t control_read(dma_registers_t *dma);
static uint8_t data_read(dma_registers_t *dma);
static uint8_t status_read(dma_registers_t *dma);
static uint8_t addr_l_read(dma_registers_t *dma);
static uint8_t addr_m_read(dma_registers_t *dma);
static uint8_t addr_h_read(dma_registers_t *dma);

// Global register handler table - aligned to 256-byte boundary for fast indexing
static reg_handler_t register_handlers[DMA_REGISTER_SPACE_SIZE] __attribute__((aligned(256)));
static bool handlers_initialized = false;

// Initialize the register handler table
static void init_register_handlers(dma_registers_t *dma) {
    if (handlers_initialized) return;
    
    // Clear all handlers first
    memset(register_handlers, 0, sizeof(register_handlers));
    
    // Set up handlers for mapped registers
    // Control register (0x00-0x0F) - write-only
    for (int i = 0x00; i < 0x10; i++) {
        register_handlers[i].read = control_read;
        register_handlers[i].write = control_write;
    }
    
    // Data register (0x10-0x1F)
    for (int i = 0x10; i < 0x20; i++) {
        register_handlers[i].read = data_read;
        register_handlers[i].write = data_write;
    }
    
    // Status register (0x20-0x2F) - read-only
    for (int i = 0x20; i < 0x30; i++) {
        register_handlers[i].read = status_read;
        register_handlers[i].write = status_write;
    }
    
    // DMA address registers (0x80-0xFF with mirroring)
    for (int i = 0x80; i < 0xA0; i++) {
        register_handlers[i].read = addr_l_read;
        register_handlers[i].write = addr_l_write;
        register_handlers[i].direct_ptr = &dma->dma_address.bytes.low;
    }
    
    for (int i = 0xA0; i < 0xC0; i++) {
        register_handlers[i].read = addr_m_read;
        register_handlers[i].write = addr_m_write;
        register_handlers[i].direct_ptr = &dma->dma_address.bytes.mid;
    }
    
    for (int i = 0xC0; i < 0x100; i++) {
        register_handlers[i].read = addr_h_read;
        register_handlers[i].write = addr_h_write;
        register_handlers[i].direct_ptr = &dma->dma_address.bytes.high;
    }
    
    handlers_initialized = true;
}

// Handler implementations

static void control_write(dma_registers_t *dma, uint8_t value) {
    bool prev_sel = (dma->control & DMA_SELECT_BIT) != 0;
    dma->control = value;

    if (value & DMA_RESET_BIT) {
        dma_device_reset(dma);
        return;
    }

    if (value & DMA_ON_LATCH_BIT) {
        dma->state.dma_enabled = (value & DMA_ON_VALUE_BIT);
    }
    
    dma->state.dma_dir_in = (value & DMA_WR_MODE_BIT);
    
    bool now_sel = (value & DMA_SELECT_BIT) != 0;
    if (now_sel && !prev_sel) {
        dma->bus_ctrl |= SASI_SEL_BIT;
        dma->selected_target = (dma->command & 0x07);
        dma->bus_ctrl |= SASI_BSY_BIT;
    } else if (!now_sel && prev_sel) {
        dma->bus_ctrl &= ~SASI_SEL_BIT;
        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
        dma->bus_ctrl &= ~SASI_INP_BIT;
        dma->state.non_dma_req = 1;
        dma_update_interrupts(dma, true);
    }
}

static uint8_t control_read(dma_registers_t *dma) {
    return 0xFF; // Write-only register
}

static void data_write(dma_registers_t *dma, uint8_t value) {
    dma->command = value;

    if (dma->control & DMA_SELECT_BIT) {
        dma->selected_target = (value & 0x07);
        dma->bus_ctrl &= ~SASI_SEL_BIT;
        dma->control &= ~DMA_SELECT_BIT;
    }

    bool pending_non_dma = dma->state.non_dma_req;
    bool host_to_device_phase = !(dma->bus_ctrl & SASI_INP_BIT);

    if (pending_non_dma && host_to_device_phase) {
        dma->state.non_dma_req = 0;
        dma->state.asserting_ack = 1;
        dma->bus_ctrl |= SASI_ACK_BIT;
    }

    if (dma->bus_ctrl & SASI_CTL_BIT) {
        handle_sasi_command_byte(dma, value);
    }
}

static uint8_t data_read(dma_registers_t *dma) {
    uint8_t data = 0xff;
    
    if (dma->state.non_dma_req) {
        if (dma->bus_ctrl & SASI_MSG_BIT) {
            data = 0x00;
            dma->state.non_dma_req = 0;
            dma->bus_ctrl &= ~SASI_REQ_BIT;
            dma->bus_ctrl &= ~(SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT);
            dma->bus_ctrl &= ~SASI_BSY_BIT;
            dma_update_interrupts(dma, false);
        } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) == (SASI_CTL_BIT | SASI_INP_BIT)) {
            data = dma->status;
            dma->bus_ctrl |= (SASI_MSG_BIT | SASI_REQ_BIT);
            dma->state.non_dma_req = 1;
            dma_update_interrupts(dma, true);
        } else {
            data = dma->command;
            dma->state.non_dma_req = 0;
        }
    }
    
    return data;
}

static void status_write(dma_registers_t *dma, uint8_t value) {
    dma->status = value;
}

static uint8_t status_read(dma_registers_t *dma) {
    uint8_t data = ((dma->bus_ctrl & SASI_INP_BIT) ? SASI_INP_BIT : 0) |
                   ((dma->bus_ctrl & SASI_CTL_BIT) ? SASI_CTL_BIT : 0) |
                   ((dma->bus_ctrl & SASI_BSY_BIT) ? SASI_BSY_BIT : 0) |
                   ((dma->bus_ctrl & SASI_REQ_BIT) ? SASI_REQ_BIT : 0) |
                   ((dma->bus_ctrl & SASI_MSG_BIT) ? SASI_MSG_BIT : 0);
    
    dma_update_interrupts(dma, false);
    return data;
}

// Simple address register handlers - use direct memory access when possible
static void addr_l_write(dma_registers_t *dma, uint8_t value) {
    dma->dma_address.full = (dma->dma_address.full & ~0xFF) | value;
}

static uint8_t addr_l_read(dma_registers_t *dma) {
    return dma->dma_address.full & 0xFF;
}

static void addr_m_write(dma_registers_t *dma, uint8_t value) {
    dma->dma_address.full = (dma->dma_address.full & ~0xFF00) | (value << 8);
}

static uint8_t addr_m_read(dma_registers_t *dma) {
    return (dma->dma_address.full >> 8) & 0xFF;
}

static void addr_h_write(dma_registers_t *dma, uint8_t value) {
    dma->dma_address.full = (dma->dma_address.full & ~0x0F0000) | ((value & 0x0F) << 16);
}

static uint8_t addr_h_read(dma_registers_t *dma) {
    return (dma->dma_address.full >> 16) & 0x0F;
}

// Optimized IRQ handler using direct indexing
void __time_critical_func(registers_irq_handler_fast)() {
    gpio_put(DEBUG_PIN, 1);
    
    // Get raw value from PIO FIFO
    uint32_t raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);
    
    // Start cycle counter
    uint32_t start_cycles = systick_hw->cvr;
    
    // Extract fields
    uint32_t address = raw_value & 0xFFFFF;
    uint8_t data = (raw_value >> 20) & 0xFF;
    bool read_flag = (raw_value >> 28) & 0x01;
    
    // Quick bounds check
    if (address < DMA_REGISTER_BASE || address >= (DMA_REGISTER_BASE + DMA_REGISTER_SPACE_SIZE)) {
        gpio_put(DEBUG_PIN, 0);
        return;
    }
    
    // Calculate offset - this is now just subtraction, no switch needed
    uint8_t offset = address - DMA_REGISTER_BASE;
    
    // Get DMA registers
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        gpio_put(DEBUG_PIN, 0);
        return;
    }
    
    // Ensure handlers are initialized
    if (!handlers_initialized) {
        init_register_handlers(dma);
    }
    
    // Direct index into handler table - this is the key optimization
    reg_handler_t *handler = &register_handlers[offset];
    
    if (read_flag) {
        // For simple address registers, use direct pointer if available
        if (handler->direct_ptr && offset >= 0x80) {
            data = *handler->direct_ptr;
        } else if (handler->read) {
            data = handler->read(dma);
        } else {
            data = 0xFF;
        }
        
        // Send data back to 8088
        uint32_t pindirs_and_data = (0xFF << 8) | (data & 0xFF);
        pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, pindirs_and_data);
    } else {
        // For writes, use direct pointer for simple registers
        if (handler->direct_ptr && offset >= 0x80) {
            *handler->direct_ptr = data;
        } else if (handler->write) {
            handler->write(dma, data);
        }
    }
    
    // Measure cycles
    uint32_t end_cycles = systick_hw->cvr;
    uint32_t cycles_used = (start_cycles >= end_cycles) ? 
                          (start_cycles - end_cycles) : 
                          (start_cycles + (0x00FFFFFF - end_cycles));
    
    // Only log if it takes more than 80ns (16 cycles at 200MHz)
    if (cycles_used > 16) {
        fast_log("Slow access: %lu cycles (%.1f ns) for offset 0x%02x\n", 
                cycles_used, cycles_used * 5.0, offset);
    }
    
    gpio_put(DEBUG_PIN, 0);
}

// Wrapper functions to maintain API compatibility
void dma_write_register_fast(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value) {
    if (!handlers_initialized) {
        init_register_handlers(dma);
    }
    
    // Apply address masking as per MAME
    if (offset >= 0x80) {
        offset &= ~0x1f;
    } else {
        offset &= ~0xf;
    }
    
    if (offset < DMA_REGISTER_SPACE_SIZE) {
        reg_handler_t *handler = &register_handlers[offset];
        if (handler->write) {
            handler->write(dma, value);
        }
    }
}

uint8_t dma_read_register_fast(dma_registers_t *dma, dma_reg_offsets_t offset) {
    if (!handlers_initialized) {
        init_register_handlers(dma);
    }
    
    // Apply address masking as per MAME
    if (offset >= 0x80) {
        offset &= ~0x1f;
    } else {
        offset &= ~0xf;
    }
    
    if (offset < DMA_REGISTER_SPACE_SIZE) {
        reg_handler_t *handler = &register_handlers[offset];
        if (handler->read) {
            return handler->read(dma);
        }
    }
    
    return 0xFF;
}
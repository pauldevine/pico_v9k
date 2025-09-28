/*
 * dma_ultra_fast.c - Ultra-optimized DMA register implementation
 * 
 * This implementation eliminates function pointers for the most common case:
 * address registers (0x80-0xFF). These are mapped directly to memory.
 * Only complex registers (0x00-0x2F) use handler functions.
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

// Define DEBUG_PIN if not already defined
#ifndef DEBUG_PIN
#define DEBUG_PIN 45  // Using UART_RX_PIN as debug output
#endif

// For pins > 31, we need to use different registers
#if DEBUG_PIN >= 32
#define DEBUG_PIN_MASK (1u << (DEBUG_PIN - 32))
#define SIO_GPIO_OUT_SET_REG (SIO_BASE + SIO_GPIO_HI_OUT_SET_OFFSET)
#define SIO_GPIO_OUT_CLR_REG (SIO_BASE + SIO_GPIO_HI_OUT_CLR_OFFSET)
#else
#define DEBUG_PIN_MASK (1u << DEBUG_PIN)
#define SIO_GPIO_OUT_SET_REG (SIO_BASE + SIO_GPIO_OUT_SET_OFFSET)
#define SIO_GPIO_OUT_CLR_REG (SIO_BASE + SIO_GPIO_OUT_CLR_OFFSET)
#endif

// Direct memory mapping for simple registers (0x80-0xFF are address registers)
// Aligned to cache line for optimal performance
static uint8_t register_memory[256] __attribute__((aligned(64)));

// Keep register handlers only for complex registers (0x00-0x2F)
typedef struct {
    void (*write)(dma_registers_t *dma, uint8_t value);
    uint8_t (*read)(dma_registers_t *dma);
} complex_handler_t;

static complex_handler_t complex_handlers[0x30];
static bool initialized = false;

// Complex register handlers (only for 0x00-0x2F)
static void handle_control_write(dma_registers_t *dma, uint8_t value) {
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

static void handle_data_write(dma_registers_t *dma, uint8_t value) {
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

static uint8_t handle_data_read(dma_registers_t *dma) {
    if (!dma->state.non_dma_req) {
        return 0xff;
    }
    
    if (dma->bus_ctrl & SASI_MSG_BIT) {
        dma->state.non_dma_req = 0;
        dma->bus_ctrl &= ~SASI_REQ_BIT;
        dma->bus_ctrl &= ~(SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT | SASI_BSY_BIT);
        dma_update_interrupts(dma, false);
        return 0x00;
    }
    
    if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) == (SASI_CTL_BIT | SASI_INP_BIT)) {
        dma->bus_ctrl |= (SASI_MSG_BIT | SASI_REQ_BIT);
        dma->state.non_dma_req = 1;
        dma_update_interrupts(dma, true);
        return dma->status;
    }
    
    dma->state.non_dma_req = 0;
    return dma->command;
}

static uint8_t handle_status_read(dma_registers_t *dma) {
    uint8_t data = ((dma->bus_ctrl & SASI_INP_BIT) ? SASI_INP_BIT : 0) |
                   ((dma->bus_ctrl & SASI_CTL_BIT) ? SASI_CTL_BIT : 0) |
                   ((dma->bus_ctrl & SASI_BSY_BIT) ? SASI_BSY_BIT : 0) |
                   ((dma->bus_ctrl & SASI_REQ_BIT) ? SASI_REQ_BIT : 0) |
                   ((dma->bus_ctrl & SASI_MSG_BIT) ? SASI_MSG_BIT : 0);
    
    dma_update_interrupts(dma, false);
    return data;
}

static void init_ultra_fast(dma_registers_t *dma) {
    if (initialized) return;
    
    // Clear everything
    memset(register_memory, 0xFF, sizeof(register_memory));
    memset(complex_handlers, 0, sizeof(complex_handlers));
    
    // Set up complex handlers for control registers
    for (int i = 0x00; i < 0x10; i++) {
        complex_handlers[i].write = handle_control_write;
        complex_handlers[i].read = NULL;  // Write-only
    }
    
    // Data register
    for (int i = 0x10; i < 0x20; i++) {
        complex_handlers[i].write = handle_data_write;
        complex_handlers[i].read = handle_data_read;
    }
    
    // Status register
    for (int i = 0x20; i < 0x30; i++) {
        complex_handlers[i].write = NULL;  // Read-only
        complex_handlers[i].read = handle_status_read;
    }
    
    // Initialize address registers in direct memory
    // These map directly to the DMA address bytes
    for (int i = 0x80; i < 0xA0; i++) {
        register_memory[i] = 0;  // Low byte
    }
    for (int i = 0xA0; i < 0xC0; i++) {
        register_memory[i] = 0;  // Mid byte
    }
    for (int i = 0xC0; i < 0x100; i++) {
        register_memory[i] = 0;  // High byte (only 4 bits used)
    }
    
    initialized = true;
}

// Ultra-optimized IRQ handler
void __time_critical_func(registers_irq_handler_ultra)() {
    gpio_put(DEBUG_PIN, 1);
    
    // Get raw value from PIO FIFO - this is our bottleneck
    uint32_t raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);
    
    // Extract fields using bit operations
    uint32_t address = raw_value & 0xFFFFF;
    uint8_t data = (raw_value >> 20) & 0xFF;
    uint32_t read_flag = raw_value & 0x10000000;  // Bit 28
    
    // Quick bounds check - optimize for common case
    uint32_t offset = address - DMA_REGISTER_BASE;
    if (offset >= 0x100) {
        gpio_put(DEBUG_PIN, 0);
        return;
    }
    
    // Get DMA registers
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        gpio_put(DEBUG_PIN, 0);
        return;
    }
    
    // Ensure initialization
    if (!initialized) {
        init_ultra_fast(dma);
    }
    
    // Fast path for address registers (0x80-0xFF) - most common case
    if (offset >= 0x80) {
        // Apply MAME-style masking
        offset &= ~0x1f;
        
        if (read_flag) {
            // Direct memory read
            data = register_memory[offset];
            
            // But we need to get actual value from DMA structure
            if (offset == 0x80) {
                data = dma->dma_address.full & 0xFF;
            } else if (offset == 0xA0) {
                data = (dma->dma_address.full >> 8) & 0xFF;
            } else if (offset == 0xC0) {
                data = (dma->dma_address.full >> 16) & 0x0F;
            }
        } else {
            // Direct memory write with DMA structure update
            register_memory[offset] = data;
            
            if (offset == 0x80) {
                dma->dma_address.full = (dma->dma_address.full & ~0xFF) | data;
            } else if (offset == 0xA0) {
                dma->dma_address.full = (dma->dma_address.full & ~0xFF00) | (data << 8);
            } else if (offset == 0xC0) {
                dma->dma_address.full = (dma->dma_address.full & ~0x0F0000) | ((data & 0x0F) << 16);
            }
        }
    } else {
        // Complex registers (0x00-0x2F) - use handlers
        offset &= ~0xf;  // MAME-style masking
        
        if (offset < 0x30) {
            if (read_flag) {
                if (complex_handlers[offset].read) {
                    data = complex_handlers[offset].read(dma);
                } else {
                    data = 0xFF;
                }
            } else {
                if (complex_handlers[offset].write) {
                    complex_handlers[offset].write(dma, data);
                }
            }
        }
    }
    
    // Send data back to 8088 if this was a read
    if (read_flag) {
        uint32_t pindirs_and_data = (0xFF << 8) | data;
        #ifndef BENCHMARK_MODE
        pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, pindirs_and_data);
        #endif
    }
    
    gpio_put(DEBUG_PIN, 0);
}

// Even more optimized version with inline assembly for critical path
void __time_critical_func(registers_irq_handler_ultra_asm)() {
    register uint32_t raw_value asm("r4");
    register uint32_t offset asm("r5");
    register uint32_t data asm("r6");

    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from PIO FIFO
    raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);

    // Extract offset
    offset = (raw_value & 0xFFFFF) - DMA_REGISTER_BASE;

    // Bounds check
    if (offset >= 0x100) {
        *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
        return;
    }

    // Get DMA registers
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
        return;
    }
    
    // Check read flag (bit 28)
    if (raw_value & 0x10000000) {
        // Read path
        if (offset >= 0x80) {
            // Fast path for address registers
            offset &= ~0x1f;
            
            if (offset == 0x80) {
                data = dma->dma_address.full & 0xFF;
            } else if (offset == 0xA0) {
                data = (dma->dma_address.full >> 8) & 0xFF;
            } else if (offset == 0xC0) {
                data = (dma->dma_address.full >> 16) & 0x0F;
            } else {
                data = 0xFF;
            }
        } else {
            // Complex register read - fall back to standard handler
            offset &= ~0xf;
            data = dma_read_register(dma, offset);
        }
        
        // Send data back
        #ifndef BENCHMARK_MODE
        pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, (0xFF00 | data));
        #endif
    } else {
        // Write path
        data = (raw_value >> 20) & 0xFF;
        
        if (offset >= 0x80) {
            // Fast path for address registers
            offset &= ~0x1f;
            
            if (offset == 0x80) {
                dma->dma_address.full = (dma->dma_address.full & ~0xFF) | data;
            } else if (offset == 0xA0) {
                dma->dma_address.full = (dma->dma_address.full & ~0xFF00) | (data << 8);
            } else if (offset == 0xC0) {
                dma->dma_address.full = (dma->dma_address.full & ~0x0F0000) | ((data & 0x0F) << 16);
            }
        } else {
            // Complex register write
            offset &= ~0xf;
            dma_write_register(dma, offset, data);
        }
    }
    
    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}
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
#include "debug_queue.h"

// Define DEBUG_PIN if not already defined
#ifndef DEBUG_PIN
#define DEBUG_PIN 44
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
// Place in time_critical section to ensure it's in RAM
static uint8_t register_memory[256] __attribute__((aligned(64), section(".time_critical.register_memory")));

#define DEFER_QUEUE_SIZE 128
#define DEFER_QUEUE_MASK (DEFER_QUEUE_SIZE - 1)
static volatile uint32_t deferred_queue[DEFER_QUEUE_SIZE];
static volatile uint32_t deferred_head = 0;
static volatile uint32_t deferred_tail = 0;

// Forward declarations
static bool initialized = false;
static void init_ultra_fast(dma_registers_t *dma);
static inline uint32_t mask_offset(uint32_t offset) {
    if (offset >= 0x80) {
        return offset & ~0x1F;
    }
    return offset & ~0x0F;
}

static inline uint8_t compute_status_value(const dma_registers_t *dma) {
    return (uint8_t)((dma->bus_ctrl & SASI_INP_BIT ? SASI_INP_BIT : 0) |
                     (dma->bus_ctrl & SASI_CTL_BIT ? SASI_CTL_BIT : 0) |
                     (dma->bus_ctrl & SASI_BSY_BIT ? SASI_BSY_BIT : 0) |
                     (dma->bus_ctrl & SASI_REQ_BIT ? SASI_REQ_BIT : 0) |
                     (dma->bus_ctrl & SASI_MSG_BIT ? SASI_MSG_BIT : 0));
}

static inline void deferred_enqueue(uint32_t value) {
    uint32_t head = deferred_head;
    uint32_t next = (head + 1) & DEFER_QUEUE_MASK;
    if (next == deferred_tail) {
        return; // queue full, drop event
    }
    deferred_queue[head] = value;
    __asm volatile("" ::: "memory");
    deferred_head = next;
}

static inline bool deferred_try_dequeue(uint32_t *value) {
    uint32_t tail = deferred_tail;
    if (tail == deferred_head) {
        return false;
    }
    *value = deferred_queue[tail];
    deferred_tail = (tail + 1) & DEFER_QUEUE_MASK;
    return true;
}

// Initialization function to pre-warm caches
void registers_irq_handler_ultra_init(void) {
    // Touch all register memory to bring it into cache
    volatile uint8_t dummy = 0;
    volatile uint32_t dummy32 = 0;

    // First pass: Initialize all register memory
    for (int i = 0; i < 256; i++) {
        register_memory[i] = 0xFF;
    }
    register_memory[REG_STATUS] = 0x00;
    register_memory[0x30] = 0x00;

    // Second pass: Read all register memory multiple times to ensure caching
    for (int iter = 0; iter < 3; iter++) {
        for (int i = 0; i < 256; i++) {
            dummy = register_memory[i];
            register_memory[i] = (uint8_t)i;  // Write pattern
        }
    }

    // Touch the most commonly accessed addresses multiple times
    for (int iter = 0; iter < 10; iter++) {
        register_memory[0x80] = 0;
        register_memory[0xA0] = 0;
        register_memory[0xC0] = 0;
        dummy = register_memory[0x80];
        dummy = register_memory[0xA0];
        dummy = register_memory[0xC0];
    }

    // Ensure complex handlers are initialized
    if (!initialized) {
        dma_registers_t *dma = dma_get_registers();
        if (dma) {
            init_ultra_fast(dma);
        }
    }

    // Touch critical SIO registers to pre-cache them
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG;
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG;

    // Touch PIO registers
    dummy32 = PIO_REGISTERS->rxf[REGISTERS_SM];
    dummy32 = PIO_REGISTERS->txf[REGISTERS_SM];
    dummy32 = PIO_REGISTERS->fstat;

    (void)dummy; // Prevent unused variable warning
    (void)dummy32;

    deferred_head = deferred_tail = 0;
}

// Keep register handlers only for complex registers (0x00-0x3F)
typedef struct {
    void (*write)(dma_registers_t *dma, uint8_t value);
    uint8_t (*read)(dma_registers_t *dma);
} complex_handler_t;

static complex_handler_t complex_handlers[0x40];

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
    
    // Status register (include 0x20 and alias at 0x30)
    for (int i = 0x20; i < 0x30; i++) {
        complex_handlers[i].write = NULL;  // Read-only
        complex_handlers[i].read = handle_status_read;
    }
    for (int i = 0x30; i < 0x40; i++) {
        complex_handlers[i].write = NULL;  // Read-only alias
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

    uint32_t raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);
    uint32_t payload_type = dma_fifo_payload_type(raw_value);

    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        gpio_put(DEBUG_PIN, 0);
        return;
    }

    if (!initialized) {
        init_ultra_fast(dma);
    }

    switch (payload_type) {
        case FIFO_PREFETCH_ADDRESS: {
            uint32_t address = dma_fifo_prefetch_address(raw_value);
            uint32_t offset = address - DMA_REGISTER_BASE;
            if (offset >= 0x100) {
                break;
            }

            uint32_t masked_offset = mask_offset(offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            uint8_t value;
            if (masked_offset == REG_STATUS) {
                value = compute_status_value(dma);
                register_memory[REG_STATUS] = value;
                register_memory[0x30] = value;
            } else {
                value = register_memory[masked_offset];
            }
    #ifndef BENCHMARK_MODE
            // Push data byte to bus_output_helper (not board_registers!)
            // bus_output_helper will output this on BD0-BD7 when it receives IRQ 1
            PIO helper_pio = dma_get_bus_helper_pio();
            int helper_sm = dma_get_bus_helper_sm();
            pio_sm_put_blocking(helper_pio, helper_sm, (uint32_t)(value & 0xFFu));
    #endif
            break;
        }

        case FIFO_READ_COMMIT: {
            deferred_enqueue(raw_value);
            break;
        }

        case FIFO_WRITE_VALUE: {
            uint32_t address = dma_fifo_write_address(raw_value);
            uint32_t offset = address - DMA_REGISTER_BASE;
            if (offset >= 0x100) {
                break;
            }

            uint32_t masked_offset = mask_offset(offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            uint8_t write_data = dma_fifo_write_data(raw_value);
            if (masked_offset == REG_ADDR_H) {
                write_data &= 0x0F;
            }
            register_memory[masked_offset] = write_data;
            if (masked_offset == REG_STATUS) {
                register_memory[0x30] = write_data;
            }

            deferred_enqueue(raw_value);
            break;
        }

        default:
            break;
    }
    gpio_put(DEBUG_PIN, 0);
}

// Even more optimized version with inline assembly for critical path
void __time_critical_func(registers_irq_handler_ultra_asm)() {
    uint32_t raw_value;
    uint32_t offset;
    uint8_t data = 0xFF;

    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from PIO FIFO
    raw_value = pio_sm_get(PIO_REGISTERS, REGISTERS_SM);

    uint32_t payload_type = dma_fifo_payload_type(raw_value);
    dma_registers_t *dma = dma_get_registers();
    if (!dma) {
        *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
        return;
    }

    switch (payload_type) {
        case FIFO_PREFETCH_ADDRESS: {
            uint32_t address = dma_fifo_prefetch_address(raw_value);
            offset = address - DMA_REGISTER_BASE;

            if (offset >= 0x100) {
                break;
            }

            if ((offset >= 0x40 && offset < 0x80) || offset >= 0xE0) {
                break;
            }

            uint32_t masked_offset = offset;
            masked_offset = mask_offset(masked_offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            if (masked_offset >= 0x80) {
                if (masked_offset == 0x80) {
                    data = dma->dma_address.bytes.low;
                } else if (masked_offset == 0xA0) {
                    data = dma->dma_address.bytes.mid;
                } else if (masked_offset == 0xC0) {
                    data = dma->dma_address.bytes.high & 0x0F;
                } else {
                    data = 0xFF;
                }
            } else {
                data = dma_read_register(dma, masked_offset);
            }

            #ifndef BENCHMARK_MODE
            // Push data byte to bus_output_helper (not board_registers!)
            PIO helper_pio = dma_get_bus_helper_pio();
            int helper_sm = dma_get_bus_helper_sm();
            pio_sm_put_blocking(helper_pio, helper_sm, (uint32_t)data);
            #endif
            break;
        }

        case FIFO_READ_COMMIT: {
            deferred_enqueue(raw_value);
            break;
        }

        case FIFO_WRITE_VALUE: {
            uint32_t address = dma_fifo_write_address(raw_value);
            offset = address - DMA_REGISTER_BASE;

            if (offset >= 0x100) {
                break;
            }

            if ((offset >= 0x40 && offset < 0x80) || offset >= 0xE0) {
                break;
            }

            uint32_t masked_offset = offset;
            masked_offset = mask_offset(masked_offset);
            if (masked_offset == 0x30) {
                masked_offset = REG_STATUS;
            }

            data = dma_fifo_write_data(raw_value);

            if (masked_offset >= 0x80) {
                if (masked_offset == 0x80) {
                    dma->dma_address.bytes.low = data;
                } else if (masked_offset == 0xA0) {
                    dma->dma_address.bytes.mid = data;
                } else if (masked_offset == 0xC0) {
                    dma->dma_address.bytes.high = data & 0x0F;
                }
            } else {
                dma_write_register(dma, masked_offset, data);
            }

            deferred_enqueue(raw_value);
            break;
        }

        default:
            break;
    }

    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

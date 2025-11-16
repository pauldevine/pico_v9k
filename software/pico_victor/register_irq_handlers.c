/*
 * irq_handlers.c - Ultra-optimized cached DMA register handler
 *
 * This implementation returns cached values immediately to meet the 200ns timing
 * requirement, then enqueues register operations for deferred processing.
 */

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <string.h>
#include "dma.h"
#include "reg_queue_processor.h"
#include "logging.h"
#include "register_irq_handlers.h"
#include "fifo_helpers.h"



// External cache and queue instances from dma_defer.h - place in time_critical section for fast access
extern defer_queue_t defer_queue;
extern cached_registers_t cached_regs;




// Pre-warm caches by touching critical memory locations
void init_register_irq_handlers(void) {
    // Initialize deferred processing system
    dma_defer_init();

    // Get cached registers and pre-warm
    cached_registers_t *cached = defer_get_cached_registers();
    volatile uint8_t dummy = 0;

    // Touch all cached values multiple times
    for (int iter = 0; iter < 3; iter++) {
        for (int i = 0; i < 256; i++) {
            dummy = cached->values[i];
        }
    }

    // Touch the most commonly accessed addresses
    for (int iter = 0; iter < 10; iter++) {
        dummy = cached->values[0x80];
        dummy = cached->values[0xA0];
        dummy = cached->values[0xC0];
        dummy = cached->values[0x20];
        dummy = cached->values[0x30];
    }

    // Touch critical SIO registers
    volatile uint32_t dummy32;
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG;
    dummy32 = *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG;

    // Touch PIO registers
    dummy32 = PIO_REGISTERS->rxf[REG_SM_CONTROL];
    dummy32 = PIO_REGISTERS->txf[REG_SM_CONTROL];
    dummy32 = PIO_REGISTERS->fstat;

    (void)dummy;
    (void)dummy32;
}

// IRQ handler for board_registers_output handles FIFO_REG_READ
void __time_critical_func(register_read_irq_isr)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data = 0;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_read_count;

    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Get value from bus_output_helper PIO FIFO
    raw_value = PIO_OUTPUT->rxf[REG_SM_OUTPUT];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    bool enque_result = false;

    // Handle different payload types
    if (payload_type==FIFO_REG_READ) {
            //case 8088 is reading a pico register
            // bus_output_helper pushes REG_READ when 8088 reads a register, some side effects can handled later
            uint32_t address = board_fifo_read_address(raw_value);
            masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
            masked_offset &= 0xFF;

            enque_result = true;

            fifo_read_count++;
            data = cached->values[masked_offset];

            // Debug: Check if PIO_OUTPUT and REG_SM_OUTPUT are correct
            fast_log("REG_READ: offset=0x%02x data=0x%02x pio=%p sm=%d\n",
                     masked_offset, data, PIO_OUTPUT, REG_SM_OUTPUT);

            // Check TX FIFO level before pushing
            uint32_t tx_before = pio_sm_get_tx_fifo_level(PIO_OUTPUT, REG_SM_OUTPUT);

            // Push data byte to bus_output_helper for output on BD0-BD7
            pio_sm_put_blocking(PIO_OUTPUT, REG_SM_OUTPUT, (uint32_t)(data & 0xFFu));

            // Check TX FIFO level after pushing
            uint32_t tx_after = pio_sm_get_tx_fifo_level(PIO_OUTPUT, REG_SM_OUTPUT);
            fast_log("REG_READ_PUSH_RESULT: tx_before=%d tx_after=%d\n", tx_before, tx_after);
    } else {
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fast_log("REG_READ: Unknown payload type: 0x%02x raw=0x%08x\n", payload_type, raw_value);
    }

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);

    if (enque_result) {
        // Enqueue for deferred processing (fast inline version)
        defer_queue_t *queue = &defer_queue;
        uint32_t head = queue->head;
        uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

        if (next != queue->tail) {
            queue->entries[head].raw_value = raw_value;
            __asm volatile("dmb" ::: "memory");
            queue->head = next;
        }
    }

    // Clear debug pin
    *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
}

// IRQ handler for board_registers_control SM handles FIFO_WRITE_VALUE
void __time_critical_func(register_write_irq_isr)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_read_count;

    // Get value from board_registers_control PIO FIFO
    raw_value = PIO_REGISTERS->rxf[REG_SM_CONTROL];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    // board_registers only pushes WRITE_VALUE payloads
    if (payload_type == FIFO_WRITE_VALUE) {
        //case 8088 is writing to a pico register
        uint32_t address = dma_fifo_write_address(raw_value);
        masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
        masked_offset &= 0xFF;

        data = dma_fifo_write_data(raw_value);
        if (masked_offset == REG_ADDR_H) {
            data &= 0x0F;
        }

        cached->values[masked_offset] = data;
        if (masked_offset == REG_STATUS) {
            cached->values[0x30] = data;
        }
        trace_flags |= FIFO_TRACE_FLAG_WRITE;

        // Enqueue for deferred processing (fast inline version)
        defer_queue_t *queue = &defer_queue;
        uint32_t head = queue->head;
        uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

        if (next != queue->tail) {
            queue->entries[head].raw_value = raw_value;
            __asm volatile("dmb" ::: "memory");
            queue->head = next;
        }
    } else {
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fast_log("BOARD_REG: Unexpected payload type: 0x%02x raw=0x%08x\n", payload_type, raw_value);
        data = 0;
    }

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);
}
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

    // Get value from bus_output_helper PIO FIFO
    raw_value = PIO_REGISTERS->rxf[REG_SM_CONTROL];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    bool enque_result = false;

    // Handle different payload types
    // NOTE: register_output PIO can ONLY produce FIFO_REG_READ (0x00) via "in null, 2"
    // Any other payload type indicates stale/corrupted FIFO data - drop silently in ISR
    if (payload_type != FIFO_REG_PREFETCH) {
        // Invalid payload type - don't respond, just trace for deferred analysis
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fifo_trace_record(raw_value, payload_type, pending_before, (uint8_t)fifo_read_count, trace_flags, 0);
        return;
    }

    // Valid REG_READ - 8088 is reading a pico register
    uint32_t address = board_fifo_read_address(raw_value);

    // Validate address is within DMA register range (0xEF300-0xEF3FF)
    if (address < DMA_REGISTER_BASE || address >= (DMA_REGISTER_BASE + 0x100)) {
        // Address out of range - stale/invalid data, drop silently
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fifo_trace_record(raw_value, payload_type, pending_before, (uint8_t)fifo_read_count, trace_flags, 0);
        return;
    }

    masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
    masked_offset &= 0xFF;

    enque_result = true;

    fifo_read_count++;
    data = cached->values[masked_offset];

    // pio outputs 8 bits of data first then 8 bits of pindirs (0xFF for output)
    uint32_t payload = (0xFF << 8) | (data & 0xFF);

    // Push data byte to bus_output_helper for output on BD0-BD7
    pio_sm_put_blocking(PIO_REGISTERS, REG_SM_CONTROL, payload);

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
    if (payload_type != FIFO_REG_WRITE) {
        // Invalid payload type - drop silently and trace for deferred analysis
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fifo_trace_record(raw_value, payload_type, pending_before, (uint8_t)fifo_read_count, trace_flags, 0);
        return;
    }

    // Valid WRITE_VALUE - 8088 is writing to a pico register
    uint32_t address = dma_fifo_write_address(raw_value);

    // Validate address is within DMA register range (0xEF300-0xEF3FF)
    if (address < DMA_REGISTER_BASE || address >= (DMA_REGISTER_BASE + 0x100)) {
        // Address out of range - stale/invalid data, drop silently
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
        fifo_trace_record(raw_value, payload_type, pending_before, (uint8_t)fifo_read_count, trace_flags, 0);
        return;
    }

    masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE);
    masked_offset &= 0xFF;

    data = dma_fifo_write_data(raw_value);
    if (masked_offset == REG_ADDR_H) {
        data &= 0x0F;
    }

    // For CONTROL register writes, check if SELECT is being asserted
    // and immediately update cached STATUS with BSY so the Victor sees
    // the device respond before deferred processing runs
    if (masked_offset == REG_CONTROL) {
        uint8_t prev_ctrl = cached->values[REG_CONTROL];
        bool prev_sel = (prev_ctrl & DMA_SELECT_BIT) != 0;
        bool now_sel = (data & DMA_SELECT_BIT) != 0;

        if (now_sel && !prev_sel) {
            // SELECT rising edge - target responds with BSY
            cached->values[REG_STATUS] = SASI_BSY_BIT;
            cached->values[0x30] = SASI_BSY_BIT;
        } else if (!now_sel && prev_sel) {
            // SELECT falling edge - enter command phase (BSY|REQ|CTL)
            uint8_t cmd_phase = SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT;
            cached->values[REG_STATUS] = cmd_phase;
            cached->values[0x30] = cmd_phase;
        }
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

    uint8_t pending_after = (uint8_t)fifo_read_count;
    fifo_trace_record(raw_value, payload_type, pending_before, pending_after, trace_flags, data);
}
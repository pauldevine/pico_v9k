/*
    * dma_irq_handler.c
    *
    * IRQ handler for DMA read operations from the bus.
    * This handler services FIFO_DMA_READ payloads from the dma_rw_output SM on the PIO_DMA_MASTER pio.
    * It retrieves the data captured from the bus during a DMA read cycle and enqueues it for deferred processing.
    *
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

// IRQ handler dma_rw_ouput.pio, handles DMA_READ (DMA_WRITE doesn't generate IRQs)
void __time_critical_func(dma_read_isr)() {
    uint32_t raw_value;
    static uint32_t masked_offset;
    uint8_t data = 0;
    uint8_t trace_flags = 0;
    uint8_t pending_before = (uint8_t)fifo_pending_prefetch;

    // Set debug pin high
    *(volatile uint32_t *)SIO_GPIO_OUT_SET_REG = DEBUG_PIN_MASK;

    // Check if SM1 RX FIFO is actually non-empty - if empty, this is a spurious IRQ
    if (pio_sm_is_rx_fifo_empty(PIO_DMA_MASTER, DMA_SM_CONTROL)) {
        *(volatile uint32_t *)SIO_GPIO_OUT_CLR_REG = DEBUG_PIN_MASK;
        return;
    }

    // Get value from bus_output_helper PIO FIFO
    raw_value = PIO_DMA_MASTER->rxf[DMA_SM_CONTROL];

    // Extract 2-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Get cached registers pointer
    cached_registers_t *cached = &cached_regs;

    bool enque_result = false;

    // Handle different payload types
    if (payload_type==FIFO_DMA_READ) {
            //case pico is doing a DMA read from the bus
            // bus_output_helper pushes DMA_READ data after capturing from bus
            enque_result = true;
            // Extract the data from the payload (bits 22-29 contain the data byte)
            data = (raw_value >> 22) & 0xFF;
            // Extract address for logging (bits 2-21 contain the 20-bit address)
            uint32_t dma_address = (raw_value >> 2) & 0xFFFFF;
    } else {
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
    }

    uint8_t pending_after = (uint8_t)fifo_pending_prefetch;
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

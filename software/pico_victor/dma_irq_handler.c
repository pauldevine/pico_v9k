/*
    * dma_irq_handler.c
    *
    * IRQ handler for DMA read operations from the bus.
    * This handler services FIFO_DMA_READ payloads from the dma_master SM on PIO_DMA_MASTER.
    * The active DMA data path uses blocking FIFO reads in dma.c, so this IRQ path is
    * diagnostic-only when enabled.
    *
*/

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "dma.h"
#include "fifo_helpers.h"

// IRQ handler dma_rw_ouput.pio, handles DMA_READ (DMA_WRITE doesn't generate IRQs)
void __time_critical_func(dma_read_isr)() {
    uint32_t raw_value;
    uint8_t data = 0;
    uint8_t trace_flags = 0;
    uint8_t pending_before = 0;

    // Get value from bus_output_helper PIO FIFO
    raw_value = PIO_DMA_MASTER->rxf[DMA_SM_CONTROL];

    // Extract 1-bit payload type flag
    uint32_t payload_type = fifo_payload_type(raw_value);

    // Handle different payload types
    if (payload_type == FIFO_DMA_READ) {
        // DMA read response payload carries data in bits 29:22.
        data = (raw_value >> 22) & 0xFF;
    } else {
        trace_flags |= FIFO_TRACE_FLAG_ERROR;
    }

    fifo_trace_record(raw_value, payload_type, pending_before, 0, trace_flags, data);
}

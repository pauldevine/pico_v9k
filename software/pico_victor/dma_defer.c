/*
 * dma_defer.c - Deferred processing implementation for DMA register operations
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"
#include <string.h>
#include "dma_defer.h"
#include "dma.h"
#include "sasi.h"
#include "logging.h"

// Static instances - place in time_critical section for fast access
defer_queue_t defer_queue __attribute__((section(".time_critical.defer_queue")));
cached_registers_t cached_regs __attribute__((section(".time_critical.cached_regs")));

// Initialize the deferred processing system
void dma_defer_init(void) {
    // Clear the queue
    memset(&defer_queue, 0, sizeof(defer_queue));
    defer_queue.head = 0;
    defer_queue.tail = 0;

    // Initialize cached registers with default values
    memset(cached_regs.values, 0xFF, sizeof(cached_regs.values));

    // Set initial values for specific registers
    cached_regs.values[REG_STATUS] = 0x00;
    cached_regs.values[0x30] = 0x00;  // Status alias
    cached_regs.values[REG_ADDR_L] = 0x00;
    cached_regs.values[REG_ADDR_M] = 0x00;
    cached_regs.values[REG_ADDR_H] = 0x00;

    // Pre-warm the cache by touching all values
    volatile uint8_t dummy;
    for (int i = 0; i < 256; i++) {
        dummy = cached_regs.values[i];
    }
    (void)dummy;
}

// Dequeue an entry for processing
bool defer_dequeue(defer_queue_t *queue, defer_entry_t *entry) {
    uint32_t tail = queue->tail;

    // Check if queue is empty
    if (tail == queue->head) {
        return false;
    }

    // Copy entry
    *entry = queue->entries[tail];

    // Memory barrier
    __asm volatile("dmb" ::: "memory");

    // Advance tail
    queue->tail = (tail + 1) & DEFER_QUEUE_MASK;
    return true;
}

// Read operations are handled entirely by the fast cached handler
// We only need to process side effects here, not update cache values
// The cache was already read and returned by the fast handler
void defer_process_read(dma_registers_t *dma, uint32_t raw_value) {
    // Extract address FIFO has 00[address] - REG_READ payload
    fast_log("RAW=0x%08x\n", raw_value);
    uint32_t address = board_fifo_read_address(raw_value);
    uint32_t offset = address - DMA_REGISTER_BASE;
    uint32_t masked_offset = dma_mask_offset(offset);

    // Handle alias for status register
    if (masked_offset == 0x30) {
        masked_offset = REG_STATUS;
    }

    // Bounds check
    if (offset >= 0x100) {
        fast_log("DEFER_READ: out of range offset=0x%02x\n", offset);
        return;
    }

    switch (masked_offset) {
        case REG_STATUS:
            // Clear interrupt on status read (side effect only)
            dma_update_interrupts(dma, false);
            break;

        case REG_DATA:
            // Data register read side effects
            if (dma->state.non_dma_req) {
                if (dma->bus_ctrl & SASI_MSG_BIT) {
                    // Message phase - clear bus
                    dma->state.non_dma_req = 0;
                    dma->bus_ctrl &= ~SASI_REQ_BIT;
                    dma->bus_ctrl &= ~(SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT |
                                        SASI_ACK_BIT | SASI_BSY_BIT);
                    dma_update_interrupts(dma, false);

                    // Update cached status to reflect new bus state
                    uint8_t new_status = 0;
                    cached_regs.values[REG_STATUS] = new_status;
                    cached_regs.values[0x30] = new_status;
                } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) ==
                            (SASI_CTL_BIT | SASI_INP_BIT)) {
                    // Status phase - set message phase
                    dma->bus_ctrl |= (SASI_MSG_BIT | SASI_REQ_BIT);
                    dma->state.non_dma_req = 1;
                    dma_update_interrupts(dma, true);

                    // Update cached status to reflect new bus state
                    uint8_t new_status = SASI_MSG_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT;
                    cached_regs.values[REG_STATUS] = new_status;
                    cached_regs.values[0x30] = new_status;
                } else {
                    dma->state.non_dma_req = 0;
                }
            }
            fast_log("DEFER_READ: offset=0x%02x end_status %d (processed side effects, cache already returned)\n", masked_offset, cached_regs.values[REG_STATUS]    );
            break;

        case REG_ADDR_L:
        case REG_ADDR_M:
        case REG_ADDR_H:
            // No side effects for address register reads
            // The cached value was already returned by the fast handler
            fast_log("DEFER_READ: offset=0x%02x (no action, cache already returned)\n", masked_offset);
            break;

        default:
            // Unknown register
            fast_log("DEFER_READ: Unknown register offset=0x%02x\n", masked_offset);
            break;
    }
}

//Write operations need to update both cache and actual DMA state
//by processing side effects
void defer_process_write(dma_registers_t *dma, uint32_t raw_value) {
    // Extract address FIFO has 10[data][address]
    uint32_t address = dma_fifo_write_address(raw_value);
    uint32_t offset = address - DMA_REGISTER_BASE;
    uint32_t masked_offset = dma_mask_offset(offset);
    uint8_t write_data = dma_fifo_write_data(raw_value);

    // Handle alias for status register
    if (masked_offset == 0x30) {
        masked_offset = REG_STATUS;
    }

    // Bounds check
    if (offset >= 0x100) {
        fast_log("DEFER_WRITE: out of range offset=0x%02x\n", offset);
        return;
    }

    switch (masked_offset) {
        case REG_CONTROL:
            // Control register write
            {
                bool prev_sel = (dma->control & DMA_SELECT_BIT) != 0;
                dma->control = write_data;

                if (write_data & DMA_RESET_BIT) {
                    dma_device_reset(dma);
                    // Reset cached status
                    cached_regs.values[REG_STATUS] = 0x00;
                    cached_regs.values[0x30] = 0x00;
                } else {
                    if (write_data & DMA_ON_LATCH_BIT) {
                        dma->state.dma_enabled = (write_data & DMA_ON_VALUE_BIT) != 0;
                    }

                    dma->state.dma_dir_in = (write_data & DMA_WR_MODE_BIT) != 0;

                    bool now_sel = (write_data & DMA_SELECT_BIT) != 0;
                    if (now_sel && !prev_sel) {
                        // Selection phase starting
                        dma->bus_ctrl |= SASI_SEL_BIT;
                        dma->selected_target = (dma->command & 0x07);
                        dma->bus_ctrl |= SASI_BSY_BIT;
                    } else if (!now_sel && prev_sel) {
                        // Selection phase ending - enter command phase
                        dma->bus_ctrl &= ~SASI_SEL_BIT;
                        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
                        dma->bus_ctrl &= ~SASI_INP_BIT;
                        dma->state.non_dma_req = 1;
                        dma_update_interrupts(dma, true);
                    }
                }
            }  
            fast_log("DEFER: Write CONTROL = 0x%02x\n", write_data);
            break;

        case REG_DATA:
            // Data register write
            dma->command = write_data;
            cached_regs.values[REG_DATA] = write_data;

            if (dma->control & DMA_SELECT_BIT) {
                // During selection, data contains target ID
                dma->selected_target = (write_data & 0x07);
                dma->bus_ctrl &= ~SASI_SEL_BIT;
                dma->control &= ~DMA_SELECT_BIT;
            }

            // Handle command/data byte
            bool pending_non_dma = dma->state.non_dma_req;
            bool host_to_device = !(dma->bus_ctrl & SASI_INP_BIT);

            if (pending_non_dma && host_to_device) {
                dma->state.non_dma_req = 0;
                dma->state.asserting_ack = 1;
                dma->bus_ctrl |= SASI_ACK_BIT;
            }

            if (dma->bus_ctrl & SASI_CTL_BIT) {
                // Command phase - process SASI command byte
                handle_sasi_command_byte(dma, write_data);
            }
            fast_log("DEFER: Write DATA = 0x%02x\n", write_data);
            break;

        case REG_ADDR_L:
            dma->dma_address.bytes.low = write_data;
            cached_regs.values[REG_ADDR_L] = write_data;
            fast_log("DEFER: Write ADDR_L = 0x%02x\n", write_data);
            break;

        case REG_ADDR_M:
            dma->dma_address.bytes.mid = write_data;
            cached_regs.values[REG_ADDR_M] = write_data;
            fast_log("DEFER: Write ADDR_M = 0x%02x\n", write_data);
            break;

        case REG_ADDR_H:
            dma->dma_address.bytes.high = write_data & 0x0F;
            cached_regs.values[REG_ADDR_H] = write_data & 0x0F;
            fast_log("DEFER: Write ADDR_H = 0x%02x\n", write_data & 0x0F);
            break;

        default:
            // Unknown register
            fast_log("DEFER_WRITE: Unknown register offset=0x%02x\n", masked_offset);
            break;
    }

    // After any write that changes bus state, update cached status
    if (masked_offset == REG_CONTROL || masked_offset == REG_DATA) {
        uint8_t new_status = ((dma->bus_ctrl & SASI_INP_BIT) ? SASI_INP_BIT : 0) |
                                ((dma->bus_ctrl & SASI_CTL_BIT) ? SASI_CTL_BIT : 0) |
                                ((dma->bus_ctrl & SASI_BSY_BIT) ? SASI_BSY_BIT : 0) |
                                ((dma->bus_ctrl & SASI_REQ_BIT) ? SASI_REQ_BIT : 0) |
                                ((dma->bus_ctrl & SASI_MSG_BIT) ? SASI_MSG_BIT : 0);
        cached_regs.values[REG_STATUS] = new_status;
        cached_regs.values[0x30] = new_status;
    }
}


// Process a deferred register operation
void defer_process_entry(dma_registers_t *dma, const defer_entry_t *entry) {
    
    uint32_t raw_value = entry->raw_value;
    //extract 2-bit payload type flag
    uint32_t payload_type = (raw_value >> 30) & 0x03;
    uint32_t address;
    uint32_t offset;

    switch (payload_type) {
        case FIFO_REG_READ:
            // Read operation - process side effects only
            defer_process_read(dma, raw_value);
            break;
        case FIFO_WRITE_VALUE:
            // Write operation - update cache and process side effects
            defer_process_write(dma, raw_value);
            break;
        default:
            // Unknown payload type
            fast_log("DEFER: Unknown payload type: 0x%02x\n", payload_type);
            return;
    }
    defer_queue.processed++;
}

// Core 1 worker main loop
void defer_worker_main(void) {
    dma_registers_t *dma = dma_get_registers();
    defer_entry_t entry;

    fast_log("DEFER: Worker starting on Core 1\n");

    while (1) {
        // Process all pending entries
        while (defer_dequeue(&defer_queue, &entry)) {
            defer_process_entry(dma, &entry);
        }

        // Emit recorded FIFO tag trace entries for debugging
        dma_fifo_trace_flush();

        // Small delay to avoid burning CPU when queue is empty
        // This is a balance between latency and CPU usage
        busy_wait_us(1);         //TODO: tune this value after we have the system working. Saw some stalls currently likely due to this being too high
    }
}

// Get the shared queue instance
defer_queue_t* defer_get_queue(void) {
    return &defer_queue;
}

// Get the cached registers instance
cached_registers_t* defer_get_cached_registers(void) {
    return &cached_regs;
}

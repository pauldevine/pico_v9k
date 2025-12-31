/*
 * reg_queue_processor.c - Deferred processing implementation for DMA register operations
 */

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"  // For __dmb() memory barrier
#include <string.h>
#include "reg_queue_processor.h"
#include "dma.h"
#include "sasi.h"
#include "logging.h"
#include "fifo_helpers.h"

// Static instances - place in time_critical section for fast access
defer_queue_t defer_queue __attribute__((section(".time_critical.defer_queue")));
cached_registers_t cached_regs __attribute__((section(".time_critical.cached_regs")));
static int read_request_counter = 0;

void cached_status_sync_from_bus(const dma_registers_t *dma) {
    if (!dma) {
        return;
    }

    uint8_t ctrl = dma->bus_ctrl;
    uint8_t new_status = 0;
    if (ctrl & SASI_INP_BIT)  new_status |= SASI_INP_BIT;
    if (ctrl & SASI_CTL_BIT)  new_status |= SASI_CTL_BIT;
    if (ctrl & SASI_BSY_BIT)  new_status |= SASI_BSY_BIT;
    if (ctrl & SASI_REQ_BIT)  new_status |= SASI_REQ_BIT;
    if (ctrl & SASI_MSG_BIT)  new_status |= SASI_MSG_BIT;

    cached_regs.values[REG_STATUS] = new_status;
    cached_regs.values[0x30] = new_status;
}

void cached_set_data(uint8_t value) {
    cached_regs.values[REG_DATA] = value;
}

// Initialize the deferred processing system
void dma_defer_init(void) {
    // Clear the queue
    memset(&defer_queue, 0, sizeof(defer_queue));
    defer_queue.head = 0;
    defer_queue.tail = 0;

    // Initialize cached registers with default values
    memset(cached_regs.values, 0xFF, sizeof(cached_regs.values));

    // Set initial values for specific registers
    cached_regs.values[REG_CONTROL] = 0x00;  // Control must start at 0 for SELECT edge detection
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
    if (++read_request_counter >= 1000) {
        read_request_counter = 0;
        //fast_log("1000 READ requests processed\n");
    }
    //fast_log("RAW=0x%08x\n", raw_value);
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
            // STATUS register reads do NOT advance bus phases - they just return current status.
            // Phase transitions happen when DATA register (0x10) is read.
            // The MAME boot log shows:
            //   1. Host polls STATUS (0x20) -> sees 0x0F (status phase)
            //   2. Host reads DATA (0x10) -> gets status byte, we advance to message phase
            //   3. Host polls STATUS (0x20) -> sees 0x1F (message phase)
            //   4. Host reads DATA (0x10) -> gets message byte, bus released
            // No action needed here - status is returned from cache by fast handler
            break;

        case REG_DATA:
            // Data register read side effects
            // NOTE: Phase transitions are now handled synchronously in the fast handler
            // (register_irq_handlers.c) to avoid race conditions. The deferred processor
            // only handles slow operations like updating dma->state and interrupts.
            if (dma->state.non_dma_req) {
                if (dma->bus_ctrl & SASI_MSG_BIT) {
                    // Message phase - bus was already released by fast handler
                    // Just update the slow state here
                    dma->state.non_dma_req = 0;
                    dma_update_interrupts(dma, false);
                } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) ==
                            (SASI_CTL_BIT | SASI_INP_BIT)) {
                    // Status phase - transition was already done by fast handler
                    // Just update the slow state here
                    if (dma->state.status_pending) {
                        dma->state.status_pending = 0;
                        dma->state.non_dma_req = 1;
                        dma_update_interrupts(dma, true);
                    }
                } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) == SASI_INP_BIT) {
                    // Data-in phase (CTL=0, INP=1) - clear request after read
                    dma->state.non_dma_req = 0;
                } else {
                    // Command phase (CTL=1, INP=0) or Data-out phase (CTL=0, INP=0)
                    // Reading DATA is unexpected here - don't clear non_dma_req
                    // to preserve the request state for the next command byte
                    fast_log("DEFER_READ: DATA read during unexpected phase, bus_ctrl=0x%02X\n", dma->bus_ctrl);
                }
            }
            fast_log("DEFER_READ: offset=0x%02x end_status %d (processed side effects, cache already returned)\n", masked_offset, cached_regs.values[REG_STATUS]);
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
                        // Selection phase starting (SASI selection byte is a bit mask)
                        dma->bus_ctrl |= SASI_SEL_BIT;
                        dma->selected_target = sasi_extract_target_id(dma->command);
                        dma->bus_ctrl |= SASI_BSY_BIT;
                        __dmb();  // Ensure Core 0 sees bus_ctrl update
                        fast_log("DEFER: SELECT asserted, bus_ctrl=0x%02x\n", dma->bus_ctrl);
                    } else if (!now_sel && prev_sel) {
                        // Selection phase ending - enter command phase
                        dma->bus_ctrl &= ~SASI_SEL_BIT;
                        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
                        dma->bus_ctrl &= ~SASI_INP_BIT;
                        __dmb();  // Ensure Core 0 sees bus_ctrl update
                        dma->state.non_dma_req = 1;
                        dma_update_interrupts(dma, true);
                        fast_log("DEFER: SELECT released, bus_ctrl=0x%02x (entering cmd phase)\n", dma->bus_ctrl);
                    }
                }
                cached_status_sync_from_bus(dma);
                fast_log("DEFER: Write CONTROL = 0x%02x prev_sel=%d bus_ctrl=0x%02x\n",
                         write_data, prev_sel, dma->bus_ctrl);
            }
            break;

        case REG_DATA:
            // Data register write
            dma->command = write_data;
            cached_regs.values[REG_DATA] = write_data;

            if (dma->control & DMA_SELECT_BIT) {
                // During selection, data contains target ID (SASI selection byte is a bit mask)
                dma->selected_target = sasi_extract_target_id(write_data);
                // Note: Don't clear dma->control here - that would break SELECT edge detection
                // when the actual CONTROL=0x00 write arrives. Only bus_ctrl.SEL is cleared.
                dma->bus_ctrl &= ~SASI_SEL_BIT;
            }

            // Handle command/data byte
            bool pending_non_dma = dma->state.non_dma_req;
            bool host_to_device = !(dma->bus_ctrl & SASI_INP_BIT);

            if (pending_non_dma && host_to_device) {
                dma->state.non_dma_req = 0;
                dma->state.asserting_ack = 1;
                dma->bus_ctrl |= SASI_ACK_BIT;
                __dmb();  // Ensure Core 0 sees bus_ctrl update before any DATA read
            }

            // Debug: log bus_ctrl to understand command processing
            fast_log("DEFER: Write DATA = 0x%02x bus_ctrl=0x%02x CTL=%d\n",
                     write_data, dma->bus_ctrl, !!(dma->bus_ctrl & SASI_CTL_BIT));

            if (dma->bus_ctrl & SASI_CTL_BIT) {
                // Command phase - process SASI command byte
                handle_sasi_command_byte(dma, write_data);
            }
            cached_status_sync_from_bus(dma);
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
}


// Process a deferred register operation
void defer_process_entry(dma_registers_t *dma, const defer_entry_t *entry) {
    
    uint32_t raw_value = entry->raw_value;
    //extract 2-bit payload type flag
    uint32_t payload_type = (raw_value >> 30) & 0x03;
    uint32_t address;
    uint32_t offset;

    switch (payload_type) {
        case FIFO_REG_READ_COMMIT:
            // Read operation - process side effects only
            defer_process_read(dma, raw_value);
            break;
        case FIFO_REG_WRITE:
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

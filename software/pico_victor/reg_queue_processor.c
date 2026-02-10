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

// Set to 1 to enable verbose defer processor logging (WARNING: causes queue overflow)
#define DEFER_VERBOSE_LOG 0

#if DEFER_VERBOSE_LOG
#define defer_log(...) defer_log(__VA_ARGS__)
#else
#define defer_log(...) ((void)0)
#endif

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

void cached_sync_dma_address(const dma_registers_t *dma) {
    if (!dma) {
        return;
    }

    cached_regs.values[REG_ADDR_L] = dma->dma_address.bytes.low;
    cached_regs.values[REG_ADDR_M] = dma->dma_address.bytes.mid;
    cached_regs.values[REG_ADDR_H] = dma->dma_address.bytes.high & 0x0F;
    __dmb();  // Ensure other core sees updated cached address bytes
}

// Initialize the deferred processing system
void dma_defer_init(void) {
    // Clear the queue
    memset(&defer_queue, 0, sizeof(defer_queue));
    defer_queue.head = 0;
    defer_queue.tail = 0;

    // Initialize cached registers with default values
    // Use loop instead of memset for volatile array
    for (int i = 0; i < 256; i++) {
        cached_regs.values[i] = 0xFF;
    }

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
        //defer_log("1000 READ requests processed\n");
    }
    //defer_log("RAW=0x%08x\n", raw_value);
    uint32_t address = board_fifo_read_address(raw_value);
    uint32_t offset = address - DMA_REGISTER_BASE;
    uint32_t masked_offset = dma_mask_offset(offset);

    // Handle alias for status register
    if (masked_offset == 0x30) {
        masked_offset = REG_STATUS;
    }

    // Bounds check
    if (offset >= 0x100) {
        defer_log("DEFER_READ: out of range offset=0x%02x\n", offset);
        return;
    }

    switch (masked_offset) {
        case REG_STATUS:
            // Clear interrupt on status read (side effect only)
            dma_update_interrupts(dma, false);
            // STATUS register reads do NOT advance bus phases - they just return current status.
            // Phase transitions happen when DATA register (0x10) is read.
            sasi_trace_event(TRACE_STATUS_READ, dma->bus_ctrl, 0, cached_regs.values[REG_STATUS]);
            break;

        case REG_DATA:
            // Data register read - handle phase transitions
            // Core 1 (deferred processor) is the sole owner of bus_ctrl to avoid race conditions.
            sasi_trace_event(TRACE_DATA_READ, dma->bus_ctrl, dma->state.status_pending, cached_regs.values[REG_DATA]);
            if (dma->bus_ctrl & SASI_MSG_BIT) {
                // Message phase - host read the message byte, release bus
                defer_log("DEFER_READ: Message phase complete, releasing bus\n");
                sasi_trace_event(TRACE_BUS_FREE, dma->bus_ctrl, 0, 0);
                dma->bus_ctrl = 0;  // Clear all bus signals (bus free)
                __dmb();
                dma->state.non_dma_req = 0;
                dma->state.status_pending = 0;
                dma_update_interrupts(dma, false);
                cached_status_sync_from_bus(dma);
            } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) ==
                        (SASI_CTL_BIT | SASI_INP_BIT)) {
                // Status phase - host read status byte, transition to message phase
                if (dma->state.status_pending) {
                    defer_log("DEFER_READ: Status phase complete, entering message phase\n");
                    sasi_trace_event(TRACE_MSG_PHASE, dma->bus_ctrl, 1, 0);
                    dma->state.status_pending = 0;
                    dma->state.non_dma_req = 1;
                    dma->bus_ctrl |= SASI_MSG_BIT;  // Add MSG bit for message phase (0x0F -> 0x1F)
                    __dmb();
                    dma_update_interrupts(dma, true);
                    cached_status_sync_from_bus(dma);
                    cached_set_data(0x00);  // Message byte = Command Complete
                }
            } else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) == SASI_INP_BIT) {
                // Data-in phase (CTL=0, INP=1) - clear request after read
                dma->state.non_dma_req = 0;
            } else if (dma->state.non_dma_req) {
                // Command phase (CTL=1, INP=0) or Data-out phase (CTL=0, INP=0)
                // Reading DATA is unexpected here
                defer_log("DEFER_READ: DATA read during unexpected phase, bus_ctrl=0x%02X\n", dma->bus_ctrl);
            }
#if DEFER_VERBOSE_LOG
            defer_log("DEFER_READ: DATA read complete, bus_ctrl=0x%02X, status_cache=0x%02X\n",
                     dma->bus_ctrl, cached_regs.values[REG_STATUS]);
#endif
            break;

        case REG_ADDR_L:
        case REG_ADDR_M:
        case REG_ADDR_H:
            // No side effects for address register reads
            // The cached value was already returned by the fast handler
#if DEFER_VERBOSE_LOG
            defer_log("DEFER_READ: offset=0x%02x (no action, cache already returned)\n", masked_offset);
#endif
            break;

        default:
            // Unknown register
            defer_log("DEFER_READ: Unknown register offset=0x%02x\n", masked_offset);
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
        defer_log("DEFER_WRITE: out of range offset=0x%02x\n", offset);
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
                    cached_regs.values[REG_DATA] = 0x00;
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
                        defer_log("DEFER: SELECT asserted, bus_ctrl=0x%02x\n", dma->bus_ctrl);
                    } else if (!now_sel && prev_sel) {
                        // Selection phase ending - enter command phase
                        dma->bus_ctrl &= ~SASI_SEL_BIT;
                        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
                        dma->bus_ctrl &= ~SASI_INP_BIT;
                        __dmb();  // Ensure Core 0 sees bus_ctrl update
                        dma->state.non_dma_req = 1;
                        dma_update_interrupts(dma, true);
                        defer_log("DEFER: SELECT released, bus_ctrl=0x%02x (entering cmd phase)\n", dma->bus_ctrl);
                    }
                }
                cached_status_sync_from_bus(dma);
#if DEFER_VERBOSE_LOG
                defer_log("DEFER: Write CONTROL = 0x%02x prev_sel=%d bus_ctrl=0x%02x\n",
                         write_data, prev_sel, dma->bus_ctrl);
#endif
            }
            break;

        case REG_DATA:
            // Data register write
            dma->command = write_data;
            // Do not mirror host DATA writes into cached REG_DATA.
            // Cached REG_DATA must represent target->host status/message bytes,
            // otherwise command bytes can leak into status/message reads.

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
                // Drop REQ to acknowledge the byte, then assert ACK.
                dma->bus_ctrl &= ~SASI_REQ_BIT;
                dma->bus_ctrl |= SASI_ACK_BIT;
                __dmb();  // Ensure Core 0 sees bus_ctrl update before any DATA read
                // CRITICAL: Sync cache IMMEDIATELY after bus_ctrl change.
                // If we wait until after command processing, the host might poll
                // status and see stale cached data (timing race that causes DATA_RD
                // during what appears to be wrong phase).
                cached_status_sync_from_bus(dma);
            }

            // Debug: log bus_ctrl to understand command processing
#if DEFER_VERBOSE_LOG
            defer_log("DEFER: Write DATA = 0x%02x bus_ctrl=0x%02x CTL=%d\n",
                     write_data, dma->bus_ctrl, !!(dma->bus_ctrl & SASI_CTL_BIT));
#endif

            // Command phase detection must be specific:
            // - Command phase: CTL=1, INP=0, MSG=0 (bus_ctrl & 0x13 == 0x02)
            // - Status phase:  CTL=1, INP=1, MSG=0 (bus_ctrl & 0x13 == 0x03)
            // - Message phase: CTL=1, INP=1, MSG=1 (bus_ctrl & 0x13 == 0x13)
            // Previously only checked CTL bit, which incorrectly matched status/message phases
            // and caused command bytes to be processed during wrong phases (protocol desync).
            uint8_t phase_bits = dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
            bool is_command_phase = (phase_bits == SASI_CTL_BIT);  // CTL=1, INP=0, MSG=0

            if (is_command_phase) {
                // Command phase - process SASI command byte
                handle_sasi_command_byte(dma, write_data);
            } else if (dma->state.data_out_expected > 0) {
                // Data-out phase - receiving parameter bytes (e.g., for 0x0C SET DRIVE PARAMS)
                // CTL=0, INP=0 indicates data-out phase
                handle_sasi_data_out_byte(dma, write_data);
            } else if (dma->bus_ctrl != 0) {
                // DATA write during unexpected phase (status, message, or data-in)
                // This indicates protocol desync - host sent data when it shouldn't
                defer_log("DEFER: DATA write 0x%02X during wrong phase, bus_ctrl=0x%02X (ignored)\n",
                         write_data, dma->bus_ctrl);
            }
            // Note: bus_ctrl==0 means bus free, write is ignored (host preparing for next cmd)
            cached_status_sync_from_bus(dma);
            break;

        case REG_ADDR_L:
            dma->dma_address.bytes.low = write_data;
            cached_regs.values[REG_ADDR_L] = write_data;
#if DEFER_VERBOSE_LOG
            defer_log("DEFER: Write ADDR_L = 0x%02x\n", write_data);
#endif
            break;

        case REG_ADDR_M:
            dma->dma_address.bytes.mid = write_data;
            cached_regs.values[REG_ADDR_M] = write_data;
#if DEFER_VERBOSE_LOG
            defer_log("DEFER: Write ADDR_M = 0x%02x\n", write_data);
#endif
            break;

        case REG_ADDR_H:
            dma->dma_address.bytes.high = write_data & 0x0F;
            cached_regs.values[REG_ADDR_H] = write_data & 0x0F;
#if DEFER_VERBOSE_LOG
            defer_log("DEFER: Write ADDR_H = 0x%02x\n", write_data & 0x0F);
#endif
            break;

        default:
            // Unknown register
            defer_log("DEFER_WRITE: Unknown register offset=0x%02x\n", masked_offset);
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
            defer_log("DEFER: Unknown payload type: 0x%02x\n", payload_type);
            return;
    }
    defer_queue.processed++;
}

// Core 1 worker main loop
void defer_worker_main(void) {
    dma_registers_t *dma = &dma_registers;
    defer_entry_t entry;

    defer_log("DEFER: Worker starting on Core 1\n");

    while (1) {
        // Process all pending entries
        while (defer_dequeue(&defer_queue, &entry)) {
            defer_process_entry(dma, &entry);
        }

        // Emit recorded FIFO tag trace entries for debugging
        // DISABLED: causes queue overflow due to fast_log overhead
        // dma_fifo_trace_flush();

        // Keep the loop tight to minimize register latency.
        tight_loop_contents();
    }
}


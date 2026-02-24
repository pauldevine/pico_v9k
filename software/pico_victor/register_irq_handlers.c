/*
 * irq_handlers.c - Ultra-optimized cached DMA register handler
 *
 * This implementation returns cached values immediately to meet the 200ns timing
 * requirement, then enqueues register operations for deferred processing.
 */

#include "pico/stdlib.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"  // For __dmb() memory barrier
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

// PIO FSTAT bit for checking if the register SM's RX FIFO is empty.
// Bit (PIO_FSTAT_RXEMPTY_LSB + N) is 1 when SM N's RX FIFO is empty.
#define REG_FIFO_RXEMPTY_BIT (1u << (PIO_FSTAT_RXEMPTY_LSB + REG_SM_CONTROL))

#ifndef REG_IRQ_FAST_TRACE_ENABLE
#define REG_IRQ_FAST_TRACE_ENABLE 0
#endif

#ifndef REG_IRQ_WARN_ENABLE
#define REG_IRQ_WARN_ENABLE 0
#endif

#if REG_IRQ_WARN_ENABLE
#define reg_irq_warn(...) fast_log(__VA_ARGS__)
#else
#define reg_irq_warn(...) ((void)0)
#endif

#define REG_IRQ_TRACE_SIZE 128u
#define REG_IRQ_TRACE_MASK (REG_IRQ_TRACE_SIZE - 1u)

enum {
    REG_IRQ_TRACE_ANOM_MSG_DATA = 0x01u,
};

typedef struct {
    uint32_t seq;
    uint8_t status_before;
    uint8_t data_before;
    uint8_t status_after;
    uint8_t data_after;
    uint8_t bus_ctrl;
    uint8_t flags;
    uint8_t reserved;
} reg_irq_trace_entry_t;

typedef struct {
    volatile uint32_t head;
    volatile uint32_t seq;
    volatile uint32_t anomaly_count;
    reg_irq_trace_entry_t entries[REG_IRQ_TRACE_SIZE];
} reg_irq_trace_t;

static reg_irq_trace_t reg_irq_trace __attribute__((section(".time_critical.reg_irq_trace")));
static uint32_t reg_irq_trace_last_dumped_anomaly = 0;

static inline uint8_t reg_irq_trace_anomaly_flags(uint8_t status_before, uint8_t data_before) {
    uint8_t flags = 0;
    uint8_t phase = status_before & (SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);

    // Message phase byte is always Command Complete (0x00).
    if (phase == (SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT) && data_before != 0x00) {
        flags |= REG_IRQ_TRACE_ANOM_MSG_DATA;
    }

    return flags;
}

static inline void reg_irq_trace_record_data_transition(uint8_t status_before,
                                                        uint8_t data_before,
                                                        uint8_t status_after,
                                                        uint8_t data_after,
                                                        uint8_t bus_ctrl) {
    uint32_t idx = reg_irq_trace.head & REG_IRQ_TRACE_MASK;
    uint8_t flags = reg_irq_trace_anomaly_flags(status_before, data_before);

    reg_irq_trace.entries[idx].seq = reg_irq_trace.seq++;
    reg_irq_trace.entries[idx].status_before = status_before;
    reg_irq_trace.entries[idx].data_before = data_before;
    reg_irq_trace.entries[idx].status_after = status_after;
    reg_irq_trace.entries[idx].data_after = data_after;
    reg_irq_trace.entries[idx].bus_ctrl = bus_ctrl;
    reg_irq_trace.entries[idx].flags = flags;
    reg_irq_trace.entries[idx].reserved = 0;
    reg_irq_trace.head++;

    if (flags) {
        reg_irq_trace.anomaly_count++;
    }
}

void register_irq_trace_init(void) {
    memset(&reg_irq_trace, 0, sizeof(reg_irq_trace));
    reg_irq_trace_last_dumped_anomaly = 0;
}

uint32_t register_irq_trace_anomaly_count(void) {
    return reg_irq_trace.anomaly_count;
}

void register_irq_trace_dump(const char *reason) {
    uint32_t head = reg_irq_trace.head;
    uint32_t count = (head > REG_IRQ_TRACE_SIZE) ? REG_IRQ_TRACE_SIZE : head;
    uint32_t start = head - count;

    if (reason && reason[0]) {
        printf("\n=== FAST IRQ DATA TRACE (%s) ===\n", reason);
    } else {
        printf("\n=== FAST IRQ DATA TRACE ===\n");
    }
    printf("entries=%lu anomalies=%lu\n",
           (unsigned long)count, (unsigned long)reg_irq_trace.anomaly_count);
    printf("Seq   S0  D0  S1  D1  BC  Flg\n");
    printf("----  --  --  --  --  --  ---\n");

    for (uint32_t i = 0; i < count; i++) {
        uint32_t idx = (start + i) & REG_IRQ_TRACE_MASK;
        const reg_irq_trace_entry_t *e = &reg_irq_trace.entries[idx];
        printf("%4lu  %02X  %02X  %02X  %02X  %02X  %02X\n",
               (unsigned long)e->seq,
               e->status_before,
               e->data_before,
               e->status_after,
               e->data_after,
               e->bus_ctrl,
               e->flags);
    }
    printf("=== END FAST IRQ DATA TRACE ===\n\n");

    reg_irq_trace_last_dumped_anomaly = reg_irq_trace.anomaly_count;
}

void register_irq_trace_note_reset(const char *reason) {
#if !REG_IRQ_FAST_TRACE_ENABLE
    (void)reason;
    return;
#else
    if (reg_irq_trace.anomaly_count == 0) {
        return;
    }
    if (reg_irq_trace.anomaly_count == reg_irq_trace_last_dumped_anomaly) {
        return;
    }
    register_irq_trace_dump(reason);
#endif
}

static inline bool is_valid_reg_offset(uint32_t masked_offset) {
    switch (masked_offset & 0xFFu) {
        case REG_CONTROL:
        case REG_DATA:
        case REG_STATUS:
        case 0x30:
        case REG_ADDR_L:
        case REG_ADDR_M:
        case REG_ADDR_H:
            return true;
        default:
            return false;
    }
}

static inline void clear_irq_on_status_read(dma_registers_t *dma) {
    if (!dma->state.interrupt_pending) {
        return;
    }
    dma->state.interrupt_pending = 0;
    gpio_put(DMA_IRQ_PIN, DMA_IRQ_DEASSERT_LEVEL);
}




// Pre-warm caches by touching critical memory locations
void init_register_irq_handlers(void) {
    // Initialize deferred processing system
    dma_defer_init();
    register_irq_trace_init();

    // Get cached registers and pre-warm
    cached_registers_t *cached = &cached_regs;
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

// IRQ handler for board_registers handles FIFO_REG_READ/FIFO_REG_WRITE payloads.
void __time_critical_func(register_read_irq_isr)() {
    static uint32_t masked_offset;

    // Drain all pending FIFO entries in a single ISR invocation.
    // This avoids tail-chain overhead (~30ns per re-entry) and ensures
    // write-then-read sequences respond within the register-cycle timing budget.
    while (!(PIO_REGISTERS->fstat & REG_FIFO_RXEMPTY_BIT)) {
        uint32_t raw_value;
        uint8_t data = 0;
        uint8_t trace_flags = 0;
        uint8_t pending_before = 0;

        // Read one tagged payload from the board_registers SM RX FIFO.
        raw_value = PIO_REGISTERS->rxf[REG_SM_CONTROL];

        // Payload type is the top bit: 0=register read, 1=register write.
        uint32_t payload_type = fifo_payload_type(raw_value);
        cached_registers_t *cached = &cached_regs;

        bool enque_result = false;

        switch (payload_type) {
            case FIFO_REG_READ: {
                uint32_t address = board_fifo_read_address(raw_value);
                if (address < DMA_REGISTER_BASE || address >= (DMA_REGISTER_BASE + 0x100)) {
                    trace_flags |= FIFO_TRACE_FLAG_ERROR;
                    fifo_trace_record(raw_value,
                                      payload_type,
                                      pending_before,
                                      0,
                                      trace_flags,
                                      0);
                    continue;
                }

                masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE) & 0xFFu;
                if (!is_valid_reg_offset(masked_offset)) {
                    trace_flags |= FIFO_TRACE_FLAG_ERROR;
                    data = 0xFF;
                } else if (masked_offset == REG_DATA) {
                    // Single-pass DATA read model:
                    // 1) return current byte now, 2) advance visible cache phase now,
                    // 3) advance bus_ctrl/state now in lockstep with cache.
                    dma_registers_t *dma = &dma_registers;
                    uint8_t cached_status_before = cached->values[REG_STATUS] & 0x1Fu;
                    uint8_t phase_before = cached_status_before & (SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
#if REG_IRQ_FAST_TRACE_ENABLE
                    uint8_t cached_data_before = cached->values[REG_DATA];
#endif

                    if (phase_before == (SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT)) {
                        // Message-in byte is always Command Complete (0x00), then bus-free.
                        data = 0x00;
                        cached->values[REG_DATA] = 0x00;
                        cached->values[REG_STATUS] = 0x00;
                        cached->values[0x30] = 0x00;

                        dma->bus_ctrl = 0x00;
                        dma->state.non_dma_req = 0;
                        dma->state.status_pending = 0;
                        dma->state.asserting_ack = 0;
                        __dmb();  // Ensure bus_ctrl update is visible before IRQ line update.
                        dma_update_interrupts(dma, false);
                    } else if (phase_before == (SASI_CTL_BIT | SASI_INP_BIT)) {
                        // Status phase: serve authoritative status byte, then advance to message phase.
                        data = dma->status;
                        uint8_t next_status = cached_status_before | SASI_MSG_BIT;
                        cached->values[REG_STATUS] = next_status;
                        cached->values[0x30] = next_status;
                        cached->values[REG_DATA] = 0x00;  // Next DATA read returns message byte.

                        dma->state.status_pending = 0;
                        dma->state.non_dma_req = 1;
                        dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT);
                        dma->bus_ctrl &= ~SASI_ACK_BIT;
                        __dmb();  // Ensure bus_ctrl update is visible before IRQ line update.
                        dma_update_interrupts(dma, true);
                    } else {
                        // Command/data/bus-free path: return the currently cached DATA byte.
                        data = cached->values[REG_DATA];
                        if (dma->state.non_dma_req) {
                            trace_flags |= FIFO_TRACE_FLAG_ERROR;
                            reg_irq_warn("REG IRQ WARN: DATA read in unexpected phase status=0x%02X bus=0x%02X\n",
                                         cached_status_before,
                                         dma->bus_ctrl);
                        }
                    }

#if REG_IRQ_FAST_TRACE_ENABLE
                    {
                        uint8_t bus_ctrl_snapshot = dma->bus_ctrl;
                        reg_irq_trace_record_data_transition(cached_status_before,
                                                             cached_data_before,
                                                             cached->values[REG_STATUS] & 0x1F,
                                                             cached->values[REG_DATA],
                                                             bus_ctrl_snapshot);
                    }
#endif
                    // DATA phase side effects are handled fully in this fast path.
                    enque_result = false;
                } else if (masked_offset == REG_STATUS || masked_offset == 0x30) {
                    // STATUS/0x30 reads clear IRQ latch but do not advance SASI phase.
                    data = cached->values[masked_offset];
                    clear_irq_on_status_read(&dma_registers);
                } else {
                    data = cached->values[masked_offset];
                }

                // PIO expects [pindirs][data] with right-shift out, so send 0xFF pindirs then data.
                uint32_t payload = (0xFFu << 8) | (data & 0xFFu);
                pio_sm_put_blocking(PIO_REGISTERS, REG_SM_CONTROL, payload);

                fifo_trace_record(raw_value,
                                  payload_type,
                                  pending_before,
                                  0,
                                  trace_flags,
                                  data);
                break;
            }

            case FIFO_REG_WRITE: {
                uint32_t address = dma_fifo_write_address(raw_value);
                if (address < DMA_REGISTER_BASE || address >= (DMA_REGISTER_BASE + 0x100)) {
                    trace_flags |= FIFO_TRACE_FLAG_ERROR;
                    fifo_trace_record(raw_value,
                                      payload_type,
                                      pending_before,
                                      0,
                                      trace_flags,
                                      0);
                    continue;
                }

                masked_offset = dma_mask_offset(address - DMA_REGISTER_BASE) & 0xFFu;
                data = dma_fifo_write_data(raw_value);
                if (masked_offset == REG_ADDR_H) {
                    data &= 0x0F;
                }

                bool valid_offset = is_valid_reg_offset(masked_offset);
                bool is_addr_reg = valid_offset &&
                                   (masked_offset == REG_ADDR_L || masked_offset == REG_ADDR_M ||
                                    masked_offset == REG_ADDR_H);
                if (!valid_offset) {
                    trace_flags |= FIFO_TRACE_FLAG_ERROR;
                }

                if (is_addr_reg) {
                    dma_registers_t *dma = &dma_registers;
                    if (masked_offset == REG_ADDR_L) {
                        dma->dma_address.bytes.low = data;
                    } else if (masked_offset == REG_ADDR_M) {
                        dma->dma_address.bytes.mid = data;
                    } else {
                        dma->dma_address.bytes.high = data & 0x0F;
                    }
                    __dmb();  // Ensure Core 0 sees updated DMA address bytes promptly.
                }

                if (masked_offset == REG_CONTROL) {
                    uint8_t prev_ctrl = cached->values[REG_CONTROL];
                    bool prev_sel = (prev_ctrl & DMA_SELECT_BIT) != 0;
                    bool now_sel = (data & DMA_SELECT_BIT) != 0;

                    if (data & DMA_RESET_BIT) {
                        // Signal Core 1 to abort any in-flight command immediately.
                        dma_registers_t *dma = &dma_registers;
                        dma->reset_requested = true;
                        __dmb();
                        // RESET overrides all other CONTROL bits; show bus-free in cache immediately.
                        cached->values[REG_STATUS] = 0x00;
                        cached->values[0x30] = 0x00;
                        cached->values[REG_DATA] = 0x00;
                    } else if (now_sel && !prev_sel) {
                        cached->values[REG_STATUS] = SASI_BSY_BIT;
                        cached->values[0x30] = SASI_BSY_BIT;
                        cached->values[REG_DATA] = 0x00;
                    } else if (!now_sel && prev_sel) {
                        uint8_t cmd_phase = SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT;
                        cached->values[REG_STATUS] = cmd_phase;
                        cached->values[0x30] = cmd_phase;
                        cached->values[REG_DATA] = 0x00;
                    }
                }

                if (valid_offset && masked_offset != REG_DATA) {
                    cached->values[masked_offset] = data;
                    if (masked_offset == REG_STATUS) {
                        cached->values[0x30] = data;
                    }
                }

                trace_flags |= FIFO_TRACE_FLAG_WRITE;
                fifo_trace_record(raw_value,
                                  payload_type,
                                  pending_before,
                                  0,
                                  trace_flags,
                                  data);
                enque_result = valid_offset && !is_addr_reg;
                break;
            }

            default:
                trace_flags |= FIFO_TRACE_FLAG_ERROR;
                fifo_trace_record(raw_value,
                                  payload_type,
                                  pending_before,
                                  0,
                                  trace_flags,
                                  0);
                continue;
        }

        if (enque_result) {
            // Enqueue for deferred processing (fast inline version)
            defer_queue_t *queue = &defer_queue;
            uint32_t head = queue->head;
            uint32_t next = (head + 1) & DEFER_QUEUE_MASK;

            if (next != queue->tail) {
                queue->entries[head].raw_value = raw_value;
                __asm volatile("dmb" ::: "memory");
                queue->head = next;
            } else {
                // Queue full - track dropped entries
                queue->drops++;
            }
        }
    }
}

// Legacy entry point used by cache warmup; delegate to unified handler
void __time_critical_func(register_write_irq_isr)() {
    register_read_irq_isr();
}

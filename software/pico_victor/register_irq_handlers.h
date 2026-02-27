#ifndef PICOVICTOR_IRQ_HANDLERS_H
#define PICOVICTOR_IRQ_HANDLERS_H

#include <stdint.h>
#include <stdbool.h>


void init_register_irq_handlers(void);
void __time_critical_func(register_read_irq_isr)();
void __time_critical_func(register_write_irq_isr)();

// Fast-path DATA transition trace helpers (RAM-only capture, dumped off critical path)
void register_irq_trace_init(void);
void register_irq_trace_dump(const char *reason);
void register_irq_trace_note_reset(const char *reason);
uint32_t register_irq_trace_anomaly_count(void);

// Lightweight diagnostic counters (volatile, defined in register_irq_handlers.c)
extern volatile uint32_t isr_call_count;
extern volatile uint32_t isr_fifo_entries_count;
extern volatile uint32_t isr_tx_fifo_full_count;

// Post-DMA diagnostic: counts ISR calls that occur after sasi_enter_status_phase()
// updates the cache.  If this is 0 after a hang, the 8088 never polled STATUS
// after the Pico was ready — the 8088 stopped accessing 0xEF3xx before the
// status phase was entered.  If non-zero, the 8088 DID poll but something
// else prevented the STATUS→MSG→BUS_FREE sequence from completing.
extern volatile uint32_t isr_post_status_phase_count;
extern volatile bool     status_phase_flag;

// Detailed status-phase diagnostic counters (reset each time status phase is entered)
extern volatile uint32_t diag_status_reads;   // STATUS/0x30 reads during status phase
extern volatile uint32_t diag_data_reads;     // DATA reads during status phase
extern volatile uint32_t diag_other_reads;    // Other register reads during status phase
extern volatile uint32_t diag_writes;         // Register writes during status phase
extern volatile uint8_t  diag_last_phase;     // Last phase_before seen in DATA read path

#endif // PICOVICTOR_IRQ_HANDLERS_H

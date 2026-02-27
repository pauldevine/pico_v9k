#ifndef PICOVICTOR_IRQ_HANDLERS_H
#define PICOVICTOR_IRQ_HANDLERS_H

#include <stdint.h>
#include <stdbool.h>

void init_register_irq_handlers(void);
void __time_critical_func(register_read_irq_isr)();
void __time_critical_func(register_write_irq_isr)();

// Fast-path DATA commit trace helpers (RAM-only capture, dumped off critical path)
void register_irq_trace_init(void);
void register_irq_trace_dump(const char *reason);
void register_irq_trace_note_reset(const char *reason);
uint32_t register_irq_trace_anomaly_count(void);

// Diagnostic counters (updated by ISR, read by UART debug commands and trace dump)
extern volatile uint32_t isr_call_count;
extern volatile uint32_t isr_fifo_entries_count;
extern volatile uint32_t isr_tx_fifo_full_count;
extern volatile uint32_t isr_post_status_phase_count;
extern volatile bool status_phase_flag;

// Status-phase breakdown counters
extern volatile uint32_t diag_status_reads;
extern volatile uint32_t diag_data_reads;
extern volatile uint32_t diag_other_reads;
extern volatile uint32_t diag_writes;
extern volatile uint32_t diag_last_phase;

#endif // PICOVICTOR_IRQ_HANDLERS_H

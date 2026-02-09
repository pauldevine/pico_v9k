#ifndef PICOVICTOR_IRQ_HANDLERS_H
#define PICOVICTOR_IRQ_HANDLERS_H

#include <stdint.h>


void init_register_irq_handlers(void);
void __time_critical_func(register_read_irq_isr)();
void __time_critical_func(register_write_irq_isr)();

// Fast-path DATA commit trace helpers (RAM-only capture, dumped off critical path)
void register_irq_trace_init(void);
void register_irq_trace_dump(const char *reason);
void register_irq_trace_note_reset(const char *reason);
uint32_t register_irq_trace_anomaly_count(void);
#endif // PICOVICTOR_IRQ_HANDLERS_H

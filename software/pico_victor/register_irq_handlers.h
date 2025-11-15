#ifndef PICOVICTOR_IRQ_HANDLERS_H
#define PICOVICTOR_IRQ_HANDLERS_H



void init_register_irq_handlers(void);
void __time_critical_func(register_read_irq_isr)();
void __time_critical_func(register_write_irq_isr)();
#endif // PICOVICTOR_IRQ_HANDLERS_H
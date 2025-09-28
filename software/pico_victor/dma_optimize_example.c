/*
 * dma_optimize_example.c - Example of using optimized IRQ handler
 * 
 * To use the optimized version, simply change the IRQ handler registration
 * in core1_main() in dma.c
 */

// In dma.c, modify core1_main() to use the optimized handler:

void core1_main_optimized() {
    // ... existing setup code ...
    
    PIO register_pio = PIO_REGISTERS;
    int register_sm = REGISTERS_SM;
    
    printf("Setting up IRQ for PIO%d SM%d\n", pio_get_index(register_pio), register_sm);
    printf("FIFO source: %d\n", fifo_sources[register_sm]);
    
    pio_set_irq0_source_enabled(register_pio, fifo_sources[register_sm], true);
    
    // CHANGE THIS LINE to use the optimized handler
    // Original:
    // irq_set_exclusive_handler(PIO1_IRQ_0, registers_irq_handler);
    
    // Option 1: Fast version (function pointer based)
    // irq_set_exclusive_handler(PIO1_IRQ_0, registers_irq_handler_fast);
    
    // Option 2: Ultra version (direct memory for address registers)
    // irq_set_exclusive_handler(PIO1_IRQ_0, registers_irq_handler_ultra);
    
    // Option 3: Ultra ASM version (fastest for address registers)
    irq_set_exclusive_handler(PIO1_IRQ_0, registers_irq_handler_ultra_asm);
    
    irq_set_enabled(PIO1_IRQ_0, true);
    printf("Core1 started with ULTRA FAST IRQ handler\n");
    
    // ... rest of function ...
}

/*
 * Performance expectations:
 * 
 * Original handler: ~35µs (7000 cycles)
 * Fast handler: ~500ns (100 cycles) for complex registers, ~200ns (40 cycles) for address registers
 * Ultra handler: ~300ns (60 cycles) for complex registers, ~100ns (20 cycles) for address registers  
 * Ultra ASM handler: ~200ns (40 cycles) for complex registers, ~60ns (12 cycles) for address registers
 * 
 * The Ultra ASM version should meet your <80ns target for address register access,
 * which is the most common operation during DMA transfers.
 */
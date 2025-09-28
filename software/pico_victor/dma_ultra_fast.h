/*
 * dma_ultra_fast.h - Ultra-optimized DMA register implementation header
 */

#ifndef DMA_ULTRA_FAST_H
#define DMA_ULTRA_FAST_H

#include "dma.h"

// Ultra-optimized IRQ handlers
void registers_irq_handler_ultra(void);
void registers_irq_handler_ultra_asm(void);

// SIO register offsets for direct GPIO manipulation
// These are correct for RP2350
#ifndef SIO_BASE
#define SIO_BASE 0xd0000000
#endif
#ifndef SIO_GPIO_OUT_SET_OFFSET
#define SIO_GPIO_OUT_SET_OFFSET 0x18  // Correct offset for RP2350
#endif
#ifndef SIO_GPIO_OUT_CLR_OFFSET
#define SIO_GPIO_OUT_CLR_OFFSET 0x20  // Correct offset for RP2350
#endif

#endif // DMA_ULTRA_FAST_H
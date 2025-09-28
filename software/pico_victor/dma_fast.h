/*
 * dma_fast.h - Optimized DMA register implementation header
 */

#ifndef DMA_FAST_H
#define DMA_FAST_H

#include "dma.h"

// Optimized IRQ handler that uses direct memory indexing
void registers_irq_handler_fast(void);

// API-compatible wrappers
void dma_write_register_fast(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value);
uint8_t dma_read_register_fast(dma_registers_t *dma, dma_reg_offsets_t offset);

#endif // DMA_FAST_H
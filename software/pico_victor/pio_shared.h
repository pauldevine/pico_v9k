/*
 * pio_shared.h - Constants shared between C code and PIO programs
 *
 * This header can be included by both .c and .pio files to ensure
 * consistent constant definitions across the codebase.
 *
 * IMPORTANT: This file should only contain #define constants and simple
 * macros that are compatible with both C and PIO assembly syntax.
 */

#ifndef PIO_SHARED_H
#define PIO_SHARED_H

/* ============================================================================
 * FIFO Payload Type Flags (2-bit identifier in bits 31-30)
 * ============================================================================
 */
#define FIFO_REG_READ    0x00  /* 8088 reading from Pico registers */
#define FIFO_WRITE_VALUE 0x01  /* 8088 writing to Pico registers */
#define FIFO_DMA_READ    0x02  /* Pico DMA read from 8088 bus */

/* ============================================================================
 * Pin Definitions (GPIO numbers)
 * ============================================================================
 */
#define BD0_PIN         1   /* Data bus D0 (base of 8-bit data bus) */
#define RD_PIN          21  /* Read strobe (active low) */
#define WR_PIN          22  /* Write strobe (active low) */
#define DTR_PIN         23  /* Data Transmit/Receive direction */
#define ALE_PIN         24  /* Address Latch Enable */
#define DEN_PIN         25  /* Data Enable (active low) */
#define HOLD_PIN        26  /* Bus hold request (active low, open drain) */
#define EXTIO_PIN       27  /* External I/O select */
#define READY_PIN       28  /* Bus ready signal */
#define HLDA_PIN        29  /* Bus hold acknowledge */
#define CLOCK_5_PIN     30  /* 5 MHz system clock */
#define CLOCK_15B_PIN   31  /* 1.5 MHz clock (inverted) */

/* ============================================================================
 * Bus and Data Sizes
 * ============================================================================
 */
#define DATA_SIZE           8   /* Data bus width (BD0-BD7) */
#define ADDRESS_BUS_SIZE    20  /* Address bus width (A0-A19, 1MB) */
#define ADDRESS_RD_WR_SIZE  22  /* Address + control bits */

/* ============================================================================
 * DMA Register Configuration
 * ============================================================================
 */
#define DMA_REGISTER_BASE       0xEF300
#define DMA_REGISTER_BITMASK    0x00000EF3  /* Top 12 bits for address matching */

/* Register offsets (used in both C and as constants for PIO) */
#define REG_CONTROL_OFFSET  0x00
#define REG_DATA_OFFSET     0x10
#define REG_STATUS_OFFSET   0x20
#define REG_ADDR_L_OFFSET   0x80
#define REG_ADDR_M_OFFSET   0xA0
#define REG_ADDR_H_OFFSET   0xC0

#endif /* PIO_SHARED_H */

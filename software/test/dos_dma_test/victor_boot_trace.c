/*
 * victor_boot_trace.c - Replay first 20 Victor BIOS DMA register accesses
 *
 * Mirrors the first 20 lines from notes/mame boot example.log to ensure
 * the Pico DMA board responds the same way as the reference hardware.
 * Build with OpenWatcom (see Makefile in this directory).
 */

#include <conio.h>
#include <dos.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DMA_BASE        0xEF30
#define CONTROL_REG     0x00
#define DATA_REG        0x10
#define STATUS_REG      0x20
#define DMA_ADDR_LOW    0x80
#define DMA_ADDR_MID    0xA0
#define DMA_ADDR_HIGH   0xC0

#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

typedef enum {
    BOOT_OP_WRITE,
    BOOT_OP_READ
} boot_op_type_t;

typedef struct {
    boot_op_type_t type;
    unsigned int offset;
    unsigned char value;
    const char *label;
} boot_trace_entry_t;

static const boot_trace_entry_t boot_trace[] = {
    {BOOT_OP_WRITE, CONTROL_REG, 0x00, "CTRL = 0x00 (reset)"},
    {BOOT_OP_WRITE, CONTROL_REG, 0x04, "CTRL = 0x04 (DMA latch)"},
    {BOOT_OP_WRITE, DMA_ADDR_HIGH, 0x05, "ADDR_H = 0x05"},
    {BOOT_OP_WRITE, DMA_ADDR_MID, 0xAA, "ADDR_M = 0xAA"},
    {BOOT_OP_WRITE, DMA_ADDR_LOW, 0x55, "ADDR_L = 0x55"},
    {BOOT_OP_READ,  DMA_ADDR_HIGH, 0x05, "ADDR_H readback"},
    {BOOT_OP_READ,  DMA_ADDR_MID,  0xAA, "ADDR_M readback"},
    {BOOT_OP_READ,  DMA_ADDR_LOW,  0x55, "ADDR_L readback"},
    {BOOT_OP_WRITE, DMA_ADDR_HIGH, 0x0A, "ADDR_H = 0x0A"},
    {BOOT_OP_WRITE, DMA_ADDR_MID,  0x55, "ADDR_M = 0x55"},
    {BOOT_OP_WRITE, DMA_ADDR_LOW,  0xAA, "ADDR_L = 0xAA"},
    {BOOT_OP_READ,  DMA_ADDR_HIGH, 0x0A, "ADDR_H readback"},
    {BOOT_OP_READ,  DMA_ADDR_MID,  0x55, "ADDR_M readback"},
    {BOOT_OP_READ,  DMA_ADDR_LOW,  0xAA, "ADDR_L readback"},
    {BOOT_OP_WRITE, CONTROL_REG,   0x24, "CTRL = 0x24 (RESET|LATCH)"},
    {BOOT_OP_WRITE, CONTROL_REG,   0x04, "CTRL = 0x04"},
    {BOOT_OP_WRITE, CONTROL_REG,   0x00, "CTRL = 0x00"},
    {BOOT_OP_WRITE, CONTROL_REG,   0x04, "CTRL = 0x04"},
    {BOOT_OP_READ,  STATUS_REG,    0x00, "STATUS expected 0x00"},
    {BOOT_OP_WRITE, DATA_REG,      0x01, "DATA = 0x01 (Test Unit Ready)"}
};

static const char *offset_name(unsigned int offset) {
    switch (offset) {
        case CONTROL_REG: return "CONTROL";
        case DATA_REG:    return "DATA";
        case STATUS_REG:  return "STATUS";
        case DMA_ADDR_LOW:  return "ADDR_L";
        case DMA_ADDR_MID:  return "ADDR_M";
        case DMA_ADDR_HIGH: return "ADDR_H";
        default: return "UNKNOWN";
    }
}

static void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

static unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

int main(void) {
    size_t i;
    size_t total = sizeof(boot_trace) / sizeof(boot_trace[0]);
    size_t passed = 0;
    size_t failed = 0;

    printf("Victor BIOS Boot Trace Test (first %u operations)\n", (unsigned)total);
    printf("Based on notes/mame boot example.log\n\n");

    for (i = 0; i < total; i++) {
        const boot_trace_entry_t *entry = &boot_trace[i];
        const char *reg_name = offset_name(entry->offset);

        if (entry->type == BOOT_OP_WRITE) {
            printf("%2u: WRITE  %-7s <- 0x%02X  (%s)\n",
                   (unsigned)(i + 1), reg_name, entry->value, entry->label);
            dma_write(entry->offset, entry->value);
        } else {
            unsigned char read_val;
            printf("%2u: READ   %-7s -> ", (unsigned)(i + 1), reg_name);
            read_val = dma_read(entry->offset);
            printf("0x%02X (expected 0x%02X) ... ", read_val, entry->value);

            if (read_val == entry->value) {
                printf("OK\n");
                passed++;
            } else {
                printf("FAIL\n");
                failed++;
            }
        }

        delay(1); /* small pause to mimic bus timing */
    }

    printf("\nSummary: %u reads checked, %u mismatches\n",
           (unsigned)(passed + failed), (unsigned)failed);
    printf("Result: %s\n", failed ? "FAIL" : "PASS");

    return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}

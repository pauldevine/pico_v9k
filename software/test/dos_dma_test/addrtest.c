/*
 * addrtest.c - Simple DMA address register test
 *
 * Focused test for debugging address register issues
 */

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <dos.h>

/* DMA Register addresses */
#define DMA_BASE        0xEF30  /* Segment for DMA registers */
#define DMA_ADDR_LOW    0x80    /* DMA address low byte */
#define DMA_ADDR_MID    0xA0    /* DMA address mid byte */
#define DMA_ADDR_HIGH   0xC0    /* DMA address high byte */

/* Far pointer macros for accessing DMA registers */
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* Write to DMA register */
void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

/* Read from DMA register */
unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

int main(void) {
    unsigned char val;
    int errors = 0;

    printf("\nDMA Address Register Debug Test\n");
    printf("================================\n\n");

    /* Test 1: Simple write/read to each register */
    printf("Test 1: Individual register access\n");

    /* Low byte */
    printf("  LOW (0x80): ");
    dma_write(DMA_ADDR_LOW, 0x12);
    delay(5);
    val = dma_read(DMA_ADDR_LOW);
    if (val == 0x12) {
        printf("OK (wrote 0x12, read 0x%02X)\n", val);
    } else {
        printf("FAIL (wrote 0x12, read 0x%02X)\n", val);
        errors++;
    }

    /* Mid byte */
    printf("  MID (0xA0): ");
    dma_write(DMA_ADDR_MID, 0x34);
    delay(5);
    val = dma_read(DMA_ADDR_MID);
    if (val == 0x34) {
        printf("OK (wrote 0x34, read 0x%02X)\n", val);
    } else {
        printf("FAIL (wrote 0x34, read 0x%02X)\n", val);
        errors++;
    }

    /* High byte */
    printf("  HIGH (0xC0): ");
    dma_write(DMA_ADDR_HIGH, 0x05);
    delay(5);
    val = dma_read(DMA_ADDR_HIGH);
    val &= 0x0F;  /* Only 4 bits valid */
    if (val == 0x05) {
        printf("OK (wrote 0x05, read 0x%02X)\n", val);
    } else {
        printf("FAIL (wrote 0x05, read 0x%02X)\n", val);
        errors++;
    }

    /* Test 2: Check if registers hold values */
    printf("\nTest 2: Value retention after all writes\n");

    /* Write all three */
    dma_write(DMA_ADDR_LOW, 0xAA);
    dma_write(DMA_ADDR_MID, 0x55);
    dma_write(DMA_ADDR_HIGH, 0x0A);
    delay(10);

    /* Read them back */
    val = dma_read(DMA_ADDR_LOW);
    printf("  LOW: 0x%02X (expected 0xAA) %s\n", val, (val == 0xAA) ? "OK" : "FAIL");
    if (val != 0xAA) errors++;

    val = dma_read(DMA_ADDR_MID);
    printf("  MID: 0x%02X (expected 0x55) %s\n", val, (val == 0x55) ? "OK" : "FAIL");
    if (val != 0x55) errors++;

    val = dma_read(DMA_ADDR_HIGH) & 0x0F;
    printf("  HIGH: 0x%02X (expected 0x0A) %s\n", val, (val == 0x0A) ? "OK" : "FAIL");
    if (val != 0x0A) errors++;

    /* Test 3: Pattern test */
    printf("\nTest 3: Pattern test (0x00, 0xFF, 0x55, 0xAA)\n");

    unsigned char patterns[] = {0x00, 0xFF, 0x55, 0xAA};

    for (int i = 0; i < 4; i++) {
        unsigned char pattern = patterns[i];
        printf("  Pattern 0x%02X: ", pattern);

        /* Write pattern to all registers */
        dma_write(DMA_ADDR_LOW, pattern);
        dma_write(DMA_ADDR_MID, pattern);
        dma_write(DMA_ADDR_HIGH, pattern & 0x0F);
        delay(10);

        /* Read back */
        unsigned char low = dma_read(DMA_ADDR_LOW);
        unsigned char mid = dma_read(DMA_ADDR_MID);
        unsigned char high = dma_read(DMA_ADDR_HIGH) & 0x0F;

        if (low == pattern && mid == pattern && high == (pattern & 0x0F)) {
            printf("OK\n");
        } else {
            printf("FAIL (L:%02X M:%02X H:%02X)\n", low, mid, high);
            errors++;
        }
    }

    /* Test 4: Check actual register offsets being used */
    printf("\nTest 4: Register offset verification\n");
    printf("  DMA_ADDR_LOW offset:  0x%02X\n", DMA_ADDR_LOW);
    printf("  DMA_ADDR_MID offset:  0x%02X\n", DMA_ADDR_MID);
    printf("  DMA_ADDR_HIGH offset: 0x%02X\n", DMA_ADDR_HIGH);
    printf("  Base segment: 0x%04X\n", DMA_BASE);
    printf("  Physical addresses:\n");
    printf("    LOW:  0x%05lX\n", ((unsigned long)DMA_BASE << 4) + DMA_ADDR_LOW);
    printf("    MID:  0x%05lX\n", ((unsigned long)DMA_BASE << 4) + DMA_ADDR_MID);
    printf("    HIGH: 0x%05lX\n", ((unsigned long)DMA_BASE << 4) + DMA_ADDR_HIGH);

    /* Summary */
    printf("\n================================\n");
    if (errors == 0) {
        printf("All tests PASSED!\n");
    } else {
        printf("FAILED with %d errors\n", errors);
        printf("\nPossible issues:\n");
        printf("- Check address masking in dma_ultra_fast.c\n");
        printf("- Verify register_memory array indexing\n");
        printf("- Check if offset &= ~0x1f is correct for 0x80-0xFF\n");
    }

    printf("\nPress any key to exit...\n");
    getch();

    return errors;
}
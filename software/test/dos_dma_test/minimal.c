/*
 * minimal.c - Minimal DMA register test for timing analysis
 *
 * Tests a single write/read to REG_ADDR_H (0xC0) to isolate timing issues
 */

#include <stdio.h>
#include <dos.h>
#include <conio.h>

/* DMA register base segment */
#define DMA_SEG 0xEF30

/* DMA register offsets */
#define DMA_ADDR_HIGH 0xC0

/* Macro to access DMA registers */
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_SEG, offset)))

/* Small delay - renamed to avoid conflict with i86.h */
void test_delay(int ms) {
    int i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 1000; j++) {
            /* Just waste time */
        }
    }
}

int main(void) {
    unsigned char value_written = 0x0A;
    unsigned char value_read;
    int i;

    printf("\nMinimal DMA Register Test\n");
    printf("=========================\n\n");

    printf("Testing single write/read to DMA_ADDR_HIGH (0x%02X)\n", DMA_ADDR_HIGH);
    printf("Physical address: 0x%05lX\n\n", (unsigned long)(DMA_SEG << 4) + DMA_ADDR_HIGH);

    /* Test 1: Single write and immediate read */
    printf("Test 1: Write 0x%02X and immediately read:\n", value_written);

    *DMA_REG(DMA_ADDR_HIGH) = value_written;
    value_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;

    printf("  Wrote: 0x%02X\n", value_written);
    printf("  Read:  0x%02X", value_read);

    if (value_read == (value_written & 0x0F)) {
        printf(" [PASS]\n");
    } else {
        printf(" [FAIL] - Expected 0x%02X\n", value_written & 0x0F);
    }

    /* Test 2: Write with delay before read */
    printf("\nTest 2: Write 0x05 with 1ms delay before read:\n");

    *DMA_REG(DMA_ADDR_HIGH) = 0x05;
    test_delay(1);
    value_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;

    printf("  Wrote: 0x05\n");
    printf("  Read:  0x%02X", value_read);

    if (value_read == 0x05) {
        printf(" [PASS]\n");
    } else {
        printf(" [FAIL] - Expected 0x05\n");
    }

    /* Test 3: Multiple rapid reads */
    printf("\nTest 3: Write 0x0F and read 5 times rapidly:\n");

    *DMA_REG(DMA_ADDR_HIGH) = 0x0F;

    printf("  Wrote: 0x0F\n");
    printf("  Reads: ");

    for (i = 0; i < 5; i++) {
        value_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;
        printf("0x%02X ", value_read);
    }

    printf("\n");

    /* Test 4: Pattern sequence */
    printf("\nTest 4: Write/read sequence (0x00, 0x05, 0x0A, 0x0F):\n");

    unsigned char patterns[] = {0x00, 0x05, 0x0A, 0x0F};
    int pass_count = 0;

    for (i = 0; i < 4; i++) {
        *DMA_REG(DMA_ADDR_HIGH) = patterns[i];
        value_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;

        printf("  Write 0x%02X, Read 0x%02X", patterns[i], value_read);

        if (value_read == patterns[i]) {
            printf(" [PASS]\n");
            pass_count++;
        } else {
            printf(" [FAIL]\n");
        }
    }

    printf("\nPattern test: %d/4 passed\n", pass_count);

    /* Test 5: Check for value stability */
    printf("\nTest 5: Write 0x07 and check stability (10 reads):\n");

    *DMA_REG(DMA_ADDR_HIGH) = 0x07;
    printf("  Wrote: 0x07\n");

    int stable = 1;
    unsigned char first_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;

    printf("  First read: 0x%02X\n", first_read);
    printf("  Checking stability... ");

    for (i = 1; i < 10; i++) {
        value_read = *DMA_REG(DMA_ADDR_HIGH) & 0x0F;
        if (value_read != first_read) {
            stable = 0;
            printf("\n  UNSTABLE! Read #%d = 0x%02X (expected 0x%02X)",
                   i+1, value_read, first_read);
        }
    }

    if (stable) {
        printf("STABLE\n");
    } else {
        printf("\n  Value changes during consecutive reads!\n");
    }

    printf("\n=========================\n");
    printf("Test complete. Press any key to exit.\n");

    getch();
    return 0;
}


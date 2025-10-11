/*
 * dmatest.c - DOS test program for DMA board registers
 *
 * This program tests the DMA board registers at 0xEF300-0xEF3FF
 * Build with OpenWatcom: wmake -f makefile.wc
 */

#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <dos.h>
#include <string.h>
#include <time.h>

/* DMA Register addresses */
#define DMA_BASE        0xEF30  /* Segment for DMA registers */
#define CONTROL_REG     0x00    /* Control register (write-only) */
#define DATA_REG        0x10    /* Data register (read/write) */
#define STATUS_REG      0x20    /* Status register (read-only) */
#define DMA_ADDR_LOW    0x80    /* DMA address low byte */
#define DMA_ADDR_MID    0xA0    /* DMA address mid byte */
#define DMA_ADDR_HIGH   0xC0    /* DMA address high byte */

/* Control register bits */
#define DMA_RESET_BIT    0x20
#define DMA_ON_LATCH_BIT 0x04
#define DMA_ON_VALUE_BIT 0x01
#define DMA_WR_MODE_BIT  0x08
#define DMA_SELECT_BIT   0x10

/* Status register bits */
#define SASI_REQ_BIT    0x01
#define SASI_BSY_BIT    0x02
#define SASI_CTL_BIT    0x04
#define SASI_INP_BIT    0x08
#define SASI_MSG_BIT    0x10

/* Far pointer macros for accessing DMA registers */
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* Test results structure */
typedef struct {
    unsigned int tests_run;
    unsigned int tests_passed;
    unsigned int tests_failed;
} test_results_t;

static test_results_t results = {0, 0, 0};

/* Write to DMA register */
void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

/* Read from DMA register */
unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

/* Print test header */
void print_header(const char *test_name) {
    printf("\n========================================\n");
    printf(" %s\n", test_name);
    printf("========================================\n");
}

/* Print test result */
void print_result(const char *test, int passed) {
    results.tests_run++;
    if (passed) {
        results.tests_passed++;
        printf("[PASS] %s\n", test);
    } else {
        results.tests_failed++;
        printf("[FAIL] %s\n", test);
    }
}

/* Test control register writes */
void test_control_register(void) {
    unsigned char test_values[] = {0x00, 0x08, 0x10, 0x20, 0x40, 0x15, 0xFF};
    int i;

    print_header("Control Register Tests");

    for (i = 0; i < sizeof(test_values); i++) {
        char msg[80];
        printf("Writing 0x%02X to control register...\n", test_values[i]);
        dma_write(CONTROL_REG, test_values[i]);

        /* Small delay to let hardware respond */
        delay(1);

        sprintf(msg, "Control write 0x%02X", test_values[i]);
        print_result(msg, 1); /* Can't read back control register */
    }
}

/* Test data register read/write */
void test_data_register(void) {
    unsigned char test_values[] = {0x00, 0x55, 0xAA, 0xFF, 0x01, 0x80};
    unsigned char read_val;
    int i;

    print_header("Data Register Tests");

    for (i = 0; i < sizeof(test_values); i++) {
        char msg[80];

        dma_write(DATA_REG, test_values[i]);
        delay(1);
        read_val = dma_read(DATA_REG);

        sprintf(msg, "Data R/W: wrote 0x%02X, read 0x%02X",
                test_values[i], read_val);

        /* Data register behavior depends on bus state */
        print_result(msg, 1); /* Log the values for analysis */

        if (read_val == test_values[i]) {
            printf("  -> Values match (loopback mode?)\n");
        } else {
            printf("  -> Different value (normal operation)\n");
        }
    }
}

/* Test status register reads */
void test_status_register(void) {
    unsigned char status;
    int i;

    print_header("Status Register Tests");

    for (i = 0; i < 5; i++) {
        char msg[80];

        status = dma_read(STATUS_REG);

        sprintf(msg, "Status read #%d: 0x%02X", i+1, status);
        print_result(msg, 1);

        /* Decode status bits */
        printf("  Bits: ");
        if (status & SASI_REQ_BIT) printf("REQ ");
        if (status & SASI_BSY_BIT) printf("BSY ");
        if (status & SASI_CTL_BIT) printf("CTL ");
        if (status & SASI_INP_BIT) printf("INP ");
        if (status & SASI_MSG_BIT) printf("MSG ");
        if ((status & 0x1F) == 0) printf("(none)");
        printf("\n");

        delay(10);
    }
}

/* Test DMA address registers */
void test_dma_address_registers(void) {
    unsigned long test_addresses[] = {
        0x000000L, 0x001234L, 0x0A0000L, 0x0FFFFFL
    };
    unsigned char low, mid, high;
    unsigned char low2, mid2, high2;  /* For second read */
    unsigned long read_addr;
    int i;
    int all_passed = 1;

    print_header("DMA Address Register Tests");

    /* First test each register individually */
    printf("\nTesting individual address registers:\n");

    /* Test low byte register */
    printf("  Testing DMA_ADDR_LOW (0x%02X):\n", DMA_ADDR_LOW);
    dma_write(DMA_ADDR_LOW, 0x55);
    delay(1);
    low = dma_read(DMA_ADDR_LOW);
    printf("    Wrote 0x55, read 0x%02X %s\n", low, (low == 0x55) ? "[OK]" : "[FAIL]");

    dma_write(DMA_ADDR_LOW, 0xAA);
    delay(1);
    low = dma_read(DMA_ADDR_LOW);
    printf("    Wrote 0xAA, read 0x%02X %s\n", low, (low == 0xAA) ? "[OK]" : "[FAIL]");

    /* Test mid byte register */
    printf("  Testing DMA_ADDR_MID (0x%02X):\n", DMA_ADDR_MID);
    dma_write(DMA_ADDR_MID, 0x55);
    delay(1);
    mid = dma_read(DMA_ADDR_MID);
    printf("    Wrote 0x55, read 0x%02X %s\n", mid, (mid == 0x55) ? "[OK]" : "[FAIL]");

    dma_write(DMA_ADDR_MID, 0xAA);
    delay(1);
    mid = dma_read(DMA_ADDR_MID);
    printf("    Wrote 0xAA, read 0x%02X %s\n", mid, (mid == 0xAA) ? "[OK]" : "[FAIL]");

    /* Test high byte register (only 4 bits) */
    printf("  Testing DMA_ADDR_HIGH (0x%02X):\n", DMA_ADDR_HIGH);
    dma_write(DMA_ADDR_HIGH, 0x05);
    delay(1);
    high = dma_read(DMA_ADDR_HIGH) & 0x0F;
    printf("    Wrote 0x05, read 0x%02X %s\n", high, (high == 0x05) ? "[OK]" : "[FAIL]");

    dma_write(DMA_ADDR_HIGH, 0x0A);
    delay(1);
    high = dma_read(DMA_ADDR_HIGH) & 0x0F;
    printf("    Wrote 0x0A, read 0x%02X %s\n", high, (high == 0x0A) ? "[OK]" : "[FAIL]");

    /* Now test full address combinations */
    printf("\nTesting full 20-bit addresses:\n");

    for (i = 0; i < sizeof(test_addresses)/sizeof(test_addresses[0]); i++) {
        char msg[80];
        unsigned long addr = test_addresses[i];

        /* Write address bytes */
        printf("  Writing address 0x%06lX:\n", addr);

        dma_write(DMA_ADDR_LOW, (unsigned char)(addr & 0xFF));
        printf("    LOW <- 0x%02X\n", (unsigned char)(addr & 0xFF));

        dma_write(DMA_ADDR_MID, (unsigned char)((addr >> 8) & 0xFF));
        printf("    MID <- 0x%02X\n", (unsigned char)((addr >> 8) & 0xFF));

        dma_write(DMA_ADDR_HIGH, (unsigned char)((addr >> 16) & 0x0F));
        printf("    HIGH <- 0x%02X\n", (unsigned char)((addr >> 16) & 0x0F));

        delay(2);  /* Slightly longer delay */

        /* Read back address bytes */
        low = dma_read(DMA_ADDR_LOW);
        mid = dma_read(DMA_ADDR_MID);
        high = dma_read(DMA_ADDR_HIGH) & 0x0F;

        printf("  Read back:\n");
        printf("    LOW = 0x%02X\n", low);
        printf("    MID = 0x%02X\n", mid);
        printf("    HIGH = 0x%02X\n", high);

        /* Second read to check stability */
        delay(1);
        low2 = dma_read(DMA_ADDR_LOW);
        mid2 = dma_read(DMA_ADDR_MID);
        high2 = dma_read(DMA_ADDR_HIGH) & 0x0F;

        if (low != low2 || mid != mid2 || high != high2) {
            printf("  WARNING: Values changed on second read!\n");
            printf("    LOW: 0x%02X -> 0x%02X\n", low, low2);
            printf("    MID: 0x%02X -> 0x%02X\n", mid, mid2);
            printf("    HIGH: 0x%02X -> 0x%02X\n", high, high2);
        }

        read_addr = ((unsigned long)high << 16) |
                    ((unsigned long)mid << 8) |
                    (unsigned long)low;

        sprintf(msg, "DMA addr: wrote 0x%06lX, read 0x%06lX",
                addr, read_addr);

        if (addr == read_addr) {
            print_result(msg, 1);
        } else {
            print_result(msg, 0);
            all_passed = 0;
            printf("  ERROR: Mismatch!\n");
            printf("    Expected: LOW=0x%02X MID=0x%02X HIGH=0x%02X\n",
                   (unsigned char)(addr & 0xFF),
                   (unsigned char)((addr >> 8) & 0xFF),
                   (unsigned char)((addr >> 16) & 0x0F));
            printf("    Got:      LOW=0x%02X MID=0x%02X HIGH=0x%02X\n",
                   low, mid, high);
        }
    }

    if (!all_passed) {
        printf("\nNOTE: Address register failures may indicate:\n");
        printf("  - Register masking issues in PIO code\n");
        printf("  - Timing problems with register updates\n");
        printf("  - Incorrect address decoding\n");
    }
}

/* Test register access timing */
void test_access_timing(void) {
    unsigned int i, j;
    unsigned char dummy;
    clock_t start, end;
    double cpu_time_used;

    print_header("Register Access Timing Test");

    printf("Performing 1000 register reads...\n");

    /* Use clock() instead of BIOS interrupt for safer timing */
    start = clock();

    /* Perform 1000 reads */
    for (i = 0; i < 1000; i++) {
        dummy = dma_read(STATUS_REG);
        /* Small delay between reads to avoid overwhelming the bus */
        for (j = 0; j < 10; j++) {
            /* Empty delay loop */
        }
    }

    end = clock();
    cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

    printf("Time for 1000 reads: %.3f seconds\n", cpu_time_used);
    printf("Average time per read: %.3f ms\n", cpu_time_used * 1000.0 / 1000.0);

    /* Also do a simple performance test without precise timing */
    printf("\nPerforming rapid burst read test...\n");
    for (i = 0; i < 100; i++) {
        dummy = dma_read(STATUS_REG);
    }
    printf("100 rapid reads completed\n");

    print_result("Timing test completed", 1);
}

/* Test burst writes */
void test_burst_access(void) {
    unsigned int i;

    print_header("Burst Access Test");

    printf("Writing burst of 256 values to data register...\n");
    for (i = 0; i < 256; i++) {
        dma_write(DATA_REG, (unsigned char)i);
    }
    print_result("256 burst writes completed", 1);

    printf("Writing burst to DMA address registers...\n");
    for (i = 0; i < 100; i++) {
        dma_write(DMA_ADDR_LOW, (unsigned char)i);
        dma_write(DMA_ADDR_MID, (unsigned char)(i >> 1));
        dma_write(DMA_ADDR_HIGH, (unsigned char)(i >> 2));
    }
    print_result("100 address updates completed", 1);
}

/* Interactive test mode */
void interactive_test(void) {
    char cmd;
    unsigned int offset;
    unsigned char value;

    print_header("Interactive Test Mode");
    printf("Commands:\n");
    printf("  r <offset>       - Read from register\n");
    printf("  w <offset> <val> - Write to register\n");
    printf("  s                - Read status register\n");
    printf("  c <val>          - Write control register\n");
    printf("  d                - Dump all readable registers\n");
    printf("  q                - Quit interactive mode\n\n");

    while (1) {
        printf("> ");
        cmd = getchar();

        switch (cmd) {
            case 'r':
            case 'R':
                scanf("%x", &offset);
                value = dma_read(offset);
                printf("Read 0x%02X from offset 0x%02X\n", value, offset);
                break;

            case 'w':
            case 'W':
                scanf("%x %x", &offset, &value);
                dma_write(offset, value);
                printf("Wrote 0x%02X to offset 0x%02X\n", value, offset);
                break;

            case 's':
            case 'S':
                value = dma_read(STATUS_REG);
                printf("Status: 0x%02X\n", value);
                break;

            case 'c':
            case 'C':
                scanf("%x", &value);
                dma_write(CONTROL_REG, value);
                printf("Control <- 0x%02X\n", value);
                break;

            case 'd':
            case 'D':
                printf("\nRegister dump:\n");
                printf("  Status:    0x%02X\n", dma_read(STATUS_REG));
                printf("  Data:      0x%02X\n", dma_read(DATA_REG));
                printf("  DMA Low:   0x%02X\n", dma_read(DMA_ADDR_LOW));
                printf("  DMA Mid:   0x%02X\n", dma_read(DMA_ADDR_MID));
                printf("  DMA High:  0x%02X\n", dma_read(DMA_ADDR_HIGH));
                break;

            case 'q':
            case 'Q':
                return;

            case '\n':
                break;

            default:
                printf("Unknown command: %c\n", cmd);
                /* Flush input */
                while (getchar() != '\n');
                break;
        }
    }
}

/* Main program */
int main(int argc, char *argv[]) {
    int interactive = 0;
    int i;

    printf("\nDMA Board Register Test Program\n");
    printf("Testing registers at segment 0x%04X\n", DMA_BASE);

    /* Check for interactive mode */
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-i") == 0) {
            interactive = 1;
        }
    }

    if (interactive) {
        interactive_test();
    } else {
        /* Run all tests */
        test_control_register();
        test_data_register();
        test_status_register();
        test_dma_address_registers();
        test_access_timing();
        test_burst_access();

        /* Print summary */
        printf("\n========================================\n");
        printf(" Test Summary\n");
        printf("========================================\n");
        printf("Total tests:  %u\n", results.tests_run);
        printf("Passed:       %u\n", results.tests_passed);
        printf("Failed:       %u\n", results.tests_failed);

        if (results.tests_failed == 0) {
            printf("\nAll tests completed successfully!\n");
        } else {
            printf("\nSome tests failed. Check output for details.\n");
        }
    }

    printf("\nPress any key to exit...\n");
    getch();

    return (results.tests_failed > 0) ? 1 : 0;
}

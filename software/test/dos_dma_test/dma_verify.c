/*
 * dma_verify.c - DMA Transfer Verification Test
 *
 * This DOS utility reads 10 sectors from the SASI disk via DMA and validates
 * that the received data matches expected CRC8 checksums. This helps diagnose
 * whether DMA transfers are corrupting data during the boot process.
 *
 * The test:
 * 1. Issues SASI READ(6) commands to read specific sectors
 * 2. Receives data via DMA into a memory buffer
 * 3. Computes CRC8 of received data
 * 4. Compares against pre-computed expected values
 *
 * Build with OpenWatcom using the Makefile in this directory.
 */

#include <conio.h>
#include <dos.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* DMA board register addresses */
#define DMA_BASE        0xEF30
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* Register offsets */
#define REG_CONTROL     0x00
#define REG_DATA        0x10
#define REG_STATUS      0x20
#define REG_ADDR_L      0x80
#define REG_ADDR_M      0xA0
#define REG_ADDR_H      0xC0

/* Status register phase bits */
#define STATUS_PHASE_MASK   0x1F
#define PHASE_IDLE          0x00
#define PHASE_BSY           0x04
#define PHASE_COMMAND       0x0E    /* BSY|REQ|CTL */
#define PHASE_STATUS        0x0F    /* BSY|REQ|CTL|INP */
#define PHASE_MESSAGE       0x1F    /* BSY|REQ|CTL|INP|MSG */
#define PHASE_DATA_IN       0x06    /* BSY|INP (data from device) */

/* Control register bits */
#define CTRL_DMA_ENABLE     0x01
#define CTRL_DMA_LATCH      0x02
#define CTRL_DMA_WRITE      0x04    /* 1=write to memory (read from disk) */
#define CTRL_SELECT         0x10
#define CTRL_RESET          0x20

/* Poll limits */
#define MAX_POLL_ITERATIONS 50000U
#define SECTOR_SIZE         512

/* Test configuration */
#define NUM_TEST_SECTORS 10

/* LBA numbers to test - selected for varied content */
static const uint16_t test_lbas[NUM_TEST_SECTORS] = {
    0, 2, 3, 7, 11, 68, 79, 95, 100, 104
};

/* Expected CRC8 values (pre-computed from disk image) */
static const uint8_t expected_crc8[NUM_TEST_SECTORS] = {
    0xF9,  /* LBA   0: drive label */
    0xDE,  /* LBA   2: volume label */
    0xFD,  /* LBA   3: FAT */
    0xFD,  /* LBA   7: FAT copy */
    0xDD,  /* LBA  11: root directory */
    0x01,  /* LBA  68: BIOS boot code start */
    0xB7,  /* LBA  79: boot code */
    0x76,  /* LBA  95: boot code */
    0x84,  /* LBA 100: boot code */
    0xC7,  /* LBA 104: boot code */
};

/* Expected first 8 bytes for visual verification */
static const uint8_t expected_first8[NUM_TEST_SECTORS][8] = {
    {0x02, 0x00, 0x01, 0x00, 0x74, 0x61, 0x6E, 0x64},  /* LBA 0 */
    {0x01, 0x00, 0x56, 0x4F, 0x4C, 0x55, 0x4D, 0x45},  /* LBA 2 */
    {0xF8, 0xFF, 0xFF, 0x03, 0x40, 0x00, 0x05, 0x60},  /* LBA 3 */
    {0xF8, 0xFF, 0xFF, 0x03, 0x40, 0x00, 0x05, 0x60},  /* LBA 7 */
    {0x56, 0x4F, 0x4C, 0x55, 0x4D, 0x45, 0x20, 0x31},  /* LBA 11 */
    {0xEB, 0x4C, 0x08, 0x42, 0x49, 0x4F, 0x53, 0x20},  /* LBA 68 */
    {0xF0, 0x03, 0xF0, 0x03, 0x01, 0x02, 0x81, 0x03},  /* LBA 79 */
    {0xCC, 0x80, 0xCC, 0x00, 0x30, 0x00, 0x30, 0x00},  /* LBA 95 */
    {0x02, 0xE8, 0xCB, 0x15, 0x07, 0x2E, 0xA1, 0xDE},  /* LBA 100 */
    {0x4E, 0x03, 0x03, 0x02, 0x12, 0x4F, 0x01, 0x12},  /* LBA 104 */
};

/* DMA receive buffer - must be in conventional memory */
static uint8_t dma_buffer[SECTOR_SIZE];

/* Test results */
typedef struct {
    uint16_t lba;
    uint8_t expected_crc;
    uint8_t actual_crc;
    uint8_t first8[8];      /* First 8 bytes received */
    uint8_t passed;
    uint8_t phase_error;    /* Non-zero if phase polling failed */
    uint16_t poll_count;    /* Status polls needed */
} sector_result_t;

static sector_result_t results[NUM_TEST_SECTORS];

/*
 * CRC-8 calculation using polynomial 0x07 (x^8 + x^2 + x + 1)
 */
static uint8_t crc8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* Direct register access functions */
static void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

static unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

/*
 * Poll status register until expected phase or timeout.
 * Returns number of iterations, or 0xFFFF on timeout.
 */
static uint16_t poll_status(uint8_t expected) {
    uint16_t iterations = 0;
    uint8_t actual;

    while (iterations < MAX_POLL_ITERATIONS) {
        actual = dma_read(REG_STATUS) & STATUS_PHASE_MASK;
        iterations++;
        if (actual == expected) {
            return iterations;
        }
    }
    return 0xFFFF;  /* Timeout */
}

/*
 * Set up DMA address registers.
 * Address is a 20-bit physical address split across three registers.
 */
static void set_dma_address(uint32_t addr) {
    dma_write(REG_ADDR_L, (uint8_t)(addr & 0xFF));
    dma_write(REG_ADDR_M, (uint8_t)((addr >> 8) & 0xFF));
    dma_write(REG_ADDR_H, (uint8_t)((addr >> 16) & 0x0F));
}

/*
 * Get physical address of our DMA buffer.
 * On DOS, far pointer segment:offset converts to physical address.
 */
static uint32_t get_buffer_address(void) {
    uint16_t seg, off;
    uint32_t phys;

    seg = FP_SEG(dma_buffer);
    off = FP_OFF(dma_buffer);
    phys = ((uint32_t)seg << 4) + off;

    return phys;
}

/*
 * Issue a SASI READ(6) command to read one sector.
 * Returns 0 on success, non-zero on error.
 * Note: lba is uint32_t to support SASI's 21-bit LBA range.
 */
static int read_sector(uint32_t lba, uint16_t *poll_count) {
    uint16_t polls;
    uint8_t status_byte;
    uint32_t dma_addr;

    *poll_count = 0;

    /* Get buffer physical address */
    dma_addr = get_buffer_address();

    /* Step 1: Configure DMA for write-to-memory mode */
    dma_write(REG_CONTROL, 0x08);           /* Clear DMA enable */
    dma_write(REG_CONTROL, 0x0C);           /* Set DMA write mode (device->memory) */

    /* Step 2: Set DMA address */
    set_dma_address(dma_addr);

    /* Step 3: Enable DMA */
    dma_write(REG_CONTROL, 0x09);           /* DMA enable + latch */
    dma_write(REG_CONTROL, 0x0D);           /* DMA enable + write mode */

    /* Step 4: Wait for bus idle */
    polls = poll_status(PHASE_IDLE);
    if (polls == 0xFFFF) {
        return 1;  /* Timeout waiting for idle */
    }
    *poll_count += polls;

    /* Step 5: Select target (target ID = 1) */
    dma_write(REG_DATA, 0x01);
    dma_write(REG_CONTROL, 0x1D);           /* Select + DMA enable + write */

    /* Step 6: Wait for BSY */
    polls = poll_status(PHASE_BSY);
    if (polls == 0xFFFF) {
        return 2;  /* Timeout waiting for BSY */
    }
    *poll_count += polls;

    /* Step 7: Drop select, keep DMA enabled */
    dma_write(REG_CONTROL, 0x0D);

    /* Step 8: Wait for command phase */
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) {
        return 3;  /* Timeout waiting for command phase */
    }
    *poll_count += polls;

    /* Step 9: Send READ(6) CDB - 6 bytes */
    /* CDB[0]: opcode 0x08 = READ(6) */
    dma_write(REG_DATA, 0x08);
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) return 4;
    *poll_count += polls;

    /* CDB[1]: LUN (bits 7-5) + LBA high (bits 4-0) */
    dma_write(REG_DATA, (uint8_t)((lba >> 16) & 0x1F));
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) return 5;
    *poll_count += polls;

    /* CDB[2]: LBA middle byte */
    dma_write(REG_DATA, (uint8_t)((lba >> 8) & 0xFF));
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) return 6;
    *poll_count += polls;

    /* CDB[3]: LBA low byte */
    dma_write(REG_DATA, (uint8_t)(lba & 0xFF));
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) return 7;
    *poll_count += polls;

    /* CDB[4]: Transfer length = 1 sector */
    dma_write(REG_DATA, 0x01);
    polls = poll_status(PHASE_COMMAND);
    if (polls == 0xFFFF) return 8;
    *poll_count += polls;

    /* CDB[5]: Control byte = 0 */
    dma_write(REG_DATA, 0x00);

    /* Step 10: Wait for status phase (DMA transfer happens automatically) */
    /* The Pico will DMA the sector data, then enter status phase */
    polls = poll_status(PHASE_STATUS);
    if (polls == 0xFFFF) {
        return 9;  /* Timeout waiting for status (DMA may have failed) */
    }
    *poll_count += polls;

    /* Step 11: Read status byte (should be 0x00 = GOOD) */
    status_byte = dma_read(REG_DATA);
    if (status_byte != 0x00) {
        return 10;  /* Bad status */
    }

    /* Step 12: Wait for message phase */
    polls = poll_status(PHASE_MESSAGE);
    if (polls == 0xFFFF) {
        return 11;  /* Timeout waiting for message */
    }
    *poll_count += polls;

    /* Step 13: Read message byte (should be 0x00 = COMMAND COMPLETE) */
    status_byte = dma_read(REG_DATA);
    if (status_byte != 0x00) {
        return 12;  /* Bad message */
    }

    /* Success! Data should now be in dma_buffer */
    return 0;
}

/*
 * Run the sector read and verification test.
 */
static void run_test(void) {
    int i, j;
    int err;
    uint8_t actual_crc;

    printf("Reading and verifying %d sectors...\n\n", NUM_TEST_SECTORS);

    for (i = 0; i < NUM_TEST_SECTORS; i++) {
        results[i].lba = test_lbas[i];
        results[i].expected_crc = expected_crc8[i];

        /* Clear buffer before read */
        memset(dma_buffer, 0xAA, SECTOR_SIZE);

        /* Read the sector */
        err = read_sector(test_lbas[i], &results[i].poll_count);

        if (err != 0) {
            results[i].phase_error = err;
            results[i].actual_crc = 0;
            results[i].passed = 0;
            printf("  LBA %3u: PHASE ERROR %d (poll count=%u)\n",
                   test_lbas[i], err, results[i].poll_count);
            continue;
        }

        results[i].phase_error = 0;

        /* Copy first 8 bytes for verification */
        for (j = 0; j < 8; j++) {
            results[i].first8[j] = dma_buffer[j];
        }

        /* Calculate CRC8 of received data */
        actual_crc = crc8(dma_buffer, SECTOR_SIZE);
        results[i].actual_crc = actual_crc;

        /* Compare with expected */
        if (actual_crc == expected_crc8[i]) {
            results[i].passed = 1;
            printf("  LBA %3u: PASS (CRC=0x%02X, polls=%u)\n",
                   test_lbas[i], actual_crc, results[i].poll_count);
        } else {
            results[i].passed = 0;
            printf("  LBA %3u: FAIL CRC expected=0x%02X actual=0x%02X\n",
                   test_lbas[i], expected_crc8[i], actual_crc);
        }
    }
}

/*
 * Print detailed results.
 */
static void print_results(void) {
    int i, j;
    int pass_count = 0;
    int fail_count = 0;
    int error_count = 0;

    printf("\n========================================\n");
    printf("DMA Transfer Verification Results\n");
    printf("========================================\n\n");

    /* Summary */
    for (i = 0; i < NUM_TEST_SECTORS; i++) {
        if (results[i].phase_error) {
            error_count++;
        } else if (results[i].passed) {
            pass_count++;
        } else {
            fail_count++;
        }
    }

    printf("Summary: %d PASS, %d FAIL, %d ERROR\n\n",
           pass_count, fail_count, error_count);

    /* Detailed results for failures */
    if (fail_count > 0 || error_count > 0) {
        printf("Detailed failure analysis:\n");
        printf("--------------------------\n");

        for (i = 0; i < NUM_TEST_SECTORS; i++) {
            if (results[i].phase_error) {
                printf("\nLBA %u: Phase error %d\n",
                       results[i].lba, results[i].phase_error);
                continue;
            }

            if (!results[i].passed) {
                printf("\nLBA %u: CRC mismatch\n", results[i].lba);
                printf("  Expected CRC: 0x%02X\n", results[i].expected_crc);
                printf("  Actual CRC:   0x%02X\n", results[i].actual_crc);

                printf("  Expected first 8: ");
                for (j = 0; j < 8; j++) {
                    printf("%02X ", expected_first8[i][j]);
                }
                printf("\n");

                printf("  Received first 8: ");
                for (j = 0; j < 8; j++) {
                    printf("%02X ", results[i].first8[j]);
                }
                printf("\n");

                /* Check for common corruption patterns */
                if (memcmp(results[i].first8, expected_first8[i], 8) == 0) {
                    printf("  Note: First 8 bytes match - corruption later in sector\n");
                } else {
                    int mismatch_pos = -1;
                    for (j = 0; j < 8; j++) {
                        if (results[i].first8[j] != expected_first8[i][j]) {
                            mismatch_pos = j;
                            break;
                        }
                    }
                    printf("  Note: First mismatch at byte %d\n", mismatch_pos);
                }
            }
        }
    }

    printf("\n");

    /* Final verdict */
    if (pass_count == NUM_TEST_SECTORS) {
        printf("*** ALL TESTS PASSED ***\n");
        printf("DMA transfers are delivering correct data.\n");
    } else {
        printf("*** TESTS FAILED ***\n");
        printf("DMA transfers may be corrupting data.\n");
        printf("Check: byte ordering, timing, address calculation.\n");
    }
}

/*
 * Pre-flight check - verify DMA board is responding.
 */
static int preflight_check(void) {
    uint8_t test_val;

    printf("Pre-flight check...\n");

    /* Try writing and reading address register */
    dma_write(REG_ADDR_H, 0x07);
    test_val = dma_read(REG_ADDR_H);

    if ((test_val & 0x0F) == 0x07) {
        printf("  DMA board responding: OK\n");

        /* Reset the board */
        dma_write(REG_CONTROL, CTRL_RESET);
        dma_write(REG_CONTROL, 0x00);

        printf("  Board reset: OK\n");
        return 1;
    }

    printf("  WARNING: DMA board may not be responding.\n");
    printf("  Wrote 0x07 to ADDR_H, read back 0x%02X\n", test_val);
    return 0;
}

/*
 * Show buffer physical address for debugging.
 */
static void show_buffer_info(void) {
    uint32_t addr = get_buffer_address();
    uint16_t seg = (uint16_t)(addr >> 4);
    uint16_t off = (uint16_t)(addr & 0x0F);

    printf("DMA buffer info:\n");
    printf("  Physical address: 0x%05lX\n", addr);
    printf("  Segment:Offset:   %04X:%04X\n", seg, off);
    printf("  Buffer size:      %u bytes\n\n", SECTOR_SIZE);
}

int main(void) {
    printf("DMA Transfer Verification Test v1.0\n");
    printf("====================================\n\n");

    /* Preflight check */
    if (!preflight_check()) {
        printf("\nContinuing anyway...\n");
    }
    printf("\n");

    /* Show buffer info */
    show_buffer_info();

    /* Run the test */
    run_test();

    /* Print detailed results */
    print_results();

    /* Wait for keypress */
    printf("\nPress any key to exit...\n");
    getch();

    return 0;
}

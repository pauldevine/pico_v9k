/*
 * victor_boot_trace.c - FujiNet sector DMA verification on a Victor 9000
 *
 * This DOS utility performs the minimum SASI sequence needed to read one
 * 512-byte sector via the Pico-based DMA board, then checks that the data
 * landed at the expected RAM address.  It mirrors the transactions issued
 * by the ROM during boot, but keeps the sequence compact so we can validate
 * data movement without needing the full BIOS.
 *
 * Build with OpenWatcom using the Makefile in this directory.
 */

#include <conio.h>
#include <dos.h>
#include <malloc.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define DMA_BASE        0xEF30
#define CONTROL_REG     0x00
#define DATA_REG        0x10
#define STATUS_REG      0x20
#define DMA_ADDR_LOW    0x80
#define DMA_ADDR_MID    0xA0
#define DMA_ADDR_HIGH   0xC0

#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* SASI phase encodings seen on the status register (lower 5 bits). */
#define SASI_PHASE_BUSY     0x04
#define SASI_PHASE_COMMAND  0x0E
#define SASI_PHASE_STATUS   0x0F
#define SASI_PHASE_MESSAGE  0x1F

/* How long we wait (in milliseconds) for each bus phase. */
#define COMMAND_TIMEOUT_MS  2000U
#define DATA_TIMEOUT_MS     8000U
#define MESSAGE_TIMEOUT_MS  2000U
#define BUSY_TIMEOUT_MS     500U

/* Test parameters.  These mirror the working settings from on_device_runner.c */
#define TARGET_ID           0x00
#define READ_LBA            0x00000002UL
#define READ_BLOCKS         1U
#define SECTOR_SIZE         512U

/* Expected header from sector 2 of the DOS boot volume. */
static const unsigned char expected_volume_header[32] = {
    0x01, 0x00, 0x56, 0x4F, 0x4C, 0x55, 0x4D, 0x45,
    0x20, 0x31, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0C, 0x13, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E
};

static inline void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

static inline unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

static void dma_reset(void) {
    dma_write(CONTROL_REG, 0x20);   /* Assert reset */
    delay(1);
    dma_write(CONTROL_REG, 0x00);   /* Release reset */
    delay(1);
}

static void dma_arm_direction(void) {
    /* 0x08 sets device->host direction, 0x0C latches DMA disabled, 0x0D re-enables it. */
    dma_write(CONTROL_REG, 0x08);
    dma_write(CONTROL_REG, 0x0C);
    dma_write(CONTROL_REG, 0x09);
    dma_write(CONTROL_REG, 0x0D);
}

static void dma_program_address(uint32_t linear) {
    printf("DMA destination -> linear 0x%05lX\n", (unsigned long)linear);

    dma_arm_direction();
    dma_write(DMA_ADDR_HIGH, (unsigned char)((linear >> 16) & 0x0F));
    dma_write(DMA_ADDR_MID,  (unsigned char)((linear >> 8) & 0xFF));
    dma_write(DMA_ADDR_LOW,  (unsigned char)(linear & 0xFF));
    dma_arm_direction();
}

/* Convert a far pointer to a 20-bit linear address */
static uint32_t far_to_linear(void far *ptr) {
    uint16_t seg = FP_SEG(ptr);
    uint16_t off = FP_OFF(ptr);
    return (((uint32_t)seg) << 4) + off;
}

/* Get current time in milliseconds using standard clock() function.
 * Victor 9000 MS-DOS typically has CLOCKS_PER_SEC = 100 (10ms resolution) */
static unsigned long get_time_ms(void) {
    return (unsigned long)((clock() * 1000UL) / CLOCKS_PER_SEC);
}

static int wait_for_phase(uint8_t expected, unsigned int timeout_ms, const char *label) {
    unsigned long start_ms = get_time_ms();
    unsigned long elapsed_ms;
    uint8_t last = 0xFF;
    uint8_t prev = 0xFF;
    unsigned long last_print_ms = 0;

    while (1) {
        elapsed_ms = get_time_ms() - start_ms;
        if (elapsed_ms >= timeout_ms) break;

        last = dma_read(STATUS_REG) & 0x1F;
        if (last == expected) {
            printf("  [%lums] Got expected %s (0x%02X)\n", elapsed_ms, label, expected);
            return 1;
        }
        if (last != prev) {
            printf("  [%lums] STATUS changed: 0x%02X -> 0x%02X\n", elapsed_ms, prev, last);
            prev = last;
        }
        if (kbhit() && getch() == 27) {  /* ESC to abort */
            printf("Aborted by user\n");
            return 0;
        }
        /* Print progress every 500ms */
        if (elapsed_ms >= last_print_ms + 500) {
            printf("  [%lums] Still waiting for %s, STATUS=0x%02X\n", elapsed_ms, label, last);
            last_print_ms = elapsed_ms;
        }
    }

    printf("Timeout waiting for %s (expected 0x%02X, last 0x%02X)\n",
           label, expected, last);
    return 0;
}

static int select_target(uint8_t target_id) {
    uint8_t status_before, status_after;
    uint8_t select_data;

    /* SASI selection: put both initiator (ID 7) and target ID on bus as bit mask */
    select_data = (1 << 7) | (1 << (target_id & 0x07));

    printf("Step: Writing select data 0x%02X to DATA_REG (initiator=7, target=%u)\n",
           select_data, target_id);
    status_before = dma_read(STATUS_REG);
    printf("  STATUS before: 0x%02X\n", status_before);

    dma_write(DATA_REG, select_data);

    printf("Step: Asserting SELECT (CONTROL = 0x1D)\n");
    dma_write(CONTROL_REG, 0x1D); /* SELECT(0x10) + LATCH(0x04) + WMODE(0x08) + DMAON(0x01) */

    status_after = dma_read(STATUS_REG);
    printf("  STATUS immediately after SELECT: 0x%02X\n", status_after);

    printf("Step: Waiting for BSY (0x04)...\n");
    if (!wait_for_phase(SASI_PHASE_BUSY, BUSY_TIMEOUT_MS, "target BSY")) {
        printf("  Final STATUS: 0x%02X (expected 0x04)\n", dma_read(STATUS_REG));
        return 0;
    }

    printf("Step: Dropping SELECT (CONTROL = 0x0D)\n");
    dma_write(CONTROL_REG, 0x0D);  /* Drop SELECT but keep DMA enabled (LATCH+WMODE+DMAON) */
    return 1;
}

static void build_read6_cdb(uint8_t *cdb, uint32_t lba, uint8_t blocks) {
    cdb[0] = 0x08; /* Standard SCSI/SASI READ(6) */
    cdb[1] = (uint8_t)((lba >> 16) & 0x1F);
    cdb[2] = (uint8_t)((lba >> 8) & 0xFF);
    cdb[3] = (uint8_t)(lba & 0xFF);
    cdb[4] = blocks ? blocks : 0x00; /* 0 means 256 blocks */
    cdb[5] = 0x00;
}

static int send_cdb(const uint8_t *cdb, size_t len) {
    size_t i;
    printf("Step: Sending %u-byte CDB\n", (unsigned)len);
    for (i = 0; i < len; i++) {
        printf("  CDB[%u]: waiting for COMMAND phase (0x0E)...\n", (unsigned)i);
        if (!wait_for_phase(SASI_PHASE_COMMAND, COMMAND_TIMEOUT_MS, "command byte request")) {
            printf("  Failed waiting for command phase at byte %u\n", (unsigned)i);
            return 0;
        }

        /* DMA is already enabled from select_target() - don't touch CONTROL here.
         * The hardware distinguishes command phase (uses non-DMA ACK) from data
         * phase (uses DMA) based on the CTL signal, not the DMA enable bit. */

        printf("  CDB[%u]: writing 0x%02X\n", (unsigned)i, cdb[i]);
        dma_write(DATA_REG, cdb[i]);
    }
    printf("  CDB sent successfully\n");
    return 1;
}

static int verify_buffer(uint8_t far *buffer) {
    size_t i;
    size_t mismatches = 0;

    for (i = 0; i < sizeof(expected_volume_header); i++) {
        unsigned char observed = buffer[i];
        unsigned char expected = expected_volume_header[i];
        if (observed != expected) {
            if (mismatches < 8) {
                printf("Mismatch at +0x%02X: got 0x%02X expected 0x%02X\n",
                       (unsigned)i, observed, expected);
            }
            mismatches++;
        }
    }

    if (mismatches) {
        printf("Sector verification failed (%u mismatches within first 32 bytes)\n",
               (unsigned)mismatches);
        return 0;
    }

    printf("Sector header matches expected Volume ID\n");
    return 1;
}

int main(void) {
    uint8_t cdb[6];
    uint8_t status_byte;
    uint8_t message_byte;
    uint8_t far *dest;
    uint32_t dest_linear;

    printf("Victor DMA FujiNet sector test\n");
    printf("---------------------------------\n");

    /* Allocate buffer from DOS heap - this gives us safe RAM above the program */
    dest = (uint8_t far *)_fmalloc(SECTOR_SIZE);
    if (!dest) {
        printf("FAILED: Could not allocate %u bytes for DMA buffer\n", SECTOR_SIZE);
        return EXIT_FAILURE;
    }
    dest_linear = far_to_linear(dest);
    printf("Allocated DMA buffer at %04X:%04X (linear 0x%05lX)\n",
           FP_SEG(dest), FP_OFF(dest), (unsigned long)dest_linear);

    /* Clear buffer so we can verify data arrived */
    _fmemset(dest, 0xAA, SECTOR_SIZE);

    printf("Step: Resetting DMA controller\n");
    dma_reset();

    printf("Step: Programming DMA address\n");
    dma_program_address(dest_linear);

    printf("Step: Selecting target %u\n", TARGET_ID);
    if (!select_target(TARGET_ID)) {
        printf("FAILED: Could not select SASI target %u\n", TARGET_ID);
        _ffree(dest);
        return EXIT_FAILURE;
    }
    printf("  Target selected successfully\n");

    build_read6_cdb(cdb, READ_LBA, READ_BLOCKS);
    printf("Step: Issuing READ(6) for LBA %lu (blocks=%u)\n",
           (unsigned long)READ_LBA, READ_BLOCKS);
    printf("  CDB: %02X %02X %02X %02X %02X %02X\n",
           cdb[0], cdb[1], cdb[2], cdb[3], cdb[4], cdb[5]);

    if (!send_cdb(cdb, sizeof(cdb))) {
        printf("FAILED: Timed out sending READ(6) command\n");
        _ffree(dest);
        return EXIT_FAILURE;
    }

    /* DMA was enabled during select_target() and stays enabled.
     * Wait for either data-in phase or status phase (DMA might complete instantly). */
    printf("Step: Waiting for DATA-IN or STATUS phase...\n");
    {
        unsigned long start_ms = get_time_ms();
        unsigned long elapsed_ms;
        unsigned long last_print_ms = 0;
        uint8_t status;
        uint8_t prev_status = 0xFF;
        int in_data_phase = 0;

        while (1) {
            elapsed_ms = get_time_ms() - start_ms;
            if (elapsed_ms >= DATA_TIMEOUT_MS) break;

            status = dma_read(STATUS_REG) & 0x1F;

            if (status != prev_status) {
                printf("  [%lums] STATUS changed: 0x%02X -> 0x%02X\n", elapsed_ms, prev_status, status);
                prev_status = status;
            }

            /* Data-in phase: INP=1, CTL=0 (status & 0x03 == 0x01) */
            if ((status & 0x03) == 0x01) {
                if (!in_data_phase) {
                    printf("  [%lums] Data-in phase detected (STATUS=0x%02X)\n", elapsed_ms, status);
                    in_data_phase = 1;
                }
                /* DMA should be running - continue waiting for status phase */
            }

            /* Status phase: CTL=1, INP=1, MSG=0 -> 0x0F */
            if (status == SASI_PHASE_STATUS) {
                printf("  [%lums] Status phase reached (STATUS=0x%02X)\n", elapsed_ms, status);
                break;
            }

            if (kbhit() && getch() == 27) {
                printf("Aborted by user\n");
                _ffree(dest);
                return EXIT_FAILURE;
            }

            /* Print progress every 500ms while waiting */
            if (elapsed_ms >= last_print_ms + 500) {
                printf("  [%lums] Waiting for status phase, STATUS=0x%02X\n", elapsed_ms, status);
                last_print_ms = elapsed_ms;
            }
        }
        if (elapsed_ms >= DATA_TIMEOUT_MS) {
            printf("FAILED: Timeout waiting for status phase (last STATUS=0x%02X)\n", status);
            _ffree(dest);
            return EXIT_FAILURE;
        }
    }
    status_byte = dma_read(DATA_REG);
    printf("  Status byte: 0x%02X\n", status_byte);

    printf("Step: Waiting for MESSAGE phase (0x1F)...\n");
    if (!wait_for_phase(SASI_PHASE_MESSAGE, MESSAGE_TIMEOUT_MS, "message phase")) {
        printf("FAILED: Timeout waiting for message phase\n");
        _ffree(dest);
        return EXIT_FAILURE;
    }
    message_byte = dma_read(DATA_REG);
    printf("  Message byte: 0x%02X\n", message_byte);

    if (status_byte != 0x00 || message_byte != 0x00) {
        printf("Controller reported error status\n");
        _ffree(dest);
        return EXIT_FAILURE;
    }

    /* Debug: dump first 32 bytes of buffer to see what arrived */
    printf("Buffer contents (first 32 bytes):\n");
    {
        int i;
        int aa_count = 0;
        for (i = 0; i < 32; i++) {
            if (i % 16 == 0) printf("  %04X: ", i);
            printf("%02X ", dest[i]);
            if (dest[i] == 0xAA) aa_count++;
            if (i % 16 == 15) printf("\n");
        }
        if (aa_count == 32) {
            printf("WARNING: Buffer still contains 0xAA pattern - DMA may not have written!\n");
        } else if (aa_count > 0) {
            printf("Note: %d bytes still contain 0xAA fill pattern\n", aa_count);
        }
    }

    if (!verify_buffer(dest)) {
        _ffree(dest);
        return EXIT_FAILURE;
    }

    printf("DMA sector transfer verified at linear 0x%05lX\n",
           (unsigned long)dest_linear);
    _ffree(dest);
    return EXIT_SUCCESS;
}

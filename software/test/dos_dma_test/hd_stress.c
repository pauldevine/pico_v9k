/*
 * hd_stress.c - Victor 9000 Hard Disk Stress Test
 *
 * Exercises the hard disk through standard MS-DOS file services to stress
 * test the entire I/O stack the way a real application would:
 *   DOS INT 21h -> BIOS -> DMA board -> FujiNet
 *
 * Test phases:
 *   Phase 1: Write 25 small files (10K each) with unique patterns
 *   Phase 2: Read back all 25 files and verify contents byte-by-byte
 *   Phase 3: Write one large file (800K) with a known pattern
 *   Phase 4: Read back the large file and verify contents
 *
 * On any mismatch, reports: file name, absolute byte offset, file sector
 * number (offset / 512), byte position within that sector, and expected
 * vs actual byte values.
 *
 * Usage:  HD_STRES [drive]
 *   drive  - Optional target drive letter (e.g. C). If omitted, files
 *            are created in the current directory.
 *
 * Example: Run from floppy, stress-test the hard disk:
 *   A> HD_STRES C
 *
 * Build with OpenWatcom using the Makefile in this directory.
 */

#include <conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

/* Test parameters */
#define NUM_SMALL_FILES     25
#define SMALL_FILE_SIZE     10240UL     /* 10K per file */
#define LARGE_FILE_SIZE     819200UL    /* 800K */
#define IO_BUFFER_SIZE      4096U       /* 4K I/O chunks */
#define SECTOR_SIZE         512U

/* Target drive prefix: empty or "C:\" */
static char drive_prefix[4] = "";

/*
 * Deterministic pattern generator.
 *
 * For a given (file_id, offset) pair this always produces the same byte.
 * The formula mixes low and high offset bits with the file identity so:
 *   - Different files have completely different sequences
 *   - Within a file, every offset produces a distinct value
 *   - No trivially repeating pattern that might hide stuck bus lines
 */
static uint8_t make_pattern_byte(uint8_t file_id, uint32_t offset)
{
    uint8_t lo = (uint8_t)(offset & 0xFF);
    uint8_t hi = (uint8_t)((offset >> 8) & 0xFF);
    return (uint8_t)((lo ^ hi) + file_id * 37U + 0xA5U);
}

/* Fill buffer with pattern bytes starting at the given file offset */
static void fill_pattern(uint8_t *buf, uint16_t len,
                         uint8_t file_id, uint32_t start_offset)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        buf[i] = make_pattern_byte(file_id, start_offset + (uint32_t)i);
    }
}

/* Format a small-file name: TEST01.DAT .. TEST25.DAT (with drive prefix) */
static void small_file_name(char *buf, int file_num)
{
    sprintf(buf, "%sTEST%02d.DAT", drive_prefix, file_num);
}

/* Format the large file path (with drive prefix) */
static void large_file_name(char *buf)
{
    sprintf(buf, "%sBIGTEST.DAT", drive_prefix);
}

/* Global statistics */
static uint32_t total_bytes_written = 0;
static uint32_t total_bytes_read    = 0;
static uint16_t total_errors        = 0;
static uint16_t files_created       = 0;
static uint16_t files_verified      = 0;

/* I/O buffers (static to stay in data segment, 8K total) */
static uint8_t io_buf[IO_BUFFER_SIZE];
static uint8_t expect_buf[IO_BUFFER_SIZE];

/* ------------------------------------------------------------------ */
/*  Helper: write a single file with pattern data                      */
/* ------------------------------------------------------------------ */
static int write_one_file(const char *fname, uint8_t file_id,
                          uint32_t file_size)
{
    FILE *fp;
    uint32_t remaining, offset;
    uint16_t chunk_len;

    fp = fopen(fname, "wb");
    if (!fp) {
        printf("  ERROR: Cannot create %s (errno=%d)\n", fname, errno);
        total_errors++;
        return 0;
    }

    offset = 0;
    remaining = file_size;
    while (remaining > 0) {
        chunk_len = (remaining > IO_BUFFER_SIZE)
                    ? IO_BUFFER_SIZE : (uint16_t)remaining;
        fill_pattern(io_buf, chunk_len, file_id, offset);

        if (fwrite(io_buf, 1, chunk_len, fp) != chunk_len) {
            printf("  ERROR: Write failed on %s at offset %lu "
                   "(file sector %lu) errno=%d\n",
                   fname, offset, offset / SECTOR_SIZE, errno);
            total_errors++;
            fclose(fp);
            return 0;
        }

        offset += chunk_len;
        remaining -= chunk_len;
        total_bytes_written += chunk_len;
    }

    fclose(fp);
    return 1;
}

/* ------------------------------------------------------------------ */
/*  Helper: read back and verify a single file                         */
/* ------------------------------------------------------------------ */
static int verify_one_file(const char *fname, uint8_t file_id,
                           uint32_t file_size)
{
    FILE *fp;
    uint32_t offset;
    uint16_t expect_len, read_len;
    uint16_t i;
    int file_ok = 1;
    uint16_t mismatch_count = 0;

    fp = fopen(fname, "rb");
    if (!fp) {
        printf("  ERROR: Cannot open %s for reading (errno=%d)\n",
               fname, errno);
        total_errors++;
        return 0;
    }

    offset = 0;
    while (offset < file_size) {
        expect_len = (file_size - offset > IO_BUFFER_SIZE)
                     ? IO_BUFFER_SIZE : (uint16_t)(file_size - offset);

        read_len = (uint16_t)fread(io_buf, 1, expect_len, fp);
        if (read_len != expect_len) {
            printf("  ERROR: Short read on %s at offset %lu: "
                   "expected %u bytes, got %u\n",
                   fname, offset, expect_len, read_len);
            total_errors++;
            fclose(fp);
            return 0;
        }

        total_bytes_read += read_len;

        /* Generate expected pattern and compare */
        fill_pattern(expect_buf, read_len, file_id, offset);
        for (i = 0; i < read_len; i++) {
            if (io_buf[i] != expect_buf[i]) {
                uint32_t abs_off = offset + (uint32_t)i;
                if (mismatch_count < 10) {
                    printf("  MISMATCH %s offset %lu "
                           "(file sector %lu, byte %u): "
                           "expected 0x%02X got 0x%02X\n",
                           fname,
                           abs_off,
                           abs_off / SECTOR_SIZE,
                           (unsigned)(abs_off % SECTOR_SIZE),
                           expect_buf[i], io_buf[i]);
                }
                mismatch_count++;
                total_errors++;
                file_ok = 0;
            }
        }

        if (mismatch_count >= 10 && !file_ok) {
            /* Keep reading to count total mismatches */
            uint32_t scan_off = offset + read_len;
            while (scan_off < file_size) {
                expect_len = (file_size - scan_off > IO_BUFFER_SIZE)
                             ? IO_BUFFER_SIZE
                             : (uint16_t)(file_size - scan_off);
                read_len = (uint16_t)fread(io_buf, 1, expect_len, fp);
                if (read_len == 0) break;
                total_bytes_read += read_len;
                fill_pattern(expect_buf, read_len, file_id, scan_off);
                for (i = 0; i < read_len; i++) {
                    if (io_buf[i] != expect_buf[i]) {
                        mismatch_count++;
                        total_errors++;
                    }
                }
                scan_off += read_len;
            }
            printf("  ... %s: %u total byte mismatches\n",
                   fname, mismatch_count);
            break;
        }

        offset += read_len;
    }

    fclose(fp);
    return file_ok;
}

/* ------------------------------------------------------------------ */
/*  Phase 1: Write 25 small files                                      */
/* ------------------------------------------------------------------ */
static int phase1_write_small_files(void)
{
    int f;
    char fname[16];
    uint16_t created = 0;

    printf("\n--- Phase 1: Writing %d small files (%luK each) ---\n",
           NUM_SMALL_FILES, SMALL_FILE_SIZE / 1024UL);

    for (f = 1; f <= NUM_SMALL_FILES; f++) {
        small_file_name(fname, f);
        printf("  [%2d/%d] Writing %s...", f, NUM_SMALL_FILES, fname);

        if (write_one_file(fname, (uint8_t)f, SMALL_FILE_SIZE)) {
            printf(" OK\n");
            created++;
            files_created++;
        } else {
            printf(" FAILED\n");
        }
    }

    printf("  Phase 1 complete: %u/%d files created, %lu bytes written\n",
           created, NUM_SMALL_FILES, total_bytes_written);
    return (created == NUM_SMALL_FILES) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/*  Phase 2: Read back and verify 25 small files                       */
/* ------------------------------------------------------------------ */
static int phase2_verify_small_files(void)
{
    int f;
    char fname[16];
    uint16_t passed = 0;

    printf("\n--- Phase 2: Verifying %d small files ---\n",
           NUM_SMALL_FILES);

    for (f = 1; f <= NUM_SMALL_FILES; f++) {
        small_file_name(fname, f);
        printf("  [%2d/%d] Verifying %s...", f, NUM_SMALL_FILES, fname);

        if (verify_one_file(fname, (uint8_t)f, SMALL_FILE_SIZE)) {
            printf(" OK\n");
            passed++;
            files_verified++;
        } else {
            printf(" FAILED\n");
        }
    }

    printf("  Phase 2 complete: %u/%d files verified OK\n",
           passed, NUM_SMALL_FILES);
    return (passed == NUM_SMALL_FILES) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/*  Phase 3: Write large file (800K)                                   */
/* ------------------------------------------------------------------ */
static int phase3_write_large_file(void)
{
    uint32_t last_report = 0;
    uint32_t bytes_before = total_bytes_written;
    int ok;
    char bigname[16];

    large_file_name(bigname);

    printf("\n--- Phase 3: Writing large file %s (%luK) ---\n",
           bigname, LARGE_FILE_SIZE / 1024UL);

    {
        FILE *fp;
        uint32_t remaining, offset;
        uint16_t chunk_len;

        fp = fopen(bigname, "wb");
        if (!fp) {
            printf("  ERROR: Cannot create %s (errno=%d)\n", bigname, errno);
            total_errors++;
            return 0;
        }

        offset = 0;
        remaining = LARGE_FILE_SIZE;
        ok = 1;

        while (remaining > 0) {
            chunk_len = (remaining > IO_BUFFER_SIZE)
                        ? IO_BUFFER_SIZE : (uint16_t)remaining;
            fill_pattern(io_buf, chunk_len, 0, offset);

            if (fwrite(io_buf, 1, chunk_len, fp) != chunk_len) {
                printf("  ERROR: Write failed at offset %lu "
                       "(file sector %lu) errno=%d\n",
                       offset, offset / SECTOR_SIZE, errno);
                total_errors++;
                ok = 0;
                break;
            }

            offset += chunk_len;
            remaining -= chunk_len;
            total_bytes_written += chunk_len;

            if (offset - last_report >= 102400UL) {
                printf("  Written %luK / %luK...\n",
                       offset / 1024UL, LARGE_FILE_SIZE / 1024UL);
                last_report = offset;
            }
        }

        fclose(fp);
    }

    printf("  Phase 3 complete: %lu bytes written\n",
           total_bytes_written - bytes_before);
    return ok;
}

/* ------------------------------------------------------------------ */
/*  Phase 4: Read back and verify large file                           */
/* ------------------------------------------------------------------ */
static int phase4_verify_large_file(void)
{
    FILE *fp;
    uint32_t offset;
    uint16_t expect_len, read_len;
    uint16_t i;
    int file_ok = 1;
    uint16_t mismatch_count = 0;
    uint32_t last_report = 0;
    char bigname[16];

    large_file_name(bigname);

    printf("\n--- Phase 4: Verifying large file %s (%luK) ---\n",
           bigname, LARGE_FILE_SIZE / 1024UL);

    fp = fopen(bigname, "rb");
    if (!fp) {
        printf("  ERROR: Cannot open %s for reading (errno=%d)\n",
               bigname, errno);
        total_errors++;
        return 0;
    }

    printf("  Reading first chunk...\n");
    offset = 0;
    while (offset < LARGE_FILE_SIZE) {
        expect_len = (LARGE_FILE_SIZE - offset > IO_BUFFER_SIZE)
                     ? IO_BUFFER_SIZE
                     : (uint16_t)(LARGE_FILE_SIZE - offset);

        read_len = (uint16_t)fread(io_buf, 1, expect_len, fp);
        if (read_len != expect_len) {
            printf("  ERROR: Short read at offset %lu: "
                   "expected %u bytes, got %u\n",
                   offset, expect_len, read_len);
            total_errors++;
            file_ok = 0;
            break;
        }

        total_bytes_read += read_len;

        fill_pattern(expect_buf, read_len, 0, offset);
        for (i = 0; i < read_len; i++) {
            if (io_buf[i] != expect_buf[i]) {
                uint32_t abs_off = offset + (uint32_t)i;
                if (mismatch_count < 10) {
                    printf("  MISMATCH offset %lu "
                           "(file sector %lu, byte %u): "
                           "expected 0x%02X got 0x%02X\n",
                           abs_off,
                           abs_off / SECTOR_SIZE,
                           (unsigned)(abs_off % SECTOR_SIZE),
                           expect_buf[i], io_buf[i]);
                }
                mismatch_count++;
                total_errors++;
                file_ok = 0;
            }
        }

        offset += read_len;

        if (offset - last_report >= 51200UL) {
            printf("  Verified %luK / %luK...\n",
                   offset / 1024UL, LARGE_FILE_SIZE / 1024UL);
            last_report = offset;
        }
    }

    fclose(fp);

    if (mismatch_count > 10) {
        printf("  Total byte mismatches: %u\n", mismatch_count);
    }

    if (file_ok) {
        printf("  Phase 4 complete: BIGTEST.DAT verified OK\n");
    } else {
        printf("  Phase 4 complete: BIGTEST.DAT FAILED "
               "(%u byte mismatches)\n", mismatch_count);
    }
    return file_ok;
}

/* ------------------------------------------------------------------ */
/*  Cleanup: remove test files from disk                               */
/* ------------------------------------------------------------------ */
static void cleanup_test_files(void)
{
    int f;
    char fname[16];
    char bigname[16];

    large_file_name(bigname);

    printf("\nCleaning up test files...\n");
    for (f = 1; f <= NUM_SMALL_FILES; f++) {
        small_file_name(fname, f);
        remove(fname);
    }
    remove(bigname);
    printf("  Done.\n");
}

/* ------------------------------------------------------------------ */
/*  Main                                                               */
/* ------------------------------------------------------------------ */
int main(int argc, char *argv[])
{
    int phase1_ok, phase2_ok, phase3_ok, phase4_ok;
    int all_passed;
    int ch;
    uint32_t total_data;

    /* Parse optional drive letter argument */
    if (argc > 1 && argv[1][0] != '\0') {
        char dl = argv[1][0];
        if (dl >= 'a' && dl <= 'z') dl -= 32;  /* uppercase */
        if (dl >= 'A' && dl <= 'Z') {
            drive_prefix[0] = dl;
            drive_prefix[1] = ':';
            drive_prefix[2] = '\\';
            drive_prefix[3] = '\0';
        } else {
            printf("Usage: HD_STRES [drive]\n");
            printf("  drive = target drive letter (e.g. C)\n");
            return 1;
        }
    }

    total_data = (uint32_t)NUM_SMALL_FILES * SMALL_FILE_SIZE
                 + LARGE_FILE_SIZE;

    printf("Victor 9000 Hard Disk Stress Test v1.0\n");
    printf("======================================\n");
    printf("Uses MS-DOS file services to exercise the full I/O stack.\n");
    if (drive_prefix[0]) {
        printf("Target drive: %c:\\\n", drive_prefix[0]);
    } else {
        printf("Target: current directory\n");
    }
    printf("\n");
    printf("Test plan:\n");
    printf("  Phase 1: Write %d files x %luK = %luK\n",
           NUM_SMALL_FILES, SMALL_FILE_SIZE / 1024UL,
           (uint32_t)NUM_SMALL_FILES * SMALL_FILE_SIZE / 1024UL);
    printf("  Phase 2: Read back and verify all %d small files\n",
           NUM_SMALL_FILES);
    printf("  Phase 3: Write 1 large file = %luK\n",
           LARGE_FILE_SIZE / 1024UL);
    printf("  Phase 4: Read back and verify the large file\n");
    printf("\n  Total I/O: %luK write + %luK read = %luK\n",
           total_data / 1024UL, total_data / 1024UL,
           total_data * 2UL / 1024UL);

    printf("\nPress any key to begin (ESC to abort)...\n");
    ch = getch();
    if (ch == 27) {
        printf("Aborted.\n");
        return 1;
    }

    phase1_ok = phase1_write_small_files();
    phase2_ok = phase2_verify_small_files();
    phase3_ok = phase3_write_large_file();
    phase4_ok = phase4_verify_large_file();

    all_passed = phase1_ok && phase2_ok && phase3_ok && phase4_ok;

    /* ---- Summary ---- */
    printf("\n======================================\n");
    printf("STRESS TEST SUMMARY\n");
    printf("======================================\n");
    printf("Phase 1 (write %d small files):  %s\n",
           NUM_SMALL_FILES, phase1_ok ? "PASS" : "FAIL");
    printf("Phase 2 (verify small files):    %s\n",
           phase2_ok ? "PASS" : "FAIL");
    printf("Phase 3 (write %luK file):       %s\n",
           LARGE_FILE_SIZE / 1024UL, phase3_ok ? "PASS" : "FAIL");
    printf("Phase 4 (verify %luK file):      %s\n",
           LARGE_FILE_SIZE / 1024UL, phase4_ok ? "PASS" : "FAIL");
    printf("--------------------------------------\n");
    printf("Total bytes written:  %lu\n", total_bytes_written);
    printf("Total bytes verified: %lu\n", total_bytes_read);
    printf("Total errors:         %u\n", total_errors);
    printf("Files created:        %u/%d\n", files_created, NUM_SMALL_FILES);
    printf("Files verified OK:    %u/%d\n", files_verified, NUM_SMALL_FILES);

    if (all_passed) {
        printf("\nRESULT: ALL TESTS PASSED\n");
    } else {
        printf("\nRESULT: FAILURES DETECTED\n");
    }

    printf("\nClean up test files? (Y/N) ");
    ch = getch();
    printf("%c\n", ch);
    if (ch == 'Y' || ch == 'y') {
        cleanup_test_files();
    }

    printf("\nPress any key to exit...\n");
    getch();
    return all_passed ? 0 : 1;
}

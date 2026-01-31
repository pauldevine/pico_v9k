/*
 * psram_trace.c - Two-tier trace buffer using PSRAM
 *
 * PSRAM initialization based on MicroPython rp2_psram.c and Arduino-Pico implementations.
 * Uses QMI peripheral to access APS6404L PSRAM on Pimoroni PGA2350.
 */

#include "psram_trace.h"
#include "debug_queue.h"
#include "pico/stdlib.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/systick.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include <stdio.h>
#include <string.h>

// FatFS for SD card dump
#include "ff.h"
#include "f_util.h"

// PSRAM CS pin on PGA2350
#ifndef PIMORONI_PGA2350_PSRAM_CS_PIN
#define PIMORONI_PGA2350_PSRAM_CS_PIN 47
#endif

// QMI register field values for quad mode
#define QMI_WIDTH_SINGLE 0
#define QMI_WIDTH_DUAL   1
#define QMI_WIDTH_QUAD   2

// Global control structure - aligned for cache efficiency
psram_trace_ctrl_t psram_trace_ctrl __attribute__((aligned(64)));

// Helper to get pointer to trace entry in PSRAM
static inline psram_trace_entry_t* psram_entry(uint32_t index) {
    return (psram_trace_entry_t*)(PSRAM_BASE_ADDR + index * sizeof(psram_trace_entry_t));
}

// Timeout helper for QMI busy waits
static bool qmi_wait_ready(uint32_t timeout_us) {
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;  // Timeout
        }
        tight_loop_contents();
    }
    return true;
}

// Initialize PSRAM via QMI peripheral
static bool psram_qmi_init(void) {
    printf("PSRAM: Configuring GPIO 47 for CS1...\n");

    // Configure GPIO 47 for XIP CS1 function
    gpio_set_function(PIMORONI_PGA2350_PSRAM_CS_PIN, GPIO_FUNC_XIP_CS1);

    // Get system clock frequency for timing calculations
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    printf("PSRAM: System clock = %lu Hz\n", (unsigned long)sys_clk_hz);

    // Calculate clock divisor to keep PSRAM at ~100MHz (max 133MHz for APS6404L)
    // Use divisor 2 for 200MHz system clock -> 100MHz PSRAM clock
    uint32_t divisor = (sys_clk_hz + 100000000 - 1) / 100000000;
    if (divisor < 2) divisor = 2;  // Minimum divisor is 2
    printf("PSRAM: Using clock divisor = %lu\n", (unsigned long)divisor);

    printf("PSRAM: Enabling direct mode...\n");

    // Enable direct mode to send QPI enable command
    // Set clock divisor high initially for SPI mode command
    qmi_hw->direct_csr = (10 << QMI_DIRECT_CSR_CLKDIV_LSB) |
                         QMI_DIRECT_CSR_EN_BITS |
                         QMI_DIRECT_CSR_AUTO_CS1N_BITS;

    // Wait for any previous operation to complete (with timeout)
    if (!qmi_wait_ready(10000)) {
        printf("PSRAM: Timeout waiting for QMI ready (1)\n");
        qmi_hw->direct_csr = 0;
        return false;
    }

    printf("PSRAM: Sending QPI enable command...\n");

    // Send QPI Enable command (0x35) in SPI mode
    // NOPUSH means don't expect a response byte
    qmi_hw->direct_tx = QMI_DIRECT_TX_NOPUSH_BITS |
                        QMI_DIRECT_TX_OE_BITS |
                        0x35;

    // Wait for command to complete (with timeout)
    if (!qmi_wait_ready(10000)) {
        printf("PSRAM: Timeout waiting for QMI ready (2)\n");
        qmi_hw->direct_csr = 0;
        return false;
    }

    // Disable direct mode
    qmi_hw->direct_csr = 0;

    printf("PSRAM: Configuring M1 timing and format...\n");

    // Configure M1 timing for PSRAM
    // Based on APS6404L datasheet and MicroPython implementation
    qmi_hw->m[1].timing =
        (1 << QMI_M1_TIMING_COOLDOWN_LSB) |         // 1 cycle cooldown
        (1 << QMI_M1_TIMING_PAGEBREAK_LSB) |        // 1024-byte page break
        (0 << QMI_M1_TIMING_SELECT_SETUP_LSB) |     // No extra setup
        (0 << QMI_M1_TIMING_SELECT_HOLD_LSB) |      // No extra hold
        (0 << QMI_M1_TIMING_MAX_SELECT_LSB) |       // No max select limit
        (1 << QMI_M1_TIMING_MIN_DESELECT_LSB) |     // Min deselect time
        (divisor << QMI_M1_TIMING_RXDELAY_LSB) |    // RX sample delay
        (divisor << QMI_M1_TIMING_CLKDIV_LSB);      // Clock divisor

    // Configure read format for Quad I/O (command 0xEB)
    qmi_hw->m[1].rfmt =
        (QMI_WIDTH_QUAD << QMI_M1_RFMT_PREFIX_WIDTH_LSB) |   // Quad command
        (QMI_WIDTH_QUAD << QMI_M1_RFMT_ADDR_WIDTH_LSB) |     // Quad address
        (QMI_WIDTH_QUAD << QMI_M1_RFMT_SUFFIX_WIDTH_LSB) |   // Quad suffix
        (QMI_WIDTH_QUAD << QMI_M1_RFMT_DUMMY_WIDTH_LSB) |    // Quad dummy
        (QMI_WIDTH_QUAD << QMI_M1_RFMT_DATA_WIDTH_LSB) |     // Quad data
        (1 << QMI_M1_RFMT_PREFIX_LEN_LSB) |                  // 8-bit command
        (0 << QMI_M1_RFMT_SUFFIX_LEN_LSB) |                  // No suffix
        (6 << QMI_M1_RFMT_DUMMY_LEN_LSB);                    // 6 dummy cycles

    // Read command: Fast Read Quad I/O (0xEB)
    qmi_hw->m[1].rcmd = 0xEB;

    // Configure write format for Quad Page Program (command 0x38)
    qmi_hw->m[1].wfmt =
        (QMI_WIDTH_QUAD << QMI_M1_WFMT_PREFIX_WIDTH_LSB) |   // Quad command
        (QMI_WIDTH_QUAD << QMI_M1_WFMT_ADDR_WIDTH_LSB) |     // Quad address
        (QMI_WIDTH_QUAD << QMI_M1_WFMT_SUFFIX_WIDTH_LSB) |   // Quad suffix
        (QMI_WIDTH_QUAD << QMI_M1_WFMT_DUMMY_WIDTH_LSB) |    // Quad dummy
        (QMI_WIDTH_QUAD << QMI_M1_WFMT_DATA_WIDTH_LSB) |     // Quad data
        (1 << QMI_M1_WFMT_PREFIX_LEN_LSB) |                  // 8-bit command
        (0 << QMI_M1_WFMT_SUFFIX_LEN_LSB) |                  // No suffix
        (0 << QMI_M1_WFMT_DUMMY_LEN_LSB);                    // No dummy for write

    // Write command: Quad Page Program (0x38)
    qmi_hw->m[1].wcmd = 0x38;

    // Enable XIP writes to M1 (PSRAM window)
    xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;

    return true;
}

// Verify PSRAM is working with a simple read/write test
static bool psram_verify(void) {
    printf("PSRAM: Running verification test...\n");

    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE_ADDR;

    // Test pattern
    const uint32_t test_pattern = 0xDEADBEEF;
    const uint32_t test_offset = 0x1000;  // Test at 4KB offset (word index)

    printf("PSRAM: Reading from 0x%08X...\n", (unsigned)(PSRAM_BASE_ADDR + test_offset * 4));

    // Save original value
    uint32_t original = psram[test_offset];
    printf("PSRAM: Original value = 0x%08lX\n", (unsigned long)original);

    // Write test pattern
    printf("PSRAM: Writing test pattern 0x%08lX...\n", (unsigned long)test_pattern);
    psram[test_offset] = test_pattern;
    __dmb();  // Memory barrier

    // Read back and verify
    uint32_t readback = psram[test_offset];
    printf("PSRAM: Read back = 0x%08lX\n", (unsigned long)readback);

    // Restore original
    psram[test_offset] = original;
    __dmb();

    if (readback == test_pattern) {
        printf("PSRAM: Verification PASSED\n");
        return true;
    } else {
        printf("PSRAM: Verification FAILED (expected 0x%08lX, got 0x%08lX)\n",
               (unsigned long)test_pattern, (unsigned long)readback);
        return false;
    }
}

bool psram_trace_init(void) {
    printf("PSRAM trace: Starting initialization...\n");

    // Clear control structure
    memset(&psram_trace_ctrl, 0, sizeof(psram_trace_ctrl));

#ifdef PSRAM_TRACE_DISABLED
    // PSRAM disabled at compile time - skip all initialization
    printf("PSRAM trace: Disabled at compile time\n");
    return false;
#endif

    // Initialize PSRAM via QMI
    if (!psram_qmi_init()) {
        printf("PSRAM trace: QMI init failed - tracing disabled\n");
        return false;
    }

    // Verify PSRAM is accessible
    if (!psram_verify()) {
        printf("PSRAM trace: Verification failed - tracing disabled\n");
        return false;
    }

    // Clear the trace buffer area (first few entries as sanity check)
    volatile uint32_t *psram = (volatile uint32_t *)PSRAM_BASE_ADDR;
    for (int i = 0; i < 256; i++) {
        psram[i] = 0;
    }
    __dmb();

    psram_trace_ctrl.initialized = true;
    printf("PSRAM trace: Initialized %lu entries (%lu KB) at 0x%08X\n",
           (unsigned long)PSRAM_TRACE_ENTRIES,
           (unsigned long)(PSRAM_TRACE_SIZE / 1024),
           PSRAM_BASE_ADDR);

    return true;
}

void psram_trace_drain(void) {
    if (!psram_trace_ctrl.initialized || psram_trace_ctrl.paused) {
        return;
    }

    uint32_t value;
    uint32_t drained = 0;

    // Drain all available entries from debug_queue to PSRAM
    while (debug_queue_pop(&value)) {
        uint32_t head = psram_trace_ctrl.head;

        // Write entry to PSRAM
        psram_trace_entry_t *entry = psram_entry(head);
        entry->timestamp = systick_hw->cvr;  // Current SysTick value
        entry->payload = value;

        // Advance head (circular buffer)
        psram_trace_ctrl.head = (head + 1) & PSRAM_TRACE_MASK;
        psram_trace_ctrl.count++;

        // Check for overflow (head caught up to tail)
        if (psram_trace_ctrl.head == psram_trace_ctrl.tail) {
            // Move tail forward, dropping oldest entry
            psram_trace_ctrl.tail = (psram_trace_ctrl.tail + 1) & PSRAM_TRACE_MASK;
            psram_trace_ctrl.overflow++;
        }

        drained++;
    }

    // Memory barrier to ensure writes complete
    if (drained > 0) {
        __dmb();
    }
}

uint32_t psram_trace_count(void) {
    if (!psram_trace_ctrl.initialized) {
        return 0;
    }

    uint32_t head = psram_trace_ctrl.head;
    uint32_t tail = psram_trace_ctrl.tail;

    if (head >= tail) {
        return head - tail;
    } else {
        return PSRAM_TRACE_ENTRIES - tail + head;
    }
}

void psram_trace_clear(void) {
    psram_trace_ctrl.head = 0;
    psram_trace_ctrl.tail = 0;
    psram_trace_ctrl.count = 0;
    psram_trace_ctrl.overflow = 0;
    __dmb();
    printf("PSRAM trace: Cleared\n");
}

void psram_trace_status(void) {
    if (!psram_trace_ctrl.initialized) {
        printf("PSRAM trace: Not initialized\n");
        return;
    }

    printf("PSRAM trace status:\n");
    printf("  Entries: %lu / %lu\n",
           (unsigned long)psram_trace_count(),
           (unsigned long)PSRAM_TRACE_ENTRIES);
    printf("  Total written: %lu\n", (unsigned long)psram_trace_ctrl.count);
    printf("  Overflow drops: %lu\n", (unsigned long)psram_trace_ctrl.overflow);
    printf("  Paused: %s\n", psram_trace_ctrl.paused ? "yes" : "no");
    printf("  Head: %lu, Tail: %lu\n",
           (unsigned long)psram_trace_ctrl.head,
           (unsigned long)psram_trace_ctrl.tail);
}

void psram_trace_toggle_pause(void) {
    psram_trace_ctrl.paused = !psram_trace_ctrl.paused;
    printf("PSRAM trace: %s\n", psram_trace_ctrl.paused ? "Paused" : "Resumed");
}

void psram_trace_dump_uart(uint32_t count) {
    if (!psram_trace_ctrl.initialized) {
        printf("PSRAM trace: Not initialized\n");
        return;
    }

    uint32_t available = psram_trace_count();
    if (count == 0 || count > available) {
        count = available;
    }

    if (count == 0) {
        printf("PSRAM trace: No entries to dump\n");
        return;
    }

    printf("PSRAM trace: Dumping %lu entries\n", (unsigned long)count);

    // Calculate starting position (dump most recent entries)
    uint32_t start;
    if (available == count) {
        start = psram_trace_ctrl.tail;
    } else {
        // Start from (head - count) to get most recent
        start = (psram_trace_ctrl.head - count) & PSRAM_TRACE_MASK;
    }

    // Dump entries in chronological order
    for (uint32_t i = 0; i < count; i++) {
        uint32_t idx = (start + i) & PSRAM_TRACE_MASK;
        psram_trace_entry_t *entry = psram_entry(idx);

        // Format: "TRACE t=XXXXXXXX p=XXXXXXXX"
        printf("TRACE t=%08lX p=%08lX\n",
               (unsigned long)entry->timestamp,
               (unsigned long)entry->payload);
    }

    printf("PSRAM trace: Dump complete\n");
}

bool psram_trace_dump_sd(const char *filename) {
    if (!psram_trace_ctrl.initialized) {
        printf("PSRAM trace: Not initialized\n");
        return false;
    }

    uint32_t count = psram_trace_count();
    if (count == 0) {
        printf("PSRAM trace: No entries to dump\n");
        return true;  // Not an error, just nothing to do
    }

    FIL file;
    FRESULT fr = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) {
        printf("PSRAM trace: Failed to open %s (error %d)\n", filename, fr);
        return false;
    }

    printf("PSRAM trace: Dumping %lu entries to %s...\n", (unsigned long)count, filename);

    // Write a simple header: magic + count
    uint32_t header[2] = { 0x50545243, count };  // "PTRC" magic
    UINT bw;
    fr = f_write(&file, header, sizeof(header), &bw);
    if (fr != FR_OK || bw != sizeof(header)) {
        printf("PSRAM trace: Header write failed\n");
        f_close(&file);
        return false;
    }

    // Write entries in batches for efficiency
    const uint32_t BATCH_SIZE = 64;
    uint32_t start = psram_trace_ctrl.tail;
    uint32_t written = 0;

    while (written < count) {
        uint32_t batch = (count - written > BATCH_SIZE) ? BATCH_SIZE : (count - written);

        // Copy batch to temporary buffer (PSRAM reads may be slow)
        psram_trace_entry_t batch_buf[BATCH_SIZE];
        for (uint32_t i = 0; i < batch; i++) {
            uint32_t idx = (start + written + i) & PSRAM_TRACE_MASK;
            psram_trace_entry_t *entry = psram_entry(idx);
            batch_buf[i] = *entry;
        }

        fr = f_write(&file, batch_buf, batch * sizeof(psram_trace_entry_t), &bw);
        if (fr != FR_OK || bw != batch * sizeof(psram_trace_entry_t)) {
            printf("PSRAM trace: Write failed at entry %lu\n", (unsigned long)written);
            f_close(&file);
            return false;
        }

        written += batch;
    }

    f_close(&file);
    printf("PSRAM trace: Dump complete (%lu bytes)\n",
           (unsigned long)(sizeof(header) + count * sizeof(psram_trace_entry_t)));

    // Clear the buffer after successful dump
    psram_trace_clear();

    return true;
}

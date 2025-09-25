#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "../pico_victor/dma.h"
#include "../sasi.h"
#include "../pico_fujinet/spi.h"

static char test_summary[1024];
static size_t test_summary_len;

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

void initialize_uart() {
    // Initialize UART with high priority
    gpio_init(UART_TX_PIN);
    gpio_init(UART_RX_PIN);

    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    uart_set_fifo_enabled(UART_ID, false);

    return;
}

static void append_test_summary(const char *fmt, ...) {
    if (test_summary_len >= sizeof(test_summary)) {
        return;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(test_summary + test_summary_len,
                            sizeof(test_summary) - test_summary_len,
                            fmt,
                            args);
    va_end(args);

    if (written < 0) {
        return;
    }

    if ((size_t)written >= sizeof(test_summary) - test_summary_len) {
        test_summary_len = sizeof(test_summary) - 1;
        test_summary[test_summary_len] = '\0';
    } else {
        test_summary_len += (size_t)written;
    }
}

// Simple test macros
#define ASSERT_EQ(msg, a, b) do { \
    unsigned _assert_eq_a = (unsigned)(a); \
    unsigned _assert_eq_b = (unsigned)(b); \
    if (_assert_eq_a != _assert_eq_b) { \
        append_test_summary("FAIL: %s (0x%02X != 0x%02X)\n", msg, _assert_eq_a, _assert_eq_b); \
        printf("FAIL: %s (0x%02X != 0x%02X)\n", msg, _assert_eq_a, _assert_eq_b); \
        return false; \
    } \
} while (0)

#define ASSERT_TRUE(msg, cond) do { \
    bool _assert_true_cond = (cond); \
    if (!_assert_true_cond) { \
        append_test_summary("FAIL: %s\n", msg); \
        printf("FAIL: %s\n", msg); \
        return false; \
    } \
} while (0)

static bool test_selection_and_status() {
    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);

    // Write target ID on data bus, then assert SELECT
    dma_write_register(dma, REG_DATA, 0x01);
    dma_write_register(dma, REG_CONTROL, 0x14); // SELECT + latch

    // Status should show BSY (0x04)
    uint8_t st = dma_read_register(dma, REG_STATUS);
    ASSERT_EQ("BSY after select", st & 0x1F, 0x04);

    // Drop select, expect command REQ (0x0E)
    dma_write_register(dma, REG_CONTROL, 0x04);
    st = dma_read_register(dma, REG_STATUS);
    ASSERT_EQ("REQ+CTL after select drop", st & 0x1F, 0x0E);
    return true;
}

// FujiNet stubs for tests (force read to use fallback pattern)
bool __attribute__((weak)) fujinet_read_sector(uint8_t device, uint32_t lba, uint8_t *buffer, size_t len) {
    (void)device; (void)lba; (void)buffer; (void)len;
    return false; // cause sasi.c to use deterministic fallback
}

bool __attribute__((weak)) fujinet_write_sector(uint8_t device, uint32_t lba, const uint8_t *buffer, size_t len) {
    (void)device; (void)lba; (void)buffer; (void)len;
    return true; // accept writes
}

#define TEST_FUJINET_KNOWN_LBA 0x00000002u
static const uint8_t EXPECTED_FUJINET_PREFIX[32] = {
    0x01, 0x00, 0x56, 0x4F, 0x4C, 0x55, 0x4D, 0x45, 
    0x20, 0x31, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
    0x20, 0x20, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x0C, 0x13, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E
};

static bool test_spi_known_sector(void) {
    printf("\n-- FujiNet SPI known sector smoke test --\n");

    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);
    dma->selected_target = 0; // Victor host selects target 0 for disk 0

    uint8_t sector[512] = {0};
    read_sector_from_disk(dma, TEST_FUJINET_KNOWN_LBA, sector);

    for (size_t i = 0; i < sizeof(EXPECTED_FUJINET_PREFIX); i++) {
        char msg[52];
        snprintf(msg, sizeof(msg), "FujiNet prefix byte %u", (unsigned)i);
        ASSERT_EQ(msg, sector[i], EXPECTED_FUJINET_PREFIX[i]);
    }

    return true;
}

static bool test_xebec_diag_sequence() {
    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);

    // Initiate selection & command phase
    dma_write_register(dma, REG_DATA, 0x01);
    dma_write_register(dma, REG_CONTROL, 0x14);
    dma_write_register(dma, REG_CONTROL, 0x04);

    // Send 6-byte CDB: E0 00 AA 55 00 00
    uint8_t cdb[6] = {0xE0, 0x00, 0xAA, 0x55, 0x00, 0x00};
    for (int i = 0; i < 6; i++) {
        dma_write_register(dma, REG_DATA, cdb[i]);
    }

    // Expect status phase (0x0F)
    uint8_t st = dma_read_register(dma, REG_STATUS);
    ASSERT_EQ("Status phase after diag", st & 0x1F, 0x0F);

    // Read status byte (0x00)
    uint8_t status_byte = dma_read_register(dma, REG_DATA);
    ASSERT_EQ("Diag GOOD status", status_byte, 0x00);

    // Expect message phase (0x1F) then 0x00
    st = dma_read_register(dma, REG_STATUS);
    ASSERT_EQ("Message phase after status", st & 0x1F, 0x1F);
    uint8_t msg = dma_read_register(dma, REG_DATA);
    ASSERT_EQ("Command complete message", msg, 0x00);
    return true;
}

static bool test_read6_one_sector() {
    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);

    // Program DMA destination address
    dma_write_register(dma, REG_ADDR_H, 0x00);
    dma_write_register(dma, REG_ADDR_M, 0x20);
    dma_write_register(dma, REG_ADDR_L, 0x00);

    // Selection & command phase
    dma_write_register(dma, REG_DATA, 0x00);
    dma_write_register(dma, REG_CONTROL, 0x14);
    dma_write_register(dma, REG_CONTROL, 0x04);

    // CDB: 0x03 00 00 02 01 00 -> Read 1 sector at LBA 0x000002
    uint8_t cdb[6] = {0x03, 0x00, 0x00, 0x02, 0x01, 0x00};
    for (int i = 0; i < 6; i++) dma_write_register(dma, REG_DATA, cdb[i]);

    // After completion, first 32 bytes should match the expected Volume Label payload
    uint8_t *mem = test_get_victor_ram();
    uint32_t addr = (0x00 << 16) | (0x20 << 8) | 0x00;
    static const uint8_t expected_volume_header[32] = {
        0x01, 0x00, 0x56, 0x4F, 0x4C, 0x55, 0x4D, 0x45,
        0x20, 0x31, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20,
        0x20, 0x20, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0C, 0x13, 0x00, 0x00, 0x00, 0x00, 0x20, 0x4E
    };
    for (size_t i = 0; i < sizeof(expected_volume_header); i++) {
        char msg[48];
        snprintf(msg, sizeof(msg), "Read6 volume byte %u", (unsigned)i);
        ASSERT_EQ(msg, mem[addr + i], expected_volume_header[i]);
    }
    return true;
}

static bool test_request_sense() {
    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);

    // Program DMA destination address
    dma_write_register(dma, REG_ADDR_H, 0x00);
    dma_write_register(dma, REG_ADDR_M, 0x10);
    dma_write_register(dma, REG_ADDR_L, 0x00);

    // Selection & command phase
    dma_write_register(dma, REG_DATA, 0x00);
    dma_write_register(dma, REG_CONTROL, 0x14);
    dma_write_register(dma, REG_CONTROL, 0x04);

    // CDB: Request Sense (0x03) alloc len 0x12
    uint8_t cdb[6] = {0x03, 0x00, 0x00, 0x00, 0x12, 0x00};
    for (int i = 0; i < 6; i++) dma_write_register(dma, REG_DATA, cdb[i]);

    // Verify zeros written
    uint8_t *mem = test_get_victor_ram();
    uint32_t addr = (0x00 << 16) | (0x10 << 8) | 0x00;
    for (int i = 0; i < 0x12; i++) ASSERT_EQ("Request Sense zeros", mem[addr + i], 0x00);
    return true;
}

static bool test_mode_select() {
    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);

    // Prepare parameter list at 0x003000
    uint8_t *mem = test_get_victor_ram();
    uint32_t addr = (0x00 << 16) | (0x30 << 8) | 0x00;
    for (int i = 0; i < 8; i++) mem[addr + i] = (uint8_t)i;

    // Program DMA source address
    dma_write_register(dma, REG_ADDR_H, 0x00);
    dma_write_register(dma, REG_ADDR_M, 0x30);
    dma_write_register(dma, REG_ADDR_L, 0x00);

    // Selection & command phase
    dma_write_register(dma, REG_DATA, 0x00);
    dma_write_register(dma, REG_CONTROL, 0x14);
    dma_write_register(dma, REG_CONTROL, 0x04);

    // CDB: Mode Select(6) (0x15) param length 8
    uint8_t cdb[6] = {0x15, 0x00, 0x00, 0x00, 0x08, 0x00};
    for (int i = 0; i < 6; i++) dma_write_register(dma, REG_DATA, cdb[i]);

    // Expect DMA address incremented by 8
    ASSERT_EQ("Mode Select dma addr low", dma->dma_address.bytes.low, 0x08);
    ASSERT_EQ("Mode Select dma addr mid", dma->dma_address.bytes.mid, 0x30);
    ASSERT_EQ("Mode Select dma addr high", dma->dma_address.bytes.high & 0x0F, 0x00);
    return true;
}

int main() {
    stdio_init_all();
    sleep_ms(500);

    set_sys_clock_khz(200000, true);

    initialize_uart();    

    printf("\nVictor DMA On-Device Tests\n");

    bool ok = true;
    // Initialize SPI bus for FujiNet storage
    spi_bus_init();

    if (!fujinet_config_boot(false)) {
        printf("FAIL: FujiNet CONFIG boot disable\n");
        ok = false;
    }

    if (!fujinet_mount_host(0, FUJINET_DISK_ACCESS_READ)) {
        printf("FAIL: FujiNet mount host 0\n");
        ok = false;
    }

    if (!fujinet_mount_disk_slot(0, FUJINET_DISK_ACCESS_READ)) {
        printf("FAIL: FujiNet mount disk slot 0\n");
        ok = false;
    }

    ok &= test_selection_and_status();
    ok &= test_xebec_diag_sequence();
    ok &= test_read6_one_sector();
    ok &= test_request_sense();
    ok &= test_mode_select();
    ok &= test_spi_known_sector();

    if (test_summary_len > 0) {
        printf("\nTest Summary:\n%s", test_summary);
    }

    printf("\nRESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

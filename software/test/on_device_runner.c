#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"

#include "../pico_victor/dma.h"
#include "../sasi.h"
#include "../pico_fujinet/spi.h"

// Simple test macros
#define ASSERT_EQ(msg, a, b) do { \
    if ((a) != (b)) { \
        printf("FAIL: %s (0x%02X != 0x%02X)\n", msg, (unsigned)(a), (unsigned)(b)); \
        return false; \
    } \
} while (0)

#define ASSERT_TRUE(msg, cond) do { \
    if (!(cond)) { printf("FAIL: %s\n", msg); return false; } \
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
static const uint8_t EXPECTED_FUJINET_PREFIX[8] = {
    0xFA, 0xCE, 0x90, 0x0D, 0xF0, 0x0D, 0xBA, 0xBE
};

static bool test_spi_known_sector(void) {
    printf("\n-- FujiNet SPI known sector smoke test --\n");

    dma_registers_t *dma = dma_get_registers();
    dma_device_reset(dma);
    dma->selected_target = 0; // Victor host selects target 0 for disk 0

    uint8_t sector[512] = {0};
    read_sector_from_disk(dma, TEST_FUJINET_KNOWN_LBA, sector);

    for (size_t i = 0; i < sizeof(EXPECTED_FUJINET_PREFIX); i++) {
        char msg[48];
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

    // CDB: 0x03 00 02 00 01 00 -> Read 1 sector at LBA 0x000200
    uint8_t cdb[6] = {0x03, 0x00, 0x02, 0x00, 0x01, 0x00};
    for (int i = 0; i < 6; i++) dma_write_register(dma, REG_DATA, cdb[i]);

    // After completion, first 16 bytes at destination should match deterministic fallback pattern
    uint8_t *mem = test_get_victor_ram();
    uint32_t addr = (0x00 << 16) | (0x20 << 8) | 0x00;
    for (int i = 0; i < 16; i++) {
        uint8_t expected = (uint8_t)((0x000200 + i) & 0xFF);
        ASSERT_EQ("Read6 data pattern", mem[addr + i], expected);
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

    printf("\nRESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

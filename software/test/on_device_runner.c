#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/clocks.h"

#include "../pico_fujinet/spi.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

static void initialize_uart(void) {
    gpio_init(UART_TX_PIN);
    gpio_init(UART_RX_PIN);

    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    uart_set_fifo_enabled(UART_ID, false);
}

// Helper to print a hex dump of a buffer
static void hex_dump(const uint8_t *data, size_t len, uint32_t base_addr) {
    for (size_t i = 0; i < len; i += 16) {
        printf("%08X: ", (unsigned)(base_addr + i));
        // Hex bytes
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
            printf("%02X ", data[i + j]);
        }
        // Padding if less than 16 bytes
        for (size_t j = len - i; j < 16 && (i + j) >= len; j++) {
            printf("   ");
        }
        // ASCII representation
        printf(" |");
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
            uint8_t c = data[i + j];
            printf("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
        }
        printf("|\n");
    }
}

static bool test_spi_read_sectors(void) {
    printf("\n-- FujiNet SPI sector read test --\n");
    printf("Reading first 3 sectors from disk image via SPI...\n\n");

    uint8_t sector[512];

    for (uint32_t lba = 0; lba < 3; lba++) {
        memset(sector, 0, sizeof(sector));

        printf("=== Sector %u (LBA 0x%08X) ===\n", (unsigned)lba, (unsigned)lba);

        bool success = fujinet_read_sector(DEVICE_DISK_BASE, lba, sector, 512);

        if (!success) {
            printf("FAIL: Could not read sector %u from FujiNet\n", (unsigned)lba);
            return false;
        }

        printf("Successfully read sector %u\n", (unsigned)lba);
        hex_dump(sector, 512, lba * 512);
        printf("\n");
    }

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

    // Run the SPI sector read test
    ok &= test_spi_read_sectors();

    printf("\nRESULT: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

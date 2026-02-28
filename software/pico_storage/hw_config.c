/* hw_config.c
 * Hardware configuration for SD card SPI interface
 * Required by no-OS-FatFS-SD-SDIO-SPI-RPi-Pico library
 *
 * Pin configuration for Pimoroni PGA2350 (RP2350) using hardware SPI1:
 *   CS   = GPIO 41 (manual GPIO)   -> SD DAT3/CS
 *   SCK  = GPIO 42 (SPI1_SCK)      -> SD CLK
 *   MOSI = GPIO 43 (SPI1_TX)       -> SD CMD/DI
 *   MISO = GPIO 44 (SPI1_RX)       -> SD DAT0/DO
 *
 * Uses hardware SPI peripheral (no PIO, no DMA IRQ).
 */

#include "hw_config.h"
#include "sd_card.h"
#include "../pico_victor/dma.h"

/* SPI hardware instance configuration.
 * Uses SPI1 peripheral on RP2350 — completely independent of PIO.
 * The library handles slow-clock init (100-400 kHz) internally,
 * then switches to this baud_rate for data transfers. */
static spi_t spi = {
    .hw_inst = spi1,
    .sck_gpio = SD_SCK_PIN,
    .mosi_gpio = SD_MOSI_PIN,
    .miso_gpio = SD_MISO_PIN,
    .baud_rate = 25 * 1000 * 1000,   // 25 MHz — standard SD SPI max
};

/* SPI interface configuration (slave select / chip select) */
static sd_spi_if_t spi_if = {
    .spi = &spi,
    .ss_gpio = SD_CS_PIN,
};

/* Hardware Configuration of the SD Card socket "object" */
static sd_card_t sd_card = {
    .type = SD_IF_SPI,
    .spi_if_p = &spi_if,
    // Card detect not used - assume card is always present
    .use_card_detect = false,
};

/* Callbacks used by the library */

size_t sd_get_num(void) {
    return 1;
}

sd_card_t* sd_get_by_num(size_t num) {
    if (0 == num) {
        return &sd_card;
    } else {
        return NULL;
    }
}

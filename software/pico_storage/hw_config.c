/* hw_config.c
 * Hardware configuration for SD card SDIO interface
 * Required by no-OS-FatFS-SD-SDIO-SPI-RPi-Pico library
 *
 * Pin configuration for Pimoroni PGA2350 (RP2350):
 *   CLK  = GPIO 40
 *   CMD  = GPIO 41
 *   D0   = GPIO 42
 *   D1   = GPIO 43 (auto-calculated)
 *   D2   = GPIO 44 (auto-calculated)
 *   D3   = GPIO 45 (auto-calculated)
 *   PIO  = pio1
 *   DMA  = DMA_IRQ_1
 */

#include "hw_config.h"
#include "sd_card.h"

/* SDIO Interface Configuration
 * For RP2350 with GPIOs >= 32, CLK_gpio must be specified explicitly
 * since the normal D0+offset calculation doesn't work for pins above 32.
 *
 * The relationship is: CLK = D0 - 2 (in mod32 arithmetic)
 * For D0=42, CLK should be 40.
 */
static sd_sdio_if_t sdio_if = {
    .CLK_gpio = 40,           // Must specify explicitly for GPIOs >= 32
    .CMD_gpio = 41,
    .D0_gpio = 42,            // D1=43, D2=44, D3=45 are auto-calculated
    .SDIO_PIO = pio2,
    .DMA_IRQ_num = DMA_IRQ_1,
    .baud_rate = 15 * 1000 * 1000,  // 15 MHz - conservative speed
};

/* Hardware Configuration of the SD Card socket "object" */
static sd_card_t sd_card = {
    .type = SD_IF_SDIO,
    .sdio_if_p = &sdio_if,
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

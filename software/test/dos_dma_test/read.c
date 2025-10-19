/*
 * read.c - Single DMA register read test
 *
 * This is the absolute minimal test - just one read from register 0xC0
 * to help debug PIO timing issues without multiple operations.
 */

#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <i86.h>

// DMA board base address
#define DMA_BASE 0xEF300

// Register offsets
#define REG_ADDR_H  0xC0  // High byte of DMA address (4 bits)

// Physical segment calculation
#define SEGMENT (DMA_BASE >> 4)
#define OFFSET(reg) ((DMA_BASE & 0x0F) + (reg))

int main(void) {
    unsigned char value;
    unsigned char far *reg_addr_h;

    printf("Single DMA Register Read Test\n");
    printf("=============================\n\n");

    // Calculate far pointer to register
    reg_addr_h = (unsigned char far *) MK_FP(SEGMENT, OFFSET(REG_ADDR_H));
    printf("Pointer = %04X:%04X\n", FP_SEG(reg_addr_h), FP_OFF(reg_addr_h));

    printf("Reading from DMA_ADDR_HIGH (0xC0)\n");
    printf("Physical address: 0x%05lX\n\n",
           ((unsigned long)SEGMENT << 4) + OFFSET(REG_ADDR_H));

    // Perform the single read
    value = *reg_addr_h;

    printf("Read value: 0x%02X\n", value);

    printf("\n=============================\n");
    printf("Test complete. Press any key to exit.\n");

    while (!kbhit()) {
        // Wait for keypress
    }
    getch();  // Consume the keypress

    return 0;
}

/*
 * addr_mid.c - Minimal DMA_ADDR_MID write/read verification
 *
 * This simple DOS program isolates the mid-byte DMA address register
 * behaviour by performing two write/read cycles: 0x55 and 0xAA.
 *
 * Usage: run under DOS (or DOSBox) after building with OpenWatcom.
 */

#include <stdio.h>
#include <dos.h>
#include <conio.h>
#include <i86.h>

// DMA board base address
#define DMA_BASE 0xEF300

// Register offset for the mid byte
#define REG_ADDR_H 0xC0
#define REG_ADDR_M 0xA0
#define REG_ADDR_L 0x80

// Helpers to translate the physical address
#define SEGMENT (DMA_BASE >> 4)
#define OFFSET(reg) ((DMA_BASE & 0x0F) + (reg))

static void write_and_read(unsigned char far *reg, unsigned char value) {
    unsigned char read_back;

    if (FP_OFF(reg) == REG_ADDR_M) {
        printf("Testing DMA_ADDR_MID (0x%02X):\n", REG_ADDR_M);
    } else if (FP_OFF(reg) == REG_ADDR_L) {
        printf("Testing DMA_ADDR_LOW (0x%02X):\n", REG_ADDR_L);
    } else if (FP_OFF(reg) == REG_ADDR_H) {
        printf("Testing DMA_ADDR_HIGH (0x%02X):\n", REG_ADDR_H);
    }
    printf("Writing 0x%02X to register...\n", value);
    *reg = value;

    read_back = *reg;
    printf("Read back: 0x%02X %s\n\n", read_back,
           (read_back == value) ? "[MATCH]" : "[MISMATCH]");
}

int main(void) {
    unsigned char far *reg_addr_l;
    unsigned char far *reg_addr_m;
    unsigned char far *reg_addr_h;

    printf("DMA_ADDR_MID Write/Read Test\n");
    printf("============================\n\n");

    reg_addr_l = (unsigned char far *)MK_FP(SEGMENT, OFFSET(REG_ADDR_L));
    reg_addr_m = (unsigned char far *)MK_FP(SEGMENT, OFFSET(REG_ADDR_M));
    reg_addr_h = (unsigned char far *)MK_FP(SEGMENT, OFFSET(REG_ADDR_H));

    printf("Pointer = %04X:%04X\n", FP_SEG(reg_addr_m), FP_OFF(reg_addr_m));
    printf("Physical address: 0x%05lX\n\n",
           ((unsigned long)SEGMENT << 4) + OFFSET(REG_ADDR_M));

    write_and_read(reg_addr_l, 0x55);
    write_and_read(reg_addr_m, 0x55);
    write_and_read(reg_addr_h, 0x55);

    write_and_read(reg_addr_l, 0xAA);
    write_and_read(reg_addr_m, 0xAA);
    write_and_read(reg_addr_h, 0xAA);

    printf("Test complete. Press any key to exit.\n");
    while (!kbhit()) {
        // spin until keypress
    }
    getch();

    return 0;
}

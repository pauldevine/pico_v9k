/*
 * sasi_perf_test.c - SASI State Machine Performance Test
 *
 * This DOS utility replays the first 250 register operations from the MAME
 * boot log to validate that the Pico SASI state machine matches MAME's
 * behavior. For status register reads expecting phase transitions, it polls
 * like real BIOS code does and measures how many iterations are needed.
 *
 * Goals:
 * 1. Validate SASI state machine behavior matches MAME
 * 2. Measure poll iterations needed for phase transitions (realistic metric)
 * 3. Execute operations efficiently with minimal I/O during test phase
 *
 * Build with OpenWatcom using the Makefile in this directory.
 */

#include <conio.h>
#include <dos.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>

/* DMA board register addresses */
#define DMA_BASE        0xEF30
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* Operation types */
#define OP_WRITE        0
#define OP_READ         1
#define OP_POLL_STATUS  2   /* Poll status register until expected value */

/* Validation flags for read operations */
#define VAL_EXACT       0   /* Must match exactly */
#define VAL_FLEXIBLE    1   /* Status register - informational only */

/* Poll limits and masks */
#define MAX_POLL_ITERATIONS 50000U  /* Max iterations before timeout */
#define STATUS_PHASE_MASK   0x1F    /* Lower 5 bits are phase */

/* Single test operation from MAME log */
typedef struct {
    uint8_t op_type;        /* OP_WRITE, OP_READ, or OP_POLL_STATUS */
    uint8_t offset;         /* Register offset (0x00, 0x10, 0x20, 0x80, 0xa0, 0xc0) */
    uint8_t data;           /* Data to write, or expected result for reads */
    uint8_t val_type;       /* Validation type for reads */
} test_op_t;

/* Result storage for operations */
typedef struct {
    uint8_t actual;         /* Actual value read */
    uint8_t expected;       /* Expected value */
    uint16_t poll_count;    /* Number of polls needed (for OP_POLL_STATUS) */
    uint8_t timed_out;      /* 1 if poll timed out */
} op_result_t;

/*
 * Test sequence from MAME boot example log (first ~250 ops)
 *
 * Key insight: Real BIOS polls status in loops. We use OP_POLL_STATUS for
 * status reads that expect a phase CHANGE (e.g., command->status->message).
 * Regular OP_READ is used for status reads that confirm current phase.
 *
 * Format: {OP_TYPE, offset, data, validation_type}
 */
static const test_op_t test_sequence[] = {
    /* Line 1-18: Reset and address register test pattern */
    {OP_WRITE, 0x00, 0x00, 0},              /* 1: Control = 0x00 */
    {OP_WRITE, 0x00, 0x04, 0},              /* 2: Control = 0x04 */
    {OP_WRITE, 0xc0, 0x05, 0},              /* 3: DMA addr high = 0x05 */
    {OP_WRITE, 0xa0, 0xaa, 0},              /* 4: DMA addr mid = 0xaa */
    {OP_WRITE, 0x80, 0x55, 0},              /* 5: DMA addr low = 0x55 */
    {OP_READ,  0xc0, 0x05, VAL_EXACT},      /* 6: Read back high */
    {OP_READ,  0xa0, 0xaa, VAL_EXACT},      /* 7: Read back mid */
    {OP_READ,  0x80, 0x55, VAL_EXACT},      /* 8: Read back low */
    {OP_WRITE, 0xc0, 0x0a, 0},              /* 9: DMA addr high = 0x0a */
    {OP_WRITE, 0xa0, 0x55, 0},              /* 10: DMA addr mid = 0x55 */
    {OP_WRITE, 0x80, 0xaa, 0},              /* 11: DMA addr low = 0xaa */
    {OP_READ,  0xc0, 0x0a, VAL_EXACT},      /* 12: Read back high */
    {OP_READ,  0xa0, 0x55, VAL_EXACT},      /* 13: Read back mid */
    {OP_READ,  0x80, 0xaa, VAL_EXACT},      /* 14: Read back low */
    {OP_WRITE, 0x00, 0x24, 0},              /* 15: Control = 0x24 (reset) */
    {OP_WRITE, 0x00, 0x04, 0},              /* 16: Control = 0x04 */
    {OP_WRITE, 0x00, 0x00, 0},              /* 17: Control = 0x00 */
    {OP_WRITE, 0x00, 0x04, 0},              /* 18: Control = 0x04 */

    /* Line 19-41: First diagnostic command (TEST_UNIT_READY 0xE0) */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 19: Poll for idle (0x00) */
    {OP_WRITE, 0x10, 0x01, 0},              /* 20: Data = 0x01 (target ID) */
    {OP_WRITE, 0x00, 0x14, 0},              /* 21: Control = 0x14 (select) */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 22: Poll for BSY (0x04) */
    {OP_WRITE, 0x00, 0x04, 0},              /* 23: Control = 0x04 (drop select) */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 24: Poll for command phase (0x0e) */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 25: Confirm command phase */
    {OP_WRITE, 0x10, 0xe0, 0},              /* 26: CDB[0] = 0xe0 (diag) */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 27: Poll for REQ (command phase) */
    {OP_WRITE, 0x10, 0x00, 0},              /* 28: CDB[1] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 29: Poll for REQ */
    {OP_WRITE, 0x10, 0xaa, 0},              /* 30: CDB[2] = 0xaa */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 31: Poll for REQ */
    {OP_WRITE, 0x10, 0x55, 0},              /* 32: CDB[3] = 0x55 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 33: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 34: CDB[4] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 35: Poll for REQ (last CDB byte) */
    {OP_WRITE, 0x10, 0x00, 0},              /* 36: CDB[5] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 37: Poll for status phase (0x0f) */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 38: Data = 0x00 (GOOD status) */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 39: Poll for message phase (0x1f) */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 40: Data = 0x00 (COMPLETE) */

    /* Line 41-70: Program DMA address and REQUEST_SENSE (0x03) */
    {OP_WRITE, 0x00, 0x08, 0},              /* 41: Control = 0x08 */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 42: Control = 0x0c */
    {OP_WRITE, 0xc0, 0x00, 0},              /* 43: DMA addr high = 0x00 */
    {OP_WRITE, 0xa0, 0x02, 0},              /* 44: DMA addr mid = 0x02 */
    {OP_WRITE, 0x80, 0xec, 0},              /* 45: DMA addr low = 0xec */
    {OP_WRITE, 0x00, 0x09, 0},              /* 46: Control = 0x09 */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 47: Control = 0x0d */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 48: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 49: Data = 0x01 (target ID) */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 50: Control = 0x1d (select) */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 51: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 52: Control = 0x0d */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 53: Poll for command phase */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 54: Confirm command phase */
    {OP_WRITE, 0x10, 0x03, 0},              /* 55: CDB[0] = 0x03 (REQ SENSE) */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 56: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 57: CDB[1] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 58: Poll for REQ */
    {OP_WRITE, 0x10, 0x02, 0},              /* 59: CDB[2] = 0x02 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 60: Poll for REQ */
    {OP_WRITE, 0x10, 0xec, 0},              /* 61: CDB[3] = 0xec */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 62: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 63: CDB[4] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 64: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 65: CDB[5] = 0x00 */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 66: Poll for status phase */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 67: Data = 0x00 (GOOD) */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 68: Poll for message phase */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 69: Data = 0x00 (COMPLETE) */

    /* Line 70-92: Another command sequence - second diagnostic (0xE4) */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 70: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 71: Target ID */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 72: Select */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 73: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 74: Drop select */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 75: Poll for command phase */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 76: Confirm command phase */
    {OP_WRITE, 0x10, 0xe4, 0},              /* 77: CDB[0] = 0xe4 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 78: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 79: CDB[1] */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 80: Poll for REQ */
    {OP_WRITE, 0x10, 0x02, 0},              /* 81: CDB[2] */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 82: Poll for REQ */
    {OP_WRITE, 0x10, 0xec, 0},              /* 83: CDB[3] */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 84: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 85: CDB[4] */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 86: Poll for REQ */
    {OP_WRITE, 0x10, 0x00, 0},              /* 87: CDB[5] */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 88: Poll for status phase */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 89: GOOD */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 90: Poll for message phase */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 91: COMPLETE */

    /* Line 92-121: Program DMA and REQUEST_SENSE again */
    {OP_WRITE, 0x00, 0x08, 0},              /* 92: */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 93: */
    {OP_WRITE, 0xc0, 0x00, 0},              /* 94: DMA high */
    {OP_WRITE, 0xa0, 0x02, 0},              /* 95: DMA mid */
    {OP_WRITE, 0x80, 0xec, 0},              /* 96: DMA low */
    {OP_WRITE, 0x00, 0x09, 0},              /* 97: */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 98: */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 99: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 100: */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 101: */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 102: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 103: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 104: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 105: */
    {OP_WRITE, 0x10, 0x03, 0},              /* 106: REQ_SENSE */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 107: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 108: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 109: */
    {OP_WRITE, 0x10, 0x02, 0},              /* 110: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 111: */
    {OP_WRITE, 0x10, 0xec, 0},              /* 112: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 113: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 114: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 115: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 116: */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 117: Poll for status */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 118: */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 119: Poll for message */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 120: */

    /* Line 121-145: Another setup and TEST_UNIT_READY (0x00) */
    {OP_WRITE, 0x00, 0x08, 0},              /* 121: */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 122: */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 123: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 124: */
    {OP_WRITE, 0x00, 0x1c, 0},              /* 125: */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 126: Poll for BSY */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 127: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 128: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 129: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 130: CDB[0] = TEST_UNIT_READY */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 131: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 132: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 133: */
    {OP_WRITE, 0x10, 0x40, 0},              /* 134: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 135: */
    {OP_WRITE, 0x10, 0x01, 0},              /* 136: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 137: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 138: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 139: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 140: */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 141: Poll for status */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 142: GOOD */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 143: Poll for message */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 144: COMPLETE */

    /* Line 145-174: DMA setup and REQUEST_SENSE */
    {OP_WRITE, 0x00, 0x08, 0},              /* 145: */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 146: */
    {OP_WRITE, 0xc0, 0x00, 0},              /* 147: */
    {OP_WRITE, 0xa0, 0x02, 0},              /* 148: */
    {OP_WRITE, 0x80, 0xec, 0},              /* 149: */
    {OP_WRITE, 0x00, 0x09, 0},              /* 150: */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 151: */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 152: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 153: */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 154: */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 155: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 156: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 157: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 158: */
    {OP_WRITE, 0x10, 0x03, 0},              /* 159: REQ_SENSE */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 160: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 161: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 162: */
    {OP_WRITE, 0x10, 0x02, 0},              /* 163: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 164: */
    {OP_WRITE, 0x10, 0xec, 0},              /* 165: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 166: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 167: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 168: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 169: */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 170: Poll for status */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 171: */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 172: Poll for message */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 173: */

    /* Line 174-196: RECALIBRATE command (0x01) */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 174: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 175: */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 176: */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 177: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 178: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 179: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 180: */
    {OP_WRITE, 0x10, 0x01, 0},              /* 181: CDB[0] = RECALIBRATE */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 182: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 183: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 184: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 185: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 186: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 187: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 188: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 189: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 190: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 191: */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 192: Poll for status */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 193: GOOD */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 194: Poll for message */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 195: COMPLETE */

    /* Line 196-225: DMA setup and REQUEST_SENSE */
    {OP_WRITE, 0x00, 0x08, 0},              /* 196: */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 197: */
    {OP_WRITE, 0xc0, 0x00, 0},              /* 198: */
    {OP_WRITE, 0xa0, 0x02, 0},              /* 199: */
    {OP_WRITE, 0x80, 0xec, 0},              /* 200: */
    {OP_WRITE, 0x00, 0x09, 0},              /* 201: */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 202: */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 203: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 204: */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 205: */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 206: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 207: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 208: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 209: */
    {OP_WRITE, 0x10, 0x03, 0},              /* 210: REQ_SENSE */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 211: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 212: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 213: */
    {OP_WRITE, 0x10, 0x02, 0},              /* 214: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 215: */
    {OP_WRITE, 0x10, 0xec, 0},              /* 216: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 217: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 218: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 219: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 220: */
    {OP_POLL_STATUS, 0x20, 0x0f, 0},        /* 221: Poll for status */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 222: GOOD */
    {OP_POLL_STATUS, 0x20, 0x1f, 0},        /* 223: Poll for message */
    {OP_READ,  0x10, 0x00, VAL_EXACT},      /* 224: COMPLETE */

    /* Line 225-249: DMA setup for READ(6) command */
    {OP_WRITE, 0x00, 0x08, 0},              /* 225: */
    {OP_WRITE, 0x00, 0x0c, 0},              /* 226: */
    {OP_WRITE, 0xc0, 0x00, 0},              /* 227: */
    {OP_WRITE, 0xa0, 0x0c, 0},              /* 228: */
    {OP_WRITE, 0x80, 0x00, 0},              /* 229: */
    {OP_WRITE, 0x00, 0x09, 0},              /* 230: */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 231: */
    {OP_POLL_STATUS, 0x20, 0x00, 0},        /* 232: Poll for idle */
    {OP_WRITE, 0x10, 0x01, 0},              /* 233: */
    {OP_WRITE, 0x00, 0x1d, 0},              /* 234: Select */
    {OP_POLL_STATUS, 0x20, 0x04, 0},        /* 235: Poll for BSY */
    {OP_WRITE, 0x00, 0x0d, 0},              /* 236: */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 237: Poll for command */
    {OP_READ,  0x20, 0x0e, VAL_FLEXIBLE},   /* 238: */
    {OP_WRITE, 0x10, 0x08, 0},              /* 239: READ(6) opcode */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 240: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 241: LBA high */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 242: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 243: LBA mid */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 244: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 245: LBA low */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 246: */
    {OP_WRITE, 0x10, 0x02, 0},              /* 247: Block count = 2 */
    {OP_POLL_STATUS, 0x20, 0x0e, 0},        /* 248: */
    {OP_WRITE, 0x10, 0x00, 0},              /* 249: Control byte */
    /* After this, DMA data transfer begins - not tested here */
};

#define NUM_TEST_OPS (sizeof(test_sequence) / sizeof(test_sequence[0]))
#define MAX_RESULTS 300

/* Result storage */
static op_result_t results[MAX_RESULTS];
static uint16_t num_results = 0;

/* Statistics */
static uint32_t total_poll_iterations = 0;
static uint16_t num_polls = 0;
static uint16_t num_timeouts = 0;
static uint16_t max_poll_count = 0;
static uint16_t min_poll_count = 0xFFFF;

/* Performance tracking */
static clock_t start_time;
static clock_t end_time;

/* Direct register access - as fast as possible */
static inline void dma_write(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

static inline unsigned char dma_read(unsigned int offset) {
    return *DMA_REG(offset);
}

/*
 * Poll status register until expected value or timeout.
 * Returns number of iterations, or 0xFFFF on timeout.
 */
static uint16_t poll_status(uint8_t expected) {
    uint16_t iterations = 0;
    uint8_t actual;

    while (iterations < MAX_POLL_ITERATIONS) {
        actual = dma_read(0x20) & STATUS_PHASE_MASK;
        iterations++;
        if (actual == expected) {
            return iterations;
        }
    }
    return 0xFFFF;  /* Timeout */
}

/*
 * Execute all test operations.
 * Polls are done inline, no printf during execution.
 */
static void execute_test_sequence(void) {
    uint16_t i;
    uint16_t result_idx = 0;
    const test_op_t *op;
    uint8_t actual;
    uint16_t poll_count;

    /* Record start time */
    start_time = clock();

    /* Execute all operations */
    for (i = 0; i < NUM_TEST_OPS; i++) {
        op = &test_sequence[i];

        switch (op->op_type) {
        case OP_WRITE:
            dma_write(op->offset, op->data);
            /* No result to store for writes */
            break;

        case OP_READ:
            actual = dma_read(op->offset);
            if (result_idx < MAX_RESULTS) {
                results[result_idx].actual = actual;
                results[result_idx].expected = op->data;
                results[result_idx].poll_count = 1;
                results[result_idx].timed_out = 0;
                result_idx++;
            }
            break;

        case OP_POLL_STATUS:
            poll_count = poll_status(op->data);
            if (result_idx < MAX_RESULTS) {
                results[result_idx].expected = op->data;
                if (poll_count == 0xFFFF) {
                    results[result_idx].actual = dma_read(0x20) & STATUS_PHASE_MASK;
                    results[result_idx].poll_count = MAX_POLL_ITERATIONS;
                    results[result_idx].timed_out = 1;
                    num_timeouts++;
                } else {
                    results[result_idx].actual = op->data;  /* Got expected */
                    results[result_idx].poll_count = poll_count;
                    results[result_idx].timed_out = 0;

                    /* Update statistics */
                    total_poll_iterations += poll_count;
                    num_polls++;
                    if (poll_count > max_poll_count) max_poll_count = poll_count;
                    if (poll_count < min_poll_count) min_poll_count = poll_count;
                }
                result_idx++;
            }
            break;
        }
    }

    /* Record end time */
    end_time = clock();
    num_results = result_idx;
}

/*
 * Analyze and print results after test completes.
 */
static void analyze_results(void) {
    uint16_t i;
    uint16_t result_idx = 0;
    uint16_t exact_match_count = 0;
    uint16_t exact_mismatch_count = 0;
    uint16_t write_count = 0;
    uint16_t poll_success_count = 0;
    const test_op_t *op;
    unsigned long elapsed_ticks;
    unsigned long elapsed_ms;
    unsigned long avg_poll;

    /* Calculate timing */
    elapsed_ticks = (unsigned long)(end_time - start_time);
    elapsed_ms = (elapsed_ticks * 1000UL) / CLOCKS_PER_SEC;

    printf("SASI State Machine Performance Test Results\n");
    printf("============================================\n\n");

    printf("Timing:\n");
    printf("  Total operations: %u\n", NUM_TEST_OPS);
    printf("  Elapsed time: %lu ms (%lu ticks)\n", elapsed_ms, elapsed_ticks);
    printf("\n");

    /* Poll statistics */
    printf("Poll Statistics (realistic BIOS-style polling):\n");
    printf("  Total polls: %u\n", num_polls);
    printf("  Timeouts: %u\n", num_timeouts);
    if (num_polls > 0) {
        avg_poll = total_poll_iterations / num_polls;
        printf("  Min iterations: %u\n", min_poll_count);
        printf("  Max iterations: %u\n", max_poll_count);
        printf("  Avg iterations: %lu\n", avg_poll);
        printf("  Total iterations: %lu\n", total_poll_iterations);
    }
    printf("\n");

    /* Categorize results */
    for (i = 0; i < NUM_TEST_OPS; i++) {
        op = &test_sequence[i];
        if (op->op_type == OP_WRITE) {
            write_count++;
        } else if (op->op_type == OP_READ) {
            if (results[result_idx].actual == results[result_idx].expected) {
                exact_match_count++;
            } else if (op->val_type == VAL_EXACT) {
                exact_mismatch_count++;
            }
            result_idx++;
        } else if (op->op_type == OP_POLL_STATUS) {
            if (!results[result_idx].timed_out) {
                poll_success_count++;
            }
            result_idx++;
        }
    }

    printf("Summary:\n");
    printf("  Writes: %u\n", write_count);
    printf("  Exact-match reads: %u/%u\n", exact_match_count,
           exact_match_count + exact_mismatch_count);
    printf("  Poll successes: %u/%u\n", poll_success_count, num_polls);
    printf("\n");

    /* Print exact-match failures if any */
    if (exact_mismatch_count > 0) {
        printf("FAILURES - Exact match mismatches:\n");
        printf("----------------------------------\n");

        result_idx = 0;
        for (i = 0; i < NUM_TEST_OPS; i++) {
            op = &test_sequence[i];
            if (op->op_type == OP_READ) {
                if (op->val_type == VAL_EXACT &&
                    results[result_idx].actual != results[result_idx].expected) {
                    printf("  Op #%3u: Read 0x%02X: expected 0x%02X, got 0x%02X\n",
                           i + 1, op->offset,
                           results[result_idx].expected,
                           results[result_idx].actual);
                }
                result_idx++;
            } else if (op->op_type == OP_POLL_STATUS) {
                result_idx++;
            }
        }
        printf("\n");
    }

    /* Print poll timeouts if any */
    if (num_timeouts > 0) {
        printf("TIMEOUTS - Polls that didn't complete:\n");
        printf("--------------------------------------\n");

        result_idx = 0;
        for (i = 0; i < NUM_TEST_OPS; i++) {
            op = &test_sequence[i];
            if (op->op_type == OP_POLL_STATUS) {
                if (results[result_idx].timed_out) {
                    printf("  Op #%3u: Poll for 0x%02X timed out (last=0x%02X)\n",
                           i + 1, results[result_idx].expected,
                           results[result_idx].actual);
                }
                result_idx++;
            } else if (op->op_type == OP_READ) {
                result_idx++;
            }
        }
        printf("\n");
    }

    /* Show poll iteration distribution */
    printf("Poll iteration counts (first 30 polls):\n");
    printf("----------------------------------------\n");
    result_idx = 0;
    {
        uint16_t poll_shown = 0;
        for (i = 0; i < NUM_TEST_OPS && poll_shown < 30; i++) {
            op = &test_sequence[i];
            if (op->op_type == OP_POLL_STATUS) {
                printf("  Op #%3u: poll for 0x%02X -> %u iterations%s\n",
                       i + 1, op->data, results[result_idx].poll_count,
                       results[result_idx].timed_out ? " (TIMEOUT)" : "");
                poll_shown++;
                result_idx++;
            } else if (op->op_type == OP_READ) {
                result_idx++;
            }
        }
    }
    printf("\n");

    /* Final verdict */
    if (exact_mismatch_count == 0 && num_timeouts == 0) {
        printf("PASS: All exact-match reads correct, all polls succeeded.\n");
    } else if (num_timeouts > 0) {
        printf("FAIL: %u poll timeouts occurred.\n", num_timeouts);
    } else {
        printf("FAIL: %u exact-match reads returned unexpected values.\n",
               exact_mismatch_count);
    }
}

/*
 * Pre-flight check - verify DMA board is responding
 */
static int preflight_check(void) {
    unsigned char test_val;

    /* Try writing and reading address register */
    dma_write(0xc0, 0x07);
    test_val = dma_read(0xc0);

    if (test_val == 0x07) {
        return 1;  /* Board responding */
    }

    printf("WARNING: DMA board may not be responding.\n");
    printf("  Wrote 0x07 to offset 0xc0, read back 0x%02X\n", test_val);
    return 0;
}

int main(void) {
    printf("SASI Performance Test v2.0 (Polling Mode)\n");
    printf("Based on MAME boot log - polls like real BIOS\n");
    printf("---------------------------------------------\n\n");

    /* Preflight check */
    if (!preflight_check()) {
        printf("Continuing anyway...\n\n");
    }

    printf("Executing %u operations with polling...\n", NUM_TEST_OPS);

    /* Execute the test sequence */
    execute_test_sequence();

    printf("Done.\n\n");

    /* Analyze and print results */
    analyze_results();

    /* Wait for keypress before exit */
    printf("\nPress any key to exit...\n");
    getch();

    return 0;
}

/*
 * boot_sequence_test.c - Victor 9000 boot-sequence SASI harness
 *
 * This DOS utility replays the key hard-disk boot command sequence observed
 * on a healthy Victor 9000 boot. It validates command-phase handshakes,
 * status/message completion, and DMA address progression per command.
 *
 * The goal is to pinpoint the first command that diverges from expected
 * behavior during real boot, while keeping output concise and deterministic.
 *
 * Build with OpenWatcom using the Makefile in this directory.
 */

#include <conio.h>
#include <dos.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* DMA board register addresses */
#define DMA_BASE        0xEF30
#define DMA_REG(offset) ((volatile unsigned char far *)(MK_FP(DMA_BASE, offset)))

/* Register offsets */
#define REG_CONTROL     0x00
#define REG_DATA        0x10
#define REG_STATUS      0x20
#define REG_ADDR_L      0x80
#define REG_ADDR_M      0xA0
#define REG_ADDR_H      0xC0

/* Status phase bits */
#define STATUS_PHASE_MASK   0x1F
#define PHASE_IDLE          0x00
#define PHASE_BSY           0x04
#define PHASE_COMMAND       0x0E
#define PHASE_STATUS        0x0F
#define PHASE_MESSAGE       0x1F

/* Control bit patterns used by existing DOS harnesses */
#define CTRL_SELECT_BIT     0x10

/* DMA control images used by the existing DOS tests and ROM traces */
#define CTRL_BASE_NONE      0x04  /* DMA disabled, strobe high */
#define CTRL_BASE_IN        0x0D  /* DMA enabled, dir=device->host, strobe high */
#define CTRL_BASE_OUT       0x05  /* DMA enabled, dir=host->device, strobe high */

/* Poll limits */
#define DEFAULT_POLL_LIMIT  50000UL
#define LONG_POLL_LIMIT    400000UL

#define TARGET_SELECT_DATA  0x01
#define SECTOR_SIZE         512UL

/* Boot-sequence DMA addresses observed in MAME/trace notes */
#define DMA_ADDR_SENSE      0x002ECUL
#define DMA_ADDR_LABEL      0x00C00UL
#define DMA_ADDR_PARAMS     0x00C24UL
#define DMA_ADDR_BOOTIMG    0xCCF40UL
#define DMA_ADDR_POSTIMG    (DMA_ADDR_BOOTIMG + (152UL * SECTOR_SIZE))

typedef enum {
    DMA_MODE_NONE = 0,
    DMA_MODE_DEV_TO_HOST = 1,
    DMA_MODE_HOST_TO_DEV = 2
} dma_mode_t;

typedef struct {
    const char *name;
    uint8_t cdb[6];
    dma_mode_t dma_mode;
    uint32_t dma_addr;
    uint32_t status_poll_limit;
} boot_step_t;

typedef struct {
    uint8_t success;
    uint8_t error_code;
    uint8_t status_byte;
    uint8_t message_byte;
    uint8_t last_status;
    uint32_t poll_idle;
    uint32_t poll_bsy;
    uint32_t poll_cmd[6];
    uint32_t poll_status;
    uint32_t poll_message;
    uint32_t dma_start;
    uint32_t dma_end;
    uint32_t dma_expected_delta;
    uint32_t dma_actual_delta;
} step_result_t;

enum {
    STEP_OK = 0,
    ERR_DMA_ADDR_PROGRAM = 1,
    ERR_IDLE_TIMEOUT = 2,
    ERR_BSY_TIMEOUT = 3,
    ERR_COMMAND_TIMEOUT = 4,
    ERR_STATUS_TIMEOUT = 5,
    ERR_BAD_STATUS_BYTE = 6,
    ERR_MESSAGE_TIMEOUT = 7,
    ERR_BAD_MESSAGE_BYTE = 8,
    ERR_DMA_DELTA = 9
};

static const boot_step_t boot_sequence[] = {
    { "XEBEC_RAM_DIAG",      { 0xE0, 0x00, 0xAA, 0x55, 0x00, 0x00 }, DMA_MODE_NONE,        0x00000UL,      DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_1",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "XEBEC_INTERNAL_DIAG", { 0xE4, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_NONE,        0x00000UL,      DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_2",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "TEST_UNIT_READY",     { 0x00, 0x00, 0x40, 0x01, 0x00, 0x00 }, DMA_MODE_NONE,        0x00000UL,      DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_3",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "RECALIBRATE",         { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 }, DMA_MODE_NONE,        0x00000UL,      DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_4",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "READ_LABEL_2SEC",     { 0x08, 0x00, 0x00, 0x00, 0x02, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_LABEL, LONG_POLL_LIMIT },
    { "REQUEST_SENSE_5",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "INIT_DRIVE_PARAMS",   { 0x0C, 0x00, 0x00, 0x02, 0x02, 0x00 }, DMA_MODE_HOST_TO_DEV, DMA_ADDR_PARAMS, DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_6",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x00 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "READ_BOOT_152SEC",    { 0x08, 0x00, 0x00, 0x44, 0x98, 0x07 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_BOOTIMG, LONG_POLL_LIMIT },
    { "REQUEST_SENSE_7",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x07 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT },
    { "READ_FOLLOWUP_1SEC",  { 0x08, 0x00, 0x00, 0xDC, 0x01, 0x07 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_POSTIMG, DEFAULT_POLL_LIMIT },
    { "REQUEST_SENSE_8",     { 0x03, 0x00, 0x02, 0xEC, 0x00, 0x07 }, DMA_MODE_DEV_TO_HOST, DMA_ADDR_SENSE, DEFAULT_POLL_LIMIT }
};

#define NUM_BOOT_STEPS (sizeof(boot_sequence) / sizeof(boot_sequence[0]))

static void dma_write_reg(unsigned int offset, unsigned char value) {
    *DMA_REG(offset) = value;
}

static unsigned char dma_read_reg(unsigned int offset) {
    return *DMA_REG(offset);
}

static uint8_t read_status_phase(void) {
    return dma_read_reg(REG_STATUS) & STATUS_PHASE_MASK;
}

static void board_reset(void) {
    dma_write_reg(REG_CONTROL, 0x20);
    delay(2);
    dma_write_reg(REG_CONTROL, 0x00);
    delay(2);
}

static uint32_t poll_status(uint8_t expected, uint32_t max_iterations, uint8_t *last_status) {
    uint32_t i;
    uint8_t status;

    for (i = 0; i < max_iterations; i++) {
        status = read_status_phase();
        if (last_status) {
            *last_status = status;
        }
        if (status == expected) {
            return i + 1;
        }
    }
    return 0;
}

static uint8_t control_base_for_mode(dma_mode_t mode) {
    if (mode == DMA_MODE_DEV_TO_HOST) {
        return CTRL_BASE_IN;
    }
    if (mode == DMA_MODE_HOST_TO_DEV) {
        return CTRL_BASE_OUT;
    }
    return CTRL_BASE_NONE;
}

static void write_dma_address(uint32_t addr) {
    dma_write_reg(REG_ADDR_L, (uint8_t)(addr & 0xFF));
    dma_write_reg(REG_ADDR_M, (uint8_t)((addr >> 8) & 0xFF));
    dma_write_reg(REG_ADDR_H, (uint8_t)((addr >> 16) & 0x0F));
}

static uint32_t read_dma_address(void) {
    uint32_t addr;
    addr = (uint32_t)(dma_read_reg(REG_ADDR_H) & 0x0F) << 16;
    addr |= (uint32_t)dma_read_reg(REG_ADDR_M) << 8;
    addr |= (uint32_t)dma_read_reg(REG_ADDR_L);
    return addr;
}

static void configure_dma_mode(dma_mode_t mode, uint32_t dma_addr) {
    if (mode == DMA_MODE_DEV_TO_HOST) {
        dma_write_reg(REG_CONTROL, 0x08);
        dma_write_reg(REG_CONTROL, 0x0C);
        write_dma_address(dma_addr);
        dma_write_reg(REG_CONTROL, 0x09);
        dma_write_reg(REG_CONTROL, 0x0D);
    } else if (mode == DMA_MODE_HOST_TO_DEV) {
        dma_write_reg(REG_CONTROL, 0x00);
        dma_write_reg(REG_CONTROL, 0x04);
        write_dma_address(dma_addr);
        dma_write_reg(REG_CONTROL, 0x01);
        dma_write_reg(REG_CONTROL, 0x05);
    } else {
        dma_write_reg(REG_CONTROL, 0x00);
        dma_write_reg(REG_CONTROL, 0x04);
    }
}

static uint32_t expected_dma_delta(const uint8_t *cdb) {
    uint8_t op = cdb[0];
    uint32_t blocks;
    uint32_t alloc;

    if (op == 0x08 || op == 0x0A) {
        blocks = cdb[4] ? (uint32_t)cdb[4] : 256UL;
        return blocks * SECTOR_SIZE;
    }

    if (op == 0x03) {
        alloc = cdb[4] ? (uint32_t)cdb[4] : 4UL;
        if (alloc > 4UL) {
            alloc = 4UL;
        }
        return alloc;
    }

    if (op == 0x0C) {
        return 8UL;
    }

    return 0UL;
}

static uint8_t read_phys_byte(uint32_t phys_addr) {
    uint16_t seg = (uint16_t)(phys_addr >> 4);
    uint16_t off = (uint16_t)(phys_addr & 0x0F);
    volatile uint8_t far *ptr = (volatile uint8_t far *)MK_FP(seg, off);
    return *ptr;
}

static uint8_t crc8_phys(uint32_t phys_addr, uint16_t len) {
    uint8_t crc = 0x00;
    uint16_t i;
    uint8_t bit;
    uint8_t value;

    for (i = 0; i < len; i++) {
        value = read_phys_byte(phys_addr + (uint32_t)i);
        crc ^= value;
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static void print_phys_bytes(uint32_t phys_addr, uint16_t len) {
    uint16_t i;
    for (i = 0; i < len; i++) {
        printf("%02X", read_phys_byte(phys_addr + (uint32_t)i));
        if (i + 1 < len) {
            putchar(' ');
        }
    }
}

static int run_boot_step(const boot_step_t *step, step_result_t *result) {
    uint8_t base_ctrl;
    uint8_t status;
    int i;
    uint32_t polls;

    memset(result, 0, sizeof(*result));
    result->success = 0;

    configure_dma_mode(step->dma_mode, step->dma_addr);
    base_ctrl = control_base_for_mode(step->dma_mode);

    if (step->dma_mode != DMA_MODE_NONE) {
        result->dma_start = read_dma_address();
        if (result->dma_start != step->dma_addr) {
            result->error_code = ERR_DMA_ADDR_PROGRAM;
            return 0;
        }
    }

    polls = poll_status(PHASE_IDLE, DEFAULT_POLL_LIMIT, &status);
    result->poll_idle = polls;
    if (polls == 0) {
        result->error_code = ERR_IDLE_TIMEOUT;
        result->last_status = status;
        return 0;
    }

    dma_write_reg(REG_DATA, TARGET_SELECT_DATA);
    dma_write_reg(REG_CONTROL, (uint8_t)(base_ctrl | CTRL_SELECT_BIT));

    polls = poll_status(PHASE_BSY, DEFAULT_POLL_LIMIT, &status);
    result->poll_bsy = polls;
    if (polls == 0) {
        result->error_code = ERR_BSY_TIMEOUT;
        result->last_status = status;
        return 0;
    }

    dma_write_reg(REG_CONTROL, base_ctrl);

    for (i = 0; i < 6; i++) {
        polls = poll_status(PHASE_COMMAND, DEFAULT_POLL_LIMIT, &status);
        result->poll_cmd[i] = polls;
        if (polls == 0) {
            result->error_code = ERR_COMMAND_TIMEOUT;
            result->last_status = status;
            return 0;
        }
        dma_write_reg(REG_DATA, step->cdb[i]);
    }

    polls = poll_status(PHASE_STATUS, step->status_poll_limit, &status);
    result->poll_status = polls;
    if (polls == 0) {
        result->error_code = ERR_STATUS_TIMEOUT;
        result->last_status = status;
        return 0;
    }

    result->status_byte = dma_read_reg(REG_DATA);
    if (result->status_byte != 0x00) {
        result->error_code = ERR_BAD_STATUS_BYTE;
        result->last_status = read_status_phase();
        return 0;
    }

    polls = poll_status(PHASE_MESSAGE, DEFAULT_POLL_LIMIT, &status);
    result->poll_message = polls;
    if (polls == 0) {
        result->error_code = ERR_MESSAGE_TIMEOUT;
        result->last_status = status;
        return 0;
    }

    result->message_byte = dma_read_reg(REG_DATA);
    if (result->message_byte != 0x00) {
        result->error_code = ERR_BAD_MESSAGE_BYTE;
        result->last_status = read_status_phase();
        return 0;
    }

    if (step->dma_mode != DMA_MODE_NONE) {
        result->dma_end = read_dma_address();
        result->dma_expected_delta = expected_dma_delta(step->cdb);
        result->dma_actual_delta = result->dma_end - result->dma_start;

        if (result->dma_actual_delta != result->dma_expected_delta) {
            result->error_code = ERR_DMA_DELTA;
            result->last_status = read_status_phase();
            return 0;
        }
    }

    result->success = 1;
    return 1;
}

static const char *error_name(uint8_t error_code) {
    switch (error_code) {
        case STEP_OK:              return "OK";
        case ERR_DMA_ADDR_PROGRAM: return "DMA_ADDR_PROGRAM";
        case ERR_IDLE_TIMEOUT:     return "IDLE_TIMEOUT";
        case ERR_BSY_TIMEOUT:      return "BSY_TIMEOUT";
        case ERR_COMMAND_TIMEOUT:  return "COMMAND_TIMEOUT";
        case ERR_STATUS_TIMEOUT:   return "STATUS_TIMEOUT";
        case ERR_BAD_STATUS_BYTE:  return "BAD_STATUS_BYTE";
        case ERR_MESSAGE_TIMEOUT:  return "MESSAGE_TIMEOUT";
        case ERR_BAD_MESSAGE_BYTE: return "BAD_MESSAGE_BYTE";
        case ERR_DMA_DELTA:        return "DMA_DELTA_MISMATCH";
        default:                   return "UNKNOWN";
    }
}

static void print_long_read_samples(void) {
    uint32_t first_addr = DMA_ADDR_BOOTIMG;
    uint32_t last_addr = DMA_ADDR_BOOTIMG + (151UL * SECTOR_SIZE);
    uint8_t crc_first = crc8_phys(first_addr, (uint16_t)SECTOR_SIZE);
    uint8_t crc_last = crc8_phys(last_addr, (uint16_t)SECTOR_SIZE);

    printf("  Long-read sample:\n");
    printf("    first sector @0x%05lX CRC8=0x%02X first8=[", first_addr, crc_first);
    print_phys_bytes(first_addr, 8);
    printf("]\n");

    printf("    last  sector @0x%05lX CRC8=0x%02X first8=[", last_addr, crc_last);
    print_phys_bytes(last_addr, 8);
    printf("]\n");
}

int main(void) {
    uint8_t preflight;
    size_t i;
    size_t passed = 0;
    step_result_t result;

    printf("Victor 9000 Boot Sequence Test Harness v1.0\n");
    printf("===========================================\n\n");

    printf("Pre-flight check...\n");
    dma_write_reg(REG_ADDR_H, 0x07);
    preflight = dma_read_reg(REG_ADDR_H) & 0x0F;
    if (preflight == 0x07) {
        printf("  DMA board responding: OK\n");
    } else {
        printf("  WARNING: DMA board may not be fully warmed.\n");
        printf("  Wrote 0x07 to ADDR_H, read back 0x%02X\n", preflight);
    }

    printf("  Board reset...\n");
    board_reset();
    printf("  Reset complete.\n\n");

    printf("Executing %u boot-sequence commands...\n\n", (unsigned)NUM_BOOT_STEPS);

    for (i = 0; i < NUM_BOOT_STEPS; i++) {
        const boot_step_t *step = &boot_sequence[i];

        printf("[%u/%u] %s  CDB=%02X %02X %02X %02X %02X %02X",
               (unsigned)(i + 1),
               (unsigned)NUM_BOOT_STEPS,
               step->name,
               step->cdb[0], step->cdb[1], step->cdb[2],
               step->cdb[3], step->cdb[4], step->cdb[5]);

        if (step->dma_mode != DMA_MODE_NONE) {
            printf("  DMA=0x%05lX", step->dma_addr);
        } else {
            printf("  DMA=none");
        }
        printf("\n");

        if (step->cdb[0] == 0x0C) {
            printf("  Params @0x%05lX: [", (unsigned long)DMA_ADDR_PARAMS);
            print_phys_bytes(DMA_ADDR_PARAMS, 8);
            printf("]\n");
        }

        if (!run_boot_step(step, &result)) {
            printf("  FAIL: %s (code=%u)\n", error_name(result.error_code), result.error_code);
            printf("  Polls: idle=%lu bsy=%lu cmd=[%lu,%lu,%lu,%lu,%lu,%lu] status=%lu msg=%lu\n",
                   result.poll_idle, result.poll_bsy,
                   result.poll_cmd[0], result.poll_cmd[1], result.poll_cmd[2],
                   result.poll_cmd[3], result.poll_cmd[4], result.poll_cmd[5],
                   result.poll_status, result.poll_message);
            printf("  Last phase status=0x%02X status_byte=0x%02X message_byte=0x%02X\n",
                   result.last_status, result.status_byte, result.message_byte);
            if (step->dma_mode != DMA_MODE_NONE) {
                printf("  DMA addr start=0x%05lX end=0x%05lX expected_delta=%lu actual_delta=%lu\n",
                       result.dma_start, result.dma_end,
                       result.dma_expected_delta, result.dma_actual_delta);
            }
            printf("\nStopped at first divergence.\n");
            break;
        }

        passed++;
        printf("  PASS: status=0x%02X message=0x%02X polls(status=%lu,msg=%lu)\n",
               result.status_byte, result.message_byte,
               result.poll_status, result.poll_message);

        if (step->dma_mode != DMA_MODE_NONE) {
            printf("  DMA: start=0x%05lX end=0x%05lX delta=%lu\n",
                   result.dma_start, result.dma_end, result.dma_actual_delta);
        }

        if (step->cdb[0] == 0x08 && step->cdb[3] == 0x44 && step->cdb[4] == 0x98) {
            print_long_read_samples();
        }

        printf("\n");
    }

    printf("===========================================\n");
    printf("Boot Sequence Harness Summary\n");
    printf("===========================================\n");
    printf("Steps passed: %u/%u\n", (unsigned)passed, (unsigned)NUM_BOOT_STEPS);

    if (passed == NUM_BOOT_STEPS) {
        printf("Result: PASS (no protocol divergence detected in harness)\n");
    } else {
        printf("Result: FAIL (divergence detected at step %u)\n", (unsigned)(passed + 1));
    }

    printf("\nPress any key to exit...\n");
    getch();
    return (passed == NUM_BOOT_STEPS) ? 0 : 1;
}

#include <stdio.h>
#include <string.h>
#include <ctype.h>


#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/structs/systick.h"
#include "hardware/clocks.h"
#include "board_registers.pio.h"
#include "dma_master.pio.h"
#include "pico_victor/dma.h"
#include "pico_victor/debug_queue.h"
#include "pico_victor/reg_queue_processor.h"
#include "pico_victor/register_irq_handlers.h"
#include "hardware/structs/iobank0.h"
#include "sasi.h"
#include "sasi_log.h"
#include "pico_storage/fatfs_guard.h"

// USE_SD_STORAGE and SD_DISK_IMAGE are provided by CMakeLists.txt
// target_compile_definitions so both dma_board.c and register_cache.c see them.
#ifndef USE_SD_STORAGE
#define USE_SD_STORAGE 1
#endif
#ifndef SD_DISK_IMAGE
#define SD_DISK_IMAGE "victor.img"
#endif

#ifndef SASI_LOG_AUTO_FLUSH_DEFAULT
#define SASI_LOG_AUTO_FLUSH_DEFAULT 0
#endif

#define DEBUG_GPIO 0

extern queue_t log_queue;

// Set by Core 1 after SD/FujiNet init + mount completes.
// Core 0 main loop checks this before calling sasi_log_flush_if_ready().
volatile bool storage_ready = false;

#ifndef CORE1_STACK_WORDS
#define CORE1_STACK_WORDS 4096u  // 16 KiB explicit Core1 stack for deferred + FatFS workload
#endif

static uint32_t core1_stack[CORE1_STACK_WORDS] __attribute__((aligned(8)));

// Stack canary: paint the bottom of Core 1's stack with a known pattern.
// Core 0's main loop checks periodically — if overwritten, stack overflow occurred.
#define STACK_CANARY_WORDS 32u
#define STACK_CANARY_PATTERN 0xDEADBEEFu

static void stack_canary_init(void) {
    for (uint32_t i = 0; i < STACK_CANARY_WORDS; i++) {
        core1_stack[i] = STACK_CANARY_PATTERN;
    }
}

// Returns the number of canary words still intact (0 = fully overwritten).
static uint32_t stack_canary_check(void) {
    uint32_t intact = 0;
    for (uint32_t i = 0; i < STACK_CANARY_WORDS; i++) {
        if (core1_stack[i] == STACK_CANARY_PATTERN) {
            intact++;
        }
    }
    return intact;
}

// =========================================================================
// HardFault handler: capture fault registers, then blink GPIO 47.
//
// The Cortex-M33 pushes {R0-R3, R12, LR, PC, xPSR} to the active stack
// on exception entry.  We extract these plus the SCB fault status registers
// (CFSR, HFSR, MMFAR, BFAR) and store them in a global struct that Core 0
// can dump over UART — even if the fault happened on Core 1.
//
// The FatFS library's crash.c normally provides isr_hardfault, but we
// compile it with PICO_RISCV=1 (see CMakeLists) to disable that handler.
// =========================================================================
#define FAULT_LED_PIN 47
#define FAULT_INFO_MAGIC 0xDEADFA17u

typedef struct {
    uint32_t r0;         // offset  0: stacked R0
    uint32_t r1;         // offset  4: stacked R1
    uint32_t r2;         // offset  8: stacked R2
    uint32_t r3;         // offset 12: stacked R3
    uint32_t r12;        // offset 16: stacked R12
    uint32_t lr;         // offset 20: stacked LR (return address)
    uint32_t pc;         // offset 24: stacked PC (faulting instruction)
    uint32_t xpsr;       // offset 28: stacked xPSR
    uint32_t cfsr;       // offset 32: Configurable Fault Status Register
    uint32_t hfsr;       // offset 36: HardFault Status Register
    uint32_t mmfar;      // offset 40: MemManage Fault Address Register
    uint32_t bfar;       // offset 44: BusFault Address Register
    uint32_t exc_return; // offset 48: EXC_RETURN from LR at handler entry
    uint32_t sp;         // offset 52: Stack pointer at time of fault
    uint32_t core_id;    // offset 56: SIO CPUID (0=Core0, 1=Core1)
    uint32_t valid;      // offset 60: set to FAULT_INFO_MAGIC when populated
} fault_info_t;

static volatile fault_info_t fault_info;

static void fault_led_init(void) {
    gpio_init(FAULT_LED_PIN);
    gpio_set_dir(FAULT_LED_PIN, GPIO_OUT);
    gpio_put(FAULT_LED_PIN, 0);
}

void __attribute__((used, naked)) isr_hardfault(void) {
    __asm volatile(
        ".syntax unified              \n"
        //
        // 1. Determine which stack pointer was active (MSP or PSP)
        //    EXC_RETURN bit 2: 0=MSP, 1=PSP
        //
        "tst   lr, #4                 \n"
        "ite   eq                     \n"
        "mrseq r0, msp               \n"
        "mrsne r0, psp               \n"
        // r0 = pointer to exception frame on faulting stack
        "mov   r3, lr                 \n"  // save EXC_RETURN
        //
        // 2. Load fault_info struct address
        //
        "ldr   r1, =fault_info        \n"
        //
        // 3. Copy 8-word exception frame {R0..xPSR}
        //
        "ldr   r2, [r0, #0]          \n"  // stacked R0
        "str   r2, [r1, #0]          \n"
        "ldr   r2, [r0, #4]          \n"  // stacked R1
        "str   r2, [r1, #4]          \n"
        "ldr   r2, [r0, #8]          \n"  // stacked R2
        "str   r2, [r1, #8]          \n"
        "ldr   r2, [r0, #12]         \n"  // stacked R3
        "str   r2, [r1, #12]         \n"
        "ldr   r2, [r0, #16]         \n"  // stacked R12
        "str   r2, [r1, #16]         \n"
        "ldr   r2, [r0, #20]         \n"  // stacked LR
        "str   r2, [r1, #20]         \n"
        "ldr   r2, [r0, #24]         \n"  // stacked PC (faulting instruction)
        "str   r2, [r1, #24]         \n"
        "ldr   r2, [r0, #28]         \n"  // stacked xPSR
        "str   r2, [r1, #28]         \n"
        //
        // 4. Read SCB fault status registers
        //    CFSR=0xE000ED28, HFSR=+4, MMFAR=+12, BFAR=+16
        //
        "ldr   r2, =0xE000ED28       \n"  // SCB CFSR base
        "ldr   r4, [r2, #0]          \n"  // CFSR
        "str   r4, [r1, #32]         \n"
        "ldr   r4, [r2, #4]          \n"  // HFSR
        "str   r4, [r1, #36]         \n"
        "ldr   r4, [r2, #12]         \n"  // MMFAR (+0x0C)
        "str   r4, [r1, #40]         \n"
        "ldr   r4, [r2, #16]         \n"  // BFAR  (+0x10)
        "str   r4, [r1, #44]         \n"
        //
        // 5. Store EXC_RETURN, SP, core ID
        //
        "str   r3, [r1, #48]         \n"  // exc_return
        "str   r0, [r1, #52]         \n"  // sp (faulting stack)
        "ldr   r2, =0xd0000000       \n"  // SIO base
        "ldr   r4, [r2, #0]          \n"  // CPUID: 0=Core0, 1=Core1
        "str   r4, [r1, #56]         \n"  // core_id
        //
        // 6. Set valid flag and barrier
        //
        "movw  r4, #0xFA17           \n"
        "movt  r4, #0xDEAD           \n"  // FAULT_INFO_MAGIC
        "str   r4, [r1, #60]         \n"
        "dsb   sy                     \n"  // ensure stores visible to other core
        //
        // 7. Blink GPIO 47 forever via raw SIO registers
        //    gpio_hi_oe_set=+0x34, gpio_hi_togl=+0x2c
        //    Bit for GPIO47 = 1<<(47-32) = 1<<15 = 0x8000
        //
        "ldr   r0, =0xd0000000       \n"  // SIO base
        "movs  r1, #1                 \n"
        "lsls  r1, r1, #15           \n"  // 0x8000
        "str   r1, [r0, #0x34]       \n"  // gpio_hi_oe_set
        "1:                            \n"
        "str   r1, [r0, #0x2c]       \n"  // gpio_hi_togl
        "ldr   r2, =2000000          \n"  // ~200ms at 200MHz
        "2:                            \n"
        "subs  r2, r2, #1             \n"
        "bne   2b                      \n"
        "b     1b                      \n"
    );
}

// Dump captured HardFault info over UART.  Safe to call from Core 0
// even while Core 1 is stuck in the LED blink loop above.
static void dump_fault_info(void) {
    if (fault_info.valid != FAULT_INFO_MAGIC) {
        printf("No HardFault recorded.\n");
        return;
    }

    printf("\n=== HARDFAULT INFO ===\n");
    printf("Core %lu faulted\n", (unsigned long)fault_info.core_id);
    printf("PC   = 0x%08lX  (faulting instruction)\n", (unsigned long)fault_info.pc);
    printf("LR   = 0x%08lX  (return address)\n", (unsigned long)fault_info.lr);
    printf("SP   = 0x%08lX\n", (unsigned long)fault_info.sp);
    printf("R0=0x%08lX R1=0x%08lX R2=0x%08lX R3=0x%08lX\n",
           (unsigned long)fault_info.r0, (unsigned long)fault_info.r1,
           (unsigned long)fault_info.r2, (unsigned long)fault_info.r3);
    printf("R12=0x%08lX  xPSR=0x%08lX\n",
           (unsigned long)fault_info.r12, (unsigned long)fault_info.xpsr);
    printf("EXC_RETURN=0x%08lX (%s)\n",
           (unsigned long)fault_info.exc_return,
           (fault_info.exc_return & 0x4) ? "PSP/Thread" : "MSP/Handler");

    uint32_t cfsr = fault_info.cfsr;
    printf("CFSR = 0x%08lX\n", (unsigned long)cfsr);
    // MemManage faults (bits 0-7)
    if (cfsr & 0x01) printf("  IACCVIOL: Instruction access violation\n");
    if (cfsr & 0x02) printf("  DACCVIOL: Data access violation\n");
    if (cfsr & 0x08) printf("  MUNSTKERR: MemManage on unstacking\n");
    if (cfsr & 0x10) printf("  MSTKERR: MemManage on stacking\n");
    if (cfsr & 0x80) printf("  MMARVALID: MMFAR = 0x%08lX\n", (unsigned long)fault_info.mmfar);
    // Bus faults (bits 8-15)
    if (cfsr & 0x0100) printf("  IBUSERR: Instruction bus error\n");
    if (cfsr & 0x0200) printf("  PRECISERR: Precise data bus error\n");
    if (cfsr & 0x0400) printf("  IMPRECISERR: Imprecise data bus error\n");
    if (cfsr & 0x0800) printf("  UNSTKERR: BusFault on unstacking\n");
    if (cfsr & 0x1000) printf("  STKERR: BusFault on stacking\n");
    if (cfsr & 0x8000) printf("  BFARVALID: BFAR = 0x%08lX\n", (unsigned long)fault_info.bfar);
    // Usage faults (bits 16-31)
    if (cfsr & 0x010000) printf("  UNDEFINSTR: Undefined instruction\n");
    if (cfsr & 0x020000) printf("  INVSTATE: Invalid state (Thumb bit)\n");
    if (cfsr & 0x040000) printf("  INVPC: Invalid PC load\n");
    if (cfsr & 0x080000) printf("  NOCP: No coprocessor\n");
    if (cfsr & 0x100000) printf("  STKOF: Stack overflow (ARMv8-M)\n");
    if (cfsr & 0x01000000) printf("  UNALIGNED: Unaligned access\n");
    if (cfsr & 0x02000000) printf("  DIVBYZERO: Divide by zero\n");

    printf("HFSR = 0x%08lX", (unsigned long)fault_info.hfsr);
    if (fault_info.hfsr & (1u << 30)) printf("  FORCED");
    if (fault_info.hfsr & (1u << 1))  printf("  VECTTBL");
    printf("\n");

    printf("=== END HARDFAULT INFO ===\n\n");
}

static void print_uart_help(void) {
    printf("\nUART commands:\n");
    printf("  h/? : show this help\n");
    printf("  t   : dump SASI trace to UART\n");
#if VERIFY_DMA_WRITES
    printf("  r   : dump DMA CRC trace (per-sector CRC-8)\n");
#endif
    printf("  f   : force SASI_LOG.TXT flush now\n");
    printf("  a   : toggle automatic log flush\n");
    printf("  p   : PIO0 state + XACK/EXTIO/CLK5/READY pin dump\n");
    printf("  i   : fast IRQ DATA transition trace dump\n");
    printf("  c   : dump HardFault crash info (if captured)\n");
    printf("\n");
}

static void dump_pio0_and_pin_state(void) {
    PIO pio = PIO_REGISTERS;
    uint sm = REG_SM_CONTROL;

    printf("\n=== PIO0 SM0 STATE ===\n");
    bool reg_sm_running = (pio->ctrl & (1u << sm)) != 0;
    printf("enabled=%d  stalled=%d  PC=0x%02x\n",
           reg_sm_running,
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_get_pc(pio, sm));
    printf("RX FIFO=%d/4  TX FIFO=%d/4\n",
           pio_sm_get_rx_fifo_level(pio, sm),
           pio_sm_get_tx_fifo_level(pio, sm));

    // XACK pin state
    uint32_t xack_status = io_bank0_hw->io[XACK_PIN].status;
    printf("XACK  (GPIO%d): OE=%d OUT=%d PAD=%d\n",
           XACK_PIN,
           !!(xack_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(xack_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(xack_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // EXTIO pin state
    uint32_t extio_status = io_bank0_hw->io[EXTIO_PIN].status;
    printf("EXTIO (GPIO%d): OE=%d OUT=%d PAD=%d\n",
           EXTIO_PIN,
           !!(extio_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(extio_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(extio_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // READY pin state
    uint32_t ready_status = io_bank0_hw->io[READY_PIN].status;
    printf("READY (GPIO%d): PAD=%d\n",
           READY_PIN,
           !!(ready_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // CLK5 - sample 3 times with 1us gaps to detect toggling
    bool clk5_samples[3];
    clk5_samples[0] = gpio_get(CLOCK_5_PIN);
    busy_wait_us(1);
    clk5_samples[1] = gpio_get(CLOCK_5_PIN);
    busy_wait_us(1);
    clk5_samples[2] = gpio_get(CLOCK_5_PIN);
    bool toggling = (clk5_samples[0] != clk5_samples[1]) || (clk5_samples[1] != clk5_samples[2]);
    printf("CLK5  (GPIO%d): samples=%d,%d,%d  %s\n",
           CLOCK_5_PIN,
           clk5_samples[0], clk5_samples[1], clk5_samples[2],
           toggling ? "TOGGLING" : "STUCK");

    // IR4 (DMA IRQ) pin state
    uint32_t ir4_status = io_bank0_hw->io[DMA_IRQ_PIN].status;
    printf("IR4   (GPIO%d): OE=%d OUT=%d PAD=%d  irq_pend=%d\n",
           DMA_IRQ_PIN,
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS),
           dma_registers.state.interrupt_pending);

    // HOLD/HLDA pin state (DMA bus mastering)
    uint32_t hold_status = io_bank0_hw->io[HOLD_PIN].status;
    printf("HOLD  (GPIO%d): OE=%d OUT=%d PAD=%d  func=%lu\n",
           HOLD_PIN,
           !!(hold_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(hold_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(hold_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS),
           (unsigned long)(io_bank0_hw->io[HOLD_PIN].ctrl & 0x1f));
    uint32_t hlda_status = io_bank0_hw->io[HLDA_PIN].status;
    printf("HLDA  (GPIO%d): PAD=%d\n",
           HLDA_PIN,
           !!(hlda_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // PIO1 (DMA master) SM0 state
    bool dma_sm_running = (PIO_DMA_MASTER->ctrl & (1u << DMA_SM_CONTROL)) != 0;
    printf("DMA SM (PIO%d SM%d): enabled=%d  PC=0x%02x  TX=%d/4  RX=%d/4\n",
           pio_get_index(PIO_DMA_MASTER), DMA_SM_CONTROL,
           dma_sm_running,
           pio_sm_get_pc(PIO_DMA_MASTER, DMA_SM_CONTROL),
           pio_sm_get_tx_fifo_level(PIO_DMA_MASTER, DMA_SM_CONTROL),
           pio_sm_get_rx_fifo_level(PIO_DMA_MASTER, DMA_SM_CONTROL));

    // SASI bus state
    printf("bus_ctrl=0x%02X (BSY:%d REQ:%d CTL:%d INP:%d MSG:%d)\n",
           dma_registers.bus_ctrl,
           !!(dma_registers.bus_ctrl & SASI_BSY_BIT),
           !!(dma_registers.bus_ctrl & SASI_REQ_BIT),
           !!(dma_registers.bus_ctrl & SASI_CTL_BIT),
           !!(dma_registers.bus_ctrl & SASI_INP_BIT),
           !!(dma_registers.bus_ctrl & SASI_MSG_BIT));
    printf("cached STATUS=0x%02X  cached DATA=0x%02X\n",
           cached_regs.values[REG_STATUS],
           cached_regs.values[REG_DATA]);

    // ISR diagnostic counters
    extern defer_queue_t defer_queue;
    printf("ISR calls=%lu  FIFO entries=%lu  TX full drops=%lu\n",
           (unsigned long)isr_call_count,
           (unsigned long)isr_fifo_entries_count,
           (unsigned long)isr_tx_fifo_full_count);
    printf("post-status-phase ISR=%lu  flag=%d\n",
           (unsigned long)isr_post_status_phase_count,
           status_phase_flag);
    printf("status-phase breakdown: DATA=%lu STATUS=%lu other=%lu writes=%lu last_phase=0x%02X\n",
           (unsigned long)diag_data_reads,
           (unsigned long)diag_status_reads,
           (unsigned long)diag_other_reads,
           (unsigned long)diag_writes,
           diag_last_phase);
    printf("Defer queue: processed=%lu  drops=%lu\n",
           (unsigned long)defer_queue.processed,
           (unsigned long)defer_queue.drops);

    // Command timing diagnostics
    printf("Cmd timing: last=%lu us  max=%lu us  >1s=%lu  >4s=%lu\n",
           (unsigned long)sasi_last_cmd_us,
           (unsigned long)sasi_max_cmd_us,
           (unsigned long)sasi_cmd_over_1s_count,
           (unsigned long)sasi_cmd_over_4s_count);
    printf("RESET count: %lu\n", (unsigned long)sasi_reset_count);

    // Per-operation timing breakdown
    printf("Op timing (max us): obtain=%lu  sd_rd=%lu  sd_wr=%lu  dma_rd=%lu  dma_wr=%lu  sync=%lu  verify=%lu\n",
           (unsigned long)sasi_op_timing.max_obtain_us,
           (unsigned long)sasi_op_timing.max_sd_read_us,
           (unsigned long)sasi_op_timing.max_sd_write_us,
           (unsigned long)sasi_op_timing.max_dma_read_us,
           (unsigned long)sasi_op_timing.max_dma_write_us,
           (unsigned long)sasi_op_timing.max_sync_us,
           (unsigned long)sasi_op_timing.max_verify_us);
    if (sasi_op_timing.stall_count > 0) {
        static const char *op_names[] = { "?", "dma_rd", "dma_wr", "sd_rd", "sd_wr", "sync", "verify", "obtain" };
        uint32_t op = sasi_op_timing.stall_last_op;
        printf("Stalls (>100ms): count=%lu  last=%s %lu us @ LBA %lu\n",
               (unsigned long)sasi_op_timing.stall_count,
               (op <= SASI_OP_OBTAIN) ? op_names[op] : "?",
               (unsigned long)sasi_op_timing.stall_last_us,
               (unsigned long)sasi_op_timing.stall_last_lba);
    } else {
        printf("Stalls (>100ms): 0\n");
    }

    // Core 1 liveness
    uint64_t now = time_us_64();
    uint64_t last_proc = defer_last_process_us;
    uint64_t defer_age_us = (last_proc > 0) ? (now - last_proc) : 0;
    printf("Defer heartbeat: last_process=%llu us ago  (head=%lu tail=%lu)\n",
           (unsigned long long)defer_age_us,
           (unsigned long)defer_queue.head,
           (unsigned long)defer_queue.tail);

    // Stack canary status
    uint32_t canary_intact = stack_canary_check();
    printf("Core1 stack canary: %lu/%lu intact%s\n",
           (unsigned long)canary_intact, (unsigned long)STACK_CANARY_WORDS,
           (canary_intact < STACK_CANARY_WORDS) ? " *** OVERFLOW DETECTED ***" : "");

#if VERIFY_DMA_WRITES
    printf("DMA verify: checked=%lu  errors=%lu  byte_mismatches=%lu\n",
           (unsigned long)dma_verify_stats.sectors_checked,
           (unsigned long)dma_verify_stats.sectors_failed,
           (unsigned long)dma_verify_stats.total_byte_mismatches);
    if (dma_verify_stats.first_error_recorded) {
        printf("  first_err: LBA=%lu addr=0x%05lX offset=%u exp=0x%02X got=0x%02X\n",
               (unsigned long)dma_verify_stats.first_err_lba,
               (unsigned long)dma_verify_stats.first_err_addr,
               dma_verify_stats.first_err_offset,
               dma_verify_stats.first_err_expected,
               dma_verify_stats.first_err_actual);
    }
    printf("DMA CRC trace: %lu sectors recorded (R=%s, W=%s), use 'r' to dump\n",
           (unsigned long)dma_crc_trace.total,
           "disk->Victor", "Victor->disk");
#endif

    // HardFault info (if captured)
    if (fault_info.valid == FAULT_INFO_MAGIC) {
        printf("*** HARDFAULT on Core %lu: PC=0x%08lX LR=0x%08lX CFSR=0x%08lX ***\n",
               (unsigned long)fault_info.core_id,
               (unsigned long)fault_info.pc,
               (unsigned long)fault_info.lr,
               (unsigned long)fault_info.cfsr);
    }

    printf("=== END PIO0 STATE ===\n\n");
}

static void handle_uart_command(int raw_ch, bool *auto_flush_enabled) {
    if (raw_ch < 0 || !auto_flush_enabled) {
        return;
    }

    if (raw_ch == '\r' || raw_ch == '\n') {
        return;
    }

    char ch = (char)tolower(raw_ch);
    switch (ch) {
        case 'h':
        case '?':
            print_uart_help();
            break;
        case 't':
            printf("\nUART: dumping SASI trace\n");
            sasi_trace_dump();
            break;
#if VERIFY_DMA_WRITES
        case 'r':
            printf("\nUART: dumping DMA CRC trace\n");
            dma_crc_trace_dump();
            break;
#endif
        case 'f':
            printf("\nUART: forcing SASI log flush\n");
            sasi_log_flush_now();
            break;
        case 'a':
            *auto_flush_enabled = !*auto_flush_enabled;
            printf("\nUART: automatic SASI log flush %s\n",
                   *auto_flush_enabled ? "ENABLED" : "DISABLED");
            break;
        case 'p':
            dump_pio0_and_pin_state();
            break;
        case 'i':
            register_irq_trace_dump("UART request");
            break;
        case 'c':
            dump_fault_info();
            break;
        default:
            printf("\nUART: unknown command '%c' (0x%02X). Press 'h' for help.\n",
                   (raw_ch >= 32 && raw_ch <= 126) ? raw_ch : '.',
                   (unsigned int)(raw_ch & 0xFF));
            break;
    }
}


void initialize_uart() {
    // Initialize UART for TX and RX
    gpio_init(UART_TX_PIN);
    gpio_init(UART_RX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // Initialize UART hardware
    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);

    //clear BD0_PIN (GPIO1) - ensure it's not claimed by stdio/UART and ready for PIO
    gpio_disable_pulls(BD0_PIN);                     // clear PUE/PDE (kills bus-keep)
    gpio_set_dir(BD0_PIN, false);                    // input
    gpio_set_function(BD0_PIN, GPIO_FUNC_NULL);      // release back to NULL or PIO later
    return;
}


        int main() {
    set_sys_clock_khz(200000, true);
    stdio_init_all();
    initialize_uart();
    fault_led_init();

    queue_init(&log_queue, sizeof(char[256]), 32); // 32 message buffer

    // Initialize the debug queue for PIO register access logging
    debug_queue_init();
    // Debug output is disabled by default for minimal performance impact
    // Enable only when actively debugging register traffic.
    // debug_queue_enable(true);

    uint32_t seconds = 3;

    printf("\n=== DMA Board Initialization ===\n");
    printf("Sleeping for %d seconds\n", seconds);
    //sleep_ms(timeout);
    printf("Awake!\n");

    // Initialize shared FatFS lock before any SD card file operations.
    fatfs_guard_init();

    // Storage backend init (SD card or FujiNet) and sasi_log_init() run on
    // Core 1 so that SDIO DMA_IRQ_1 is registered on Core 1's NVIC.  This
    // eliminates IRQ contention with Core 0's register ISR (PIO0_IRQ_0)
    // during inter-batch DMA gaps when the 8088 polls STATUS rapidly.
    // See register_cache.c::core1_main() for the actual init sequence.

    bool sasi_log_auto_flush_enabled = SASI_LOG_AUTO_FLUSH_DEFAULT != 0;
    print_uart_help();

    //configure GPIO pulls and output strenght/skew etc
    one_time_pin_setup();
    
    // configure the board_registers PIO, which controls the DMA board registers which 
    // house control & meta data about the SASI bus
    PIO register_pio = PIO_REGISTERS;
    int reg_sm_control = REG_SM_CONTROL;
    pio_sm_claim (register_pio, REG_SM_CONTROL);
    int outcome = pio_set_gpio_base(register_pio, LOWER_PIN_BASE);
    printf("register_pio pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int board_registers_program_offset = pio_add_program(register_pio, &board_registers_program);
    if (board_registers_program_offset < 0) {
        printf("ERROR: Failed to load board_registers_program - PIO%d SM%d instruction memory full!\n", pio_get_index(register_pio), reg_sm_control);
        return -1;
    }

    // Store offset so reset_register_pio_sm() can JMP to wrap_target after DMA.
    dma_set_board_reg_program_offset(board_registers_program_offset);

    board_registers_program_init(register_pio, reg_sm_control, board_registers_program_offset);

    //initialize state machine, it stores a 12-bit bitmask (0x00000EF3) of the registeraddress MSBs to match against for register accesses
    //we pull in the full 20-bit address and then drop the lower 8 bits in the PIO program to compare against the 12 MSBs to match our register range
    pio_sm_put_blocking(register_pio, reg_sm_control, DMA_REGISTER_BITMASK);  
    printf("board_registers initialized on PIO%d SM%d\n",
           pio_get_index(register_pio), reg_sm_control);
    
    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)

    // configure the pio state machines for DMA reading and writing
    PIO pio_dma_master = PIO_DMA_MASTER;
    outcome = pio_set_gpio_base(pio_dma_master, LOWER_PIN_BASE);
    printf("dma_control_pio pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int dma_master_program_offset = pio_add_program(pio_dma_master, &dma_master_program);

    if (dma_master_program_offset < 0) {
        printf("ERROR: Failed to load dma_master_program - PIO%d SM%d instruction memory full!\n", pio_get_index(pio_dma_master), DMA_SM_CONTROL);
        return -1;
    }
    pio_sm_claim(pio_dma_master, DMA_SM_CONTROL); 
    int dma_control_sm = DMA_SM_CONTROL;
    dma_master_program_init(pio_dma_master, dma_control_sm, dma_master_program_offset, BD0_PIN);
    
    printf("dma_master PIO initialized on PIO%d SM%d\n",
           pio_get_index(pio_dma_master), dma_control_sm);

    // Debug PIO state
    printf("PIO state: enabled=%d, stalled=%d, PC=0x%x\n", 
           pio_sm_is_claimed(register_pio, reg_sm_control),
           pio_sm_is_exec_stalled(register_pio, reg_sm_control),
           pio_sm_get_pc(register_pio, reg_sm_control));
     
    dma_device_reset(&dma_registers);
    sasi_trace_init();  // Initialize diagnostic trace buffer
    sasi_op_timing_init();  // Initialize per-operation timing breakdown
#if VERIFY_DMA_WRITES
    dma_crc_trace_init();
#endif
    printf("DMA device reset complete\n");

    // Paint Core 1 stack with canary pattern BEFORE launch.
    stack_canary_init();

    // Launch core 1 only after both PIO state machines and reset state are stable.
    // This avoids cross-core races where core1 cache warmup touches PIO1 SM0 while
    // core0 is still configuring dma_master_program.
    multicore_launch_core1_with_stack(core1_main, core1_stack, sizeof(core1_stack));

#if DEBUG_GPIO
    pio_debug_state();
#endif


    printf("waiting for DMA register access...\n");

    // Stuck-state detector state. The window must be well above the slowest
    // legitimate command (~0.5 s seen with SD write/sync stalls), because the
    // host spins on status reads (ISR-only, never queued) while Core 1 is busy.
    const uint64_t STUCK_WINDOW_US = 2000000;
    const uint32_t STUCK_MIN_ISR_CALLS = 20000;
    uint32_t last_defer_processed = 0;
    uint32_t last_isr_calls = 0;
    uint64_t last_defer_check_us = time_us_64();
    uint64_t last_stuck_check_us = last_defer_check_us;
    bool stuck_already_reported = false;
    bool stack_overflow_reported = false;
    bool fault_info_dumped = false;

    uint64_t iterations = INT64_MAX;
    for (uint64_t i = 0; i<INT64_MAX; i++) {
        int cmd = getchar_timeout_us(0);
        if (cmd >= 0) {
            handle_uart_command(cmd, &sasi_log_auto_flush_enabled);
        }
        if (sasi_log_auto_flush_enabled && storage_ready) {
            sasi_log_flush_if_ready(&dma_registers);
        }

        // Auto stuck-state detector: every ~2s check if Core 1 is making progress
        uint64_t now = time_us_64();
        if (now - last_stuck_check_us > STUCK_WINDOW_US) {
            uint32_t current_processed = defer_queue.processed;
            uint32_t current_isr_calls = isr_call_count;
            bool queue_pending = defer_queue.head != defer_queue.tail;

            if (storage_ready &&
                !sasi_in_dma_transfer &&
                queue_pending &&
                current_processed == last_defer_processed &&
                current_isr_calls > last_isr_calls + STUCK_MIN_ISR_CALLS &&
                !stuck_already_reported) {
                // Work is queued and the host is active, but Core 1 hasn't
                // retired anything for a full window — stuck!
                printf("\n!!! STUCK DETECTED: ISR calls=%lu (+%lu) defer_processed=%lu (stalled) !!!\n",
                       (unsigned long)current_isr_calls,
                       (unsigned long)(current_isr_calls - last_isr_calls),
                       (unsigned long)current_processed);
                dump_pio0_and_pin_state();
                sasi_trace_dump();
                register_irq_trace_dump("auto-stuck");
                stuck_already_reported = true;
            }

            // Reset stuck flag if Core 1 resumes processing
            if (current_processed != last_defer_processed) {
                stuck_already_reported = false;
            }

            last_defer_processed = current_processed;
            last_isr_calls = current_isr_calls;
            last_stuck_check_us = now;
        }

        if (now - last_defer_check_us > 100000) {
            // Stack canary check (runs every ~100ms, very cheap)
            if (!stack_overflow_reported) {
                uint32_t canary_intact = stack_canary_check();
                if (canary_intact < STACK_CANARY_WORDS) {
                    printf("\n!!! CORE1 STACK OVERFLOW: canary %lu/%lu intact !!!\n",
                           (unsigned long)canary_intact, (unsigned long)STACK_CANARY_WORDS);
                    dump_pio0_and_pin_state();
                    stack_overflow_reported = true;
                }
            }

            last_defer_check_us = now;
        }

        tight_loop_contents();
    }
}

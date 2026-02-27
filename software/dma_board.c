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

extern queue_t log_queue;

// Set by Core 1 after SD/FujiNet init + mount completes.
// Core 0 main loop checks this before calling sasi_log_flush_if_ready().
volatile bool storage_ready = false;

#ifndef CORE1_STACK_WORDS
#define CORE1_STACK_WORDS 2048u  // 8 KiB explicit Core1 stack for deferred + FatFS workload
#endif

static uint32_t core1_stack[CORE1_STACK_WORDS] __attribute__((aligned(8)));

static void print_uart_help(void) {
    printf("\nUART commands:\n");
    printf("  h/? : show this help\n");
    printf("  t   : dump SASI trace to UART\n");
    printf("  f   : force SASI_LOG.TXT flush now\n");
    printf("  a   : toggle automatic log flush\n");
    printf("  p   : PIO0 state + XACK/EXTIO/CLK5/READY pin dump\n");
    printf("  i   : fast IRQ DATA transition trace dump\n");
    printf("\n");
}

static void dump_pio0_and_pin_state(void) {
    PIO pio = PIO_REGISTERS;
    uint sm = REG_SM_CONTROL;

    printf("\n=== PIO0 SM0 STATE ===\n");
    printf("enabled=%d  stalled=%d  PC=0x%02x\n",
           pio_sm_is_claimed(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_get_pc(pio, sm));
    printf("RX FIFO=%d/4  TX FIFO=%d/4\n",
           pio_sm_get_rx_fifo_level(pio, sm),
           pio_sm_get_tx_fifo_level(pio, sm));

    // XACK (GPIO26) pin state
    uint32_t xack_status = io_bank0_hw->io[XACK_PIN].status;
    printf("XACK  (GPIO%d): OE=%d OUT=%d PAD=%d\n",
           XACK_PIN,
           !!(xack_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(xack_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(xack_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // EXTIO (GPIO27) pin state
    uint32_t extio_status = io_bank0_hw->io[EXTIO_PIN].status;
    printf("EXTIO (GPIO%d): OE=%d OUT=%d PAD=%d\n",
           EXTIO_PIN,
           !!(extio_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(extio_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(extio_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // READY (GPIO28) pin state
    uint32_t ready_status = io_bank0_hw->io[READY_PIN].status;
    printf("READY (GPIO%d): PAD=%d\n",
           READY_PIN,
           !!(ready_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS));

    // CLK5 (GPIO30) - sample 3 times with 1us gaps to detect toggling
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

    // IR4 (DMA IRQ) pin state — shows whether interrupt is asserted to 8259 PIC
    uint32_t ir4_status = io_bank0_hw->io[DMA_IRQ_PIN].status;
    printf("IR4   (GPIO%d): OE=%d OUT=%d PAD=%d  irq_pend=%d\n",
           DMA_IRQ_PIN,
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS),
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS),
           !!(ir4_status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS),
           dma_registers.state.interrupt_pending);

    // SASI bus state — shows current phase from Pico's perspective
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

    // ISR diagnostic counters — compare two 'p' dumps to see if host is still active
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
        default:
            printf("\nUART: unknown command '%c' (0x%02X). Press 'h' for help.\n",
                   (raw_ch >= 32 && raw_ch <= 126) ? raw_ch : '.',
                   (unsigned int)(raw_ch & 0xFF));
            break;
    }
}


void initialize_uart() {
    // Initialize UART for TX only
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
    ontime_pin_setup();
    
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
    printf("DMA device reset complete\n");

    // Launch core 1 only after both PIO state machines and reset state are stable.
    // This avoids cross-core races where core1 cache warmup touches PIO1 SM0 while
    // core0 is still configuring dma_master_program.
    multicore_launch_core1_with_stack(core1_main, core1_stack, sizeof(core1_stack));
    
#if DEBUG_GPIO
    pio_debug_state();
#endif


    printf("waiting for DMA register access...\n");
    uint64_t iterations = INT64_MAX;
    for (uint64_t i = 0; i<INT64_MAX; i++) {
        int cmd = getchar_timeout_us(0);
        if (cmd >= 0) {
            handle_uart_command(cmd, &sasi_log_auto_flush_enabled);
        }
        if (sasi_log_auto_flush_enabled && storage_ready) {
            sasi_log_flush_if_ready(&dma_registers);
        }
    
        tight_loop_contents();
    }
}

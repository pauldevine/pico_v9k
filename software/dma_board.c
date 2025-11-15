#include <stdio.h>
#include <string.h>


#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/structs/systick.h"
#include "hardware/clocks.h"
#include "board_registers_control.pio.h"
#include "board_registers_output.pio.h"
#include "dma_rw_control.pio.h"
#include "dma_rw_output.pio.h"
#include "iom_helper.pio.h"
#include "pico_victor/dma.h"
#include "pico_victor/debug_queue.h"
#include "pico_fujinet/spi.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 45

extern queue_t log_queue;


void initialize_uart() {
    // Initialize UART for TX only
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    
    // Initialize UART hardware
    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);

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
    // Uncomment the next line to enable debug output
    debug_queue_enable(true);

    int ch=0;
    uint32_t millis_per_second = 1000;
    uint32_t seconds = 3;
    uint32_t timeout = seconds * millis_per_second;

    printf("\n=== DMA Board Initialization ===\n");
    printf("Sleeping for %d seconds\n", seconds);
    sleep_ms(timeout);
    printf("Awake!\n");

    // Initialize SPI bus for FujiNet storage
    spi_bus_init();

    //setup our debug pin
    gpio_init(DEBUG_PIN);
    gpio_set_function(DEBUG_PIN, GPIO_FUNC_SIO);  // Explicitly set to SIO (GPIO) mode
    gpio_disable_pulls(DEBUG_PIN);  // Disable all pulls
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);  // Set as output

    // Test toggles with delays to see restart on logic analyzer
    gpio_put(DEBUG_PIN, 0);
    sleep_ms(1);
    gpio_put(DEBUG_PIN, 1);
    sleep_ms(1);
    gpio_put(DEBUG_PIN, 0);
    sleep_ms(1);
    gpio_put(DEBUG_PIN, 1);
    sleep_ms(1);
    gpio_put(DEBUG_PIN, 0);
    
    // configure the register controller PIO, which controls the board registers timing for both read and write
    // but only handles data for 8088 write cycles, which means pico is reading from the 8088 bus
    // ouputting read data is handled by the board_registers_output PIO state machine
    PIO register_pio = PIO_REGISTERS;
    int reg_sm_control = REG_SM_CONTROL;
    pio_sm_claim (register_pio, REG_SM_CONTROL);
    int outcome = pio_set_gpio_base(register_pio, LOWER_PIN_BASE);
    printf("register_pio pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int board_registers_program_offset = pio_add_program(register_pio, &board_registers_program);
    board_registers_program_init(register_pio, reg_sm_control, board_registers_program_offset);
    //initialize state machine, it stores a 12-bit bitmask (0x00000EF3) of the registeraddress MSBs to match against for register accesses
    //we pull in the full 20-bit address and then drop the lower 8 bits in the PIO program to compare against the 12 MSBs to match our register range
    pio_sm_put_blocking(register_pio, reg_sm_control, DMA_REGISTER_BITMASK);  
    printf("board_registers initialized on PIO%d SM%d\n",
           pio_get_index(register_pio), reg_sm_control);

    // configure the board_register_output PIO state machine to manage BD0-A19 outputs
    // this PIO handles outputting data when the 8088 is reading from pico registers
    // it's on a separate PIO instance to share output pins without conflict with the DMA output PIO
    PIO pio_output = PIO_OUTPUT;
    int reg_sm_output = REG_SM_OUTPUT;
    pio_sm_claim(pio_output, reg_sm_output);
    int reg_output_offset = pio_add_program(pio_output, &register_output_program);
    register_output_program_init(pio_output, reg_sm_output, reg_output_offset);
    pio_sm_set_enabled(pio_output, reg_sm_output, true);
    printf("register_output initialized on PIO%d SM%d\n",
           pio_get_index(pio_output), reg_sm_output);
    
    // Launch core 1 to handle DMA operations
    multicore_launch_core1(core1_main);

    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)

    // configure the two pio state machines for DMA reading and writing
    // dma_rw_control.pio handles the bus control and timing for both read and write cycles
    // dma_rw_output.pio handles inputting and outputting addresses and data during DMA read and write cycles
    //they're on separate PIO instances to share output pins without conflict with board_registers PIO
    PIO dma_control_pio = PIO_DMA_CONTROL;
    //configure the PIO state machine to upper GPIOs because we share the pio with iom_helper which uses a pin >32
    outcome = pio_set_gpio_base(dma_control_pio, UPPER_PIN_BASE);
    printf("dma_control_pio pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int dma_rw_control_program_offset = pio_add_program(dma_control_pio, &dma_rw_control_program);
    pio_sm_claim(dma_control_pio, DMA_SM_CONTROL); 
    int dma_sm = DMA_SM_CONTROL;
    dma_rw_control_program_init(dma_control_pio, dma_sm, dma_rw_control_program_offset, BD0_PIN);
    pio_sm_set_enabled(dma_control_pio, dma_sm, true);
    printf("dma_rw_control PIO initialized on PIO%d SM%d\n",
           pio_get_index(dma_control_pio), dma_sm);

    //configure the dma_rw_output PIO state machine to manage BD0-A19 inputs and outputs
    PIO dma_output_pio = PIO_OUTPUT;
    int dma_output_sm = DMA_SM_OUTPUT;
    pio_sm_claim(dma_output_pio, dma_output_sm);
    int dma_output_offset = pio_add_program(dma_output_pio, &dma_rw_output_program);
    dma_rw_output_program_init(dma_output_pio, dma_output_sm, dma_output_offset);
    pio_sm_set_enabled(dma_output_pio, dma_output_sm, true);
    printf("dma_rw_output PIO initialized on PIO%d SM%d\n",
           pio_get_index(dma_output_pio), dma_output_sm);

    // Preload pindirs value for DMA read cycles (0xFFF00), A19-A8 as outputs, data pins as inputs
    pio_sm_put_blocking(dma_output_pio, dma_output_sm, DMA_READ_T2_PINDIRS);

    //configure the IO/M helper PIO state machine to manage the IO/M pin during register accesses
    PIO iom_pio = PIO_DMA_CONTROL;
    int iom_sm = IOM_SM;
    outcome = pio_set_gpio_base(iom_pio, UPPER_PIN_BASE);
    printf("iom pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    pio_sm_claim (iom_pio, iom_sm);
    int iom_program_offset = pio_add_program(iom_pio, &iom_helper_program);
    iom_helper_program_init(iom_pio, iom_sm, iom_program_offset);
    pio_sm_set_enabled(iom_pio, iom_sm, true);
    printf("IOM PIO initialized\n");

    // The unified dma_read_write state machine handles both read and write operations
    // It determines the operation type from bit 0 of the FIFO payload
    // The unified SM stays enabled and blocks on FIFO when idle

    // Debug PIO state
    printf("PIO state: enabled=%d, stalled=%d, PC=0x%x\n", 
           pio_sm_is_claimed(register_pio, reg_sm_control),
           pio_sm_is_exec_stalled(register_pio, reg_sm_control),
           pio_sm_get_pc(register_pio, reg_sm_control));
     
    dma_device_reset(dma_get_registers());
    printf("DMA device reset complete\n");
    pio_debug_state();
    debug_dump_pin(BD0_PIN);

    //todo: remove after debugging
    uint hold_function = GPIO_FUNC_PIO0 + pio_get_index(PIO_OUTPUT);
    gpio_set_function(HOLD_PIN, hold_function);
    pio_sm_set_consecutive_pindirs(PIO_OUTPUT, reg_sm_output, HOLD_PIN, 1, true);

    printf("HOLD_PIN Pin %d func: %d, sio dir: %d\n",
           HOLD_PIN, gpio_get_function(HOLD_PIN), gpio_get_dir(HOLD_PIN));

    printf("HOLD_PIN PIO dir: %d\n", (PIO_OUTPUT == pio0 ? pio0 : pio1)->dbg_padoe >> HOLD_PIN & 1);

    // Enable cross-PIO IRQ synchronization (PIO0 → PIO1)
    // Required for board_registers "irq next" to signal register_output_helper
    PIO_CTRL_NEXTPREV_CLKDIV_RESTART_BITS;

    printf("PIO0 CTRL: 0x%08x (cross-PIO IRQ enabled)\n", pio0->ctrl);

    printf("waiting for DMA register access...\n");
    uint64_t iterations = INT64_MAX;
    for (uint64_t i = 0; i<INT64_MAX; i++) {
        char buffer[256];
        if (queue_try_remove(&log_queue, buffer)) {
            printf("%s", buffer);
        } else {
            int level = queue_get_level(&log_queue);
            //printf("Log queue empty, current level: %d\n", level);
            if (i % 100000 == 0) {
            printf (".");
            }
        }

        dma_process_deferred_events_cached();

        // Process debug queue entries (non-blocking)
        debug_queue_process();    
            
        // Every 10M iterations, check PIO state
        if (i % 10000000 == 0) {
            printf("\nPIO check: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(register_pio, reg_sm_control),
                    pio_sm_get_tx_fifo_level(register_pio, reg_sm_control),
                    pio_sm_get_pc(register_pio, reg_sm_control),
                    pio_sm_is_exec_stalled(register_pio, reg_sm_control));

            // Check register_output state
            printf("register_output: TX FIFO=%d/8, RX FIFO=%d/8, PC=0x%x, stalled=%d\n",
                    pio_sm_get_tx_fifo_level(pio_output, reg_sm_output),
                    pio_sm_get_rx_fifo_level(pio_output, reg_sm_output),
                    pio_sm_get_pc(pio_output, reg_sm_output),
                    pio_sm_is_exec_stalled(pio_output, reg_sm_output));

            // Check IRQ status for each IRQ on all PIOs
            for (int pio_idx = 0; pio_idx < 3; pio_idx++) {
                PIO pio_inst = (pio_idx == 0) ? pio0 : (pio_idx == 1) ? pio1 : pio2;
                printf("PIO%d IRQ status: IRQ0=%d, IRQ1=%d, IRQ2=%d, IRQ3=%d, IRQ4=%d, IRQ5=%d, IRQ6=%d, IRQ7=%d\n",
                        pio_idx,
                        pio_interrupt_get(pio_inst, 0) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 1) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 2) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 3) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 4) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 5) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 6) ? 1 : 0,
                        pio_interrupt_get(pio_inst, 7) ? 1 : 0);
            }

        }
    
        tight_loop_contents();
    }
}

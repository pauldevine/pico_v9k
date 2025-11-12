#include <stdio.h>
#include <string.h>


#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/structs/systick.h"
#include "hardware/clocks.h"
#include "board_registers.pio.h"
#include "dma_read_write.pio.h"
#include "bus_output_helper.pio.h"
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
    
    //configure the register PIO, used to read/write the DMA registers for controlling the DMA card behavior
    PIO register_pio = PIO_REGISTERS;
    int register_sm = REGISTERS_SM;
    pio_sm_claim (register_pio, REGISTERS_SM);
    int outcome = pio_set_gpio_base(register_pio, LOWER_PIN_BASE);
    printf("register_pio pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int board_registers_program_offset = pio_add_program(register_pio, &board_registers_program);
    board_registers_program_init(register_pio, register_sm, board_registers_program_offset);
    multicore_launch_core1(core1_main);

    //configure the IO/M helper PIO state machine to manage the IO/M pin during register accesses
    PIO iom_pio = PIO_IOM;
    int iom_sm = IOM_SM;
    outcome = pio_set_gpio_base(iom_pio, UPPER_PIN_BASE);
    printf("iom pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    pio_sm_claim (iom_pio, iom_sm);
    int iom_program_offset = pio_add_program(iom_pio, &iom_helper_program);
    iom_helper_program_init(iom_pio, iom_sm, iom_program_offset);
    pio_sm_set_enabled(iom_pio, iom_sm, true);
    printf("IOM PIO initialized\n");

    //configure the bus_output_helper PIO state machine to manage BD0-A19 outputs
    PIO bus_helper_pio = PIO_BUS_HELPER;
    int bus_helper_sm = BUS_HELPER_SM;
    pio_sm_claim(bus_helper_pio, bus_helper_sm);
    int bus_helper_offset = pio_add_program(bus_helper_pio, &bus_output_helper_program);
    bus_output_helper_program_init(bus_helper_pio, bus_helper_sm, bus_helper_offset);
    pio_sm_set_enabled(bus_helper_pio, bus_helper_sm, true);
    printf("bus_output_helper PIO initialized\n");

    // CRITICAL: Preload pindirs value for DMA read cycles (0xFFF00)
    // This satisfies the preamble in bus_output_helper.pio lines 12-13
    pio_sm_put_blocking(bus_helper_pio, bus_helper_sm, 0xFFF00);

    // Store the bus_helper SM info for IRQ handlers to use
    dma_set_bus_helper_sm(bus_helper_pio, bus_helper_sm);
    printf("bus_output_helper initialized on PIO%d SM%d\n",
           pio_get_index(bus_helper_pio), bus_helper_sm);

    printf("pio: %d register_sm: %d board_registers_program_offset: %d pin: %d\n", register_pio, register_sm, board_registers_program_offset, BD0_PIN);
    printf("about to iniitialize board_registers with offfset %X \n", DMA_REGISTER_BITMASK);

    //initialize state machine dma offset location
    pio_sm_put_blocking(register_pio, register_sm, DMA_REGISTER_BITMASK);  

    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)
    

    // configure the two pio state machines for DMA reading and writing, the same code is used for both reading and writing
    PIO dma_pio = PIO_DMA;
    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    pio_sm_claim(dma_pio, DMA_SM);  // Only need one SM for unified read/write
    int dma_sm = DMA_SM;
    dma_read_write_program_init(dma_pio, dma_sm, dma_read_write_program_offset, BD0_PIN);
    dma_set_unified_sm(dma_pio, dma_sm);
    // Keep DMA SM disabled until a DMA transfer is requested
    pio_sm_set_enabled(dma_pio, dma_sm, false);

    // After initializing the DMA PIO, return bus ownership to the register PIO (PIO0)
    // This is needed so board_registers can control its side-set pin (HOLD_PIN/pin 26)
    setup_pio_instance(PIO_REGISTERS, register_sm);


    printf("pio: %d dma_sm: %d dma_read_write_program_offset: %d pin: %d\n", dma_pio, dma_sm, dma_read_write_program_offset, BD0_PIN);

    // The unified dma_read_write state machine handles both read and write operations
    // It determines the operation type from bit 0 of the FIFO payload
    // The unified SM stays enabled and blocks on FIFO when idle

    // Debug PIO state
    printf("PIO state: enabled=%d, stalled=%d, PC=0x%x\n", 
           pio_sm_is_claimed(register_pio, register_sm),
           pio_sm_is_exec_stalled(register_pio, register_sm),
           pio_sm_get_pc(register_pio, register_sm));
     
    dma_device_reset(dma_get_registers());
    printf("DMA device reset complete\n");
    pio_debug_state();
    debug_dump_pin(BD0_PIN);

    //todo: remove after debugging
    uint hold_function = GPIO_FUNC_PIO0 + pio_get_index(PIO_BUS_HELPER);
    gpio_set_function(HOLD_PIN, hold_function);

    // Enable cross-PIO IRQ synchronization (PIO0 → PIO1)
    // Required for board_registers "irq next" to signal bus_output_helper
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

#ifdef CACHED_MODE
        dma_process_deferred_events_cached();
#else
        dma_process_deferred_events();
#endif

        // Process debug queue entries (non-blocking)
        debug_queue_process();    
            
        // Every 10M iterations, check PIO state
        if (i % 10000000 == 0) {
            printf("\nPIO check: FIFO RX level=%d, TX level=%d, PC=0x%x, stalled=%d\n",
                    pio_sm_get_rx_fifo_level(register_pio, register_sm),
                    pio_sm_get_tx_fifo_level(register_pio, register_sm),
                    pio_sm_get_pc(register_pio, register_sm),
                    pio_sm_is_exec_stalled(register_pio, register_sm));
            bool dma_enabled = !!(PIO_DMA->ctrl & (1u << (PIO_CTRL_SM_ENABLE_LSB + dma_sm)));
            printf("DMA SM enabled? %d\n", dma_enabled);

            // Check bus_output_helper state
            printf("bus_output_helper: TX FIFO=%d/8, RX FIFO=%d/8, PC=0x%x, stalled=%d\n",
                    pio_sm_get_tx_fifo_level(bus_helper_pio, bus_helper_sm),
                    pio_sm_get_rx_fifo_level(bus_helper_pio, bus_helper_sm),
                    pio_sm_get_pc(bus_helper_pio, bus_helper_sm),
                    pio_sm_is_exec_stalled(bus_helper_pio, bus_helper_sm));

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

#include <stdio.h>
#include <string.h>


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
    if (!fujinet_config_boot(false)) {
        printf("FujiNet: failed to clear boot config\n");
    }
    if (!fujinet_mount_host(0, FUJINET_DISK_ACCESS_READ)) {
        printf("FujiNet: failed to mount host slot 0\n");
    }
    if (!fujinet_mount_disk_slot(0, FUJINET_DISK_ACCESS_READ)) {
        printf("FujiNet: failed to mount disk slot 0\n");
    }

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

    board_registers_program_init(register_pio, reg_sm_control, board_registers_program_offset);

    //initialize state machine, it stores a 12-bit bitmask (0x00000EF3) of the registeraddress MSBs to match against for register accesses
    //we pull in the full 20-bit address and then drop the lower 8 bits in the PIO program to compare against the 12 MSBs to match our register range
    pio_sm_put_blocking(register_pio, reg_sm_control, DMA_REGISTER_BITMASK);  
    printf("board_registers initialized on PIO%d SM%d\n",
           pio_get_index(register_pio), reg_sm_control);
    
    // Launch core 1 to handle board access operations
    multicore_launch_core1(core1_main);

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
     
    dma_device_reset(dma_get_registers());
    printf("DMA device reset complete\n");
    pio_debug_state();
    debug_dump_pin(BD0_PIN);


    printf("waiting for DMA register access...\n");
    uint64_t iterations = INT64_MAX;
    for (uint64_t i = 0; i<INT64_MAX; i++) {
        char buffer[256];
        if (queue_try_remove(&log_queue, buffer)) {
            printf("%s", buffer);
        } else if (i % 100000 == 0) {
            printf(".");
        }

        // Process debug queue entries (non-blocking)
        debug_queue_process();    
            
        // Every 10M iterations, check PIO state
        if (i % 10000000 == 0) {
            // Check board_registers state machine (PIO0)
            printf("\nboard_registers (PIO%d SM%d): PC=0x%02x, stalled=%d, RX=%d/4, TX=%d/4\n",
                   pio_get_index(register_pio), reg_sm_control,
                   pio_sm_get_pc(register_pio, reg_sm_control),
                   pio_sm_is_exec_stalled(register_pio, reg_sm_control),
                   pio_sm_get_rx_fifo_level(register_pio, reg_sm_control),
                   pio_sm_get_tx_fifo_level(register_pio, reg_sm_control));


            // Check DMA control state machine (PIO2)
            bool dma_enabled = !!(pio_dma_master->ctrl & (1u << (PIO_CTRL_SM_ENABLE_LSB + dma_control_sm)));
            printf("dma_read_write (PIO%d SM%d): PC=0x%02x, stalled=%d, RX=%d/4, TX=%d/4, enabled=%d\n",
                   pio_get_index(pio_dma_master), dma_control_sm,
                   pio_sm_get_pc(pio_dma_master, dma_control_sm),
                   pio_sm_is_exec_stalled(pio_dma_master, dma_control_sm),
                   pio_sm_get_rx_fifo_level(pio_dma_master, dma_control_sm),
                   pio_sm_get_tx_fifo_level(pio_dma_master, dma_control_sm),
                   dma_enabled);

            // Check IRQ flags on all PIOs
            printf("\nIRQ Flags:\n");
            for (int pio_idx = 0; pio_idx < 3; pio_idx++) {
                PIO pio_inst = (pio_idx == 0) ? pio0 : (pio_idx == 1) ? pio1 : pio2;
                printf("  PIO%d: ", pio_idx);
                for (int irq = 0; irq < 8; irq++) {
                    if (pio_interrupt_get(pio_inst, irq)) {
                        printf("IRQ%d=1 ", irq);
                    }
                }
                printf("\n");
            }

            // Check defer queue stats
            defer_queue_t *q = defer_get_queue();
            if (q->drops > 0) {
                printf("DEFER QUEUE: drops=%lu processed=%lu\n",
                       (unsigned long)q->drops, (unsigned long)q->processed);
            }

        }
    
        tight_loop_contents();
    }
}

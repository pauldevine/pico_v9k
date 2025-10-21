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
#include "pico_victor/dma.h"
#include "pico_victor/debug_queue.h"
#include "pico_fujinet/spi.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
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

    return;
}

 int main() {

    // DO NOT call stdio_init_all() - it claims GPIO0/1 for UART which conflicts with BD0/BD1
    // We manually initialize UART on pin 46 instead

    set_sys_clock_khz(200000, true);

    initialize_uart();    

    queue_init(&log_queue, sizeof(char[256]), 32); // 32 message buffer

    // Initialize the debug queue for PIO register access logging
    debug_queue_init();
    // Debug output is disabled by default for minimal performance impact
    // Uncomment the next line to enable debug output
    debug_queue_enable(true);

    int ch=0;
    uint32_t millis_per_second = 1000000;
    uint32_t seconds = 5;
    uint32_t timeout = seconds * millis_per_second;

    printf("\n=== DMA Board Initialization ===\n");
    printf("Sleeping for %d seconds\n", seconds);
    //sleep_ms(timeout);
    printf("Awake!\n");

    // Initialize SPI bus for FujiNet storage
    spi_bus_init();

    //setup our debug pin
    gpio_init(DEBUG_PIN);
    gpio_set_function(DEBUG_PIN, GPIO_FUNC_SIO);  // Explicitly set to SIO (GPIO) mode
    gpio_disable_pulls(DEBUG_PIN);  // Disable all pulls
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);  // Set as output

    // Test toggles with delays to see on logic analyzer
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
    printf("pio_set_gpio_base outcome: %d PICO_PIO_USE_GPIO_BASE %d\n", outcome, PICO_PIO_USE_GPIO_BASE);
    int dma_registers_program_offset = pio_add_program(register_pio, &dma_registers_program);
    dma_registers_program_init(register_pio, register_sm, dma_registers_program_offset);
    multicore_launch_core1(core1_main);

    printf("pio: %d register_sm: %d dma_registers_program_offset: %d pin: %d\n", register_pio, register_sm, dma_registers_program_offset, BD0_PIN);
    printf("about to iniitialize dma_registers with offfset %X \n", DMA_REGISTER_BITMASK);

    //initialize state machine dma offset location
    pio_sm_put_blocking(register_pio, register_sm, DMA_REGISTER_BITMASK);  

    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)

    printf("back to main loop\n");
    printf("HOLD_PIN=%d\n", HOLD_PIN);
    printf("Pin %d func: %d, dir: %d\n", HOLD_PIN, gpio_get_function(HOLD_PIN), gpio_get_dir(HOLD_PIN));
    

    // configure the two pio state machines for DMA reading and writing, the same code is used for both reading and writing
    // the init controls which mode the program runs in, ends up with 2 state machines with different
    // configs running the same code, saving on PIO space
    // Keep DMA on the other PIO instance (PIO0). We'll switch GPIO mux
    // between PIO1 (registers) and PIO0 (DMA) around DMA operations.
    PIO dma_pio = PIO_DMA;
    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int dma_sm = pio_claim_unused_sm(dma_pio, true);  // Only need one SM for unified read/write
    dma_read_write_program_init(dma_pio, dma_sm, dma_read_write_program_offset, BD0_PIN);
    dma_set_unified_sm(dma_pio, dma_sm);
    // Keep DMA SM disabled until a DMA transfer is requested
    pio_sm_set_enabled(dma_pio, dma_sm, false);
    // After initializing the DMA PIO, return bus ownership to the register PIO (PIO1)
    {
        uint func = GPIO_FUNC_PIO0 + pio_get_index(PIO_REGISTERS);
        for (int pin = BD0_PIN; pin <= IO_M_PIN; ++pin) {
            gpio_set_function(pin, func);
        }
        for (int pin = READY_PIN; pin <= IR_4_PIN; ++pin) {
            gpio_set_function(pin, func);
            gpio_set_input_enabled(pin, true);
        }
    }
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

        }
    
        tight_loop_contents();
    }
}

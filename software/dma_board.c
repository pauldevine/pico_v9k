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
#include "pico_fujinet/spi.h"

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

extern queue_t log_queue;

void initialize_uart() {
    // Initialize UART with high priority
    gpio_init(UART_TX_PIN);
    gpio_init(UART_RX_PIN);

    gpio_set_dir(UART_TX_PIN, GPIO_OUT);
    gpio_set_dir(UART_RX_PIN, GPIO_IN);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, UART_RX_PIN);
    uart_set_fifo_enabled(UART_ID, false);

    return;
}


int main() {

    stdio_init_all();

    set_sys_clock_khz(200000, true);

    initialize_uart();    

    queue_init(&log_queue, sizeof(char[256]), 32); // 32 message buffer

    int ch=0;
    uint32_t millis_per_second = 1000000;
    uint32_t seconds = 5;
    uint32_t timeout = seconds * millis_per_second;

    printf("Sleeping for %d seconds\n", seconds);
    //sleep_ms(timeout);
    printf("Awake!\n");

    // Initialize SPI bus for FujiNet storage
    spi_bus_init();
    
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
    printf("LOW_ADDR_DIR=%d, HOLD_PIN=%d\n", LOW_ADDR_DIR, HOLD_PIN);
    printf("Pin %d func: %d, dir: %d\n", LOW_ADDR_DIR, gpio_get_function(LOW_ADDR_DIR), gpio_get_dir(LOW_ADDR_DIR));
    printf("Pin %d func: %d, dir: %d\n", HOLD_PIN, gpio_get_function(HOLD_PIN), gpio_get_dir(HOLD_PIN));
    

    // configure the two pio state machines for DMA reading and writing, the same code is used for both reading and writing
    // the init controls which mode the program runs in, ends up with 2 state machines with different
    // configs running the same code, saving on PIO space
    PIO dma_pio = PIO_DMA;
    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int read_sm = pio_claim_unused_sm(dma_pio, true);
    int write_sm = pio_claim_unused_sm(dma_pio, true);
    dma_read_write_program_init(dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN, DMA_READ); 
    dma_read_write_program_init(dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN, DMA_WRITE); 
    printf("pio: %d dma_read: %d dma_read_write_program_offset: %d pin: %d\n", dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN);
    printf("pio: %d dma_write: %d dma_read_write_program_offset: %d pin: %d\n", dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN);

    printf("about to iniitialize read_sm with x %d and y %X\n", DMA_READ, DMA_READ_T2_PINDIRS);
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ);  //initialize state machine for read operation
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ_T2_PINDIRS);  //feed pindirs for RD operation to state machine

    printf("about to iniitialize write_sm with x %d and y %X\n", DMA_WRITE, DMA_WRITE_T2_PINDIRS);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE);  //initialize state machine for write operation
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE_T2_PINDIRS);  //not used in WR operation, but needed to keep the state machine happy

    //state machines are already disabled from initialization, will enable when a DMA operation is requested

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
            
        // Every 10M iterations, check PIO state
        if (i % 10000000 == 0) {
            printf("\nPIO check: FIFO level=%d, PC=0x%x, stalled=%d\n", 
                    pio_sm_get_rx_fifo_level(register_pio, register_sm),
                    pio_sm_get_pc(register_pio, register_sm),
                    pio_sm_is_exec_stalled(register_pio, register_sm));
        }
    
        tight_loop_contents();
    }
}

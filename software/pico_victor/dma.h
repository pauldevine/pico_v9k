#ifndef DMA_VICTOR_H
#define DMA_VICTOR_H
#include <stdio.h>
#include "hardware/pio.h"

#define REGISTERS_SM  0
#define PIO_BUS pio0
#define PIO_ADDRESS pio1

#define PIO_DMA pio0
#define PIO_REGISTERS pio1

#define ADDR_START_PIN 1 // DMA address output
#define LOWER_PIN_BASE 0
#define UPPER_PIN_BASE 16
#define DATA_SIZE 8
#define ADDRESS_BUS_SIZE 20
#define ADDRESS_AND_CONTROL_SIZE 31
#define ADDRESS_AND_DATA_SIZE 28
#define ADDRESS_DATA_CONBIT_SIZE 29  // 20-bit address, 8-bit data, 1-bit control
#define PIO_FULL_SIZE 32

#define BD0_PIN 1
#define RD_PIN 21
#define WR_PIN 22
#define DTR_PIN 23       
#define ALE_PIN 24      
#define DEN_PIN 25 
#define HOLD_PIN 26  
#define IO_M_PIN 27
#define READY_PIN 28
#define HLDA_PIN 29
#define CLOCK_5_PIN 30
#define CLOCK_15B_PIN 31
#define IR_4_PIN 32
#define XACK_PIN 33
#define EXTIO_PIN 34
#define IR_5_PIN 35
#define SSO_PIN 36
#define DLATCH_PIN 37
#define CSEN_PIN 38
#define PHASE_2_PIN 39

#define TEMP_DEBUG_PIN HOLD_PIN
#define DEBUG_PIN 45 //TODO: After debugging completes move back to pin 45.

#define ADDRESS_DIR_PINCNT 2
#define DMA_READ 1
#define DMA_WRITE 0

// FIFO operation types, defines for the 2-bit pio payload type flag
#define FIFO_PREFETCH_ADDRESS 0x00
#define FIFO_READ_COMMIT 0x01
#define FIFO_WRITE_VALUE 0x02
#define FIFO_UNUSED 0x03

static inline uint32_t dma_fifo_payload_type(uint32_t raw_value) {
    return (raw_value >> 30) & 0x03u;
}

static inline uint32_t dma_fifo_prefetch_address(uint32_t raw_value) {
    return (raw_value >> 10) & 0xFFFFFu;
}

static inline uint32_t dma_fifo_commit_address(uint32_t raw_value) {
    return (raw_value >> 10) & 0xFFFFFu;
}

static inline uint32_t dma_fifo_write_address(uint32_t raw_value) {
    return (raw_value >> 2) & 0xFFFFFu;
}

static inline uint8_t dma_fifo_write_data(uint32_t raw_value) {
    return (raw_value >> 22) & 0xFFu;
}

static inline uint32_t dma_mask_offset(uint32_t offset) {
    if (offset >= 0x80) {
        return offset & ~0x1Fu;
    }
    return offset & ~0x0Fu;
}

static inline uint32_t dma_fifo_encode_prefetch(uint32_t address) {
    return ((address & 0xFFFFFu) << 10) | ((uint32_t)FIFO_PREFETCH_ADDRESS << 30);
}

static inline uint32_t dma_fifo_encode_commit(uint32_t address) {
    return ((address & 0xFFFFFu) << 10) | ((uint32_t)FIFO_READ_COMMIT << 30);
}

static inline uint32_t dma_fifo_encode_write(uint32_t address, uint8_t data) {
    return ((address & 0xFFFFFu) << 2) |
           (((uint32_t)data & 0xFFu) << 22) |
           ((uint32_t)FIFO_WRITE_VALUE << 30);
}

static void setup_pio_instance(PIO pio, int sm) {

    for (int pin = BD0_PIN; pin <= CLOCK_15B_PIN; ++pin) {
        uint function = GPIO_FUNC_PIO0 + pio_get_index(pio);
        gpio_set_function(pin, function);
        pio_gpio_init(pio, pin);
        gpio_pull_down(pin);
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
        pio_sm_set_pins_with_mask(pio, sm, 0u, 1u << pin); // preload latch low
        pio_sm_set_pindirs_with_mask(pio, sm, 0u, 1u << pin); // input

        printf("dma gpio_init %d  ", pin);
        printf("GPIO%d_CTRL = 0x%08x\n", pin, io_bank0_hw->io[pin].ctrl);
    }
}


// DMA operations use a TWO-WORD FIFO protocol (see dma_read_write.pio for details):
//
// WRITE: pio_sm_put(pio, sm, (addr << 1) | 1);        // Word 1: address + write flag
//        pio_sm_put(pio, sm, (addr & 0xFFF00) | data); // Word 2: A8-A19 + data byte
//
// READ:  pio_sm_put(pio, sm, (addr << 1) | 0);        // Word 1: address + read flag
//        pio_sm_put(pio, sm, 0xFFF00);                 // Word 2: pindirs control

#define DMA_REGISTER_BASE 0xEF300
#define DMA_REGISTER_BITMASK 0x00000EF3  //the top 12 bits for the PIO instance to match address against
#define DMA_BUS_START_POSITION (0xDF6 & 0xFFF)  // Sets up the BUS CONTROL pins to start DMA transfers
#define DMA_READ_START_POSITION (0xDF6 & 0xFFF)  // Sets up the BUS CONTROL pins to start DMA transfers
#define DMA_READ_T2_PINDIRS 0xFFF00 // Sets up the Address pins so 0-7 are inputs and 8-19 are outputs
#define DMA_WRITE_T2_PINDIRS 0x07FFFFFF //Sets up the address and DMA control pins so 0-27 are outputs, 28-31 are inputs

// Convert SM number to the corresponding IRQ source
static const enum pio_interrupt_source fifo_sources[] = {
    pis_sm0_rx_fifo_not_empty,
    pis_sm1_rx_fifo_not_empty, 
    pis_sm2_rx_fifo_not_empty,
    pis_sm3_rx_fifo_not_empty
};

#pragma pack(push, 1)
typedef struct {
    // Control register (write-only)
    uint8_t control;

    // Command/Status register (read/write)
    uint8_t command;
    uint8_t status;

    // Bus status register emulation (read-only)
    struct {
        uint8_t REQ : 1;
        uint8_t BSY : 1;
        uint8_t C_D : 1;
        uint8_t I_O : 1;
        uint8_t MSG : 1;
        uint8_t reserved : 3;
    } bus;

    // DMA address registers (3 bytes, written by CPU)
    union {
        struct {
            uint8_t low;   // hddmal
            uint8_t mid;   // hddmam
            uint8_t high;  // hddmah
        } bytes;
        uint32_t full;     // Internal Pico usage
    } dma_address;

    // Block count (as passed by LRB)
    union {
        struct {
            uint8_t low;
            uint8_t high;
        } bytes;
        uint16_t full;
    } block_count;

    // Logical Block Address (sector number)
    union {
        struct {
            uint8_t b0;
            uint8_t b1;
            uint8_t b2;
            uint8_t b3;
        } bytes;
        uint32_t full;
    } logical_block;

    // Optional data buffer (e.g., for load params or sense commands)
    struct {
        uint8_t data[16];
        uint8_t index;
        uint8_t length;
    } buffer;

    // Internal state flags (not exposed to CPU)
    struct {
        uint8_t dma_enabled : 1;
        uint8_t dma_strobe  : 1;
        uint8_t dma_dir_in  : 1; // 1 = device → RAM (read), 0 = RAM → device
        uint8_t interrupt_pending : 1;
        uint8_t asserting_ack : 1;
        uint8_t non_dma_req : 1;
    } state;
    
    // SASI bus control state (tracks bus signals for status register)
    uint8_t bus_ctrl;

    // Selected SASI target ID (from data bus during selection)
    uint8_t selected_target;
} dma_registers_t;
#pragma pack(pop)

// Register offsets
typedef enum {
    REG_CONTROL = 0x00,
    REG_DATA    = 0x10,
    REG_STATUS  = 0x20, 
    REG_ADDR_L  = 0x80,
    REG_ADDR_M  = 0xA0,
    REG_ADDR_H  = 0xC0
} dma_reg_offsets_t;

// Control register bits
typedef enum {
    DMA_ON_VALUE_BIT = 0x01,  // Matches BIT_DMA_Enable (enable value latched by strobe)
    DMA_LOCKOUT_BIT  = 0x02,  // Matches BIT_CPU_Lockout
    DMA_ON_LATCH_BIT = 0x04,  // Matches BIT_DMA_Strobe latch pulse
    DMA_WR_MODE_BIT  = 0x08,  // Matches BIT_DMA_Dir (1 = device -> host)
    DMA_SELECT_BIT   = 0x10,  // Matches BIT_Select (assert SEL/)
    DMA_RESET_BIT    = 0x20   // Matches BIT_Reset
} dma_control_bits_t;

// SASI bus state bits (for internal state tracking)
typedef enum {
    SASI_INP_BIT = 0x01,
    SASI_CTL_BIT = 0x02,
    SASI_BSY_BIT = 0x04,
    SASI_REQ_BIT = 0x08,
    SASI_MSG_BIT = 0x10,
    SASI_SEL_BIT = 0x20,
    SASI_ACK_BIT = 0x40
} sasi_status_bits_t;

void debug_dump_pin(uint pin); 
void pio_debug_state();
void core1_main();
void setup_bus_control();
dma_registers_t* dma_get_registers();
// Unified DMA SM management (single-SM dma_read_write.pio)
void dma_set_unified_sm(PIO pio, int sm);
int dma_get_unified_sm();
PIO dma_get_unified_pio();
void dma_write_to_victor_ram(PIO write_pio, int write_sm, uint8_t *data, size_t length, uint32_t start_address);
void dma_read_from_victor_ram(PIO read_pio, int read_sm, uint8_t *data, size_t length, uint32_t start_address);
void registers_irq_handler();
void dma_process_deferred_events(void);

// Cached/deferred processing functions
#ifdef CACHED_MODE
void registers_irq_handler_cached(void);
void registers_irq_handler_cached_asm(void);
void registers_irq_handler_cached_init(void);
void dma_process_deferred_events_cached(void);
#endif
void dma_write_register(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value);
uint8_t dma_read_register(dma_registers_t *dma, dma_reg_offsets_t offset);
void dma_device_reset(dma_registers_t *dma);
void dma_update_interrupts(dma_registers_t *dma, bool irq_state);
void dma_handle_sasi_req(dma_registers_t *dma);


#ifdef UNIT_TEST
uint8_t* test_get_victor_ram();
size_t test_get_victor_ram_size();
#endif

#endif

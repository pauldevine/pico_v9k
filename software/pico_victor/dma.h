#ifndef DMA_VICTOR_H
#define DMA_VICTOR_H
#include "hardware/pio.h"

#define WRITE_SM 0
#define READ_SM  1
#define REGISTERS_SM  0
#define PIO_BUS pio0
#define PIO_ADDRESS pio1

#define PIO_DMA pio0
#define PIO_REGISTERS pio1

#define ADDR_START_PIN 0 // DMA address output
#define LOWER_PIN_BASE 0
#define UPPER_PIN_BASE 16
#define DATA_SIZE 8
#define ADDRESS_BUS_SIZE 20
#define ADDRESS_AND_CONTROL_SIZE 31
#define ADDRESS_AND_DATA_SIZE 28
#define ADDRESS_DATA_CONBIT_SIZE 29  // 20-bit address, 8-bit data, 1-bit control
#define PIO_FULL_SIZE 32

#define BD0_PIN 0
#define RD_PIN 20
#define WR_PIN 21
#define DTR_PIN 22       
#define LOW_ADDR_DIR 23    // LOW ADDRESS direction pin   
#define HOLD_PIN 24
#define BUS_CNTRL_DIR 25   // BUS CONTROL signals direction pins
#define DEN_PIN 26 
#define ALE_PIN 27
#define Ready_PIN 28
#define HLDA_PIN 29
#define CLOCK_5_PIN 30
#define CLOCK_15B_PIN 31
#define XACK_PIN 32
#define EXTIO_PIN 33
#define IR_4_PIN 34
#define IOM_PIN 35
#define SSO_PIN 36
#define DLATCH_PIN 37
#define CSEN_PIN 38
#define PHASE_2_PIN 39

#define ADDRESS_DIR_PINCNT 2
#define DMA_READ 1
#define DMA_WRITE 0

#define DMA_REGISTER_BASE 0xEF300
#define DMA_REGISTER_BITMASK 0x00000EF3  //the top 12 bits for the PIO instance to match address against
#define DMA_BUS_START_POSITION (0xDF6 & 0xFFF)  // Sets up the BUS CONTROL pins to start DMA transfers
#define DMA_READ_START_POSITION (0xDF6 & 0xFFF)  // Sets up the BUS CONTROL pins to start DMA transfers
#define DMA_READ_T2_PINDIRS 0xFFF00 // Sets up the Address pins so 0-7 are inputs and 8-19 are outputs
#define DMA_WRITE_T2_PINDIRS 0xFFFFF //these aren't actually used, but are here to send something to the state machine

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
    DMA_ON_VALUE_BIT = 0x01,  // DMA value bit
    DMA_LOCKOUT_BIT  = 0x02,  // DMA lockout
    DMA_ON_LATCH_BIT = 0x04,  // DMA on latch bit
    DMA_WR_MODE_BIT  = 0x08,  // Write mode (1=write to memory, 0=read from memory)
    DMA_SELECT_BIT   = 0x10,  // Target selection
    DMA_RESET_BIT    = 0x20   // Reset bit
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

void core1_main();
void setup_bus_control();
dma_registers_t* dma_get_registers();
void dma_write_to_victor_ram(PIO write_pio, int write_sm, uint8_t *data, size_t length, uint32_t start_address);
void dma_read_from_victor_ram(PIO read_pio, int read_sm, uint8_t *data, size_t length, uint32_t start_address);
void registers_irq_handler();
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

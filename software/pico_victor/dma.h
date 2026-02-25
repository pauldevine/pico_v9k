#ifndef DMA_VICTOR_H
#define DMA_VICTOR_H
#include <stdio.h>
#include "hardware/pio.h"

#define UART_ID uart0
#define BAUD_RATE 230400

#define PIO_REGISTERS   pio0
#define REG_SM_CONTROL  0   

#define PIO_DMA_MASTER pio1
#define DMA_SM_CONTROL  0

#define ADDR_START_PIN 1 // DMA address output
#define LOWER_PIN_BASE 0
#define UPPER_PIN_BASE 16
#define DATA_SIZE 8
#define ADDRESS_BUS_SIZE 20
#define ADDRESS_AND_CONTROL_SIZE 31
#define ADDRESS_AND_DATA_SIZE 28
#define ADDRESS_DATA_CONBIT_SIZE 29  // 20-bit address, 8-bit data, 1-bit control
#define PIO_FULL_SIZE 32

#define DMA_BATCH_SIZE 32   // Bytes per DMA batch transfer (interleaves host status polls)
#define DMA_SHARE_WAIT_US 8   // Small bus-share pause between batches (microseconds)
#define DMA_TIMEOUT_US 2000  // Timeout for DMA master acquisition (in microseconds)

#define UART_TX_PIN 0
#define BD0_PIN 1
#define A19_PIN 20
#define RD_PIN 21
#define WR_PIN 22
#define DTR_PIN 23       
#define ALE_PIN 24      
#define HOLD_PIN 25 
#define XACK_PIN 26  
#define EXTIO_PIN 27
#define READY_PIN 28
#define HLDA_PIN 29
#define CLOCK_5_PIN 30
#define CLOCK_15B_PIN 31
#define IR_4_PIN 32
#define UART_RX_PIN 33
#define IR_5_PIN 34
#define IO_M_PIN 35

#define SSO_PIN 36
#define DLATCH_PIN 37
#define CSEN_PIN 38
#define PHASE_2_PIN 39
#define DEN_PIN 40

#define SDIO_CLK 41
#define SDIO_CMD 42
#define SDIO_D0 43
#define SDIO_D1 44
#define SDIO_D2 45
#define SDIO_D3 46

#define ADDRESS_DIR_PINCNT 2
#define DMA_READ 1
#define DMA_WRITE 0

// DMA board IRQ line (IR4 by default; jumperable to IR5 per manual).
#define DMA_IRQ_PIN IR_4_PIN
#define DMA_IRQ_ASSERT_LEVEL 1
#define DMA_IRQ_DEASSERT_LEVEL 0

// Extract target ID from SASI selection byte (bit mask format)
// In SASI selection, the data byte is a bit mask where each bit represents an ID:
// - Bit 7 (0x80) = Initiator (typically ID 7)
// - Bits 0-6 = Target IDs (one bit set for the selected target)
// Example: 0x81 = initiator 7 + target 0, 0x82 = initiator 7 + target 1
static inline uint8_t sasi_extract_target_id(uint8_t selection_byte) {
    uint8_t target_mask = selection_byte & 0x7F;  // Remove initiator bit (bit 7)
    if (target_mask == 0) return 0;
    // Find lowest set bit (target ID) using builtin
    return (uint8_t)__builtin_ctz(target_mask);
}

// FIFO operation types encoded in bit 31 of the payload word.
#define FIFO_REG_READ    0x0
#define FIFO_REG_WRITE   0x1

#define FIFO_DMA_READ        0x0
#define FIFO_DMA_WRITE       0x1

static inline uint32_t fifo_payload_type(uint32_t raw_value) {
    // PIO payload varies by operation RD vs WR, but payload-type identifier is always in bit 31
    //[RD/WR flag 1 bit][WR data 8 bits][address 20 bits]
    return (raw_value >> 31) & 0x01u; 
}

static inline uint32_t board_fifo_read_address(uint32_t raw_value) {
    // read payload is MSB 0[address 20 bits]
    return (raw_value >> 11) & 0xFFFFFu;  // Address is in bits 30-9 (board_registers uses right-shift ISR)
}

static inline uint32_t dma_fifo_write_address(uint32_t raw_value) {
    // write payload is MSB 1[data 8 bits][address 20 bits] 
    return (raw_value >> 3) & 0xFFFFFu;  // Write address is in bits 22-3 (ISR restored at line 66, then shifts by 9)
}

static inline uint8_t dma_fifo_write_data(uint32_t raw_value) {
    // write payload is MSB 1[data 8 bits][address 20 bits] 
    return (raw_value >> 23) & 0xFFu;  // Write data is in bits 30-23
}

static inline uint32_t dma_mask_offset(uint32_t offset) {
    if (offset >= 0x80) {
        return offset & ~0x1Fu;
    }
    return offset & ~0x0Fu;
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
#define DMA_READ_T2_PINDIRS 0xFFF00 // Sets up the Address pins so [A19-A8] are outputs FFF, [BD7-BD0] 00 data pins are inputs
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
    // NOTE: volatile required - written by Core 1 (sasi_enter_status_phase),
    // read by Core 0 ISR during single-pass DATA reads.
    volatile uint8_t status;

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
    // NOTE: Keep these as separate volatile bytes (not bitfields). Bitfields cause
    // read-modify-write collisions between fast IRQ paths and deferred processing.
    volatile struct {
        uint8_t dma_enabled;
        uint8_t dma_strobe;
        uint8_t dma_dir_in;          // 1 = device -> RAM (read), 0 = RAM -> device
        uint8_t interrupt_pending;
        uint8_t asserting_ack;
        uint8_t non_dma_req;
        uint8_t status_pending;      // 1 = waiting for host to read status byte
        uint8_t data_out_expected;   // Non-zero = expecting this many data-out bytes (e.g., for 0x0C params)
    } state;

    // Set by Core 0 ISR when a RESET write is detected on the control register.
    // Checked by Core 1 command loops to abort long-running commands promptly.
    volatile bool reset_requested;

    // SASI bus control state (tracks bus signals for status register)
    // NOTE: volatile required - accessed by both Core 0 (fast handler) and Core 1 (deferred processor)
    volatile uint8_t bus_ctrl;

    // Selected SASI target ID (from data bus during selection)
    // NOTE: volatile - written by Core 1 (defer processor), read cross-core
    volatile uint8_t selected_target;
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

void ontime_pin_setup();
void debug_dump_pin(uint pin);
void pio_debug_state();
void core1_main();
void setup_bus_control();
// Global DMA registers — lives in scratch_x RAM for contention-free Core 1 access.
extern dma_registers_t dma_registers;
bool dma_write_to_victor_ram(uint8_t *data, size_t length, uint32_t start_address);
bool dma_read_from_victor_ram(uint8_t *data, size_t length, uint32_t start_address);

// Hold/release the Victor 8088 bus via HOLD/HLDA handshake.
// Use around slow SD card I/O to prevent the BIOS timeout counter from advancing.
// Does NOT touch PIO state machines — safe to call independently of DMA transfers.
bool hold_victor_bus(void);
void release_victor_bus(void);
void dma_process_deferred_events(void);
void dma_process_deferred_events_cached(void);

void dma_write_register(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value);
uint8_t dma_read_register(dma_registers_t *dma, dma_reg_offsets_t offset);
void dma_device_reset(dma_registers_t *dma);
void dma_update_interrupts(dma_registers_t *dma, bool irq_state);
void dma_handle_sasi_req(dma_registers_t *dma);

// Store the board_registers PIO program offset so reset_register_pio_sm()
// can JMP back to wrap_target (T0) after a safe restart.
void dma_set_board_reg_program_offset(int offset);

// Safe PIO0 SM0 restart: releases XACK/EXTIO, clears FIFOs, JMPs to T0.
// Preserves Y register (0xEF3 bitmask). Call after DMA transfers or abort.
void reset_register_pio_sm(void);

// DMA read IRQ control - enable when DMA is active, disable when idle
void enable_dma_read_irq(void);
void disable_dma_read_irq(void);


#ifdef UNIT_TEST
uint8_t* test_get_victor_ram();
size_t test_get_victor_ram_size();
#endif

#endif // DMA_VICTOR_H

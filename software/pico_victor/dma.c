#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dma.h"
#include "sasi.h"
#include "logging.h"
#include "reg_queue_processor.h"
#include "pico_fujinet/spi.h"

#define SASI_SECTOR_SIZE 512

void debug_dump_pin(uint pin) {
    uint32_t ctrl   = iobank0_hw->io[pin].ctrl;    // contains FUNCSEL, INOVER, OEOVER, OUTOVER
    uint32_t status = iobank0_hw->io[pin].status;  // effective OE/OUT, input sample, etc.
    uint32_t pad    = padsbank0_hw->io[pin];       // IE, PUE, PDE, DRIVE, SCHMITT, SLEWFAST
    printf("GPIO%u: CTRL=%08x STATUS=%08x PADREG=%08x\n", pin, ctrl, status, pad);
}

void pio_debug_state() {
    for (uint pin = BD0_PIN; pin < BD0_PIN + DATA_SIZE; ++pin) {
        uint32_t status = io_bank0_hw->io[pin].status;
        bool oe = status & IO_BANK0_GPIO0_STATUS_OETOPAD_BITS;
        bool out_level = status & IO_BANK0_GPIO0_STATUS_OUTTOPAD_BITS;
        bool pad_level = status & IO_BANK0_GPIO0_STATUS_INFROMPAD_BITS;
        printf("GPIO%u state: OE=%d OUT=%d PAD=%d ",
               pin,
               oe ? 1 : 0,
               out_level ? 1 : 0,
               pad_level ? 1 : 0);
    
        gpio_function_t func = gpio_get_function(pin);
        bool pull_up = gpio_is_pulled_up(pin);
        bool pull_down = gpio_is_pulled_down(pin);
        const char *pull_desc = (!pull_up && !pull_down) ? "off" :
                                (pull_up && !pull_down) ? "pull-up" :
                                (!pull_up && pull_down) ? "pull-down" :
                                                          "both";
        printf("function=%u, pulls=%s\n",
               func,
               pull_desc);
    }
}

static inline uint8_t sasi_dma_target_device(const dma_registers_t *dma) {
    uint8_t target = dma ? (dma->selected_target & 0x07) : 0;
    return DEVICE_DISK_BASE + target;
}

static inline uint32_t sasi_dma_sector_count(const dma_registers_t *dma) {
    return dma ? dma->block_count.full : 0;
}

static inline uint32_t sasi_dma_bytes_requested(const dma_registers_t *dma) {
    return sasi_dma_sector_count(dma) * SASI_SECTOR_SIZE;
}

static bool sasi_dma_device_to_ram(dma_registers_t *dma) {
    uint8_t device = sasi_dma_target_device(dma);
    uint32_t lba = dma->logical_block.full;
    uint32_t sectors_remaining = sasi_dma_sector_count(dma);
    uint32_t addr = dma->dma_address.full;


    if (sectors_remaining == 0) {
        return false;
    }

    while (sectors_remaining > 0) {
        uint8_t sector[SASI_SECTOR_SIZE];

        if (!fujinet_read_sector(device, lba, sector, SASI_SECTOR_SIZE)) {
            printf("Warning: FujiNet read LBA %lu failed\n", (unsigned long)lba);
            return false;
        }

        dma_write_to_victor_ram(sector, SASI_SECTOR_SIZE, addr);

        addr += SASI_SECTOR_SIZE;
        sectors_remaining--;
        lba++;
    }

    dma->dma_address.full = addr;
    dma->logical_block.full = lba;
    dma->block_count.full = 0;

    return true;
}

static bool sasi_dma_ram_to_device(dma_registers_t *dma) {
    uint8_t device = sasi_dma_target_device(dma);
    uint32_t lba = dma->logical_block.full;
    uint32_t sectors_remaining = sasi_dma_sector_count(dma);
    uint32_t addr = dma->dma_address.full;

    if (sectors_remaining == 0) {
        return false;
    }

    while (sectors_remaining > 0) {
        uint8_t sector[SASI_SECTOR_SIZE];

        dma_read_from_victor_ram(sector, SASI_SECTOR_SIZE, addr);

        if (!fujinet_write_sector(device, lba, sector, SASI_SECTOR_SIZE)) {
            printf("Warning: FujiNet write LBA %lu failed\n", (unsigned long)lba);
            return false;
        }

        addr += SASI_SECTOR_SIZE;
        sectors_remaining--;
        lba++;
    }

    dma->dma_address.full = addr;
    dma->logical_block.full = lba;
    dma->block_count.full = 0;

    return true;
}

// Place the actual registers in time_critical section
static dma_registers_t registers_storage __attribute__((section(".time_critical.dma_registers")));
static dma_registers_t *registers = NULL;

dma_registers_t* dma_get_registers() {
    if (!registers) {
        // Use static storage in time_critical section for fast access
        registers = &registers_storage;
        // Initialize all registers to zero
        memset(registers, 0, sizeof(dma_registers_t));
        printf("DMA registers initialized in time_critical section\n");
    }

    return registers;

}

void debug_pio_state(PIO pio, uint sm) {
    // Check FIFO levels
    printf("pio: %d sm: %d RX FIFO: %d/4 ", pio, sm, pio_sm_get_rx_fifo_level(pio, sm));

    // Check if the state machine is enabled
    printf("SM stalled: %d  ", pio_sm_is_exec_stalled(pio, sm));
    
    // Check program counter
    printf("PC: 0x%x  ", pio_sm_get_pc(pio, sm));

    printf("IRQ 0: %d ", pio_interrupt_get(pio, 0));
    printf("IRQ 1: %d ", pio_interrupt_get(pio, 1));
    printf("IRQ 2: %d   ", pio_interrupt_get(pio, 2));
    printf("IRQ 3: %d   ", pio_interrupt_get(pio, 3));
    
    // Check stall status
    printf("rx empty: %d \n", pio_sm_is_rx_fifo_empty(pio, sm));

}


void print_segment_offset(uint32_t addr) {
    // Assumes addr < 1MB.
    uint16_t seg = addr >> 4;
    uint16_t off = addr & 0xF;
    printf("RD %04X:%04X\n", seg, off);
}

void dma_write_register(dma_registers_t *dma, dma_reg_offsets_t offset, uint8_t value) {
    //printf("dma_write_register offset (0x%x), value: 0x%02x\n", offset, value);
    // Lower address bits are ignored (based on MAME implementation)
    if (offset >= 0x80) {
        offset &= ~0x1f;
    } else {
        offset &= ~0xf;
    }

    switch (offset) {
        case REG_CONTROL: // 0x00 - Control register
        {
            uint8_t prev_control = dma->control;
            bool prev_sel = (prev_control & DMA_SELECT_BIT) != 0;
            dma->control = value;

            // RESET is a pulse - trigger only on rising edge and auto-clear
            if ((value & DMA_RESET_BIT) && !(prev_control & DMA_RESET_BIT)) {
                // Handle reset
                dma_device_reset(dma);
                dma->control &= ~DMA_RESET_BIT;
                value &= ~DMA_RESET_BIT;
            }

            // Handle DMA enable with latch mechanism 
            if (value & DMA_ON_LATCH_BIT) {
                dma->state.dma_enabled = (value & DMA_ON_VALUE_BIT);
            }
            
            // Set write mode (1 = write to memory, 0 = read from memory)
            dma->state.dma_dir_in = (value & DMA_WR_MODE_BIT);
            
            // Handle target selection transitions
            bool now_sel = (value & DMA_SELECT_BIT) != 0;
            if (now_sel && !prev_sel) {
                // Assert SEL
                dma->bus_ctrl |= SASI_SEL_BIT;
                printf("SASI SEL asserted\n");
                // Latch target ID from last data bus value (SASI selection byte is a bit mask)
                dma->selected_target = sasi_extract_target_id(dma->command);
                // Target responds busy after selection (matches log: status 0x04)
                dma->bus_ctrl |= SASI_BSY_BIT;
            } else if (!now_sel && prev_sel) {
                // Deassert SEL and request first command byte (BSY|REQ|CTL)
                dma->bus_ctrl &= ~SASI_SEL_BIT;
                dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
                dma->bus_ctrl &= ~SASI_INP_BIT; // host -> device
                dma->state.non_dma_req = 1;
                dma_update_interrupts(dma, true);
            }
            // Sync cached status after any bus_ctrl changes from SELECT edges
            cached_status_sync_from_bus(dma);
            break;
        }

        case REG_DATA: // 0x10 - Data register
            dma->command = value; // Store last written data value

            // If writing DATA while SELECT is asserted, treat byte as target ID (bit mask format)
            if (dma->control & DMA_SELECT_BIT) {
                dma->selected_target = sasi_extract_target_id(value);
                // Workaround like MAME: ensure SEL isn't asserted during data bus change
                dma->bus_ctrl &= ~SASI_SEL_BIT;
                dma->control &= ~DMA_SELECT_BIT;
                // Initiate command phase request immediately when selection completes via data write
                // Mirrors behavior when control write deasserts SELECT
                dma->bus_ctrl |= (SASI_BSY_BIT | SASI_REQ_BIT | SASI_CTL_BIT);
                dma->bus_ctrl &= ~SASI_INP_BIT; // host -> device
                dma->state.non_dma_req = 1;
                dma_update_interrupts(dma, true);
            }

            bool pending_non_dma = dma->state.non_dma_req;
            bool host_to_device_phase = !(dma->bus_ctrl & SASI_INP_BIT);

            // Ack host->device non-DMA cycles (command/message out) before parsing the byte
            if (pending_non_dma && host_to_device_phase) {
                dma->state.non_dma_req = 0;
                dma->state.asserting_ack = 1;
                dma->bus_ctrl |= SASI_ACK_BIT;
            }

            // During command phase (CTL asserted and REQ expected), route byte to parser
            if (dma->bus_ctrl & SASI_CTL_BIT) {
                handle_sasi_command_byte(dma, value);
            }
            // Sync cached status after any bus_ctrl changes from DATA writes
            cached_status_sync_from_bus(dma);
            break;

        case REG_STATUS: // 0x20 - Status register
            // Read status information
            value = dma->status;
            break;

        case REG_ADDR_L: // 0x80 - DMA address low byte
            dma->dma_address.full = (dma->dma_address.full & ~0xFF) | value;
            break;

        case REG_ADDR_M: // 0xA0 - DMA address middle byte
            dma->dma_address.full = (dma->dma_address.full & ~0xFF00) | (value << 8);
            break;

        case REG_ADDR_H: // 0xC0 - DMA address high byte (4 bits only)
            dma->dma_address.full = (dma->dma_address.full & ~0x0F0000) | ((value & 0x0F) << 16);
            break;

        default:
            printf("Write to unknown register offset (0x%x), value: 0x%02x\n", offset, value);
            break;
    }
}

uint8_t dma_read_register(dma_registers_t *dma, dma_reg_offsets_t offset) {
    uint8_t orig_offset = offset;
    // Lower address bits are ignored (based on MAME implementation)
    if (offset >= 0x80) {
        offset &= ~0x1f;
    } else {
        offset &= ~0xf;
    }

    uint8_t data = 0xff;

    switch (offset) {
        case REG_CONTROL: // 0x00 - Control register (write-only)
            // Write-only register, return 0xFF
            break;

        case REG_DATA: // 0x10 - Data register (non-DMA control/status transfers)
            if (dma->state.non_dma_req) {
                // If in message phase, return 0x00 then release bus
                if (dma->bus_ctrl & SASI_MSG_BIT) {
                    data = 0x00; // Command Complete
                    dma->state.non_dma_req = 0;
                    // Drop REQ then release bus to idle
                    dma->bus_ctrl &= ~SASI_REQ_BIT;
                    dma->bus_ctrl &= ~(SASI_CTL_BIT | SASI_INP_BIT | SASI_MSG_BIT | SASI_ACK_BIT);
                    dma->bus_ctrl &= ~SASI_BSY_BIT;
                    dma_update_interrupts(dma, false);
                }
                // If in status phase, return status then move to message phase
                else if ((dma->bus_ctrl & (SASI_CTL_BIT | SASI_INP_BIT)) == (SASI_CTL_BIT | SASI_INP_BIT)) {
                    data = dma->status; // e.g. 0x00 (GOOD)
                    // Remain asserting BSY and CTL/INP, switch to MSG and REQ for message-in
                    dma->bus_ctrl |= (SASI_MSG_BIT | SASI_REQ_BIT);
                    dma->state.non_dma_req = 1; // next read will fetch message byte
                    dma_update_interrupts(dma, true);
                } else {
                    // Unexpected non-DMA read, just return last data value
                    data = dma->command;
                    dma->state.non_dma_req = 0;
                }
            }
            break;

        case REG_STATUS:
        case 0x30: // Treat 0x30 as status alias
            // For status register reads, construct from bus control state
            data = ((dma->bus_ctrl & SASI_INP_BIT) ? SASI_INP_BIT : 0) |
                   ((dma->bus_ctrl & SASI_CTL_BIT) ? SASI_CTL_BIT : 0) |
                   ((dma->bus_ctrl & SASI_BSY_BIT) ? SASI_BSY_BIT : 0) |
                   ((dma->bus_ctrl & SASI_REQ_BIT) ? SASI_REQ_BIT : 0) |
                   ((dma->bus_ctrl & SASI_MSG_BIT) ? SASI_MSG_BIT : 0);

            fast_log("returning 0x%02x (BSY:%d REQ:%d CTL:%d INP:%d MSG:%d)\n", 
                   data, 
                   !!(dma->bus_ctrl & SASI_BSY_BIT),
                   !!(dma->bus_ctrl & SASI_REQ_BIT), 
                   !!(dma->bus_ctrl & SASI_CTL_BIT),
                   !!(dma->bus_ctrl & SASI_INP_BIT),
                   !!(dma->bus_ctrl & SASI_MSG_BIT));

            // Clear interrupt on status read
            dma_update_interrupts(dma, false);
            break;

        case REG_ADDR_L: // 0x80 - DMA address low byte
            data = dma->dma_address.full & 0xFF;
            break;

        case REG_ADDR_M: // 0xA0 - DMA address middle byte
            data = (dma->dma_address.full >> 8) & 0xFF;
            break;

        case REG_ADDR_H: // 0xC0 - DMA address high byte (4 bits only)
            data = (dma->dma_address.full >> 16) & 0x0F;
            break;

        default:
            fast_log("Read from unknown register offset (0x%x)\n", offset);
            break;
    }
    fast_log("dma_read_register offset (0x%x) returning 0x%02x\n", offset, data);
    return data;
}

void ontime_pin_setup() {
    //setup pin basics like enable pulls and set slew rate
    for (int pin = BD0_PIN; pin <= PHASE_2_PIN; ++pin) {
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_2MA);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_SLOW);
        gpio_pull_down(pin); // enable pull-downs
        gpio_put(pin, 0);  // by default drive low
        gpio_set_dir(pin, GPIO_IN); // set as output to assert HOLD/
        
        printf("ontime gpio_setup %d  ", pin);
    }

    // for the bus control pins, to avoid floating pins during DMA handoff to 8088
    // we need to set appropriate pull-ups or pull-downs
    // we defaulted everything to pull low above, so just need to fix the active low pins
    gpio_pull_up(RD_PIN);   // RD is active low, so pull-up
    gpio_pull_up(WR_PIN);   // WR is active low, so pull-up
    gpio_pull_up(DTR_PIN);  // DTR is active low, so pull-up 
    gpio_pull_up(DEN_PIN);  // DEN is active low, so pull-up
    gpio_pull_up(SSO_PIN);  // SSO is active low, so pull-up
    gpio_pull_up(DLATCH_PIN); // DLATCH is active low, so pull-up
}

static inline void setup_pin_dma_control(uint32_t pin, bool is_output, bool preload_level) {
    gpio_set_function(pin, GPIO_FUNC_PIO0 + pio_get_index(PIO_DMA_MASTER));
    pio_gpio_init(PIO_DMA_MASTER, pin);
    pio_sm_set_pins_with_mask(PIO_DMA_MASTER, DMA_SM_CONTROL, (preload_level ? 1u : 0u) << pin, 1u << pin); // preload level
    pio_sm_set_pindirs_with_mask(PIO_DMA_MASTER, DMA_SM_CONTROL, (is_output ? 1u : 0u) << pin, 1u << pin); // direction
}

static inline void setup_pins_dma_control() {
    
    //setup data pins BD0 to A19 to be owned by PIO as inputs
    for (int pin = BD0_PIN; pin <= A19_PIN; ++pin) {
        uint function = GPIO_FUNC_PIO0 + pio_get_index(PIO_DMA_MASTER);
        gpio_set_function(pin, function);
        pio_gpio_init(PIO_DMA_MASTER, pin);
        pio_sm_set_pins_with_mask(PIO_DMA_MASTER, DMA_SM_CONTROL, 0u, 1u << pin); // preload latch low
        pio_sm_set_pindirs_with_mask(PIO_DMA_MASTER, DMA_SM_CONTROL, 0u, 1u << pin); // input

        printf("dma gpio_init %d  ", pin);
        printf("GPIO%d_CTRL = 0x%08x\n", pin, io_bank0_hw->io[pin].ctrl);
    }

    // Assign control pins to PIO, set direction and preload levels
    uint32_t high = 1;
    uint32_t low = 0;
    setup_pin_dma_control(RD_PIN, GPIO_OUT, high);   // RD/ output, preload high
    setup_pin_dma_control(WR_PIN, GPIO_OUT, high);   // WR/ output, preload high
    setup_pin_dma_control(DTR_PIN, GPIO_OUT, high);  // DTR/ output, preload high

    setup_pin_dma_control(ALE_PIN, GPIO_OUT, low);   // ALE/ output, preload low
    setup_pin_dma_control(DEN_PIN, GPIO_OUT, low);   // DEN/ output, preload low
    
    setup_pin_dma_control(READY_PIN, GPIO_IN, low); // READY/ input, preload low
    setup_pin_dma_control(CLOCK_5_PIN, GPIO_IN, low); // CLOCK_5 input, preload low
    setup_pin_dma_control(CLOCK_15B_PIN, GPIO_IN, low); // CLOCK_15B input, preload low

    gpio_init(SSO_PIN);
    gpio_put(SSO_PIN, low); 
    gpio_set_dir(SSO_PIN, GPIO_OUT);

    gpio_init(IO_M_PIN);
    gpio_put(IO_M_PIN, low); 
    gpio_set_dir(IO_M_PIN, GPIO_OUT);
    
    printf("dma control setup pins RD:%d WR:%d DTR:%d\n", RD_PIN, WR_PIN, DTR_PIN);
}   

static inline void setup_pins_register_inputs() {
    //setup data pins BD0 to A19 to be owned by board register PIO as inputs
    for (int pin = BD0_PIN; pin <= CLOCK_15B_PIN; ++pin) {
        uint function = GPIO_FUNC_PIO0 + pio_get_index(PIO_REGISTERS);
        gpio_set_function(pin, function);

        printf("PIO_REGISTERS gpio_init %d  ", pin);
        printf("GPIO%d_CTRL = 0x%08x\n", pin, io_bank0_hw->io[pin].ctrl);
    }
} 


static inline bool obtain_dma_master() {
    //wait for low then high so we can sync to CLOCK_5 edge to help meta-stability of board
    while (gpio_get(CLOCK_5_PIN)) { tight_loop_contents(); } // wait low
    while (!gpio_get(CLOCK_5_PIN)) { tight_loop_contents(); } // then wait high

    gpio_init(HOLD_PIN);
    gpio_put(HOLD_PIN, 0);  // sink the line, HOLD/ is open drain on Victor
    gpio_set_dir(HOLD_PIN, GPIO_OUT); // set as output to assert HOLD/
    gpio_init(HLDA_PIN);
    gpio_set_dir(HLDA_PIN, GPIO_IN);

    absolute_time_t deadline = make_timeout_time_us(DMA_TIMEOUT_US);
    while (!gpio_get(HLDA_PIN)) {
        if (DMA_TIMEOUT_US && absolute_time_diff_us(get_absolute_time(), deadline) <= 0) {
            return false;
        }
        tight_loop_contents();
    }

    return true;
}

static inline bool start_dma_control() {
    if (!obtain_dma_master()) {
        printf("Failed to obtain DMA master within timeout\n");
        return false;
    }
    //setup all the pins to be controlled by PIO DMA SM
    setup_pins_dma_control();
    pio_sm_clear_fifos(PIO_DMA_MASTER, true);
    pio_sm_set_enabled(PIO_DMA_MASTER, DMA_SM_CONTROL, true);
    return true;
}


static inline void release_dma_master() {
    //turn off DMA PIO and give back control to register PIO
    pio_sm_set_enabled(PIO_DMA_MASTER, DMA_SM_CONTROL, false);
    setup_pins_register_inputs();
    pio_sm_set_enabled(PIO_REGISTERS, REG_SM_CONTROL, true);

    //wait for low then high so we can sync to CLOCK_5 edge to help meta-stability of board
    while (gpio_get(CLOCK_5_PIN)) { tight_loop_contents(); } // wait low
    while (!gpio_get(CLOCK_5_PIN)) { tight_loop_contents(); } // then wait high
    
    gpio_set_dir(HOLD_PIN, GPIO_IN); // release HOLD/ is open drain on Victor
}


#ifndef UNIT_TEST
// Write function using two-word FIFO protocol:
//  Word 1: bits 0=W/R flag (1=write), bits 1-20=address A0-A19
//  Word 2: bits 0-7=data byte, bits 8-19=address A8-A19 (MSB)
void dma_write_to_victor_ram(uint8_t *data, size_t length, uint32_t start_address) {
    printf("Starting DMA write to Victor RAM\n");
    printf("Length: %zu\n", length);
    print_segment_offset(start_address);

    printf("write_pio: %p, write_sm: %d\n", PIO_DMA_MASTER, DMA_SM_CONTROL);
    debug_pio_state(PIO_DMA_MASTER, DMA_SM_CONTROL);

    uint32_t full_batch_count = length / DMA_BATCH_SIZE;
    uint32_t remainder_bytes = length % DMA_BATCH_SIZE;

    if (remainder_bytes > 0) {
        full_batch_count += 1;
    }

    for (uint32_t batch = 0; batch < full_batch_count; batch++) {
        if (!start_dma_control()) {
            printf("Failed to obtain DMA master\n");
            return;
        }
        printf("Starting DMA transfer batch %d\n", batch);

        for (size_t i = 0; i < 128; i++) {
            uint32_t addr = (start_address + (batch * 128) + i) & 0xFFFFF;  // 20-bit address
            uint8_t byte = data[(batch * 128) + i];                         // Data byte to write

            // printf("Writing %02X to Victor RAM at address %08X ", byte, addr);
            // print_segment_offset(addr);

            // Send two FIFO payloads per memory address =  the PIO protocol
            uint32_t fifo_t1 = ((addr & 0xFFFFFu) << 1) | 1;                // T1: [20 bit address][1-bit write=1 flag in LSB] 
            pio_sm_put_blocking(PIO_DMA_MASTER, DMA_SM_CONTROL, fifo_t1);

            uint32_t fifo_t2 = (addr & 0xFFF00u) | byte;                    // T2: [12 bits address A19-A8][8 bits data byte]
            pio_sm_put_blocking(PIO_DMA_MASTER, DMA_SM_CONTROL, fifo_t2);

            if ((batch * 128) + i + 1 >= length) {
                break; // Exit if we've written all requested bytes
            }
        }

        // Wait for TX FIFO to empty before releasing DMA master
        while (pio_sm_get_tx_fifo_level(PIO_DMA_MASTER, DMA_SM_CONTROL) > 0) {
            tight_loop_contents();
        }
        sleep_us(3); // small delay to ensure last data is processed

        release_dma_master();
        
        // give 8088 time to work background interrupts like serial port or other I/O before resuming DMA
        printf("DMA master released after batch %d\n", batch);
        sleep_us(DMA_SHARE_WAIT_US);
    }
    printf("Finished DMA write to Victor RAM\n");
   
    return;
}

// Read function (PIO-based) using two-word FIFO protocol:
//  Word 1: bits 0=W/R flag (0=read), bits 1-20=address A0-A19
//  Word 2: pindirs control value 0xFFF00 (A8-A19 outputs, BD0-BD7 inputs)
void dma_read_from_victor_ram(uint8_t *data, size_t length, uint32_t start_address) {

    // Debug logging disabled for performance during DMA
    printf("Starting DMA read from Victor RAM\n");
    printf("Length: %zu, start_address: %d ", length, start_address);
    print_segment_offset(start_address);

    uint8_t *temp = malloc((length + 1) * sizeof(uint8_t));
    if (!temp) {
        printf("Failed to allocate memory for DMA transfer\n");
        return;
    }

    printf("dma_read_pio: %p, read_sm: %d\n", PIO_DMA_MASTER, DMA_SM_CONTROL);
    debug_pio_state(PIO_DMA_MASTER, DMA_SM_CONTROL);
    printf("Reading from Victor RAM at address %08X\n", start_address);

    uint32_t full_batch_count = length / DMA_BATCH_SIZE;
    uint32_t remainder_bytes = length % DMA_BATCH_SIZE;

    if (remainder_bytes > 0) {
        full_batch_count += 1;
    }

     for (uint32_t batch = 0; batch < full_batch_count; batch++) {
        if (!start_dma_control()) {
            printf("Failed to obtain DMA master\n");
            return;
        }
        printf("Starting DMA transfer batch %d\n", batch);

        for (size_t i = 0; i < 128; i++) {
            uint32_t addr = (start_address + (batch * 128) + i) & 0xFFFFF;  // 20-bit address


            // Send two FIFO words per the PIO protocol
            uint32_t fifo_t1 = ((addr & 0xFFFFFu) << 1) | 0; // T1: [20 bit address][1-bit read=0 flag in LSB]
            pio_sm_put_blocking(PIO_DMA_MASTER, DMA_SM_CONTROL, fifo_t1);

            uint32_t fifo_t2 = 0xFFF00; // pindirs value only, no data or address T2: [A8-A19 remain outputs][BD0-BD7 inputs]
            pio_sm_put_blocking(PIO_DMA_MASTER, DMA_SM_CONTROL, fifo_t2);

            // Get the data byte that was read
            uint32_t char_data = pio_sm_get_blocking(PIO_DMA_MASTER, DMA_SM_CONTROL);
            temp[i] = char_data & 0xFF;
            if ((batch * 128) + i + 1 >= length) {
                break; // Exit if we've read all requested bytes
            }

        }

        release_dma_master();
        
        // give 8088 time to work background interrupts like serial port or other I/O before resuming DMA
        printf("DMA master released after batch %d\n", batch);
        sleep_us(DMA_SHARE_WAIT_US);
    }

    memcpy(data, temp, length);
    free(temp);
    printf("\n\nFinished DMA read from Victor RAM\n");
}
#else
// Unit-test in-memory Victor RAM model (64 KiB to fit SRAM)
static uint8_t test_victor_ram[1 << 16];
static const size_t TEST_VICTOR_RAM_SIZE = (1 << 16);

void dma_write_to_victor_ram(uint8_t *data, size_t length, uint32_t start_address) {
    for (size_t i = 0; i < length; i++) {
        uint32_t addr = (start_address + i) & 0xFFFFF;
        if (addr < TEST_VICTOR_RAM_SIZE) test_victor_ram[addr] = data[i];
    }
}

void dma_read_from_victor_ram(uint8_t *data, size_t length, uint32_t start_address) {
    for (size_t i = 0; i < length; i++) {
        uint32_t addr = (start_address + i) & 0xFFFFF;
        data[i] = (addr < TEST_VICTOR_RAM_SIZE) ? test_victor_ram[addr] : 0x00;
    }
}

uint8_t* test_get_victor_ram() { return test_victor_ram; }
size_t test_get_victor_ram_size() { return TEST_VICTOR_RAM_SIZE; }
#endif

// Device reset function (based on MAME implementation)
void dma_device_reset(dma_registers_t *dma) {
    dma->state.dma_enabled = 0;
    dma->state.dma_dir_in = 0;
    dma->dma_address.full = 0;
    dma->state.asserting_ack = 0;
    dma->state.non_dma_req = 0;
    dma->state.status_pending = 0;
    dma->command = 0;
    dma->bus_ctrl = 0;
    dma->control = 0;
    dma->selected_target = 0;
    dma->block_count.full = 0;
    dma->logical_block.full = 0;

    // Clear interrupt
    dma_update_interrupts(dma, false);

    // Reset SASI command accumulator state
    sasi_reset_command_state();

    printf("DMA device reset\n");
}

// Update interrupt state
void dma_update_interrupts(dma_registers_t *dma, bool irq_state) {
    if (irq_state != dma->state.interrupt_pending) {
        dma->state.interrupt_pending = irq_state;
        // In real implementation, this would trigger actual interrupt to CPU
        printf("DMA interrupt %s\n", irq_state ? "asserted" : "cleared");
    }
}

// Handle SASI REQ signal and DMA transfers (based on MAME ctrl_change_handler)
void dma_handle_sasi_req(dma_registers_t *dma) {
    if (dma->bus_ctrl & SASI_REQ_BIT) {
        // Only data (not control or status) transfers can use DMA
        if (dma->state.dma_enabled && !(dma->bus_ctrl & SASI_CTL_BIT)) {
            bool ok = false;
            if (dma->state.dma_dir_in) {
                // Device → Victor RAM
                ok = sasi_dma_device_to_ram(dma);
                printf("DMA write to RAM at 0x%06X\n", dma->dma_address.full);
            } else {
                // Victor RAM → Device
                ok = sasi_dma_ram_to_device(dma);
                printf("DMA read from RAM at 0x%06X\n", dma->dma_address.full);
            }

            if (!ok) {
                printf("Warning: SASI DMA transfer failed\n");
            }
            
            // Auto-increment address after each transfer (key missing piece!)
            dma->state.asserting_ack = 1;
            dma->bus_ctrl |= SASI_ACK_BIT;
        } else {
            // ACK will be generated upon direct data register access
            dma->state.non_dma_req = 1;
        }
    } else if (dma->state.asserting_ack) {
        dma->bus_ctrl &= ~SASI_ACK_BIT;
        dma->state.asserting_ack = 0;
    }
    
    // Command or status request from controller generates interrupt
    if ((dma->bus_ctrl & (SASI_REQ_BIT|SASI_CTL_BIT)) == (SASI_REQ_BIT|SASI_CTL_BIT)) {
        dma_update_interrupts(dma, true);
    }
}
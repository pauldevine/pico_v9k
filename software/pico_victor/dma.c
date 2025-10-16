#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/structs/systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "dma.h"
#include "sasi.h"
#include "logging.h"
#include "pico_fujinet/spi.h"
#include "dma_ultra_fast.h"

#define SASI_SECTOR_SIZE 512

// Forward declarations for GPIO mux switching helpers
static inline void switch_gpio_to_registers_pio(void);
static inline void switch_gpio_to_dma_pio(void);
static inline void set_bus_pins_function(uint func);

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
    bool success = false;

    PIO dma_pio = dma_get_unified_pio();
    int dma_sm = dma_get_unified_sm();

    // Disable register SM and switch GPIO mux to DMA PIO
    pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, false);
    switch_gpio_to_dma_pio();
    if (dma_sm >= 0) {
        pio_sm_set_enabled(dma_pio, dma_sm, true);
    }

    if (sectors_remaining == 0) {
        fast_log("SASI DMA dev->RAM requested with zero sectors\n");
        goto cleanup;
    }

    fast_log("SASI DMA dev->RAM start: LBA=%lu count=%lu addr=0x%06lx\n",
             (unsigned long)lba,
             (unsigned long)sectors_remaining,
             (unsigned long)addr);

    while (sectors_remaining > 0) {
        uint8_t sector[SASI_SECTOR_SIZE];

        if (!fujinet_read_sector(device, lba, sector, SASI_SECTOR_SIZE)) {
            fast_log("FujiNet read LBA %lu failed\n", (unsigned long)lba);
            break;
        }

        dma_write_to_victor_ram(dma_pio, dma_sm, sector, SASI_SECTOR_SIZE, addr);

        addr += SASI_SECTOR_SIZE;
        sectors_remaining--;
        lba++;
    }

    success = (sectors_remaining == 0);
    dma->dma_address.full = addr;
    dma->logical_block.full = lba;
    dma->block_count.full = sectors_remaining;

    if (success) {
        fast_log("SASI DMA dev->RAM complete\n");
    } else if (sectors_remaining > 0) {
        fast_log("SASI DMA dev->RAM incomplete, remaining=%lu\n", (unsigned long)sectors_remaining);
    }

cleanup:
    if (dma_sm >= 0) {
        pio_sm_set_enabled(dma_pio, dma_sm, false);
    }
    switch_gpio_to_registers_pio();
    pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, true);
    return success;
}

static bool sasi_dma_ram_to_device(dma_registers_t *dma) {
    uint8_t device = sasi_dma_target_device(dma);
    uint32_t lba = dma->logical_block.full;
    uint32_t sectors_remaining = sasi_dma_sector_count(dma);
    uint32_t addr = dma->dma_address.full;
    bool success = false;

    PIO dma_pio = dma_get_unified_pio();
    int dma_sm = dma_get_unified_sm();

    // Disable register SM and switch GPIO mux to DMA PIO
    pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, false);
    switch_gpio_to_dma_pio();
    if (dma_sm >= 0) {
        pio_sm_set_enabled(dma_pio, dma_sm, true);
    }

    if (sectors_remaining == 0) {
        fast_log("SASI DMA RAM->dev requested with zero sectors\n");
        goto cleanup;
    }

    fast_log("SASI DMA RAM->dev start: LBA=%lu count=%lu addr=0x%06lx\n",
             (unsigned long)lba,
             (unsigned long)sectors_remaining,
             (unsigned long)addr);

    while (sectors_remaining > 0) {
        uint8_t sector[SASI_SECTOR_SIZE];

        dma_read_from_victor_ram(dma_pio, dma_sm, sector, SASI_SECTOR_SIZE, addr);

        if (!fujinet_write_sector(device, lba, sector, SASI_SECTOR_SIZE)) {
            fast_log("FujiNet write LBA %lu failed\n", (unsigned long)lba);
            break;
        }

        addr += SASI_SECTOR_SIZE;
        sectors_remaining--;
        lba++;
    }

    success = (sectors_remaining == 0);
    dma->dma_address.full = addr;
    dma->logical_block.full = lba;
    dma->block_count.full = sectors_remaining;

    if (success) {
        fast_log("SASI DMA RAM->dev complete\n");
    } else if (sectors_remaining > 0) {
        fast_log("SASI DMA RAM->dev incomplete, remaining=%lu\n", (unsigned long)sectors_remaining);
    }

cleanup:
    if (dma_sm >= 0) {
        pio_sm_set_enabled(dma_pio, dma_sm, false);
    }
    switch_gpio_to_registers_pio();
    pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, true);
    return success;
}

// Place the actual registers in time_critical section
static dma_registers_t registers_storage __attribute__((section(".time_critical.dma_registers")));
static dma_registers_t *registers = NULL;

// Unified DMA SM storage and accessors
static PIO unified_dma_pio = NULL;
static int unified_dma_sm = -1;

void dma_set_unified_sm(PIO pio, int sm) {
    unified_dma_pio = pio;
    unified_dma_sm = sm;
}

int dma_get_unified_sm() {
    return unified_dma_sm;
}

PIO dma_get_unified_pio() {
    return unified_dma_pio ? unified_dma_pio : PIO_DMA;
}

// --- GPIO function switching between PIO instances ---
static inline void set_bus_pins_function(uint func) {
    // Map outputs and side-set pins
    for (int pin = BD0_PIN; pin <= IO_M_PIN; ++pin) {
        gpio_set_function(pin, func);
    }
    // Map input pins used by wait/in
    for (int pin = READY_PIN; pin <= IR_4_PIN; ++pin) {
        gpio_set_function(pin, func);
        gpio_set_input_enabled(pin, true);
    }
}

static inline void switch_gpio_to_registers_pio(void) {
    uint func = GPIO_FUNC_PIO0 + pio_get_index(PIO_REGISTERS);
    set_bus_pins_function(func);
}

static inline void switch_gpio_to_dma_pio(void) {
    uint func = GPIO_FUNC_PIO0 + pio_get_index(dma_get_unified_pio());
    set_bus_pins_function(func);
}

void core1_main() {
    //run IRQ processing on separate core
    //configure the IRQs used to indicate a Register has been accessed on the 8088 bus
    // Set up IRQ when RX FIFO has >= 1 item

    PIO register_pio = PIO_REGISTERS;
    int register_sm = REGISTERS_SM;

    printf("Setting up IRQ for PIO%d SM%d\n", pio_get_index(register_pio), register_sm);
    printf("FIFO source: %d\n", fifo_sources[register_sm]);

    // Pre-initialize and warm up DMA registers
    registers_irq_handler_ultra_init();  // Initialize ultra handler's static data
    dma_registers_t *dma = dma_get_registers();
    if (dma) {
        // Touch all the critical memory locations to bring them into cache
        volatile uint8_t dummy = 0;

        // Touch the DMA address registers (most commonly accessed)
        dma->dma_address.full = 0;
        dummy = dma->dma_address.bytes.low;
        dummy = dma->dma_address.bytes.mid;
        dummy = dma->dma_address.bytes.high;

        // Touch control and status registers
        dma->control = 0;
        dma->status = 0;
        dummy = dma->control;
        dummy = dma->status;

        // Pre-warm the IRQ handler by calling it with dummy data
        // This loads the code into I-cache and warms up branch predictors
        printf("Pre-warming IRQ handler cache...\n");

        // Temporarily disable IRQ while we warm up
        irq_set_enabled(PIO1_IRQ_0, false);

        // Set handler
        irq_set_exclusive_handler(PIO1_IRQ_0, registers_irq_handler_ultra_asm);

        // Call handler multiple times with different operations to fully warm caches
        // This ensures all code paths and data are cached
        for (int warm_iter = 0; warm_iter < 10; warm_iter++) {
            // Test different register types to warm all code paths
            uint32_t test_addresses[] = {
                0x000EF300,  // Control register write
                0x100EF310,  // Data register read
                0x100EF320,  // Status register read
                0x000EF380,  // DMA address low write
                0x100EF380,  // DMA address low read
                0x000EF3A0,  // DMA address mid write
                0x000EF3C0,  // DMA address high write
            };

            for (int i = 0; i < 7; i++) {
                if (pio_sm_is_tx_fifo_empty(register_pio, register_sm)) {
                    // Put test data
                    pio_sm_put(register_pio, register_sm, test_addresses[i]);

                    // Call the handler
                    registers_irq_handler_ultra_asm();

                    // Clear any response data
                    if (!pio_sm_is_rx_fifo_empty(register_pio, register_sm)) {
                        pio_sm_get(register_pio, register_sm);
                    }
                }
            }
        }

        printf("Cache pre-warming complete (70 handler calls)\n");

        // Small delay to let caches settle
        busy_wait_us(100);

        // One more round of warming after the delay
        for (int i = 0; i < 5; i++) {
            if (pio_sm_is_tx_fifo_empty(register_pio, register_sm)) {
                pio_sm_put(register_pio, register_sm, 0x000EF380);
                registers_irq_handler_ultra_asm();
                if (!pio_sm_is_rx_fifo_empty(register_pio, register_sm)) {
                    pio_sm_get(register_pio, register_sm);
                }
            }
        }

        (void)dummy; // Prevent unused variable warning
    }

    pio_set_irq0_source_enabled(register_pio, fifo_sources[register_sm], true);
    irq_set_enabled(PIO1_IRQ_0, true);
    printf("Core1 started, PIO: %d SM: %d\n", register_pio, register_sm);
    
    // Add a test to see if IRQ is working
    printf("Testing IRQ setup... FIFO level: %d\n", pio_sm_get_rx_fifo_level(register_pio, register_sm));

    systick_hw->csr = 0x5; // Enable, use processor clock, no interrupt
    systick_hw->rvr = 0x00FFFFFF; // Max reload value (24-bit)


    while (true) {
        tight_loop_contents();
    }
}



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
    printf("pio: %d sm: %d RX FIFO: %d/8 ", pio, sm, pio_sm_get_rx_fifo_level(pio, sm));

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
            bool prev_sel = (dma->control & DMA_SELECT_BIT) != 0;
            dma->control = value;

            if (value & DMA_RESET_BIT) {
                // Handle reset
                dma_device_reset(dma);
                break;
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
                // Latch target ID from last data bus value
                dma->selected_target = (dma->command & 0x07);
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
            break;
        }

        case REG_DATA: // 0x10 - Data register
            dma->command = value; // Store last written data value

            // If writing DATA while SELECT is asserted, treat byte as target ID
            if (dma->control & DMA_SELECT_BIT) {
                dma->selected_target = (value & 0x07);
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

 void __time_critical_func(registers_irq_handler)() {
    gpio_put(DEBUG_PIN, 1);
    // Handle IRQ for register PIO
    static int irq_count = 0;
    irq_count++;
    //printf("Register IRQ triggered\n");
    PIO pio = PIO_REGISTERS; 
    uint sm = REGISTERS_SM; 

    //printf("getting address\n");
    uint32_t raw_value = pio_sm_get(pio, sm); //retrieve the address from the FIFO
    
    uint32_t start_cycles = systick_hw->cvr;
    uint32_t end_cycles;

    uint32_t address = raw_value & 0xFFFFF; 
    dma_reg_offsets_t offset = address - DMA_REGISTER_BASE;
    
    uint8_t data = (raw_value >> 20) & 0xFF;
    bool read_flag = (bool)((raw_value >> 28) & 0xF);

    if (address >= 0xEF300) {
        fast_log("[IRQ#%d] Address: %05X Offset: %02X data: %02X read_flag: %d\n", irq_count, address, offset, data, read_flag);
    } else {
        fast_log("[IRQ#%d] Unknown address: %05X Offset: %02X data: %02X read_flag: %d\n", irq_count, address, offset, data, read_flag);
    }
    
    // Additional debug for status register attempts
    if (offset == 0x20 || offset == 0x30) {
        fast_log("STATUS REGISTER ACCESS: offset=0x%02x read=%d\n", offset, read_flag);
    }
    
    dma_registers_t *my_register = dma_get_registers();
    if (!my_register) {
        fast_log("Failed to get DMA registers\n");
        return;
    }
    
    if (read_flag) {
        data = dma_read_register(my_register, offset);
        fast_log("[IRQ#%d] dma_read_register returned 0x%02x for offset 0x%02x\n", irq_count, data, offset);
        uint32_t pindirs_and_data = (0xFF << 8) | (data & 0xFF); // Set all pins to output / combine with data payload
        
        // Check TX FIFO status before blocking
        uint tx_level = pio_sm_get_tx_fifo_level(pio, sm);
        fast_log("[IRQ#%d] TX FIFO level before put: %d\n", irq_count, tx_level);
        
        #ifndef BENCHMARK_MODE
        pio_sm_put_blocking(pio, sm, pindirs_and_data); //send the data back to the 8088
        #endif
        fast_log("[IRQ#%d] Read complete - sent 0x%02x to 8088\n", irq_count, data);
        
        // Force a small delay to ensure data is stable
        busy_wait_us(1);
    } else {
        dma_write_register(my_register, offset, data);
        fast_log("Write address: %08X offset: %02X data: %02X \n", address, offset, data);
    }
  
    end_cycles = systick_hw->cvr;
    
    uint32_t cycles_used;
    if (start_cycles >= end_cycles) {
        cycles_used = start_cycles - end_cycles;
    } else {
        // Handle wraparound
        cycles_used = start_cycles + (0x00FFFFFF - end_cycles);
    }
    //fast_log("cycles: %lu (%.1f ns)\n", cycles_used, cycles_used * 5.0);
    gpio_put(DEBUG_PIN, 0);

}


#ifndef UNIT_TEST
// Write function using two-word FIFO protocol:
//  Word 1: bits 0=W/R flag (1=write), bits 1-20=address A0-A19
//  Word 2: bits 0-7=data byte, bits 8-19=address A8-A19 (MSB)
void dma_write_to_victor_ram(PIO write_pio, int write_sm, const uint8_t *data, size_t length, uint32_t start_address) {
    if (!length || !data || write_sm < 0) {
        return;
    }

    fast_log("DMA write->Victor addr=0x%06lx len=%u\n",
             (unsigned long)(start_address & 0xFFFFF),
             (unsigned)length);

    for (size_t i = 0; i < length; i++) {
        uint32_t addr = (start_address + i) & 0xFFFFF;  // 20-bit address
        uint32_t payload = (addr & 0xFFF00) | data[i];

        // Word 1: address with write flag
        pio_sm_put_blocking(write_pio, write_sm, (addr << 1) | 1);
        // Word 2: MSBs of address + data byte
        pio_sm_put_blocking(write_pio, write_sm, payload);
    }
}

// Read function (PIO-based) using two-word FIFO protocol:
//  Word 1: bits 0=W/R flag (0=read), bits 1-20=address A0-A19
//  Word 2: pindirs control value 0xFFF00 (A8-A19 outputs, BD0-BD7 inputs)
void dma_read_from_victor_ram(PIO read_pio, int read_sm, uint8_t *data, size_t length, uint32_t start_address) {
    if (!length || !data || read_sm < 0) {
        return;
    }

    fast_log("DMA read<-Victor addr=0x%06lx len=%u\n",
             (unsigned long)(start_address & 0xFFFFF),
             (unsigned)length);

    for (size_t i = 0; i < length; i++) {
        uint32_t addr = (start_address + i) & 0xFFFFF;  // 20-bit address

        // Send two FIFO words per the PIO protocol
        // Word 1: address shifted left with read flag (0) in LSB
        pio_sm_put_blocking(read_pio, read_sm, (addr << 1) | 0);
        // Word 2: pindirs value to set BD0-BD7 as inputs, A8-A19 as outputs
        pio_sm_put_blocking(read_pio, read_sm, 0xFFF00);

        // Get the data byte that was read
        uint32_t char_data = pio_sm_get_blocking(read_pio, read_sm);
        data[i] = (uint8_t)(char_data & 0xFF);
    }
}
#else
// Unit-test in-memory Victor RAM model (64 KiB to fit SRAM)
static uint8_t test_victor_ram[1 << 16];
static const size_t TEST_VICTOR_RAM_SIZE = (1 << 16);

void dma_write_to_victor_ram(PIO write_pio, int write_sm, const uint8_t *data, size_t length, uint32_t start_address) {
    (void)write_pio; (void)write_sm;
    for (size_t i = 0; i < length; i++) {
        uint32_t addr = (start_address + i) & 0xFFFFF;
        if (addr < TEST_VICTOR_RAM_SIZE) test_victor_ram[addr] = data[i];
    }
}

void dma_read_from_victor_ram(PIO read_pio, int read_sm, uint8_t *data, size_t length, uint32_t start_address) {
    (void)read_pio; (void)read_sm;
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
    dma->command = 0;
    dma->bus_ctrl = 0;
    dma->control = 0;
    dma->selected_target = 0;
    dma->block_count.full = 0;
    dma->logical_block.full = 0;

    // Clear interrupt
    dma_update_interrupts(dma, false);

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

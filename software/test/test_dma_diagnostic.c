/*
 * Diagnostic test program to identify DMA hardware issues
 * This program performs step-by-step testing with extensive logging
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "dma_read_write.pio.h"
#include "pico_victor/dma.h"
#include "hardware/structs/pio.h"
#include "hardware/structs/ioqspi.h"

typedef enum {
    DMA_STAGE_UNKNOWN = -1,
    DMA_STAGE_T0_ADDR = 1,
    DMA_STAGE_WAIT_HLDA_ASSERT = 4,
    DMA_STAGE_T1_CLK5_HIGH = 7,
    DMA_STAGE_T1_CLK5_LOW = 8,
    DMA_STAGE_BRANCH_WRITE = 9,
    DMA_STAGE_T2_CLK5_HIGH = 13,
    DMA_STAGE_T2_CLK5_LOW = 14,
    DMA_STAGE_T2_WAIT_READY = 15,
    DMA_STAGE_T3_CLK5_HIGH = 16,
    DMA_STAGE_T3_CLK5_LOW_RELEASE = 17,
    DMA_STAGE_T4_CLK5_HIGH = 20,
    DMA_STAGE_T4_CLK15B_EDGE = 22,
    DMA_STAGE_WAIT_HLDA_DEASSERT = 24,
    DMA_STAGE_WRITE_LOAD_DATA = 27,
    DMA_STAGE_WRITE_DATA_OUT = 28,
    DMA_STAGE_WRITE_STROBES = 29
} dma_stage_index_t;

typedef struct {
    uint32_t pc;
    uint8_t instr_index;
    bool stalled;
} dma_sm_snapshot_t;

static inline pio_hw_t *pio_hw_from_inst(PIO pio) {
    switch (pio_get_index(pio)) {
        case 0: return pio0_hw;
#if NUM_PIOS > 1
        case 1: return pio1_hw;
#endif
#if NUM_PIOS > 2
        case 2: return pio2_hw;
#endif
        default: return NULL;
    }
}

static dma_sm_snapshot_t dma_capture_sm_snapshot(PIO pio, uint sm, uint offset) {
    dma_sm_snapshot_t snap = {
        .pc = pio_sm_get_pc(pio, sm),
        .stalled = pio_sm_is_exec_stalled(pio, sm)
    };

    if (snap.pc >= offset) {
        snap.instr_index = (uint8_t)(snap.pc - offset);
    } else {
        snap.instr_index = (uint8_t)snap.pc;
    }

    return snap;
}

static const char *dma_stage_label(bool is_write, uint8_t instr_index) {
    switch (instr_index) {
        case 0: return "Init: OUT X (mode)";
        case DMA_STAGE_T0_ADDR: return "T0: address on bus";
        case 2: return "T0: preset control lines";
        case 3: return "T0: preload data byte";
        case DMA_STAGE_WAIT_HLDA_ASSERT: return "Wait HLDA↑ (bus grant)";
        case 5: return "Assert HOLD, drive address/data";
        case 6: return "Drive control pindirs";
        case DMA_STAGE_T1_CLK5_HIGH: return "T1: wait CLK5↑";
        case DMA_STAGE_T1_CLK5_LOW: return "T1: wait CLK5↓";
        case DMA_STAGE_BRANCH_WRITE: return is_write ? "Branch to write path" : "Stay on read path";
        case 10: return "T2(read): drive RD/";
        case 11: return "T2(read): move byte to OSR";
        case 12: return "T2(read): set BD as inputs";
        case DMA_STAGE_T2_CLK5_HIGH: return "T2: wait CLK5↑";
        case DMA_STAGE_T2_CLK5_LOW: return "T2: wait CLK5↓";
        case DMA_STAGE_T2_WAIT_READY: return "T2: wait READY↑";
        case DMA_STAGE_T3_CLK5_HIGH: return "T3: wait CLK5↑";
        case DMA_STAGE_T3_CLK5_LOW_RELEASE: return "T3: wait CLK5↓ (release HOLD)";
        case 18: return is_write ? "Skip read sampling" : "Read path continues";
        case 19: return "T3(read): sample data bus";
        case DMA_STAGE_T4_CLK5_HIGH: return "T4: wait CLK5↑";
        case 21: return "T4: restore control lines";
        case DMA_STAGE_T4_CLK15B_EDGE: return "T4: wait CLK15B edge";
        case 23: return "T4: end of cycle (DEN high)";
        case DMA_STAGE_WAIT_HLDA_DEASSERT: return "Wait HLDA↓ (bus release)";
        case 25: return "Release control pindirs";
        case 26: return "Release address bus";
        case DMA_STAGE_WRITE_LOAD_DATA: return "T2(write): load data byte";
        case DMA_STAGE_WRITE_DATA_OUT: return "T2(write): present data";
        case DMA_STAGE_WRITE_STROBES: return "T2(write): assert WR/";
        case 30: return "Loop to T2";
        default: return "Unknown/idle";
    }
}

static void dma_log_wait_context(const dma_sm_snapshot_t *snap, bool is_write, pio_hw_t *hw) {
    switch (snap->instr_index) {
        case DMA_STAGE_WAIT_HLDA_ASSERT:
            printf("    HOLD(dir)=%d HOLD(val)=%d HLDA=%d\n",
                   (hw->dbg_padoe >> HOLD_PIN) & 1,
                   (hw->dbg_padout >> HOLD_PIN) & 1,
                   gpio_get(HLDA_PIN));
            break;
        case DMA_STAGE_T1_CLK5_HIGH:
        case DMA_STAGE_T1_CLK5_LOW:
        case DMA_STAGE_T2_CLK5_HIGH:
        case DMA_STAGE_T2_CLK5_LOW:
        case DMA_STAGE_T3_CLK5_HIGH:
        case DMA_STAGE_T3_CLK5_LOW_RELEASE:
        case DMA_STAGE_T4_CLK5_HIGH:
            printf("    CLK5=%d\n", gpio_get(CLOCK_5_PIN));
            if (snap->instr_index == DMA_STAGE_T3_CLK5_LOW_RELEASE) {
                printf("    HOLD(dir)=%d HLDA=%d\n",
                       (hw->dbg_padoe >> HOLD_PIN) & 1,
                       gpio_get(HLDA_PIN));
            }
            break;
        case DMA_STAGE_T2_WAIT_READY:
            printf("    READY=%d (expect 1)\n", gpio_get(READY_PIN));
            break;
        case DMA_STAGE_T4_CLK15B_EDGE:
            printf("    CLK15B=%d\n", gpio_get(CLOCK_15B_PIN));
            break;
        case DMA_STAGE_WAIT_HLDA_DEASSERT:
            printf("    HLDA=%d HOLD(dir)=%d HOLD(val)=%d\n",
                   gpio_get(HLDA_PIN),
                   (hw->dbg_padoe >> HOLD_PIN) & 1,
                   (hw->dbg_padout >> HOLD_PIN) & 1);
            break;
        default:
            (void)is_write;
            break;
    }
}

static void dma_log_sm_state(const char *tag, PIO pio, uint sm, uint offset, bool is_write) {
    dma_sm_snapshot_t snap = dma_capture_sm_snapshot(pio, sm, offset);
    pio_hw_t *hw = pio_hw_from_inst(pio);
    uint fifo_level = pio_sm_get_tx_fifo_level(pio, sm);

    printf("%s: SM%d PC=0x%02x (idx %u: %s) stalled=%d TX=%u RX_empty=%d\n",
           tag, sm, snap.pc, snap.instr_index,
           dma_stage_label(is_write, snap.instr_index),
           snap.stalled, fifo_level,
           pio_sm_is_rx_fifo_empty(pio, sm));

    if (hw && snap.stalled) {
        dma_log_wait_context(&snap, is_write, hw);
    }
}

static void dma_monitor_progress(const char *prefix,
                                 PIO pio, uint sm, uint offset,
                                 bool is_write) {
    static const uint32_t checkpoints_us[] = {2, 10, 50, 200, 1000};

    for (size_t i = 0; i < sizeof(checkpoints_us) / sizeof(checkpoints_us[0]); i++) {
        busy_wait_us(checkpoints_us[i]);
        char tag[64];
        snprintf(tag, sizeof(tag), "%s +%u us", prefix, checkpoints_us[i]);
        dma_log_sm_state(tag, pio, sm, offset, is_write);
    }
}

#define TEST_SIZE 2  // Start with just 2 bytes for debugging
#define TEST_ADDRESS 0x10000  // Test address in Victor RAM (segment 0x1000:0x0000)
#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 46
#define UART_RX_PIN 45

// Simple core1 stub
void core1_test_stub() {
    while (1) {
        sleep_ms(100);
    }
}


void dump_pio_versions(void) {
    printf("PIO0 VERSION = 0x%x\n", (pio0_hw->dbg_cfginfo >> 28) & 0xF);
    printf("PIO1 VERSION = 0x%x\n", (pio1_hw->dbg_cfginfo >> 28) & 0xF);
#if NUM_PIOS > 2
    printf("PIO2 VERSION = 0x%x\n", (pio2_hw->dbg_cfginfo >> 28) & 0xF);
#endif
}

static void dump_pio_dbg(void) {
    printf("PIO0_DBG_PADOE = 0x%08x\n", pio0_hw->dbg_padoe);
    printf("PIO0_DBG_PADOUT = 0x%08x\n", pio0_hw->dbg_padout);
}

void initialize_uart() {
    gpio_init(UART_TX_PIN);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);

    uart_init(UART_ID, BAUD_RATE);
    uart_set_fifo_enabled(UART_ID, false);
    stdio_uart_init_full(UART_ID, BAUD_RATE, UART_TX_PIN, -1);
}

void print_gpio_states() {
    printf("\n=== GPIO Pin States ===\n");

    // NOTE: We do NOT initialize pins here anymore as it interferes with PIO
    // We just read the current state of pins

    // Address & Data pins
    printf("Address pins (A0-A19): ");
    for (int i = 0; i < 20; i++) {
        printf("%d:%d ", i, gpio_get(i));
    }
    printf("\n");

    // Control pins
    printf("Control pins:\n");
    printf("  RD (pin %d) = %d\n", RD_PIN, gpio_get(RD_PIN));
    printf("  WR (pin %d) = %d\n", WR_PIN, gpio_get(WR_PIN));
    printf("  HOLD (pin %d) = %d\n", HOLD_PIN, gpio_get(HOLD_PIN));
    printf("  ALE (pin %d) = %d\n", ALE_PIN, gpio_get(ALE_PIN));
    printf("  HLDA (pin %d) = %d\n", HLDA_PIN, gpio_get(HLDA_PIN));
    printf("  READY (pin %d) = %d\n", READY_PIN, gpio_get(READY_PIN));
    printf("  CLOCK_5 (pin %d) = %d\n", CLOCK_5_PIN, gpio_get(CLOCK_5_PIN));
    printf("  CLOCK_15B (pin %d) = %d\n", CLOCK_15B_PIN, gpio_get(CLOCK_15B_PIN));
}

void check_bus_signals() {
    printf("\n=== Bus Signal Check ===\n");

    // Check clock signals after initing them on the arm core
    // Just read their current state
    printf("Clock signals (reading after init):\n");
    int clk5_samples = 0;
    int clk15_samples = 0;

    // Sample clocks without initializing them
    for (int i = 0; i < 1000; i++) {
        clk5_samples += gpio_get(CLOCK_5_PIN);
        clk15_samples += gpio_get(CLOCK_15B_PIN);
        busy_wait_us(1);
    }
    printf("  CLK5 activity (out of 1000): %d\n", clk5_samples);
    printf("  CLK15B activity (out of 1000): %d\n", clk15_samples);

    // Check for transitions
    int clk5_transitions = 0;
    int clk15_transitions = 0;
    int last_clk5 = gpio_get(CLOCK_5_PIN);
    int last_clk15 = gpio_get(CLOCK_15B_PIN);

    for (int i = 0; i < 1000; i++) {
        int current_clk5 = gpio_get(CLOCK_5_PIN);
        int current_clk15 = gpio_get(CLOCK_15B_PIN);

        if (current_clk5 != last_clk5) clk5_transitions++;
        if (current_clk15 != last_clk15) clk15_transitions++;

        last_clk5 = current_clk5;
        last_clk15 = current_clk15;

        busy_wait_us(1);
    }
    printf("  CLK5 transitions detected: %d\n", clk5_transitions);
    printf("  CLK15B transitions detected: %d\n", clk15_transitions);

    printf("\nWARNING: Skipping HOLD/HLDA test to avoid interfering with PIO\n");
    printf("The PIO will test HOLD/HLDA when it runs\n");

    printf("READY signal = %d (should be 1 for normal operation)\n", gpio_get(READY_PIN));
}

void test_pio_single_write(PIO pio, uint sm, uint32_t address, uint8_t data) {
    printf("\n=== Single Write Test ===\n");
    printf("Writing 0x%02X to address 0x%05X\n", data, address);

    // Check PIO state before
    printf("Before: SM%d PC=0x%x, stalled=%d, TX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_tx_fifo_empty(pio, sm));

    // Prepare the data for PIO
    uint32_t addr_only = address & 0xFFFFF;  // 20-bit address
    uint32_t addr_with_data =  addr_only | (data << 20); ;  // Address with data in low byte

    printf("Sending to PIO: addr=0x%05X, addr_with_data=0x%05X\n", addr_only, addr_with_data);

    // Send to PIO
    pio_sm_put_blocking(pio, sm, addr_with_data);

    // Wait for operation
    sleep_ms(10);

    // Check PIO state after
    printf("After: SM%d PC=0x%x, stalled=%d, TX FIFO empty=%d\n",
           sm, pio_sm_get_pc(pio, sm),
           pio_sm_is_exec_stalled(pio, sm),
           pio_sm_is_tx_fifo_empty(pio, sm));
}

void test_pio_single_read(PIO pio, uint sm, uint offset, uint32_t address) {
    printf("\n=== Single Read Test ===\n");
    printf("Reading from address 0x%05X\n", address);

    dma_log_sm_state("Read SM before request", pio, sm, offset, false);

    // Send address to PIO
    uint32_t addr_only = address & 0xFFFFF;
    uint32_t pindirs = 0;
    uint32_t addr_with_pindirs = addr_only | (pindirs << 20); // Address with pin directions in high bits
    printf("Sending to PIO: addr=0x%07X\n", addr_with_pindirs);
    pio_sm_put_blocking(pio, sm, addr_with_pindirs);

    dma_monitor_progress("Read SM after address", pio, sm, offset, false);

    sleep_ms(10);
    dma_log_sm_state("Read SM after 10ms", pio, sm, offset, false);

    uint32_t data = 0xFF;  // Default value
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        data = pio_sm_get(pio, sm);
        printf("Read data: 0x%02X\n", data & 0xFF);
    } else {
        printf("No data in RX FIFO!\n");
    }

    dma_log_sm_state("Read SM final state", pio, sm, offset, false);
}

// Forward declarations for signal simulator
void signal_simulator_init();
void signal_simulator_stop();
void signal_simulator_test();

int main() {
    stdio_init_all();
    set_sys_clock_khz(200000, true);
    initialize_uart();

    // WARNING: These pins will be controlled by PIO as outputs during DMA cycles
    // Do NOT initialize them here as it will conflict with PIO
    printf("Skipping ARM GPIO init for RD (pin %d) - PIO will control\n", RD_PIN);
    printf("Skipping ARM GPIO init for WR (pin %d) - PIO will control\n", WR_PIN);
    printf("Skipping ARM GPIO init for HOLD (pin %d) - PIO will control\n", HOLD_PIN);
    printf("Skipping ARM GPIO init for ALE (pin %d) - PIO will control\n", ALE_PIN);
    // WARNING: Do NOT use gpio_init() on pins that PIO will read with wait instructions!
    // gpio_init() sets the pin to GPIO_FUNC_SIO which disconnects it from PIO
    // Just leave these pins alone - the PIO init will route them properly
    printf("Skipping ARM GPIO init for HLDA (pin %d) - PIO will handle\n", HLDA_PIN);
    printf("Skipping ARM GPIO init for READY (pin %d) - PIO will handle\n", READY_PIN);
    printf("Skipping ARM GPIO init for CLOCK_5 (pin %d) - PIO will handle\n", CLOCK_5_PIN);
    printf("Skipping ARM GPIO init for CLOCK_15B (pin %d) - PIO will handle\n", CLOCK_15B_PIN);

    printf("\n=== DMA Hardware Diagnostic Test ===\n");
    printf("Running comprehensive diagnostics...\n\n");

    dump_pio_versions();
    
    gpio_init(DEBUG_PIN);
    gpio_set_dir(DEBUG_PIN, GPIO_OUT);
    gpio_put(DEBUG_PIN, 0);

    // Sleep briefly to ensure serial output is visible
    sleep_ms(5);

    // Check initial GPIO states
    print_gpio_states();

    // Check bus signals (should now show activity)
    check_bus_signals();

    printf("\n=== ARM Test Skipped (to avoid PIO interference) ===\n");
    printf("Previous runs confirmed HOLD/HLDA hardware works correctly\n");

    // Launch stub core1
    multicore_launch_core1(core1_test_stub);

    // Configure DMA PIO state machines
    PIO dma_pio = PIO_DMA;

    int outcome = pio_set_gpio_base(dma_pio, BD0_PIN);
    printf("pio_set_gpio_base outcome: %d\n", outcome);

    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int read_sm = pio_claim_unused_sm(dma_pio, true);
    int write_sm = pio_claim_unused_sm(dma_pio, true);

    printf("\n=== PIO Configuration ===\n");
    printf("PIO: %d\n", pio_get_index(dma_pio));
    printf("Read SM: %d\n", read_sm);
    printf("Write SM: %d\n", write_sm);
    printf("Program offset: 0x%02x\n", dma_read_write_program_offset);
    printf("Program length: %d instructions\n", dma_read_write_program.length);

    // Initialize state machines
    dma_read_write_program_init(dma_pio, read_sm, dma_read_write_program_offset, BD0_PIN, DMA_READ);
    dma_read_write_program_init(dma_pio, write_sm, dma_read_write_program_offset, BD0_PIN, DMA_WRITE);

    // Don't prime here - we'll do it right before enabling each SM
    printf("\n=== State Machine Configuration Complete ===\n");

    // Check if FIFOs are ready
    printf("Read SM TX FIFO empty: %d\n", pio_sm_is_tx_fifo_empty(dma_pio, read_sm));
    printf("Write SM TX FIFO empty: %d\n", pio_sm_is_tx_fifo_empty(dma_pio, write_sm));

    // Check SM states after init
    printf("\nState after init:\n");
    printf("Read SM: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, read_sm),
           pio_sm_is_exec_stalled(dma_pio, read_sm));
    printf("Write SM: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, write_sm),
           pio_sm_is_exec_stalled(dma_pio, write_sm));

    // Test 1: Single write operation (TX FIFO only has 4 slots!)
    printf("\n=== Test 1: Single Write Operation ===\n");

    // Load initialization values first (1 slots)
    printf("Loading initialization value...\n");

    dump_pio_dbg();

    //one time initialization of X direction
    //x = DMA direction (read or write)
    gpio_put(DEBUG_PIN, 1);
    printf("  Init Slot 1: DMA_WRITE (0x%08x)\n", DMA_WRITE);
    pio_sm_put_blocking(dma_pio, write_sm, DMA_WRITE);
    gpio_put(DEBUG_PIN, 0);

     dump_pio_dbg();

    // Check FIFO before enabling
    printf("Before enabling - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm));
        
    gpio_put(DEBUG_PIN, 1);
    printf("Enabling write SM (will consume init value)...\n");
    pio_sm_set_enabled(dma_pio, write_sm, true);
    gpio_put(DEBUG_PIN, 0);

    dma_monitor_progress("Write SM post-enable", dma_pio, write_sm,
                         dma_read_write_program_offset, true);

    // Check DBG registers after SM starts
    printf("After SM enable - DBG_PADOE: 0x%08x, DBG_PADOUT: 0x%08x\n",
           pio0_hw->dbg_padoe, pio0_hw->dbg_padout);

    // // Test: Can we manually set pindirs via PIO exec?
    // printf("\nTesting manual pindir control via PIO exec...\n");
    // // Try to set pin 25 (HOLD) as output via exec
    // pio_sm_exec(dma_pio, write_sm, pio_encode_set(pio_pindirs, 1));  // Set pindirs to 1
    // sleep_us(10);
    // printf("After manual SET PINDIRS 1 - DBG_PADOE: 0x%08x\n", pio0_hw->dbg_padoe);

    // // Try mov pindirs, ~null (all outputs)
    // // Using 0xbc63 which is the encoding for MOV PINDIRS, ~NULL
    // pio_sm_exec(dma_pio, write_sm, 0xbc63);  // mov pindirs, ~null
    // sleep_us(10);
    // printf("After MOV PINDIRS ~NULL - DBG_PADOE: 0x%08x\n", pio0_hw->dbg_padoe);

    // Reset back to inputs
    // pio_sm_exec(dma_pio, write_sm, pio_encode_set(pio_pindirs, 0));
    // sleep_us(10);
    // printf("After SET PINDIRS 0 - DBG_PADOE: 0x%08x\n", pio0_hw->dbg_padoe);

    // Now load the actual DMA cycle data
    printf("\nLoading DMA cycle data...\n");

    // First write operation
    uint32_t addr = TEST_ADDRESS & 0xFFFFF;
    uint32_t data = (0xAA & 0xFF);
    uint32_t addr_data = addr | (data << 20);  // Address in low 20 bits, data in high 8 bits

    printf("  Data Slot 1: Address plus data (0x%07x)\n", addr_data);
    gpio_put(DEBUG_PIN, 1);
    pio_sm_put_blocking(dma_pio, write_sm, addr_data);
    dump_pio_dbg();
    gpio_put(DEBUG_PIN, 0);
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("Write SM after slot 1", dma_pio, write_sm,
                         dma_read_write_program_offset, true);

    addr++;
    data = (0x55 & 0xFF);
    addr_data = addr | (data << 20);
    printf("  Data Slot 2: Address plus data (0x%07x)\n", addr_data);
    pio_sm_put_blocking(dma_pio, write_sm, addr_data);
    dump_pio_dbg();
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("Write SM after slot 2", dma_pio, write_sm,
                         dma_read_write_program_offset, true);

    addr++;
    data = (0x55 & 0xFF);
    addr_data = addr | (data << 20);
    printf("  Data Slot 3: Address plus data (0x%07x)\n", addr_data);
    pio_sm_put_blocking(dma_pio, write_sm, addr_data);
    dump_pio_dbg();
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("Write SM after slot 3", dma_pio, write_sm,
                         dma_read_write_program_offset, true);

    // Check immediately after loading data
    printf("After loading data - TX FIFO: %d/4, PC=0x%x\n",
           pio_sm_get_tx_fifo_level(dma_pio, write_sm),
           pio_sm_get_pc(dma_pio, write_sm));

    // Check HOLD and HLDA states - both ARM view and PIO view
    printf("HOLD pin (ARM view): %d, HLDA pin (ARM view): %d\n", gpio_get(HOLD_PIN), gpio_get(HLDA_PIN));

    // Check what PIO actually sees on the pins - try different registers
    uint32_t pio_pins = pio0_hw->input_sync_bypass;
    uint32_t gpio_in = sio_hw->gpio_in;
    printf("PIO0 input_sync_bypass: 0x%08x\n", pio_pins);
    printf("SIO gpio_in register: 0x%08x\n", gpio_in);
    // Check if we're on the right GPIO base (RP2350 specific)
    printf("PIO0 DBG_CFGINFO: 0x%08x\n", pio0_hw->dbg_cfginfo);

    // Check both PIO and SIO views
    printf("  HLDA (pin %d) - PIO: %d, SIO: %d\n", HLDA_PIN,
           (pio_pins >> HLDA_PIN) & 1, (gpio_in >> HLDA_PIN) & 1);
    printf("  READY (pin %d) - PIO: %d, SIO: %d\n", READY_PIN,
           (pio_pins >> READY_PIN) & 1, (gpio_in >> READY_PIN) & 1);
    printf("  CLK5 (pin %d) - PIO: %d, SIO: %d\n", CLOCK_5_PIN,
           (pio_pins >> CLOCK_5_PIN) & 1, (gpio_in >> CLOCK_5_PIN) & 1);
    printf("  CLK15B (pin %d) - PIO: %d, SIO: %d\n", CLOCK_15B_PIN,
           (pio_pins >> CLOCK_15B_PIN) & 1, (gpio_in >> CLOCK_15B_PIN) & 1);

    print_gpio_states();

    dma_log_sm_state("Write SM after 1ms", dma_pio, write_sm,
                     dma_read_write_program_offset, true);

    // Poll ALL pins to find which one has HLDA signal
    printf("Polling all pins for changes after HOLD asserted:\n");
    uint32_t initial_pins = 0;
    uint32_t changed_pins_mask = 0;

    // Get initial state
    pio_sm_exec(dma_pio, write_sm, pio_encode_in(0, 32));
    pio_sm_exec(dma_pio, write_sm, pio_encode_push(false, false));
    sleep_us(10);
    if (!pio_sm_is_rx_fifo_empty(dma_pio, write_sm)) {
        initial_pins = pio_sm_get(dma_pio, write_sm);
        printf("  Initial pins state: 0x%08x\n", initial_pins);
    }

    // Poll for changes
    int found_changes = 0;
    for (int i = 0; i < 100; i++) {
        pio_sm_exec(dma_pio, write_sm, pio_encode_in(0, 32));
        pio_sm_exec(dma_pio, write_sm, pio_encode_push(false, false));
        sleep_us(100);

        if (!pio_sm_is_rx_fifo_empty(dma_pio, write_sm)) {
            uint32_t pins = pio_sm_get(dma_pio, write_sm);
            uint32_t changes = pins ^ initial_pins;
            if (changes && !found_changes) {
                found_changes = 1;
                changed_pins_mask |= changes;
                // Print which pins changed
                printf("  Changes detected at iteration %d:\n", i);
                for (int p = 0; p < 32; p++) {
                    if ((changes >> p) & 1) {
                        printf("    Pin %d: %d -> %d %s\n",
                               p, (initial_pins >> p) & 1, (pins >> p) & 1,
                               (p == HLDA_PIN) ? "(Expected HLDA)" : "");
                    }
                }
            }
        }
    }

    if (!found_changes) {
        printf("  NO PIN CHANGES DETECTED!\n");
    }
    printf("  Expected HLDA on pin %d: %s\n", HLDA_PIN,
           (changed_pins_mask & (1 << HLDA_PIN)) ? "CHANGED" : "NO CHANGE");

    // Check PIO view again - try multiple ways
    pio_pins = pio0_hw->input_sync_bypass;
    gpio_in = sio_hw->gpio_in;

    // Check the PIO's DBG registers which show what the PIO sees
    uint32_t pio_dbg_padout = pio0_hw->dbg_padout;
    uint32_t pio_dbg_padoe = pio0_hw->dbg_padoe;

    printf("After 1ms wait:\n");
    printf("  PIO0 input_sync_bypass: 0x%08x\n", pio_pins);
    printf("  PIO0 dbg_padout: 0x%08x\n", pio_dbg_padout);
    printf("  PIO0 dbg_padoe: 0x%08x\n", pio_dbg_padoe);
    printf("  SIO gpio_in: 0x%08x\n", gpio_in);

    printf("PIO sees HLDA=%d, READY=%d, CLK5=%d, CLK15B=%d\n",
           (pio_pins >> HLDA_PIN) & 1, (pio_pins >> READY_PIN) & 1,
           (pio_pins >> CLOCK_5_PIN) & 1, (pio_pins >> CLOCK_15B_PIN) & 1);
    printf("ARM sees HOLD=%d, HLDA=%d, READY=%d, CLK5=%d, CLK15B=%d\n",
           gpio_get(HOLD_PIN), gpio_get(HLDA_PIN), gpio_get(READY_PIN),
           gpio_get(CLOCK_5_PIN), gpio_get(CLOCK_15B_PIN));

    // Try forcing a simple PIO instruction to read GPIO
    printf("\nTesting PIO pin reading with 'IN PINS, 32' instruction:\n");
    // First, we need to configure the IN pin base for this state machine
    // The write SM might not have IN pins configured, so let's set it
    pio_sm_set_in_pins(dma_pio, write_sm, 0);  // Set IN base to pin 0

    // Execute IN PINS, 32 to read all 32 pins
    // PIO_SRC_PINS is defined as 0 in the SDK
    pio_sm_exec(dma_pio, write_sm, pio_encode_in(0, 32));  // 0 = pins source
    sleep_us(10);

    // Also try pushing ISR to RX FIFO
    pio_sm_exec(dma_pio, write_sm, pio_encode_push(false, false));
    sleep_us(10);

    if (!pio_sm_is_rx_fifo_empty(dma_pio, write_sm)) {
        uint32_t pins_read = pio_sm_get(dma_pio, write_sm);
        printf("  PIO read pins via IN: 0x%08x\n", pins_read);
        printf("  Pin 27=%d, 28=%d, 29=%d, 30=%d\n",
               (pins_read >> 27) & 1, (pins_read >> 28) & 1,
               (pins_read >> 29) & 1, (pins_read >> 30) & 1);
    } else {
        printf("  PIO IN instruction didn't produce data\n");
    }

    sleep_ms(10);
    dma_log_sm_state("Write SM after 10ms", dma_pio, write_sm,
                     dma_read_write_program_offset, true);

    // Check if state machine is waiting at expected point
    sleep_ms(100);
    dma_log_sm_state("Write SM after 100ms", dma_pio, write_sm,
                     dma_read_write_program_offset, true);
    printf("    TX FIFO level: %d/4\n", pio_sm_get_tx_fifo_level(dma_pio, write_sm));

    printf("DMA write test complete. Disabling write SM.\n");
    pio_sm_set_enabled(dma_pio, write_sm, false);

    // Test 2: Single read operation
    printf("\n=== Test 2: Single Read Operation ===\n");

    // Initialize the read state machine with required values
    printf("Loading read SM FIFO with init values...\n");
    pio_sm_put_blocking(dma_pio, read_sm, DMA_READ);  // Direction

    printf("Before enabling read SM - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, read_sm));

    pio_sm_set_enabled(dma_pio, read_sm, true);
    dma_monitor_progress("Read SM post-enable", dma_pio, read_sm,
                         dma_read_write_program_offset, false);

    test_pio_single_read(dma_pio, read_sm, dma_read_write_program_offset, TEST_ADDRESS);

    sleep_ms(100);
    dma_log_sm_state("Read SM after 100ms", dma_pio, read_sm,
                     dma_read_write_program_offset, false);

    pio_sm_set_enabled(dma_pio, read_sm, false);

    // Final GPIO state check
    printf("\n=== Final GPIO State ===\n");
    print_gpio_states();

    // Check for any PIO errors
    printf("\n=== PIO Error Check ===\n");
    uint32_t fdebug = pio0_hw->fdebug;
    printf("PIO0 FDEBUG register: 0x%08X\n", fdebug);
    if (fdebug & 0x0F) {
        printf("  TX STALL detected on SM: %d\n", fdebug & 0x0F);
    }
    if ((fdebug >> 8) & 0x0F) {
        printf("  TX OVER detected on SM: %d\n", (fdebug >> 8) & 0x0F);
    }
    if ((fdebug >> 16) & 0x0F) {
        printf("  RX UNDER detected on SM: %d\n", (fdebug >> 16) & 0x0F);
    }
    if ((fdebug >> 24) & 0x0F) {
        printf("  RX STALL detected on SM: %d\n", (fdebug >> 24) & 0x0F);
    }

    printf("\n=== Diagnostic Complete ===\n");
    printf("Key observations:\n");
    printf("1. Check if HLDA responds to HOLD signal\n");
    printf("2. Check if clocks are running\n");
    printf("3. Check PIO state machine progression\n");
    printf("4. Check for PIO errors\n");

    // Keep running for monitoring
    while (1) {
        sleep_ms(5000);
    }

    return 0;
}

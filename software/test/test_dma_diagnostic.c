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
#include "hardware/structs/iobank0.h"

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
    // Use the correct PIO instance based on PIO_DMA
    pio_hw_t *pio_hw = (PIO_DMA == pio0) ? pio0_hw : (PIO_DMA == pio1) ? pio1_hw : pio2_hw;
    int pio_idx = pio_get_index(PIO_DMA);
    printf("PIO%d_DBG_PADOE = 0x%08x\n", pio_idx, pio_hw->dbg_padoe);
    printf("PIO%d_DBG_PADOUT = 0x%08x\n", pio_idx, pio_hw->dbg_padout);
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

    dma_log_sm_state("DMA SM before request", pio, sm, offset, false);

    // Send address to PIO using new format
    uint32_t addr_data = DMA_FORMAT_READ(address & 0xFFFFF);
    printf("Sending to PIO: Read from 0x%05X (formatted: 0x%08X)\n",
           address & 0xFFFFF, addr_data);
    pio_sm_put_blocking(pio, sm, addr_data);

    dma_monitor_progress("DMA SM after address", pio, sm, offset, false);

    sleep_ms(10);
    dma_log_sm_state("DMA SM after 10ms", pio, sm, offset, false);

    uint32_t data = 0xFF;  // Default value
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        data = pio_sm_get(pio, sm);
        printf("Read data: 0x%02X\n", data & 0xFF);
    } else {
        printf("No data in RX FIFO!\n");
    }

    dma_log_sm_state("DMA SM final state", pio, sm, offset, false);
}

// Forward declarations for signal simulator
void signal_simulator_init();
void signal_simulator_stop();
void signal_simulator_test();

// Helper to get the correct PIO hardware instance
static inline pio_hw_t* get_pio_hw(PIO pio) {
    return (pio == pio0) ? pio0_hw : (pio == pio1) ? pio1_hw : pio2_hw;
}

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

    //int outcome = pio_set_gpio_base(dma_pio, BD0_PIN);
    //printf("pio_set_gpio_base outcome: %d\n", outcome);
    printf("pio_set_gpio_base  commented out to avoid issues\n");

    int dma_read_write_program_offset = pio_add_program(dma_pio, &dma_read_write_program);
    int dma_sm = pio_claim_unused_sm(dma_pio, true);  // Single SM for both read and write

    printf("\n=== PIO Configuration ===\n");
    printf("PIO: %d\n", pio_get_index(dma_pio));
    printf("DMA SM: %d\n", dma_sm);
    printf("Program offset: 0x%02x\n", dma_read_write_program_offset);
    printf("Program length: %d instructions\n", dma_read_write_program.length);

    // Initialize single state machine for both read and write operations
    dma_read_write_program_init(dma_pio, dma_sm, dma_read_write_program_offset, BD0_PIN);

    // Don't prime here - we'll do it right before enabling
    printf("\n=== State Machine Configuration Complete ===\n");

    // Check if FIFO is ready
    printf("DMA SM TX FIFO empty: %d\n", pio_sm_is_tx_fifo_empty(dma_pio, dma_sm));

    // Check SM state after init
    printf("\nState after init:\n");
    printf("DMA SM: PC=0x%x, stalled=%d\n",
           pio_sm_get_pc(dma_pio, dma_sm),
           pio_sm_is_exec_stalled(dma_pio, dma_sm));

    // Test 1: Single write operation (TX FIFO only has 4 slots!)
    printf("\n=== Test 1: Single Write Operation ===\n");

    // No initialization value needed with new format
    printf("Preparing write operation...\n");

    dump_pio_dbg();

    // Check FIFO before enabling
    printf("Before enabling - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, dma_sm));

    gpio_put(DEBUG_PIN, 1);
    printf("Enabling DMA SM...\n");
    pio_sm_set_enabled(dma_pio, dma_sm, true);
    gpio_put(DEBUG_PIN, 0);

    dma_monitor_progress("DMA SM post-enable", dma_pio, dma_sm,
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

    // First write operation using new format
    uint32_t addr = TEST_ADDRESS & 0xFFFFF;
    uint8_t data = 0xAA;
    uint32_t addr_data = DMA_FORMAT_WRITE(addr, data);  // Use new format macro

    printf("  Data Slot 1: Write to 0x%05x with data 0x%02x (formatted: 0x%08x)\n",
           addr, data, addr_data);
    gpio_put(DEBUG_PIN, 1);
    pio_sm_put_blocking(dma_pio, dma_sm, addr_data);
    dump_pio_dbg();
    gpio_put(DEBUG_PIN, 0);
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("DMA SM after slot 1", dma_pio, dma_sm,
                         dma_read_write_program_offset, true);
    gpio_init(HOLD_PIN);
    gpio_set_dir(HOLD_PIN, GPIO_OUT);
    gpio_put(HOLD_PIN, 1);
    // After the state machine has processed some data
    printf("\n=== Manual HOLD Pin Control Test ===\n");

    // Stop the SM first to take control
    pio_sm_set_enabled(dma_pio, dma_sm, false);

    // Check current state
    printf("Before manual control - DBG_PADOE: 0x%08x\n", pio0_hw->dbg_padoe);
    printf("HOLD pin (ARM): %d\n", gpio_get(HOLD_PIN));

    // Test 1: Explicitly set HOLD as output and drive low
    printf("\n1. Setting HOLD as output (drive low)...\n");
      
    sleep_us(10);
    printf("   DBG_PADOE: 0x%08x, HOLD=%d\n", pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Test 2: Set HOLD pin value high (while still output)
    printf("\n2. Setting HOLD output value HIGH...\n");
    pio_sm_exec(dma_pio, dma_sm, pio_encode_set(pio_pins, (1u << HOLD_PIN)));
    sleep_us(10);
    printf("   DBG_PADOE: 0x%08x, HOLD=%d\n", pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Test 3: Set HOLD as input (should float high with Victor pull-up)
    printf("\n3. Setting HOLD as input (float)...\n");
    pio_sm_exec(dma_pio, dma_sm, pio_encode_set(pio_pindirs, 0));
    sleep_us(10);
    printf("   DBG_PADOE: 0x%08x, HOLD=%d\n", pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Test 4: Try MOV instead of SET for pindirs
    printf("\n4. Using MOV PINDIRS, NULL (all inputs)...\n");
    pio_sm_exec(dma_pio, dma_sm, pio_encode_mov(pio_pindirs, pio_null));
    sleep_us(10);
    printf("   DBG_PADOE: 0x%08x, HOLD=%d\n", pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Test 5: Verify we can drive it low again
    printf("\n5. Setting HOLD as output again (drive low)...\n");
    pio_sm_exec(dma_pio, dma_sm, pio_encode_set(pio_pindirs, (1u << HOLD_PIN)));
    pio_sm_exec(dma_pio, dma_sm, pio_encode_set(pio_pins, 0));
    sleep_us(10);  
    printf("   DBG_PADOE: 0x%08x, HOLD=%d\n", pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Verify the SM's pin configuration (use correct PIO instance)
    pio_hw_t *pio_hw = (dma_pio == pio0) ? pio0_hw : (dma_pio == pio1) ? pio1_hw : pio2_hw;
    uint32_t pinctrl = pio_hw->sm[dma_sm].pinctrl;
    uint32_t execctrl = pio_hw->sm[dma_sm].execctrl;

    printf("\n=== SM Pin Mapping Check ===\n");
    printf("SET_BASE = %d (should be %d for RD_PIN)\n", (pinctrl >> 5) & 0x1F, RD_PIN);
    printf("OUT_BASE = %d (should be %d for BD0_PIN)\n", pinctrl & 0x1F, BD0_PIN);
    printf("SIDESET_BASE = %d (should be %d for HOLD_PIN)\n", (pinctrl >> 10) & 0x1F, HOLD_PIN);

    // Also check if side_set is affecting the right pins
    printf("SIDE_EN = %d (should be 1)\n", (execctrl >> 30) & 0x1);
    printf("SIDE_PINDIR = %d (should be 1)\n", (execctrl >> 29) & 0x1);

    // Check which SM owns pin 25
    printf("\n=== Pin 25 Ownership Investigation ===\n");

    // Check GPIO function
    printf("GPIO25_CTRL = 0x%08x\n", iobank0_hw->io[25].ctrl);
    uint32_t func = iobank0_hw->io[25].ctrl & 0x1f;
    printf("  Function = %d (", func);
    switch(func) {
        case GPIO_FUNC_SIO: printf("SIO"); break;
        case GPIO_FUNC_PIO0: printf("PIO0"); break;
        case GPIO_FUNC_PIO1: printf("PIO1"); break;
        case GPIO_FUNC_PIO2: printf("PIO2"); break;
        default: printf("OTHER:%d", func);
    }
    printf(")\n");

    // Check all SMs to see who's controlling pin 25
    for (int sm = 0; sm < 4; sm++) {
        uint32_t sm_pinctrl = pio0_hw->sm[sm].pinctrl;
        uint32_t sm_execctrl = pio0_hw->sm[sm].execctrl;

        // Check OUT pins
        uint32_t out_base = sm_pinctrl & 0x1f;
        uint32_t out_count = (sm_pinctrl >> 20) & 0x3f;
        if (25 >= out_base && 25 < out_base + out_count) {
            printf("  SM%d OUT pins include pin 25 (base=%d, count=%d)\n",
                   sm, out_base, out_count);
        }

        // Check SET pins
        uint32_t set_base = (sm_pinctrl >> 5) & 0x1f;
        uint32_t set_count = (sm_pinctrl >> 26) & 0x7;
        if (25 >= set_base && 25 < set_base + set_count) {
            printf("  SM%d SET pins include pin 25 (base=%d, count=%d)\n",
                   sm, set_base, set_count);
        }

        // Check SIDESET pins
        uint32_t side_base = (sm_pinctrl >> 10) & 0x1f;
        uint32_t side_count = (sm_pinctrl >> 29) & 0x7;
        if (25 >= side_base && 25 < side_base + side_count) {
            printf("  SM%d SIDESET pins include pin 25 (base=%d, count=%d)\n",
                   sm, side_base, side_count);
            if (sm_execctrl & (1 << 29)) {
                printf("    SIDE_PINDIR is enabled for this SM\n");
            }
        }
    }

    // Try to force control back to SM1
    printf("\nAttempting to force pin 25 control to SM1...\n");

    // First, disable all SMs temporarily
    bool sm_enabled[4];
    for (int sm = 0; sm < 4; sm++) {
        // Check if SM is enabled by looking at CTRL register
        sm_enabled[sm] = (pio0_hw->ctrl & (1u << sm)) != 0;
        pio_sm_set_enabled(pio0, sm, false);
    }

    // Explicitly set pin 25 to PIO0
    gpio_set_function(25, GPIO_FUNC_PIO0);

    // Try manual control again
    pio_sm_exec(pio0, 1, pio_encode_set(pio_pindirs, 0));  // All inputs
    sleep_us(10);
    printf("After forcing input: DBG_PADOE=0x%08x, HOLD=%d\n",
           pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    pio_sm_exec(pio0, 1, pio_encode_set(pio_pindirs, 1 << 25));  // HOLD output
    sleep_us(10);
    printf("After forcing output: DBG_PADOE=0x%08x, HOLD=%d\n",
           pio0_hw->dbg_padoe, gpio_get(HOLD_PIN));

    // Re-enable the SMs that were enabled
    for (int sm = 0; sm < 4; sm++) {
        if (sm_enabled[sm]) {
            pio_sm_set_enabled(pio0, sm, true);
        }
    }

    addr++;
    data = 0x55;
    addr_data = DMA_FORMAT_WRITE(addr, data);  // Use new format
    printf("  Data Slot 2: Write to 0x%05x with data 0x%02x (formatted: 0x%08x)\n",
           addr, data, addr_data);
    pio_sm_put_blocking(dma_pio, dma_sm, addr_data);
    dump_pio_dbg();
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("DMA SM after slot 2", dma_pio, dma_sm,
                         dma_read_write_program_offset, true);

    addr++;
    data = 0x33;
    addr_data = DMA_FORMAT_WRITE(addr, data);  // Use new format
    printf("  Data Slot 3: Write to 0x%05x with data 0x%02x (formatted: 0x%08x)\n",
           addr, data, addr_data);
    pio_sm_put_blocking(dma_pio, dma_sm, addr_data);
    dump_pio_dbg();
    printf("DMA cycle data loaded.\n");
    dma_monitor_progress("DMA SM after slot 3", dma_pio, dma_sm,
                         dma_read_write_program_offset, true);

    // Check immediately after loading data
    printf("After loading data - TX FIFO: %d/4, PC=0x%x\n",
           pio_sm_get_tx_fifo_level(dma_pio, dma_sm),
           pio_sm_get_pc(dma_pio, dma_sm));

    // Check HOLD and HLDA states - both ARM view and PIO view
    printf("HOLD pin (ARM view): %d, HLDA pin (ARM view): %d\n", gpio_get(HOLD_PIN), gpio_get(HLDA_PIN));

    // Check what PIO actually sees on the pins - try different registers
    {
        pio_hw_t *pio_hw = get_pio_hw(dma_pio);
        uint32_t pio_pins = pio_hw->input_sync_bypass;
    uint32_t gpio_in = sio_hw->gpio_in;
    printf("PIO%d input_sync_bypass: 0x%08x\n", pio_get_index(dma_pio), pio_pins);
    printf("SIO gpio_in register: 0x%08x\n", gpio_in);
    // Check if we're on the right GPIO base (RP2350 specific)
    printf("PIO%d DBG_CFGINFO: 0x%08x\n", pio_get_index(dma_pio), pio_hw->dbg_cfginfo);

    // Check both PIO and SIO views
    printf("  HLDA (pin %d) - PIO: %d, SIO: %d\n", HLDA_PIN,
           (pio_pins >> HLDA_PIN) & 1, (gpio_in >> HLDA_PIN) & 1);
    printf("  READY (pin %d) - PIO: %d, SIO: %d\n", READY_PIN,
           (pio_pins >> READY_PIN) & 1, (gpio_in >> READY_PIN) & 1);
    printf("  CLK5 (pin %d) - PIO: %d, SIO: %d\n", CLOCK_5_PIN,
           (pio_pins >> CLOCK_5_PIN) & 1, (gpio_in >> CLOCK_5_PIN) & 1);
    printf("  CLK15B (pin %d) - PIO: %d, SIO: %d\n", CLOCK_15B_PIN,
           (pio_pins >> CLOCK_15B_PIN) & 1, (gpio_in >> CLOCK_15B_PIN) & 1);
    }  // End scope for pio_hw

    print_gpio_states();

    dma_log_sm_state("DMA SM after 1ms", dma_pio, dma_sm,
                     dma_read_write_program_offset, true);

    // Poll ALL pins to find which one has HLDA signal
    printf("Polling all pins for changes after HOLD asserted:\n");
    uint32_t initial_pins = 0;
    uint32_t changed_pins_mask = 0;

    // Get initial state
    pio_sm_exec(dma_pio, dma_sm, pio_encode_in(0, 32));
    pio_sm_exec(dma_pio, dma_sm, pio_encode_push(false, false));
    sleep_us(10);
    if (!pio_sm_is_rx_fifo_empty(dma_pio, dma_sm)) {
        initial_pins = pio_sm_get(dma_pio, dma_sm);
        printf("  Initial pins state: 0x%08x\n", initial_pins);
    }

    // Poll for changes
    int found_changes = 0;
    for (int i = 0; i < 100; i++) {
        pio_sm_exec(dma_pio, dma_sm, pio_encode_in(0, 32));
        pio_sm_exec(dma_pio, dma_sm, pio_encode_push(false, false));
        sleep_us(100);

        if (!pio_sm_is_rx_fifo_empty(dma_pio, dma_sm)) {
            uint32_t pins = pio_sm_get(dma_pio, dma_sm);
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
    {
        pio_hw_t *pio_hw = get_pio_hw(dma_pio);
        uint32_t pio_pins = pio_hw->input_sync_bypass;
        uint32_t gpio_in = sio_hw->gpio_in;

        // Check the PIO's DBG registers which show what the PIO sees
        uint32_t pio_dbg_padout = pio_hw->dbg_padout;
        uint32_t pio_dbg_padoe = pio_hw->dbg_padoe;

        printf("After 1ms wait:\n");
        printf("  PIO%d input_sync_bypass: 0x%08x\n", pio_get_index(dma_pio), pio_pins);
        printf("  PIO%d dbg_padout: 0x%08x\n", pio_get_index(dma_pio), pio_dbg_padout);
        printf("  PIO%d dbg_padoe: 0x%08x\n", pio_get_index(dma_pio), pio_dbg_padoe);
        printf("  SIO gpio_in: 0x%08x\n", gpio_in);

        printf("PIO sees HLDA=%d, READY=%d, CLK5=%d, CLK15B=%d\n",
               (pio_pins >> HLDA_PIN) & 1, (pio_pins >> READY_PIN) & 1,
               (pio_pins >> CLOCK_5_PIN) & 1, (pio_pins >> CLOCK_15B_PIN) & 1);
        printf("ARM sees HOLD=%d, HLDA=%d, READY=%d, CLK5=%d, CLK15B=%d\n",
               gpio_get(HOLD_PIN), gpio_get(HLDA_PIN), gpio_get(READY_PIN),
               gpio_get(CLOCK_5_PIN), gpio_get(CLOCK_15B_PIN));
    }  // End scope - properly close the block

    // Try forcing a simple PIO instruction to read GPIO
    printf("\nTesting PIO pin reading with 'IN PINS, 32' instruction:\n");
    // First, we need to configure the IN pin base for this state machine
    pio_sm_set_in_pins(dma_pio, dma_sm, 0);  // Set IN base to pin 0

    // Execute IN PINS, 32 to read all 32 pins
    // PIO_SRC_PINS is defined as 0 in the SDK
    pio_sm_exec(dma_pio, dma_sm, pio_encode_in(0, 32));  // 0 = pins source
    sleep_us(10);

    // Also try pushing ISR to RX FIFO
    pio_sm_exec(dma_pio, dma_sm, pio_encode_push(false, false));
    sleep_us(10);

    if (!pio_sm_is_rx_fifo_empty(dma_pio, dma_sm)) {
        uint32_t pins_read = pio_sm_get(dma_pio, dma_sm);
        printf("  PIO read pins via IN: 0x%08x\n", pins_read);
        printf("  Pin 27=%d, 28=%d, 29=%d, 30=%d\n",
               (pins_read >> 27) & 1, (pins_read >> 28) & 1,
               (pins_read >> 29) & 1, (pins_read >> 30) & 1);
    } else {
        printf("  PIO IN instruction didn't produce data\n");
    }

    sleep_ms(10);
    dma_log_sm_state("DMA SM after 10ms", dma_pio, dma_sm,
                     dma_read_write_program_offset, true);

    // Check if state machine is waiting at expected point
    sleep_ms(100);
    dma_log_sm_state("DMA SM after 100ms", dma_pio, dma_sm,
                     dma_read_write_program_offset, true);
    printf("    TX FIFO level: %d/4\n", pio_sm_get_tx_fifo_level(dma_pio, dma_sm));

    printf("DMA write test complete. Disabling DMA SM.\n");
    pio_sm_set_enabled(dma_pio, dma_sm, false);

    // Test 2: Single read operation
    printf("\n=== Test 2: Single Read Operation ===\n");

    // Re-enable the DMA SM for read operation
    printf("Re-enabling DMA SM for read operation...\n");

    printf("Before enabling DMA SM - TX FIFO level: %d/4\n",
           pio_sm_get_tx_fifo_level(dma_pio, dma_sm));

    pio_sm_set_enabled(dma_pio, dma_sm, true);
    dma_monitor_progress("DMA SM post-enable (read)", dma_pio, dma_sm,
                         dma_read_write_program_offset, false);

    test_pio_single_read(dma_pio, dma_sm, dma_read_write_program_offset, TEST_ADDRESS);

    sleep_ms(100);
    dma_log_sm_state("DMA SM after 100ms (read)", dma_pio, dma_sm,
                     dma_read_write_program_offset, false);

    pio_sm_set_enabled(dma_pio, dma_sm, false);

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

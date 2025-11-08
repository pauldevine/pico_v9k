# ARM-PIO Integration Analysis for bus_output_helper Architecture

## Executive Summary

The new three-state-machine PIO architecture requires significant changes to the ARM-side code. The critical issue is that **register read responses must now be pushed to `bus_output_helper` TX FIFO** instead of directly to `board_registers` TX FIFO.

## Current ARM Architecture

### IRQ Handler Flow (dma_ultra_fast.c)

Currently, the ultra-fast IRQ handler (`registers_irq_handler_ultra`) processes three types of FIFO messages from `board_registers`:

```c
// Line 297-323: PREFETCH_ADDRESS handling
case FIFO_PREFETCH_ADDRESS: {
    uint32_t address = dma_fifo_prefetch_address(raw_value);
    uint32_t offset = address - DMA_REGISTER_BASE;
    // ... mask and lookup register value ...
    uint8_t value = register_memory[masked_offset];

    // CRITICAL: Response pushed to board_registers SM TX FIFO
    uint32_t response = (0xFF00u | (uint32_t)(value & 0xFFu));
    pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, response);
    break;
}
```

**Problem**: Line 320 pushes the response to `PIO_REGISTERS, REGISTERS_SM` TX FIFO. But `board_registers` no longer outputs data - that's now `bus_output_helper`'s job!

### Current Initialization (dma_board.c)

```c
// Lines 87-94: board_registers setup
PIO register_pio = PIO_REGISTERS;  // PIO1
int register_sm = REGISTERS_SM;     // SM 0
pio_sm_claim(register_pio, REGISTERS_SM);
int dma_registers_program_offset = pio_add_program(register_pio, &dma_registers_program);
dma_registers_program_init(register_pio, register_sm, dma_registers_program_offset);

// Lines 97-106: extio_helper setup
PIO extio_pio = PIO_EXTIO;  // PIO2
int extio_sm = EXTIO_SM;     // SM 0
```

**Missing**: No initialization for `bus_output_helper` SM!

## Required Changes

### 1. Add bus_output_helper Initialization

**File**: `dma_board.c`

**Location**: After extio_helper init (around line 106), add:

```c
// Configure the bus_output_helper PIO state machine to manage BD0-A19 outputs
PIO bus_helper_pio = PIO_BUS_HELPER;  // Need to define: suggest PIO0
int bus_helper_sm = BUS_HELPER_SM;     // Need to define: suggest SM 1
pio_sm_claim(bus_helper_pio, bus_helper_sm);
int bus_helper_offset = pio_add_program(bus_helper_pio, &bus_output_helper_program);
bus_output_helper_program_init(bus_helper_pio, bus_helper_sm, bus_helper_offset);

// CRITICAL: Preload pindirs value for DMA read cycles (0xFFF00)
// This satisfies the preamble in bus_output_helper.pio lines 12-13
pio_sm_put_blocking(bus_helper_pio, bus_helper_sm, 0xFFF00);

printf("bus_output_helper initialized on PIO%d SM%d\n",
       pio_get_index(bus_helper_pio), bus_helper_sm);
```

**New definitions needed in `dma.h`**:

```c
#define PIO_BUS_HELPER pio0
#define BUS_HELPER_SM 1
```

### 2. Create bus_output_helper Access Functions

**File**: `software/pico_victor/dma.h` and `dma.c`

Add to header:
```c
// bus_output_helper SM management
void dma_set_bus_helper_sm(PIO pio, int sm);
int dma_get_bus_helper_sm();
PIO dma_get_bus_helper_pio();
```

Add to dma.c:
```c
static PIO bus_helper_pio = NULL;
static int bus_helper_sm = -1;

void dma_set_bus_helper_sm(PIO pio, int sm) {
    bus_helper_pio = pio;
    bus_helper_sm = sm;
}

int dma_get_bus_helper_sm() {
    return bus_helper_sm;
}

PIO dma_get_bus_helper_pio() {
    return bus_helper_pio ? bus_helper_pio : PIO_BUS_HELPER;
}
```

Call in dma_board.c after initialization:
```c
dma_set_bus_helper_sm(bus_helper_pio, bus_helper_sm);
```

### 3. Fix IRQ Handler to Push to bus_output_helper

**File**: `software/pico_victor/dma_ultra_fast.c`

**Change line 318-320** from:
```c
#ifndef BENCHMARK_MODE
    uint32_t response = (0xFF00u | (uint32_t)(value & 0xFFu));
    pio_sm_put_blocking(PIO_REGISTERS, REGISTERS_SM, response);
#endif
```

To:
```c
#ifndef BENCHMARK_MODE
    // Push data byte to bus_output_helper (not board_registers!)
    // bus_output_helper will output this on BD0-BD7 when it receives IRQ 1
    uint32_t response = (uint32_t)(value & 0xFFu);  // Just the data byte
    PIO helper_pio = dma_get_bus_helper_pio();
    int helper_sm = dma_get_bus_helper_sm();
    pio_sm_put_blocking(helper_pio, helper_sm, response);
#endif
```

**Same fix needed in `registers_irq_handler_ultra_asm`** around line 424 (need to check exact line).

### 4. Fix Standard IRQ Handler (Non-Ultra)

**File**: `software/pico_victor/dma.c`

**Change line 643-647** from:
```c
uint8_t data = dma_read_register(registers_ptr, masked_offset);
uint32_t pindirs_and_data = (0xFFu << 8) | (data & 0xFFu);

#ifndef BENCHMARK_MODE
pio_sm_put_blocking(pio, sm, pindirs_and_data); //send the data back to the 8088
#endif
```

To:
```c
uint8_t data = dma_read_register(registers_ptr, masked_offset);

#ifndef BENCHMARK_MODE
// Push data byte to bus_output_helper, not board_registers
PIO helper_pio = dma_get_bus_helper_pio();
int helper_sm = dma_get_bus_helper_sm();
pio_sm_put_blocking(helper_pio, helper_sm, (uint32_t)(data & 0xFFu));
#endif
```

## Timing Analysis

### Register Read Cycle Timing

```
T0: board_registers detects 0xEF300 match
T1: board_registers latches address
T2: board_registers pushes PREFETCH to ARM RX FIFO
    board_registers raises IRQ 0 to extio_helper

    ARM IRQ handler wakes (typical: ~500ns from IRQ assertion)
    ARM processes PREFETCH (~200-300ns with ultra handler)
    ARM pushes data to bus_output_helper TX FIFO (~50ns)
    Total ARM response time: ~750-850ns

T3: board_registers waits for READY (~800ns window available)
    board_registers signals IRQ 1 to bus_output_helper
    bus_output_helper pulls data from its TX FIFO
    bus_output_helper outputs on BD0-BD7
    8088 samples data

T4: board_registers signals IRQ 4 to bus_output_helper
    bus_output_helper tri-states BD0-BD7
```

**Timing window**: 800ns is sufficient for ARM to respond. The current ultra handler completes in ~500-700ns based on the benchmark data in the project.

### Potential Timing Issues

1. **FIFO underrun**: If ARM doesn't push data before IRQ 1, `bus_output_helper` will block on `pull block`
   - **Mitigation**: The `wait READY` in board_registers.pio:50 provides the 800ns window
   - **Risk**: Medium - should work but needs testing

2. **Multiple register reads**: If 8088 does back-to-back reads, could FIFO overflow?
   - bus_output_helper TX FIFO depth: 4 entries (default PIO FIFO)
   - **Risk**: Low - register reads are typically slower than ARM can respond

3. **IRQ latency variance**: ARM IRQ response isn't deterministic
   - Core1 runs in tight loop waiting for IRQ (core1_main, line 370-372)
   - IRQ handler is marked `__time_critical_func` (in RAM)
   - **Risk**: Low - latency variance is ~100-200ns, well within 800ns budget

## PIO Resource Allocation

After these changes, PIO allocation will be:

| PIO | SM | Program | Purpose |
|-----|-----|---------|---------|
| PIO0 | 0 | dma_read_write | DMA read/write cycles |
| PIO0 | 1 | bus_output_helper | BD0-A19 output arbiter |
| PIO1 | 0 | board_registers | Register address decode & timing |
| PIO2 | 0 | extio_helper | EXTIO/ control for register reads |

**GPIO Ownership**:
- BD0-A19 (GPIO 1-20): PIO0 (bus_output_helper and dma_read_write share via helper)
- RD, WR, etc. (GPIO 21-31): PIO0 (dma_read_write) and PIO1 (board_registers) both read as inputs
- EXTIO (GPIO 34): PIO2 (extio_helper)

## Testing Strategy

### Phase 1: Initialization Test
1. Add bus_output_helper init code
2. Verify PIO programs load without errors
3. Check that pindirs preload (0xFFF00) is in FIFO
4. **Expected**: No errors, FIFO should show 1 entry in bus_output_helper

### Phase 2: Register Write Test (Simplest)
1. Test 8088 writing to register (e.g., 0xEF380 = address low)
2. This bypasses bus_output_helper entirely
3. **Expected**: Writes work exactly as before

### Phase 3: Register Read Test (Critical)
1. Test 8088 reading from register (e.g., read 0xEF3A0 = address mid)
2. Use logic analyzer to verify:
   - T2: PREFETCH FIFO push
   - T2: ARM pushes to bus_output_helper
   - T3: IRQ 1 asserted
   - T3: BD0-BD7 driven with correct data
   - T4: IRQ 4 asserted, BD0-BD7 tri-stated
3. **Expected**: Correct data returned, timing within spec

### Phase 4: Status Polling Test (Most Common)
1. Repeatedly read 0xEF320 (status register)
2. Verify ARM can keep up with repeated reads
3. **Expected**: No FIFO underruns, consistent timing

### Phase 5: DMA Transfer Test
1. Test DMA write (pico → 8088 RAM)
2. Test DMA read (8088 RAM → pico)
3. **Expected**: Same behavior as current working code

## Open Questions

### Q1: Which PIO instance for bus_output_helper?

**Recommendation**: PIO0, SM 1

**Rationale**:
- dma_read_write is already on PIO0 SM 0
- Both need to output to BD0-A19
- Same PIO instance means they share GPIO mux configuration
- Avoids conflicts

**Alternative**: Could use PIO1 SM 1, but then need to manage GPIO ownership switching between PIO0 and PIO1

### Q2: What about GPIO ownership during DMA?

Currently (dma.c lines 71-96), DMA transfers disable register SM:
```c
pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, false);  // Line 71
// ... do DMA ...
pio_sm_set_enabled(PIO_REGISTERS, REGISTERS_SM, true);   // Line 96
```

With new architecture:
- **board_registers** can stay enabled (it only inputs on BD0-BD7 anyway)
- **bus_output_helper** needs to stay enabled (DMA uses it for output)
- **GPIO ownership** is managed by `setup_pio_instance()` calls

**Recommendation**: Remove the register SM disable/enable around DMA. Let all three SMs run concurrently. The IRQ protocol ensures correct coordination.

### Q3: What if ARM can't respond in time?

If ARM fails to push data before IRQ 1:
- `bus_output_helper` blocks on `pull block` (line 15)
- `board_registers` blocks on `wait READY` (line 50)
- 8088 bus stalls waiting for data

**Recovery**: None - system hangs. This is a critical failure.

**Mitigation**:
1. Ensure ARM IRQ priority is highest
2. Keep IRQ handler in time_critical section (already done)
3. Add timeout in production code (future work)
4. Monitor FIFO depth during development

## Summary of Required Code Changes

| File | Lines | Change Type | Description |
|------|-------|-------------|-------------|
| dma.h | Add | New defines | PIO_BUS_HELPER, BUS_HELPER_SM |
| dma.h | Add | New prototypes | bus_helper access functions |
| dma.c | Add | New functions | bus_helper getters/setters |
| dma.c | 643-647 | Modify | Change FIFO target to bus_helper |
| dma_board.c | ~106 | Add | Initialize bus_output_helper SM |
| dma_board.c | Add | Add | Call dma_set_bus_helper_sm |
| dma_ultra_fast.c | 318-320 | Modify | Change FIFO target to bus_helper |
| dma_ultra_fast.c | ~424 | Modify | Same fix in _asm variant |
| bus_output_helper.pio | N/A | Already done | PIO program ready |

**Estimated effort**: 2-3 hours coding + 4-6 hours testing

**Risk level**: Medium-High
- Core functionality change
- Timing-critical code path
- Multiple integration points
- Requires hardware testing

**Rollback plan**: Keep old PIO programs available, use `#ifdef NEW_ARCHITECTURE` guards during development

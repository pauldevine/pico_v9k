# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico RP2350 project that implements DMA (Direct Memory Access) functionality for interfacing with a Victor 9000 computer, a 1982 machine that uses an 8088. The PIO programming will be responsible for emulating the DMA Expansion Card, which in the legacy hardware spoke to a SASI Xebec 1410 controller that fronted MFM drives. The pico will be speaking SPI to a fujinet that will act as the storage unit. The pico will not need to emulate the full SASI protocol or the xebec, but instead react to the registers in the DMA card that are metadata for controlling the interaction as well as doing DMA to the 8088 to fulfill read and write operations against the virtual drive in the fujinet. The project uses PIO (Programmable I/O) state machines to handle bus timing and data transfer operations between the pico and the 8088. One state machine of the pio will be setup to listen to a memory mapped set of registers that control the SASI card. It receives metadata like what sector to be retrieved, where to place the resulting data in 8088 memory on a read, etc. The other state machine will be used to do actual DMA transfers against the 8088 memory space.

## Build Commands

Build the project:
```bash
cd build
cmake ..
make
```

Clean build:
```bash
rm -rf build/*
cd build
cmake ..
make
```

## Architecture

### Core Components

- **Main executable**: `dma_board.c` - The primary firmware that runs on the Pico
- **DMA Controller**: `pico_victor/dma.c` - Implements Victor 9000 DMA register emulation and SCSI/SASI bus control
- **PIO Programs** (`*.pio` files): Handle low-level bus timing and control
- `pico_victor/board_registers.pio` - Low-level bus interface using RP2350's PIO state machines
- `pico_victor/dma_read_write.pio`: Overall 8088 DMA read and Write bus interactions
- **FujiNet Integrations** (`pico_fujinet/spi.c`): Handles integrating with the FujiNet over SPI
- **Logging System**: `logging.c` - Non-blocking logging for timing-critical code

### Key Architecture Details

- **Dual-core operation**: Core 0 runs main logic, Core 1 handles IRQ processing for register access
- **PIO-based bus interface**: Uses PIO state machines to monitor the 8088 address/data bus
- **Register mapping**: DMA registers mapped at base address `0xEF300` with specific offsets:
  - `0x00`: Control register (write-only)
  - `0x10`: Data/Status register (read/write)
  - `0x80`: DMA address low byte
  - `0xA0`: DMA address middle byte
  - `0xC0`: DMA address high byte (4 bits)

### Hardware Configuration

- **Target board**: Pimoroni PGA2350 (RP2350-based)
- **Platform**: rp2350-arm-s
- **GPIO count**: 48 pins
- **Clock**: 200MHz system clock
- **Memory target**: Default firmware builds run from flash; select test targets (for example `test_dma_cleaned`) use the `no_flash` RAM configuration

## Development Workflow

1. The project uses CMake with the Pico SDK
2. PIO programs are automatically compiled to headers during build
3. UART output uses `uart0` with TX on GPIO 0 at 115200 baud; RX is not connected and GPIO 45 is repurposed for debugging/logic-analyzer triggers
4. The system includes timing-critical interrupt handlers marked with `__time_critical_func`

## Key Libraries Used

- `pico_stdlib` - Standard Pico SDK functions
- `pico_multicore` - Dual-core functionality
- `hardware_pio` - PIO state machine control
- `hardware_dma` - DMA peripheral access
- `pico_util` - Queue system for logging

## Testing

The main test runs in `test/register_test.c` and:
- Initializes dual-core operation
- Sets up PIO state machines for bus monitoring
- Waits for DMA register access from the Victor 9000
- Processes register reads/writes through interrupt handlers

## Important Notes
- NEVER EVER touch the pio files directly. Recommend courses of action only on those files. They are complex and you always mess them up!
- This is embedded firmware for hardware interface emulation
- Timing is critical - uses cycle-accurate measurements with SysTick
- The code includes Victor 9000-specific memory addressing and SCSI/SASI protocol handling
- Register access patterns follow MAME emulator implementation for compatibility
- **Direct GPIO connection**: The RP2350 is 5V tolerant, allowing direct connection to the 8088 bus without level shifters
- The 8088 uses a multiplexed address/data bus where AD0-AD7 carry address bits during T1 (when ALE is high) and data during T2-T3
- Pin direction management is handled directly through PIO `pindirs` instructions without external buffer control

## Current Status (2025-nov-16)
### Register Read Output SM Debugging
- **EXTIO sampling fixed**: Added an extra `nop` delay when capturing the bus address so the `jmp pin` gate in `board_registers_output.pio` samples EXTIO ~40 ns later; analyzer now shows the state machine only falls through to T3 when EXTIO is genuinely low.
- **Handshake swap**: Replaced the cross-PIO IRQ handshake with explicit `wait 0/1 GPIO EXTIO_PIN` sequencing. This avoids sticky IRQ flags and keeps the control/output SMs aligned without relying on `irq next`.
- **FIFO payload format**: `register_read_irq_isr()` now pushes a combined word (`0xFF00 | data`) so `out pins, 8` and `out pindirs, 8` read back-to-back bytes from the same payload. Shift direction confirmed correct (no byte-swapping needed even with 0xFFFFFFFF test vectors).
- **Open issue – bus release too early**: Despite the payload fix, BD0-BD7 drop back to input roughly 2 µs after the debug sideset pulse. Likely causes are (a) `mov pindirs, null` running immediately because EXTIO is already high, or (b) `dma_rw_output` reasserting its own `mov pindirs`. Need instrumentation to determine which path is clearing the bus.

### Immediate Next Steps
1. Guard the release with a `wait 0 GPIO EXTIO_PIN` before the existing `wait 1` so the output SM blocks until control explicitly re-asserts and then releases EXTIO.
2. Capture `PIO_OUTPUT->dbg_padout` and `PIO_OUTPUT->dbg_padoe` around the T3 window to verify what the PIO hardware believes it is driving when the analyzer sees the drop.
3. Temporarily halt `dma_rw_output` (disable SM1 or add an explicit wait) while exercising register reads to confirm it is not rewriting `pindirs`.
4. Once the data bus stays driven through T3, re-test vintage Victor register reads (e.g., 0xEF320) to ensure the cached ISR responds within the required 200 ns window.

## Previous Status (2025-nov-03)
### EXTIO line isolation fix for register reads
- **Root cause**: Read cycles were failing because the Victor CPU board never released BD0–BD7. The Pico wasn't asserting `EXTIO/`, so the host's 74LS245/373 buffers stayed enabled and overrode our register responses.
- **Fix**: Added a dedicated PIO helper (`pico_victor/extio_helper.pio`) running on PIO3 with a shifted GPIO base. The register listener SM now raises an IRQ on EF3xx hits, and the helper asserts `EXTIO/` through the whole T2/T3 read window, releasing it after `RD/` returns high. This finally let the Pico drive data back to the 8088.
- **Result**: Logic-analyzer traces now show the expected bidirectional hand‑off; the system reliably services read requests from the DMA register block.
- **Project status**: With register readback working, focus shifts to polishing DMA transfers and integrating the FujiNet path. Remaining tasks include validating CSEN timing for edge cases and resuming firmware clean‑up paused during the EXTIO chase.

## Current Status (2025-oct-25)
### GPIO Pin Migration - BD0 moved from GPIO0 to GPIO1
- **Hardware change**: After weeks of debugging GPIO0 misbehavior, the hardware was rewired to move the 8088 data bus (BD0-BD7) from GPIO0-7 to GPIO1-8
- **Root cause**: GPIO0 has special handling in the Pico SDK for default UART functionality, causing persistent conflicts with PIO control even after UART reconfiguration
- **Migration impact**: All pin definitions in `dma.h` updated to reflect BD0_PIN=1 (was 0), with all other pins shifted accordingly
- **Code fixes applied**:
  - Fixed `board_registers.pio` line 85: removed double-addition bug (`BD0_PIN + i` → `i`)
  - Fixed `dma.h` line 98: corrected `setup_pio_instance()` to start at `BD0_PIN` instead of `BD0_PIN + 1` (was skipping GPIO1)
  - Updated `dma_board.c` line 35: corrected comment to reflect BD0_PIN is now GPIO1
- **Status**: Pin initialization now correctly includes GPIO1 (BD0), side-set functionality should be restored
- **UART configuration**: UART TX remains on GPIO0, properly released before PIO initialization to avoid bus contention

## Previous Status (2025-oct-21)
- Python `fifo_trace_analyzer.py` now decodes the cached FIFO logs, flags out-of-range addresses, and correlates PREFETCH/COMMIT mismatches. Latest traces confirm the Pico occasionally commits register reads with an address outside the 0xEF300 window, matching logic-analyzer evidence that the register SM slips during T3.
- The logic analyzer shows ALE decode is stable but the read T3 window is erratic: data is sometimes driven too early/late, causing the Victor to sample the address byte (0x80/0xA0) instead of the programmed register contents. When the slip occurs once, subsequent cycles stay misaligned, explaining the persistent failures in `dmatest`.
- A 10 ns blip on BD0-BD7 during reads traced back to the default stdio UART still owning GPIO 0/1 (BD0/BD1). We need to disable the auto-UART (`pico_enable_stdio_uart(... 0)`, link `pico_stdio_uart` manually, and override `PICO_DEFAULT_UART_*` before `pico_sdk_init()` so stdio stays on TX=46 and stops fighting the PIO.
- The DMA SM remains disabled during these tests, ruling it out as the source of bus contention. Focus is on the register SM timing and ensuring nothing else reclaims the bus after `out pins/out pindirs`.
- Next steps: rebuild with the UART fix, re-run the analyzer to confirm no more out-of-range commits, and re-check the T3 timing under LA to see if the data bus now stays asserted through the read window. If the slip persists, instrument the PIO around the `out pins`/`out pindirs` instructions to pinpoint where the cycle loses alignment.

## Vintage Hardware Documentation

### Core Technical Manuals
- **SASI and DMA Card Hardware**: `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf` - Complete documentation of the hard disk subsystem including SASI interface and DMA controller
- **Hardware Reference Manual**: `notes/Manuals/Victor 9000 Hardware Reference Rev 0 - 10.5.1983.pdf` - Overall system technical documentation
- **256K CPU Theory of Operation**: `notes/Manuals/Victor 9000 Theory of Operation for the 256K CPU.pdf` - Detailed CPU architecture and operation
- **Service Manual**: `notes/Manuals/VictorServiceMan_Chapt1-5 OCR.pdf` - Field service and maintenance procedures
- **Systems Programmer's Toolkit**: `notes/Manuals/Victor 9000 Systems Programmer's Tool Kit II Vol II.pdf` - Low-level system programming reference

### BIOS and Boot Documentation
- **Boot BIOS Implementation Notes**: `notes/Boot BIOS ASM source 3.6/BT1INFO.DOC` - Detailed implementation notes for the boot BIOS
- **Boot BIOS Source Files**: `notes/Boot BIOS ASM source 3.6/` - Complete assembly source for boot BIOS
- **Difference Files**: `notes/Boot BIOS ASM source 3.6/DIFFER.TXT` and `DIFFER1.TXT` - Version differences in BIOS implementations

### Operating System Source Code
- **MS-DOS 3.1 Assembly Listings**: `notes/MS-DOS 3.1 Listings/` - Complete assembly and PL/M-86 source listings including:
  - Device drivers: `DIOV9000.LST`, `DIOVICKI.LST` (disk I/O)
  - Console I/O: `CI.LST`, `CO.LST`, `CONS.LST`
  - Buffer management: `BUFFER.LST`, `BUFFERFL.LST`, `BUFFERHD.LST`
  - System components: `BELV9000.LST`, `CBV9000.LST`, `ERRORMAP.LST`

### Additional Technical References
- **Schematics and Circuit Diagrams**:
  - `notes/Manuals/Victor 9000 Schematics Rev #11.pdf`
  - `notes/Manuals/Sirius Systems Technology Victor 9000 Schematics.pdf`
  - `notes/Manuals/Victor_9000_Schematics_revH.pdf`
- **Memory Architecture**: `notes/Manuals/Victor 9000 Memory Map.pdf`
- **Programmer's References**:
  - `notes/Manuals/Victor 9000 Programmers Toolkit Volume I.pdf`
  - `notes/Manuals/Victor 9000 Programmers Toolkit Volume II.pdf`
  - `notes/Manuals/Victor 9000 Applications Programmers Toolkit II Volume I.pdf`
  - `notes/Manuals/Victor 9000 Applications Programmers Toolkit II Volume II.pdf`
- **Technical Reference Disk**: `notes/Manuals/Technical Reference Disk in ascii/` - Contains detailed technical specifications in text format
- **June 1982 Technical Reference**: `notes/Manuals/Victor9000TechRef_Jun82_OCR.pdf` - Early technical reference documentation

## Mame Emulator for Reference
- Mame also has an emulation of the same hardware in the `notes/victor9k_hdc.cpp` file. It works today to emulate this behavior and is a good reference implementation. In `notes/mame boot example.log` there's the output from booting mame with hard drive register interaction logging enabled. You can see what the typical boot sequence looks like.

## Recent Development Status

### Current Branch: `pico_rewire_oct_b_25`
Hardware redesign branch to remove 74LVC245 level shifters and connect RP2350 directly to 8088 bus.

### Major Hardware Changes (October 2025)

#### 74LVC245 Buffer Removal
- **Hardware simplification**: Removed all 74LVC245 bidirectional buffers from the design
- **RP2350 5V tolerance**: Leveraging the RP2350's certified 5V-tolerant GPIO for direct 8088 bus connection
- **PIO code updates**: Modified both `dma_read_write.pio` and `board_registers.pio` to remove buffer control logic:
  - Removed `.side_set` directives that controlled buffer direction pins
  - Eliminated all references to LOW_ADDR_DIR and BUS_CNTRL_DIR pins
  - Simplified pin direction management using only PIO `pindirs` instructions
- **Code review completed**: Both PIO programs verified to correctly handle 8088 bus protocol without buffers

### Recent Accomplishments (September-October 2025)

#### DMA Read/Write PIO Program - Complete Rewrite
- **Major breakthrough**: After extensive debugging and troubleshooting efforts, successfully rewrote `dma_read_write.pio` from scratch
- **Validation complete**: Successfully tested bidirectional DMA transfers - wrote two characters to Victor 9000 RAM and read them back correctly
- **Critical timing**: The PIO code is highly timing-dependent at the nanosecond level. Small changes can have significant impact. The fundamental framework is now working and should not be modified without careful consideration and discussion first.
- **Bus arbitration resolved**: HOLD/HLDA signaling now working correctly after implementing proper handshaking sequences

#### Performance Optimizations (September 2025)
- **Ultra-fast DMA implementation** (`dma_ultra_fast.c/h`): New optimized register handler that eliminates function pointers for address registers (0x80-0xFF), using direct memory mapping for improved performance
- **Debug queue system** (`debug_queue.c/h`): Added non-blocking debug logging system for timing-critical code sections
- **Benchmarking infrastructure** (`test/benchmark_irq.c`): Created IRQ performance benchmarking tools to measure and optimize response times
- **Cache optimization**: Aligned register memory to cache lines and added pre-warming initialization routines

#### DMA Improvements
- **Timing debug pins**: Added GPIO pin toggling for precise timing measurements with oscilloscope
- **Fast DMA variants**: Implemented both `dma_fast.c` and `dma_ultra_fast.c` with progressively optimized register access patterns
- **LBA support**: Fixed and improved Logical Block Addressing handling in the DMA controller

#### SPI/FujiNet Integration
- **Protocol cleanup**: Streamlined SPI handshaking and communication with FujiNet device
- **Transaction phases**: Added proper SPI transaction phase management with `spi_end_transaction_phase()`
- **Mount process**: Fixed disk mounting process with increased timeout for slow operations
- **Error handling**: Improved timeout handling and error recovery in SPI communications

#### Testing Infrastructure
- **DOS DMA test** (`test/dos_dma_test/`): Added DOS-based DMA testing utilities
- **Register access patterns**: Documented and tested Victor 9000 register access sequences
- **Performance metrics**: Established baseline measurements for IRQ response times
- **test_dma_cleaned**: Minimal hardware DMA harness that validates HOLD/HLDA hand-off and a single read/write cycle
- **test_clock_check**: Simple on-device diagnostic that watches CLK5/CLK15 activity and basic GPIO wiring
- **benchmark_irq**: Measures register IRQ latency across representative Victor register transactions

### Current Work in Progress (as of October 13, 2025)

#### DMA Board + SASI Emulation Status

After reconciling the previous fix list with the current code, here is an updated, actionable view of what works and what still needs attention for reliable boot.

**What's Working:**
- ✅ DMA read/write PIO layer (validated with small read/write test)
- ✅ Register decode at `0xEF300` base with MAME-style low-bit masking
- ✅ Address register read-back (0x80/0xA0/0xC0)
- ✅ FujiNet SPI integration and sector I/O
- ✅ Command byte collection and basic opcode routing
- ✅ Single unified DMA state machine claimed in `dma_board.c`/`dma.c` and used for both read and write transfers
- ✅ Status register reads at 0x20 and 0x30 return the live SASI bus flags in both standard and ultra-fast handlers

**Updated Gap List (reconciled):**

1. Host IRQ signaling (Decision required)
- `dma_update_interrupts()` only logs; no Victor IRQ pin is driven. Polling may be sufficient for boot, but proper IRQ improves fidelity.
- Action: Either confirm polling-only boot path or map a chosen `IR_*` pin and assert/deassert alongside `dma_update_interrupts()` during command/status phases.

2. Command-phase REQ/ACK modeling (Medium)
- Command writes assert ACK but don’t explicitly drop REQ between bytes. While the current approach may work with polling, SASI expects REQ/ACK handshake per byte.
- Action: On each non-DMA command byte, set ACK and clear REQ in `bus_ctrl`, then re-assert REQ when requesting the next byte (via `sasi_request_cmd_byte`).

3. Data-phase model consistency (Clarification)
- The code uses direct data-phase handlers (`sasi.c`) and has an unused `dma_handle_sasi_req()` path. DMA transfers do occur; the previous note that DMA “never triggers” is not accurate.
- Action: Prefer the direct handler approach for now, or adopt the REQ-driven path consistently. Remove or comment the unused path to reduce confusion.

4. Command buffer ownership (Nice-to-have)
- `handle_sasi_command_byte()` uses static buffers; `dma->buffer` exists for this purpose.
- Action: Move command accumulation to `dma->buffer` for clarity/reentrancy (single target is fine today).

5. Reset semantics (Nice-to-have)
- `DMA_RESET_BIT` clears internal state but doesn’t reset the backing device.
- Action: Clear `bus_ctrl` to idle and optionally reinit/reset the selected FujiNet target on reset.

6. Mode Select(6) parameters (Nice-to-have)
- Parameter list is read but not interpreted. For boot, returning GOOD is typically fine.
- Action: Accept standard 512-byte sector configuration; log unsupported parameters for future mapping if needed.

7. Tests and boundaries (Recommended)
- Add tests that cross 64K boundaries (verify 20-bit carry) and confirm status reads at both 0x20 and 0x30 during boot traces.

Notes on previously listed items:
- “DMA transfer never triggered” — Not accurate: DMA transfers occur in `sasi.c` handlers.
- “Per-sector REQ pulsing” — Not required for host-visible status during DMA; the DMA board hides REQ/ACK during bus mastering. Keep as internal-only if you later emulate SASI wires explicitly.
- “Incomplete diagnostic support” — Command bytes are consumed by the collector; returning GOOD is acceptable for boot.

**Reference Documentation for Fixes:**
- SASI handshake: `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.txt` §3.1.4.2
- DMA board state machine: same document §2.1.3
- BIOS command sequences: `notes/mame boot example.log`
- Register layout: `notes/MS-DOS 3.1 Listings/SASIDMA.LST`

#### Immediate Next Steps
1. Wire single-SM usage through DMA helpers and SASI code; remove stale per-SM/pindirs toggles
2. Add status alias handling for 0x30 in both handlers (standard + ultra)
3. Decide on IRQ strategy; if polling-only, document that choice; otherwise map and drive an `IR_*` pin
4. Tighten command-phase REQ/ACK bookkeeping for robustness
5. Run extended 512B+ DMA tests and a boot smoke test; capture traces

### PIO GPIO Initialization Requirements

#### Critical Warnings
- **DO NOT MODIFY `dma_read_write.pio`** without careful consideration and discussion first. This code has been extensively debugged and tested at the nanosecond timing level. It is now working correctly for basic DMA operations.
- **Side-set note**: With `.side_set N opt` the assembler automatically sets the optional-enable bit for any instruction that uses a `side` value, so missing activity on DEN/ or IO/M/ almost always traces back to pin direction or pin mapping mistakes rather than the literal side value.

#### Critical Pin Initialization Rules (as of October 11, 2025)

**Output pins (controlled by PIO via pindirs):**
- A0-A19 (address bus) - pins 0-19
- RD/, WR/, DT/R/, ALE, DEN/ (control outputs) - pins 20-24
- HOLD/, IO/M/ (side-set outputs) - pins 25-26
- **Initialization**: Use `pio_gpio_init()` to give PIO full control including pin direction

**Input-only pins (read by PIO wait instructions):**
- READY - pin 27
- HLDA - pin 28
- CLOCK_5 - pin 29
- CLOCK_15B - pin 30
- IR pins - pins 31+
- **Initialization**: MUST use `pio_gpio_init()` followed by `gpio_set_dir(pin, GPIO_IN)`
- **Critical RP2350 Issue**: Despite SDK documentation stating pio_gpio_init is only needed for outputs, on RP2350 it is REQUIRED for all pins that PIO needs to read with wait or IN instructions
- **Symptom if not done correctly**: PIO wait instructions will hang forever even though ARM can read the pin correctly

**Common initialization mistakes:**
1. Calling `gpio_init()` on pins that PIO needs to read - this sets the function to GPIO_FUNC_SIO, disconnecting from PIO
2. NOT calling `pio_gpio_init()` on input pins - on RP2350 this is required despite SDK docs
3. Test/debug code interfering by re-initializing pins after PIO setup
4. Missing the function routing entirely - PIO wait instructions will hang forever

### Previously Resolved Issues (Historical Reference)

These issues have been fully resolved but are documented here for historical context and to help understand the evolution of the project:

- **RP2350 PIO side-set pindirs initialization bug**: On RP2350, the PIO side-set pindirs functionality doesn't work until the state machine has executed at least one explicit pindirs instruction.
  - **Workaround**: Execute `pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, 0))` after `pio_sm_init()` to "unlock" side-set pindirs
  - **Symptoms**: DBG_PADOE remains at 0x00000000 despite side-set pindirs operations in the PIO program
  - Without this workaround, no pins will be driven as outputs even though the side-set configuration is correct
  - This appears to be an undocumented RP2350 silicon behavior

- **PIO `wait` instruction**: Now works correctly once the pindirs issue is resolved
  - The apparent wait instruction bug was actually caused by HOLD never being asserted due to the pindirs issue
  - Once pindirs are working, the `wait gpio` instructions function properly

- **DMA Write Failure**: Writes to Victor RAM were not completing successfully
  - Data written wasn't appearing in Victor memory when checked via debugger
  - Read operations were returning sequential data (00,01,02,03...) instead of test pattern
  - Bus arbitration HOLD/HLDA signaling required jmp pin workaround
  - **Resolution**: After a complete rewrite of `dma_read_write.pio`, successfully validated writing two characters to Victor RAM and reading them back correctly. The fundamental DMA read/write protocol is now functioning.

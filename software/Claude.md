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
- **Memory target**: Runs from RAM (no_flash configuration)

## Development Workflow

1. The project uses CMake with the Pico SDK
2. PIO programs are automatically compiled to headers during build
3. UART output is enabled on UART1 (pins 4/5) at 115200 baud
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

- This is embedded firmware for hardware interface emulation
- Timing is critical - uses cycle-accurate measurements with SysTick
- The code includes Victor 9000-specific memory addressing and SCSI/SASI protocol handling
- Register access patterns follow MAME emulator implementation for compatibility
- **Direct GPIO connection**: The RP2350 is 5V tolerant, allowing direct connection to the 8088 bus without level shifters
- The 8088 uses a multiplexed address/data bus where AD0-AD7 carry address bits during T1 (when ALE is high) and data during T2-T3
- Pin direction management is handled directly through PIO `pindirs` instructions without external buffer control

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
- **test_dma_hardware**: Validates DMA read/write operations with 100-byte test patterns
- **test_dma_diagnostic**: Comprehensive hardware diagnostic that checks:
  - Bus signal verification (HOLD/HLDA handshaking)
  - Clock signals (CLK5, CLK15B)
  - GPIO pin states for all data/address/control lines
  - Direct GPIO pin direction management
  - PIO state machine progression and error states

### Current Work in Progress (as of October 13, 2025)

#### Immediate Next Steps
1. **Integration validation**: Need to pull back in the overall project and validate the changes to the PIO protocol have been updated throughout the codebase
2. **Extended DMA testing**: Test writing and reading larger blocks (512 byte sections) of memory with test sequences to validate robustness
3. **Register subsystem testing**: Validate the disk register handling and SASI command processing. This was in progress when focus shifted to fixing the DMA read/write layer.

#### Integration Priorities
- Ensure all C code that interfaces with `dma_read_write.pio` is updated to match the new protocol
- Verify timing margins and add additional cycle delays if needed for reliability
- Test full boot sequence from Victor 9000 BIOS through to DOS disk operations

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

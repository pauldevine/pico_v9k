# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Raspberry Pi Pico RP2350 project that implements DMA (Direct Memory Access) functionality for interfacing with a Victor 9000 computer, a 1982 machine that uses an 8088. The PIO programming will be responsible for emulating the DMA Expansion Card, which in the legacy hardware spoke to a SASI Xebec 1410 controller that fronted MFM drives. The pico will be speaking SPI to a fujinet that will act ast the storage unit. The pico will not need to emulate the full SASI protocol or the xebec, but instead react to the registers in the DMA card that are metadata for controlling the interaction as well as doing DMA to the 8088 to fullfil read and write operations against the virutal drive in the fujinet. The project uses PIO (Programmable I/O) state machines to handle bus timing and data transfer operations between the pico and the 8088. One state machine of the pio will be setup to listen to a memory mapped set of registers that control the SASI card. It receives metadata like what sector to be retrieved, where to place the resulting data in 8088 memory on a read, etc. The other state machine will be used to do actual DMA transfers against the 8088 memory space.

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
- **DMA Controller**: `dma.c/dma.h` - Implements Victor 9000 DMA register emulation and SCSI/SASI bus control
- **PIO Programs** (`*.pio` files): Handle low-level bus timing and control
- `board_registers.pio` - Low-level bus interface using RP2350's PIO state machines
- `dma_read_write.pio`: Overall 8088 DMA read and Write bus interactions
- **Logging System**: `logging.c/logging.h` - Non-blocking logging for timing-critical code

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

## Mame Emulator for Reference
- Mame also has an emulation of the same hardware in the victor9k_hdc.cpp file. It works today to emulate this behavior and is a good reference implementation.
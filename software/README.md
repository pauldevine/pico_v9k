# Victor 9000 DMA Replacement Firmware

This project turns a Pimoroni PGA2350 (RP2350) board into a modern replacement for the Victor 9000/Sirius 1 DMA expansion card. The firmware emulates the original DMA registers, arbitrates the 8088 bus, and streams disk data from a FujiNet device over SPI so the vintage system can boot and run without the original hard-disk controller.

## Who This Project Serves
- Victor 9000 and Sirius 1 owners who need a functional hard-disk subsystem
- Enthusiasts who already use FujiNet on other retro systems and want to add the Victor 9000 to their setup
- Developers interested in RP2350 PIO/DMA techniques for vintage bus emulation

> ⚠️ **Status:** Active development. Core DMA transfers, register decoding, and basic FujiNet integration work, but expect rough edges and limited diagnostics while the project matures.

## Hardware You Will Need
- Pimoroni PGA2350 (RP2350) board or equivalent RP2350 hardware running at 200 MHz
- Victor 9000/Sirius 1 host with access to the DMA expansion slot edge connector
- FujiNet device (SPI-capable variant) with storage images configured for the Victor
- Level-safe wiring harness from the RP2350 GPIO to the Victor bus and FujiNet SPI header
- USB connection to a development machine for flashing and logging

The RP2350 is 5 V tolerant on the GPIOs used here, so no level shifting is normally required. Ensure the wiring preserves the original DMA card pinout (address/data bus, ALE, RD/, WR/, DEN/, HOLD/, HLDA, READY, etc.) exactly; incorrect mappings will prevent the 8088 from releasing the bus.

## Firmware Capabilities
- Mirrors the EF300h DMA register map expected by the Victor BIOS and MS-DOS 3.1
- Uses one PIO state machine for register snooping and one for 8088 DMA bursts
- Emulates SASI control/status handshakes on behalf of the legacy Xebec 1410 controller
- Streams 512-byte sectors between the Victor and FujiNet-backed virtual media
- Provides non-blocking UART logging on GPIO 46 at 115200 baud for bring-up and tracing

## Building the Firmware
Prerequisites: CMake, a C compiler toolchain, and the Pico SDK available to the build environment.

```bash
mkdir -p build
cd build
cmake ..
make
```

The resulting UF2/ELF artifacts in `build/` can be flashed with your usual RP2350 programming flow (e.g., `picotool load --family=rp2350 build/dma_board.uf2` while the board is in BOOTSEL mode).

To perform a clean build, remove the cached CMake files before re-running configure:

```bash
rm -rf build/*
cd build
cmake ..
make
```

## Installing and Wiring
1. **Flash the PGA2350** with the freshly built firmware.
2. **Cable the Victor bus** to the matching RP2350 GPIO pins as documented in `pico_victor/board_registers.pio`. Use short, shielded runs where possible to keep the 8 MHz bus stable.
3. **Connect FujiNet** to the SPI pins defined in `pico_fujinet/spi.c` (SCK, MOSI, MISO, CS, and IRQ lines). Confirm the FujiNet card image is configured with a Victor-compatible disk image.
4. **Apply power** to the Victor 9000; the RP2350 should come up with system power and immediately begin presenting the DMA register window at EF300h.
5. **Monitor UART** output on GPIO 46 for status messages during initial boot attempts.

If the Victor fails to start DMA transfers, double-check HOLD/HLDA wiring and confirm the READY line is being sampled by the RP2350 (the PIO will stall forever if READY is not initialized with `pio_gpio_init()`).

## Operating Tips
- Register status is accessible at both `0xEF320` and `0xEF330`; some BIOS paths probe both locations.
- The firmware currently assumes standard 512-byte sectors; unusual formats may need additional logic.
- SASI diagnostic commands beyond simple reads/writes return a generic GOOD status while deeper emulation is still in progress.
- Timing is tight—avoid adding blocking logging or USB operations while the Victor bus is active.

## Troubleshooting & Known Limitations
- **DMA never starts:** Verify the control register shadow matches the BIOS writes and that DMA enable is asserted (`0x01` bit). Compare with logic-analyzer traces if available.
- **No SASI handshakes:** Ensure FujiNet is powered and its CS/IRQ lines are mapped correctly; REQ/ACK activity is hidden during bus mastering by design.
- **Cross-boundary transfers:** Testing across 64 KB boundaries is ongoing; report anomalies with captured register logs.
- **Interrupt signalling:** The original DMA board could raise an IRQ; this firmware currently relies on polling. Interrupt support is planned but not yet committed.

## Learning More
- `Claude.md` — detailed developer notes, hardware caveats, and debugging history
- `Technical Overview of the Project.md` — deeper architectural background on the Victor DMA subsystem
- `notes/Manuals/` — scanned Victor 9000 technical manuals referenced throughout development
- `test/dos_dma_test/dmatest.c` — example harness used to exercise register reads/writes on the RP2350

Community feedback, trace captures, and bug reports are welcome. Please document wiring, FujiNet firmware versions, and Victor ROM revisions when sharing results so issues can be reproduced.

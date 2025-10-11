# Technical Overview of the Victor 9000 DMA Replacement Project

## Project Goals and Current Direction
Retrofit the Victor 9000/Sirius 1 hard-disk subsystem with a modern controller built around a Raspberry Pi RP2350 (Pimoroni PGA2350 board). The new design must emulate the original DMA expansion card well enough that the legacy BIOS, MS-DOS 3.1, and application stack continue to run unmodified while storage is backed by a Fujinet device over SPI. Two PIO state machines are envisaged: one to mirror the DMA board’s memory-mapped control/status registers and another to stream DMA bursts on the 8088 bus.

## Legacy System Primer

### Victor 9000 Architecture Highlights
- The Victor 9000/Sirius 1 is based on an Intel 8088 CPU with an 8-bit multiplexed external bus, backed by up to 896 KB RAM and standard expansion slots for peripherals such as the DMA board.[^ref-ch1]
- Its hard-disk subsystem paired the host with a Xebec S1410 SASI controller via a dedicated DMA expansion board that decoded the EF30h I/O page and arbitrated bus ownership.[^ref-victor-dma]
- Because the 8088 multiplexes address/data lines, the DMA hardware latches A0–A7 on ALE, controls buffers, and responds to `RD/` and `WR/` strobes to place bytes on the host bus—behaviour the replacement must preserve.[^ref-8088-bus][^ref-victor-dma]

### Original DMA Board Responsibilities
- Address decoding: jumpers on the board default to base address EF300h, producing chip-select strobes such as `WS-B0/` and `RS-C0/` that gate register writes and reads.[^ref-victor-dma]
- Data path: an 8-bit bidirectional buffer (IC 1K) bridges the host AD bus to internal latches; write strobes (`DLATCH/`, `T4/`, `HWR/`) pick the correct capture edge for on-board, expansion, or CPU cycles.[^ref-victor-dma]
- Control register: write-only latch 2J drives DMA enable, controller select, reset, and other board functions; additional logic ensures DMA is disabled after system reset.[^ref-victor-dma]
- SASI handshake: the board must honour `SEL`, `BUSY`, `REQ`, and `ACK` timing for the S1410 controller, presenting command/data phases distinguished by the `C/D` and `I/O` lines.[^ref-victor-handshake]
- Status passes back completion/error bytes after each command, and the host issues six-byte Device Control Blocks (DCBs) specifying logical block addresses and transfer parameters.[^ref-victor-controller]

### SASI/Xebec Controller Interfaces
- The S1410 provides interlocked data transfer, a sector buffer (256/512 bytes), ECC, and Seagate ST506-compatible drive signals. Host-side connector P2 carries the SASI data (DB0–DB7) plus `BUSY`, `ACK`, `RST`, `MSG`, `SEL`, `C/D`, `REQ`, and `I/O` lines the RP2350 design must reproduce electrically.[^ref-victor-controller]
- Host commands (DCBs) and completion status follow handshakes where each `REQ` from the controller must be answered with an `ACK` within ~500 ns typical, ensuring deterministic byte pacing.[^ref-victor-handshake]

### Legacy Software Surfaces
- MS-DOS 3.1 BIOS listings, including `SASIDMA.LST`, describe software expectations of the DMA interface (`notes/MS-DOS 3.1 Listings/`). BIOS boot ROM sources (e.g., `notes/Boot BIOS ASM source 3.6/BT1HDDVC.ASM`) reveal startup probes and fallback paths; they should guide register emulation strategy.
- Additional diagnostics (e.g., `HDFIELD`, `DMATEST`) referenced in the manuals validate DMA and controller paths; preserving their observable behaviour will aid bring-up.[^ref-ch8]

### SASI/DMA Register Map (decoded from BIOS)
The MS-DOS PL/M module `SASI_DMA` defines the hardware layout the firmware expects at base address EF300h.[^ref-sasidma-plm] The structure below is 16-byte aligned, matching the etched address decoding on the physical board.

| Offset | Register | Access | Purpose | Relevant Bits |
| - | - | - | - | - |
| +0x00 | `CTL` | write-only | Control latch shadowed in software | `BIT_DMA_Enable` (0×01) arms DMA bus mastering, `BIT_CPU_Lockout` (0×02) isolates CPU, `BIT_DMA_Strobe` (0×04) toggles write sequencing, `BIT_DMA_Dir` (0×08) selects read (0) vs write (1), `BIT_Select` (0×10) asserts controller `SEL/`, `BIT_Reset` (0×20) pulses SASI reset |
| +0x10 | `CSD` | read/write | Command, status, and data byte port | Driven according to bus state machine; handshake matched to `BIT_Command`, `BIT_Input_Mode`, etc. |
| +0x20 | `BUS` | read-only | Snapshot of SASI bus phase | `BIT_Input_Mode` (0×01), `BIT_Command` (0×02), `BIT_Busy` (0×04), `BIT_Request` (0×08), `BIT_Message` (0×10); firmware masks with `0x1F` for phase decoding |
| +0x80 | `Low` | write/read | DMA target address bits 7–0 | Latched before enabling DMA |
| +0xA0 | `Mid` | write/read | DMA target address bits 15–8 | |
| +0xC0 | `High` | write/read | DMA target address bits 19–16 | Assembles 20-bit physical address used by 8088 DMA cycles |

Software mirrors every control-port write in `Control_Image` so it can reconstruct the shadow register before each update, a behaviour the emulator should replicate to stay consistent with legacy expectations.[^ref-sasidma-plm]

## Replacement Hardware Platform

### RP2350 System-on-Chip Capabilities
- Dual Cortex-M33 / Hazard3 cores share a symmetric AHB5 fabric, hardware boot ROM, and configurable low-power states—useful for splitting bus servicing from storage protocol handling.[^ref-rp2350-soc]
- Three PIO blocks (four state machines each) deliver deterministic bit-level IO, with per-state-machine FIFOs, scratch registers, and interrupt routing ideal for cycle-accurate 8088 bus interactions.[^ref-rp2350-pio]
- The 16-channel DMA controller can issue one read and one write per clock, supports 32-bit transfers, address increment/reverse options, and chainable control blocks—suitable for moving sector payloads between PIO FIFOs and RAM without core involvement.[^ref-rp2350-dma]

### Pimoroni PGA2350 Carrier Board
- Exposes the RP2350B’s full GPIO matrix, power rails (IOVDD 1.8–3.3 V, USB/ADC 3.3 V, VREG VIN 2.7–5.5 V), and QSPI pads, while providing 3.0–5.5 V VBUS input and labelled pin rows that simplify ribbon-cable breakouts to the Victor backplane.[^ref-pga2350]
- The schematic highlights dedicated analog rails (`ADC_AVDD`, `ADC_VREF`) and convenient test pads, informing grounding and level-shifting strategy when interfacing with the 5 V Victor bus.

## Proposed Emulation Architecture
- **Bus front-end PIO state machine**: sample `ALE` to latch address bits, replicate the DMA board’s decode to expose the EF30h register file, and present read data in the T2 window. The same state machine can generate `ACK` strobes aligned with controller handshakes.
- **DMA transfer PIO state machine**: assert bus request/hold-equivalent signals, stream bytes using the 8088 ready cycle timing, and coordinate with the RP2350 DMA engine to push/pull sector buffers from memory, mirroring the original board’s `DRD/` and `DWR/` phases.
- **Core firmware responsibilities**: service Fujinet SPI transactions, translate SASI command phases, maintain DCB/state machines, and expose diagnostics hooks that mimic original logging (e.g., completion status bytes).

### PIO Handshake Targets
- Controller selection requires `SEL/` low and address bit `DB0` asserted before the controller raises `BUSY/`; `SEL/` must be released before command completion.[^ref-victor-handshake]
- Command/data burst pacing follows a `REQ`/`ACK` handshake with ≤250 ns setup before `ACK`, and acknowledgement must drop after each `REQ` returns high; message phases pull `MSG/` low until the final status byte is acknowledged.[^ref-victor-handshake]
- BIOS polls `BUS` for composite states (`Data_In_State`, `Command_Out_State`, etc.) computed from the `0x1F` mask, implying the emulation must drive those bits synchronously with `REQ` transitions to avoid firmware timeouts.[^ref-sasidma-plm]
- RP2350 PIO code should therefore gate each byte transfer on GPIO waits mirroring the documented CLK5/CLK15B edges and align `ACK` strobes with the <500 ns typical window described in the subsystem manual, ensuring deterministic behaviour when the controller toggles between command, data, and status phases.[^ref-victor-handshake]

## Repository Orientation
- `pico_victor/`: Pico-side source (`dma.h`, PIO programs) implementing bus logic and DMA orchestration.
- `pico_victor/tests/`: Timing and unit tests for PIO state machines (e.g., `test_clock_sideset.c`).
- `dma_board.c`, `sasi.c`: Host emulation scaffolding for the Victor side of the link.
- `notes/`: Full archival library, including manuals, schematics, and software listings referenced below.

## Reference Library (key starting points)
- `notes/Manuals/Technical Reference Disk in ascii/CH1.txt` – General system overview, memory map, expansion philosophy.[^ref-ch1]
- `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf` – Primary reference for DMA board schematics, timing, SASI protocol, and Xebec controller behaviour.[^ref-victor-dma][^ref-victor-handshake][^ref-victor-controller]
- `notes/Manuals/8088 Bus good explanation.pdf` – Detailed description of 8088 bus multiplexing, cycle timing, and latch requirements.[^ref-8088-bus]
- `notes/Manuals/rp2350-datasheet.pdf` – Comprehensive RP2350 feature set, including PIO and DMA chapters needed for deterministic bus emulation.[^ref-rp2350-soc][^ref-rp2350-pio][^ref-rp2350-dma]
- `notes/Manuals/Pimoroni_PGA2350_Schematic.pdf` – Carrier-board pinout, power domains, and expansion headers for physical integration.[^ref-pga2350]
- `notes/MS-DOS 3.1 Listings/` – BIOS/MS-DOS source listings (e.g., `SASIDMA.LST`, `HDVIO.LST`) revealing software-visible register semantics.
- `notes/Boot BIOS ASM source 3.6/` – Boot ROM assembly for power-on sequencing and disk detection logic requirements.
- `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf` Appendix sections – Diagnostic utilities (`HDFIELD`, `DMATEST`) and maintenance workflows useful for validation.[^ref-ch8]

## Next Research/Implementation Steps
1. Derive an exact register map and expected side-effects from the BIOS listings; cross-check against DMA board schematics.
2. Prototype PIO programs that reproduce ALE-driven latching and SASI handshake timings; validate with logic-analyser captures against the documented waveforms.
3. Design level-shifting/interface circuitry guided by the PGA2350 schematic and Victor bus voltage requirements.
4. Build integration tests that replay MS-DOS driver sequences (e.g., reproducing `SASIDMA` routines) to ensure behavioural fidelity.

---

[^ref-ch1]: `notes/Manuals/Technical Reference Disk in ascii/CH1.txt`, System overview and expansion summary (§1.1–1.3).
[^ref-victor-dma]: `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf`, DMA board functional description (§2.1) and address decoding notes.
[^ref-8088-bus]: `notes/Manuals/8088 Bus good explanation.pdf`, Intel 8088 bus multiplexing and timing concepts (pp. 1–4).
[^ref-victor-handshake]: `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf`, Controller-host handshake timing (§3.1.4, Figures 3.6–3.8).
[^ref-victor-controller]: `notes/Manuals/Victor 9000 Sirius 1 Hard Disk Subsystem.pdf`, S1410 controller specifications and DCB format (§3.1.1–3.1.5).
[^ref-ch8]: `notes/Manuals/Technical Reference Disk in ascii/CH8.txt`, Diagnostic tooling references for the hard disk subsystem (§8.4).
[^ref-rp2350-soc]: `notes/Manuals/rp2350-datasheet.pdf`, Chip overview and dual-core architecture (§1.1).
[^ref-rp2350-pio]: `notes/Manuals/rp2350-datasheet.pdf`, PIO overview and capabilities (Chapter 11).
[^ref-rp2350-dma]: `notes/Manuals/rp2350-datasheet.pdf`, DMA controller architecture (§12.6).
[^ref-pga2350]: `notes/Manuals/Pimoroni_PGA2350_Schematic.pdf`, RP2350B carrier schematic (pp. 1–2).
[^ref-sasidma-plm]: `notes/MS-DOS 3.1 Listings/SASIDMA.LST`, PL/M definition of SASI/DMA hardware ports (pp. 3–5).

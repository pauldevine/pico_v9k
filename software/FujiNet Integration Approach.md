# FujiNet Integration Approach for the Victor 9000 DMA Board

Status: proposal, September 2026. No code yet. This is the plan for re-attaching the
working RP2350 DMA/SASI board to FujiNet the way the FujiNet project now brings up new
platforms.

## 0. What this is based on

Read for this document (all current as of 2026-09-03):

- **`fujinet-manuals/firmware-platform-bringup-guide`**, the official *FujiNet Platform
  Bring-Up Guide* (Revision 4, June 2026), linked from
  [fujinet.online/developer-manuals](https://fujinet.online/developer-manuals/). Its
  closing design exercise is an 8-bit ISA (8088) bus, the nearest thing to the Victor
  anyone upstream has written down. Also from the same repo: the MS-DOS INT F5 Technical
  Reference, the MS-DOS Getting Started guide, the Intellivision Programmer's Guide,
  *Writing Cross-Platform FujiNet Apps*, and *Connecting an Emulator to FujiNet-PC*.
- **`fujinet-bringup`**: the "start here" repo the guide points at (byte relay + `iotest`).
- `FujiNetWIFI/fujinet-firmware` master (last commit 2026-09-03): `lib/bus/rs232`,
  `lib/bus/bus.h`, `lib/device/rs232`, `lib/media/rs232`, `lib/hardware/ACMChannel.*`,
  `pico/intellivision`, `build-platforms/platformio-fujiversal-*.ini`,
  `include/pinmap/fujiversal-*.h`, `fujinet_pc.cmake`.
- `fujinet-firmware.wiki`: FEP-004, Intellivision Mailbox Protocol, DIY FujiNet for the
  Intellivision, MSX Technical Overview, RS232 Quickstart, Definition of Done, Development
  Guidelines, Software Changes for New Platforms.
- `FozzTexx/fujiversal` (RP2350 DBC firmware for MSX and CoCo) and
  `FozzTexx/fujiversal-pcb-prototype` (Core2350B + ESP32-S3 bring-up board).
- `fujinet-hardware` (`INTV/FujiNet-INTV-Rev0`, `MSX/Prototype-2`).
- `fujinet-msdos` (formerly `fujinet-rs232`: `fujinet.sys`, INT F5, `fujicom`) and
  `fujinet-lib/msdos`.
- **`FozzTexx/fujinet-lib-experimental`** (last commit 2026-09-03): the FujiBus-native
  client library the bring-up guide calls the template for a new platform, plus its
  `testing/` on-machine integration suite.
- This repo's README, Technical Overview, CLAUDE.md, `pico_fujinet/spi.*`, `pico_storage/*`,
  `sasi.c`, and the KiCad schematic net list.

Not reachable from the build container: `~/Documents/Victor9k/FujiNet/`, the Discord
exports, `mastodon.fozztexx.com`, and msx.org. Two wiki-versus-code notes: the
Intellivision mailbox wiki page still describes
`rs232Disk::mountROM()`, which moved to `lib/media/rs232/diskTypeROM.cpp` in August 2026;
and the `fujinet-bringup` README promises an `rp2350/` relay directory that does not exist
in the repo yet (only `esp32/` and `iotest/` do). Where documents disagree, this follows
the code.

## 1. Where the FujiNet project is going

### 1.1 "Fujiversal" is real and is the sanctioned path for new platforms

FEP-004 (FozzTexx, October 2025, v1.1 November 2025) is the design document:

> Future FujiNet devices will use a Raspberry Pi RP2040 or RP2350 as the physical bus
> interface, with an ESP32 or host computer managing device emulation and network
> communication.

The Platform Bring-Up Guide turns that into a method. The bus-side MCU is the DBC (Data
Bus Controller), the ESP32-S3 is the "FujiNet core", and the link between them is FujiBus
packets, SLIP-framed, over USB CDC-ACM with the ESP32-S3 as USB host and the RP2xxx as USB
device. The guide calls this the "tandem design" and states the payoff plainly:

> For a tandem bus-based platform you usually write no new bus class and no new device
> classes on the ESP32. You add a build target and a pin map, and reuse rs232.

Evidence this is mainline, not an experiment:

- `fujinet-firmware` ships `fujiversal-rs232`, `fujiversal-drivewire` (CoCo) and
  `fujiversal-intv`, all `esp32-s3-wroom-1-n16r8` with the USB host stack enabled.
- `lib/hardware/ACMChannel.cpp` is the ESP32-S3 USB host driver. It takes the first
  CDC-ACM function it finds (May 2026) and survives the RP2xxx resetting (August 2026).
- `pico/intellivision/` in `fujinet-firmware` is a vendored RP2350 cartridge firmware whose
  FujiNet half (`fujibus.c`, `fujibus_usb.c`, `fujinet.c`, `fuji_mailbox.h`) is a reference
  DBC client in plain C.
- `fujinet-hardware/INTV/FujiNet-INTV-Rev0` is an all-in-one board: RP2354A on the console
  bus, ESP32-S3-WROOM-1-N16R8 as USB host, linked on-board over native USB, one external
  USB-C via CP2102N. Unrouted, but it is the template for production fujiversal hardware.
- `fujiversal-pcb-prototype` is the bench rig: Waveshare Core2350B + Freenove ESP32-S3 CAM,
  with an ISA-8-bit-shaped universal bus header and CoCo/MSX adapters.
- The manuals on fujinet.online (MS-DOS, Intellivision, bring-up) are all dated 2026 and
  generated against current source, so they track the code closely.

### 1.2 The firmware refactor that makes this cheap (July to September 2026)

- `SystemBusBase` (`lib/bus/bus.h`) is the only contract a device may use:
  `transaction_accept(NO_GET|WILL_GET)`, `transaction_get()`, `transaction_send()`,
  `transaction_success()`, `transaction_error()`. Every bus inherits it. Devices never
  touch transports.
- Each bus has a packet class satisfying the `FujiPacketLike` concept; `fujiDevice` and its
  mixins (AppKey, Base64, Hash, QR) are shared, and `tests/check_no_build_ifdefs.py`
  forbids `BUILD_*` ifdefs inside `lib/device/fujiDevice`.
- `DaisyChain` centralizes device IDs; `fujiDeviceID_t` and `fujiCommandID_t` are
  `enum class` as of last week.

Consequence: the `fujiversal-rs232` build has no idea what computer is on the far end of
the USB link. The Intellivision uses it unchanged. So can the Victor.

### 1.3 The bring-up method the project now prescribes

From the guide and `fujinet-bringup`, in order:

1. **Two-way bytes first.** Port `iotest` (a tiny host program with a `portio` contract:
   `port_init`, `port_available`, `port_getc`, `port_getc_timeout`, `port_getbuf`,
   `port_putc`, `port_putbuf`) to the machine, and put a dumb byte relay on the
   microcontroller. Press a key on the retro machine, see it on a USB terminal, and back.
2. **Hello World against a PC build.** Point the relay's USB port at fujinet-firmware
   built for the desktop (`fujinet-pc`, `FUJINET_TARGET=RS232`, board
   `fujinet-lwm-rs232`) and fetch the adapter config. No ESP32 involved yet.
3. **Then the tandem design**: PIO for the bus, byte pipe + emulated boot ROM on the
   RP2350, stock `fujiversal-rs232` on the ESP32-S3, a `fujinet-lib` backend and a
   `fujinet-config` target on the host.

The guide's two design decisions:

- **Decision 1: ride an existing disk interface, or speak FEP-004 directly?** "Favor
  whatever boots from bare metal."
- **Decision 2: ESP32 or RP2350 on the bus?** More than 8 signal lines means RP2350, which
  takes 5 V directly. (The guide's first edition got this wrong and now says so.)

Its milestone ladder: M0 boards seated; M1 RP2350 enumerates as CDC; M2 ESP32 alive; M3
byte-pipe loopback; M4 address decode fires only on our cycles; M5 first FujiBus ACK; M6
CONFIG boots; M7 mount and boot a disk; M8 `N:` works. Milestones 1 to 5 need no bus
adapter and no ESP32.

### 1.4 The canonical DBC is a transparent byte pipe

In `fujiversal` (and the guide's Chapter 8), the RP2350 does not parse FujiBus at all:

> SLIP framing is the business of the two endpoints, the host's 8-bit client library and
> the ESP32, not the RP2350, which is a transparent byte pipe between them.

The host sees four registers: `GETC`, `STATUS` (bit = byte available), `PUTC`, and
`CONTROL` (host-to-RP2350 commands, today "activate the loaded ROM image"). Offsets and
bit positions are per-platform choices (MSX: `0xBFFC`, avail = 0x80; CoCo: `0xFF41`, avail
= 0x02). The one exception is device ID `0xFF` (DBC): `fujiversal/main.cpp` watches the
stream for a SLIP frame addressed to `0xFF` and handles it locally (ROM upload, reset),
forwarding everything else untouched. The Intellivision cartridge is the other style, a
proxy mailbox where the RP2350 builds the packets itself, chosen because the CP-1610 side
is IntyBASIC with PEEK/POKE only.

### 1.5 The FujiBus wire format (FEP-004 as implemented)

`lib/bus/rs232/FujiBusPacket.cpp` is authoritative; `pico/intellivision/firmware/src/fujibus.c`
is a bit-compatible plain-C port; `fujinet-msdos/sys/fujicom.c` is the 8088 client.

```
SLIP framing:  END 0xC0, ESC 0xDB, ESC_END 0xDC, ESC_ESC 0xDD; frame = END ... END
Header (6 bytes, little-endian):
  u8  device      0x31..0x3F disk, 0x40 printer, 0x45 clock, 0x70 FujiNet,
                  0x71..0x78 network, 0xFF DBC (the RP2xxx itself)
  u8  command     include/fujiCommandID.h
  u16 length      total decoded length including header
  u8  checksum    8-bit sum with end-around carry, computed with this byte = 0
  u8  descr       first field descriptor (bit 7 = another descriptor byte follows)
Descriptor low 3 bits: 0 none | 1..4 = N x u8 | 5 = 1 x u16 | 6 = 2 x u16 | 7 = 1 x u32
Then params (LE), then payload = everything left up to `length`.
Reply: same shape, command = 0x06 ACK or 0x15 NAK, payload = reply data.
```

Known-good vectors from `fujibus_selftest()`:

```
GET_ADAPTERCONFIG_EXTENDED to 0x70:   C0 70 C4 06 00 3B 00 C0
bare ACK from 0x70:                   C0 70 06 06 00 7C 00 C0
bare NAK from 0x70:                   C0 70 15 06 00 8B 00 C0
```

Disk device on the RS232 bus (`lib/device/rs232/disk.cpp`, `lib/media/rs232/diskTypeImg.cpp`):

| Command | Params | Payload in | Payload out | Notes |
|---|---|---|---|---|
| `R` 0x52 DISK_READ | u32 sector (descr 7) | none | 512 bytes | offset = sector * 512 |
| `P` 0x50 DISK_PUT | u32 sector | 512 bytes | none | write, no verify |
| `W` 0x57 DISK_WRITE | u32 sector | 512 bytes | none | write with verify |
| `S` 0x53 DISK_STATUS | | | 4 bytes | Atari-shaped |

Media type `.img` is a raw sector image, 512-byte sectors, 32-bit LBA. That is the Victor
hard-disk image format you already boot from SD. The guide says it directly: "Start flat."
Nothing on the ESP32 changes to serve Victor images.

### 1.6 The MS-DOS side already exists

`fujinet-msdos` (renamed from `fujinet-rs232`, latest commit 2026-08-31) is the MS-DOS
stack, all Open Watcom:

- `sys/fujinet.sys`: DOS block device driver exposing FujiNet disk slots as drive letters,
  and the INT F5 entry point (`intf5.c`).
- `sys/fujicom.c`: the FujiBus client (`fuji_bus_call(device, cmd, fields, aux1..4, data,
  len, reply, rlen)`). Hardware-specific code is confined to `portio.asm` (8250 UART).
- `fmount`, `ncopy`, `nget`, `nput`, `fnshare`, `setssid`, `iss`, `fujiprn.sys`, and
  `fujinet-config`'s msdos target (replaced 2026-09-02), all on INT F5.

INT F5 register contract (canonical in `sys/intf5.c` and the INT F5 Technical Reference):
`DL` direction (0x00 none, 0x40 read, 0x80 write), `DH` field descriptor, `AL` device,
`AH` command, `CL/CH` aux1/aux2, `SI` aux3/aux4, `ES:BX` far buffer, `DI` length.
Returns `AL` = 'C', 'E' or 'N'. The older `fujinet-bios.md` register layout is obsolete.

`fujinet-bringup/iotest/src/msdos/portio.s` is the same `portio` contract on an 8250 UART,
built with Open Watcom via `makefiles/platforms/msdos.mk`. Only that file changes for the
Victor.

### 1.6a fujinet-lib-experimental: the client library and its test suite

`FozzTexx/fujinet-lib-experimental` is v5.0.0 of `fujinet-lib`, rebuilt around FujiBus.
Layout: `common/` (network, JSON, Fuji, clock, AppKey, QR code, shared verbatim),
`bus/<platform>/` (one backend each for adam, apple2, atari, c64, coco, lynx, msdos, msx),
`include/` (`fujinet-bus.h`, `fujinet-commands.h`, `fujinet-bus-ezcall.h` with the
`DEVCALL_*`/`FUJICALL_*`/`NETCALL_*` macros keyed by field descriptor). CI builds all
eight in the `fozztexx/defoogi` container. Two backends matter for the Victor:

- **`bus/msdos/`** does no framing at all: `fuji_bus_call()` is a `#pragma aux` wrapper
  around `INT F5`, so it depends on `fujinet.sys` being loaded. `portio.s` there is the
  8250 version.
- **`bus/msx/`** is the byte-pipe backend: `fujinet-bus-msx.c` builds the header, packs
  AUX fields, computes the checksum, SLIP-encodes in place, streams out through
  `port_putbuf()`, then reads back to the closing `END` with `port_get_until()` and
  validates the reply. `portio.s` is nine memory-mapped Z80 routines against `0xBFFC`.
  It is self-contained and needs no driver.

`testing/` is the acceptance suite the project runs on real machines: `flibtest`,
`fnettest`, `fclktest`, `fdsktest`, `fqrctest`, `fnfstest`, `fapktest`, `devlist`, one
binary per test group, `PASS`/`FAIL` per check, abort on first failure, and a `fntests`
disk image per platform. `msdos` is a supported target; its image is a 360 KB
`mformat`/`mcopy` FAT12 floppy. Tests need a live FujiNet with WiFi, host slot 0 pointing
at a TNFS share, and an SD host slot. `constants.h` records a per-bus quirk you inherit:
on the RS232 device set the extended directory entry header is 12 bytes, not 13.

### 1.7 Open items in FEP-004 you will meet

1. Reply semantics: settled in code (reply = ACK/NAK packet with payload).
2. Signaling the host that data is waiting: the guide resolves it as "poll the STATUS
   register's available bit, no interrupt line required." The Victor board has IR4 wired,
   so an interrupt is available as an extra.
3. Direct ESP32-to-RP2xxx communication: device `0xFF` (DBC). Used today for ROM push and
   the Intellivision BOOTSEL doorbell.
4. Behaviour with no RP2xxx: unsolved upstream. For the Victor this is "no ESP32 present",
   handled by keeping the SD backend.

## 2. Recommended architecture for the Victor

```
Victor 9000 8088 bus
   |  (existing PIO register snoop + DMA master, unchanged)
   v
RP2350 (PGA2350) = DMA board + Xebec S1410 emulation + Victor DBC
   |  native USB, RP2350 = CDC-ACM device
   v
ESP32-S3-WROOM-1-N16R8 = USB host, stock `fujiversal-rs232`
   |
   WiFi / microSD / TNFS / HTTP / N: protocols / web UI
```

### 2.1 The two decisions, answered for the Victor

**Decision 1: both, and the guide's own tie-breaker says why.** The Victor's boot ROM
boots from SASI natively. Riding the SASI interface for the boot disk *is* the bare-metal
boot the guide wants, with no option ROM, no driver to side-load, and no BIOS change. For
everything else (CONFIG, `N:`, clock, printer, mounting from DOS) speak FEP-004 directly
through a byte pipe, exactly like MSX and the ISA design exercise.

**Decision 2: RP2350**, already decided by a year of PIO work. The guide's ISA chapter
confirms the wide-bus, 5 V-direct choice.

### 2.2 Drop the SPI link

The SPI slave transport in `pico_fujinet/spi.c` was your own protocol on both ends. Nothing
upstream speaks it, and the handshake-pin dance was a large part of the earlier flakiness.
USB CDC is what every fujiversal target uses, needs zero GPIOs on the RP2350 (dedicated
PHY), and the ESP32 side is maintained upstream. The device and command numbering in your
`fujiCmd.h` carries over; only the framing and ack semantics change.

### 2.3 Two host-facing surfaces on the RP2350

**Surface A: transparent SASI disk (no software change on the Victor).**
BIOS and MS-DOS 3.1 keep talking Xebec to the board. Inside the firmware, SASI Read(6) /
Write(6) on target T, LBA L become FujiBus `DISK_READ` / `DISK_PUT` on device `0x31 + T`
with a single u32 param. Here the RP2350 *is* a FujiBus client (the Intellivision role),
because the Victor BIOS cannot be taught SLIP. Implement it as a new
`STORAGE_BACKEND_FUJINET` in `pico_storage/storage.h`'s `storage_ops_t`, so `sasi.c` does
not change.

**Surface B: the canonical byte pipe, in the DMA register page.**
Four registers (`GETC`, `STATUS`, `PUTC`, `CONTROL`) at spare offsets of the EF300 page,
decoded by the same register-snoop path as the DMA registers. The Victor CPU streams SLIP
frames through them and parses replies itself, using `fujinet-lib` and `fujinet-msdos`
with a Victor `portio`. This is the guide's Layer 3 verbatim; the guide explicitly says
"only the access method and addresses change," and memory-mapped registers are what MSX
and CoCo use. It keeps the DOS side 100% canonical: `iotest`, `fujinet.sys`, `fmount`,
`nget`, CONFIG all run above a 100-line `portio`. `CONTROL` is free for Victor-specific
uses (reset the link, select SD fallback, ask for BOOTSEL).

**Why not the Intellivision mailbox style for Surface B?** It would work, and the Victor
board could even DMA payloads straight into caller buffers, which no cartridge can. But it
departs from the pattern every new platform is being built on, it makes the RP2350 parse
FujiBus for the host, and it would need a Victor-specific `fujicom` backend instead of a
`portio`. Keep it in the back pocket as an optimization (section 4, step 9) once the
canonical path works. The 8088 pushing 512 bytes through a memory-mapped register is
already far faster than the 115200-baud UART the PC driver lives on today.

**Link arbitration.** Two originators share one USB link: the SASI path (RP2350-built
frames) and the byte pipe (host-built frames). The ESP32 RS232 bus is strictly one
transaction at a time, and any non-SLIP bytes it sees are handed to the modem device. So
the RP2350 must multiplex at frame granularity: `fujiversal/main.cpp` already tracks
SLIP_END boundaries on the host stream to intercept DBC frames, so extend that state
machine into a lock. A host frame in progress blocks SASI frames until its reply has been
forwarded; a SASI transaction in progress buffers host bytes in the existing 1 KB ring.
Route each ESP32 reply to whichever side issued the outstanding request.

**Interrupt.** IR4 is already wired and named `DMA_IRQ_PIN`. Pulse it when a reply lands
in the byte pipe and on the RS232 bus's `rs232_poll_interrupt()` events, so the DOS driver
can sleep instead of spinning on `STATUS`. Polling stays the baseline, per the guide.

**Fallback.** When `tud_cdc_connected()` is false, serve from the local SD card as today.
That preserves the configuration you have working and answers FEP-004 item 4 for the
Victor.

### 2.4 What stays the same

Everything under `pico_victor/` (register snoop PIO, DMA master PIO, cache, IRQ handlers,
bus-release timing) and `sasi.c`. The year of electrical work is not touched.

## 3. Hardware plan

### 3.1 Phase H0: bench rig, no PCB change

Mirror `fujiversal-pcb-prototype` and the DIY Intellivision guide so others can reproduce
it:

- Current Victor board as is. Its USB-C connector carries the RP2350 native USB.
- For milestones 1 to 5, **no ESP32 at all**: cable the RP2350 to a laptop running
  `fujinet-pc` built with `FUJINET_TARGET=RS232`, serial port set to the RP2350's CDC
  device. This is the `fujinet-bringup` "Hello World" path.
- Then an ESP32-S3-WROOM-1-N16R8 dev board with two USB-C ports (Freenove ESP32-S3 CAM, or
  the dual-port N16R8 DevKitC clones the wiki lists). Native/OTG port to the Victor board
  with a data cable; UART port to a PC for flashing, monitoring and power.
- microSD on the ESP32-S3 (fujiversal pinmap: CS 41, SCK 39, MISO 40, MOSI 38). That SD is
  the FujiNet's; the RP2350's SD remains the offline fallback.
- WS2812 status LED on GPIO 48, white = WiFi, orange flicker = bus, per the pinmaps.

Verify on the bench, in order:

1. The S3 board's host port supplies VBUS in host mode (some clones do not). The RP2350 is
   powered from the Victor regardless; check whether the PGA2350's VBUS sense matters for
   enumeration.
2. The RP2350 enumerates as CDC-ACM on the S3 (`./build.sh -m` shows `ACMChannel` opening
   it). pico-sdk's default VID:PID is 0x2E8A:000A; `fujiversal-rs232` accepts any CDC-ACM.
   Pick a deliberate PID so it never collides with RP2350 BOOTSEL (0x2E8A:000F), which the
   INTV build rejects by VID filter.
3. Flashing the RP2350 while the S3 owns its USB: SWD pads already on the board (Pico
   Debug Probe), or unplug and BOOTSEL. PICOBOOT-over-ESP32 is still on the
   `intv-combined-flash` branch, not master.
4. The guide's warning: a verbose `printf`-over-CDC debug build "steals the same CDC
   channel the ESP32 needs." Keep debug on the UART you already have.

### 3.2 Phase H1: Rev B PCB with the ESP32-S3 on board

Template: `fujinet-hardware/INTV/FujiNet-INTV-Rev0` (CERN-OHL-W-2.0). Carry over:

- ESP32-S3-WROOM-1-N16R8, USB host pins (GPIO19/20) wired directly to RP2350 USB D+/D-
  (90 ohm pair, short, series resistors per the RP2350 datasheet).
- One external USB-C via CP2102N (or CH343) with DTR/RTS to EN/IO0 auto-program for the
  S3. The RP2350 loses its own connector.
- RP2350 recovery: two S3 GPIOs driving transistors on RUN and QSPI_SS (INTV uses GPIO4
  and GPIO5), plus a BOOTSEL request via the byte pipe's `CONTROL` register or a DBC
  frame. Keep the SWD header.
- microSD on the S3, WS2812 status LED.
- Power: 3.3 V buck (AP63203 in the INTV design) from the slot's +5 V with bulk
  capacitance for WiFi bursts (INTV estimates ~80 mA average, ~400 mA peak). The Victor
  slot also offers +12 V; bucking from it isolates the S3's bursts from the 5 V logic rail.
- ESD on the external USB-C.

Victor-specific:

- Cards live inside a metal chassis. Plan for ESP32-S3-WROOM-1U-N16R8 (U.FL) with an
  antenna at the rear bracket, or measure in-chassis RSSI with the PCB-antenna part first.
- GPIO budget on the PGA2350 is nearly spent (0-44 used, 45 and 46 free). USB costs none.
- Licensing: `fujinet-hardware` is CERN-OHL-W-2.0 and requires a link to the FujiNet GitHub
  on the silkscreen. This repo has no LICENSE file; pick one before deriving from INTV.

### 3.3 Phase H2: upstream

Add `Victor9000/` to `fujinet-hardware` with a README in the `INTV/` style, plus a wiki
quickstart.

## 4. RP2350 firmware plan (this repo)

Each step is testable on its own. Steps 1 to 4 are the guide's milestones 1 to 5 and
need no ESP32.

1. **USB CDC device.** TinyUSB device config with a dedicated CDC interface for FujiBus
   (interface 0, since `ACMChannel::newDevice()` takes the first CDC-ACM IAD). Debug stays
   on the UART. `tud_task()` runs on the core that does not service the bus PIO IRQs.
   Storage already lives on Core 1 so SDIO IRQs land on its NVIC; USB fits the same place.
2. **Byte pipe registers.** Decode `GETC`/`STATUS`/`PUTC`/`CONTROL` at four spare offsets
   of the EF300 page. Verify against `SASIDMA.LST` and `BT1HDDVC.ASM` that the BIOS never
   probes those offsets and that the original board's decode did not alias them. Copy the
   `fujiversal` mechanism: `PUTC` writes go into a ring toward USB, `GETC` reads pop a ring
   filled from USB, `STATUS` reports availability. This is the loopback of guide
   milestone 3: a terminal on the CDC port, a Victor `iotest` on the other side.
3. **FujiBus Hello World.** Run `fujinet-pc` (RS232 target) on a laptop pointed at the
   RP2350's CDC port; `iotest` graduates to a `fuji_bus_call()` for
   `GET_ADAPTERCONFIG_EXTENDED`. Milestone 5.
4. **DBC intercept.** Watch the host stream for SLIP frames addressed to `0xFF` and answer
   them locally (reset, version, "which SASI targets map to which slots", BOOTSEL). This is
   `fujiversal/main.cpp`'s `process_command()` pattern. NAK anything unknown.
5. **FujiBus client for the SASI path.** Port `pico/intellivision/firmware/src/fujibus.c`
   and `fujibus_usb.c` (plain C, hardware-free, desktop self-test), or write the ~200-line
   codec fresh from FEP-004 and check it against the vectors in 1.5. They are GPLv3 as part
   of fujinet-firmware; decide the license for this repo first.
6. **`STORAGE_BACKEND_FUJINET`.** `read_sector` = DISK_READ, `write_sector` = DISK_PUT,
   `is_mounted` = FujiNet STATUS with `STATUS_MOUNT_TIME` (the call `fujinet.sys` uses for
   media-change detection), `sync` = no-op. Capacity is not needed: the Xebec has no READ
   CAPACITY and the Victor keeps geometry in the on-disk label. Replace the
   `fujinet_read_sector`/`fujinet_write_sector` calls in `sasi.c` and `dma.c` with the
   storage layer.
7. **Link arbitration and backend selection.** One frame-granular lock as described in
   2.3. At boot wait a bounded time for the CDC link (the Intellivision waits 3 s); if
   absent, fall back to SD; re-check per SASI command so plugging in the S3 later works.
8. **Latency.** A SASI Read(6) with block count N becomes N transactions. Measure. Cheap
   wins: read-ahead of the next few sectors, invalidated on write or on a mount-time
   change. A multi-sector `DISK_READ` upstream is a small `MediaType` change
   (`DISK_SECTORBUF_SIZE` is 512) worth proposing once single-sector is solid.
9. **Optional later: DMA-assisted transfers.** If DOS-side throughput through `GETC`/`PUTC`
   ever matters, add a Victor-specific DBC command ("deliver the next reply payload by DMA
   to physical address X") using the existing `dma_master.pio` burst engine. The byte pipe
   stays the protocol; DMA is an accelerator the host opts into.
10. **Diagnostics.** Keep the non-blocking UART logger. Never block the bus core on USB.

Keep `pico_fujinet/spi.*` until step 6 works, then delete it.

## 5. ESP32-S3 firmware plan (fujinet-firmware)

### 5.1 Nothing to write at first

Stock `fujiversal-rs232`:

```
./build.sh -s fujiversal-rs232     # writes platformio.local.ini
./build.sh -cbu                    # clean, build, upload
./build.sh -f                      # web UI filesystem
./build.sh -m                      # monitor
```

`fnconfig.ini` with WiFi credentials on the S3's SD, mount a Victor `.img` into device
slot 1 from the web UI (`fujinet.local`), Mount All. This is the RS232 quickstart
workflow.

### 5.2 Then the five artifacts the guide names

Per the guide's Chapter 12 and the wiki's "Software Changes for New Platforms":

- `build-platforms/platformio-fujiversal-victor9k.ini`: copy of the intv one
  (`build_bus = RS232`, `build_platform = BUILD_RS232`, `-D PINMAP_FUJIVERSAL_VICTOR9K`,
  the two USB host defines, `esp32-s3-wroom-1-n16r8`).
- `include/pinmap/fujiversal-victor9k.h`: copy of `fujiversal-intv.h` including the two
  RP2350 RUN/BOOTSEL recovery GPIOs.
- `data/webui/config/fujiversal-victor9k.yaml`: copy of `fujiversal-rs232.yaml`.
- `sdkconfig.fujiversal-victor9k`: copy of `sdkconfig.fujiversal-rs232`.
- Later, a `.github/workflows` entry and `autobuild.yml`.

### 5.3 The CONFIG boot problem

The RS232 build boots CONFIG from a PC MS-DOS image (`LOBBY_URL` is
`tnfs://tnfs.fujinet.online/MSDOS/lobby.img`) and its `CONFIG.EXE` targets PC BIOS video.
Neither runs on a Victor. The guide's boot-device goal is met by SASI, so:

1. Now: `config_enabled=0` in `fnconfig.ini` (or `FUJI_CONFIG_BOOT` at runtime), a Victor
   MS-DOS 3.1 image in slot 1, Mount All at startup. Configure from the web UI.
2. Later: a `victor9k` target in `fujinet-config` (loader = nothing, since the BIOS boots
   SASI; CONFIG = a DOS program with a Victor screen layer) baked into a Victor-bootable
   image, and a per-platform boot image hook upstream. The hook is
   `fujiDevice::insert_boot_device()` / `_lobbyDiskURL`; cleanest as a config key.

### 5.4 Small upstream asks once the basics work

- Multi-sector `DISK_READ`.
- A platform name in `AdapterConfig` / web UI tweaks so the UI says Victor 9000.
- Any DBC (`0xFF`) commands you standardize, so Intellivision, MSX and Victor agree.

Follow the guidelines: feature branch off clean `master`, PRs squash-merged, install
`coding-standard.py --addhook`, no `BUILD_*` ifdefs in `lib/device/fujiDevice`, and the
Definition of Done order: Fuji device, disk, then network. Merge early rather than work in
a corner.

## 6. Victor MS-DOS software plan

One Victor `portio.s` (8088, Open Watcom `wasm`, memory-mapped reads and writes of the
four byte-pipe registers instead of 8250 port I/O) serves three consumers in turn. The
toolchain is the one in `test/dos_dma_test/makefile.wc`; the shared makefiles compile with
`wcc -0 -bt=dos`, so nothing above 8086 is emitted.

1. **`iotest` first.** Add `iotest/src/victor9k/portio.s` to `fujinet-bringup`, seven
   routines, and a `makefiles/platforms/victor9k.mk` copied from `msdos.mk`. Guide
   milestone 3.
2. **Driverless `fujinet-lib` backend.** Add `bus/victor9k/` to
   `fujinet-lib-experimental`, cloned from `bus/msx/` rather than `bus/msdos/`:
   `fujinet-bus-victor9k.c` is the MSX C framing code with the MSX jiffy-timeout macros
   swapped for a DOS tick source, and `portio.s` grows the two extra routines the MSX
   backend uses (`port_discard_until`, `port_get_until`). Add `victor9k` to `PLATFORMS`
   with `BUILD_VICTOR9K`, and pick the `fuji_bus_call` prototype in `fujinet-bus.h` (the
   varargs form the MSX uses is fine on Open Watcom). This gives the guide's milestone 5
   Hello World with no driver installed, and lets `testing/` build for the Victor.
3. **INT F5 shim and `fujinet.sys` port.** Confirm INT F5 is free on Victor MS-DOS (the
   Victor uses INT 41h for the SIO and INT FFh for boot; check `ROMNOTES.txt` and
   `BT1INFO.DOC` in `notes/`). Build `fujinet-msdos/sys` for a `victor9k` target: replace
   `portio.asm` with the byte-pipe version and drop the 8250 init; keep `fujicom.c`,
   `intf5.c`, `commands.c` (block driver), `dispatch.c`, `init.c`, `fujiprn.sys`.
4. **Avoid double mounting.** `fujinet.sys` exposes FujiNet slots as DOS drives via
   FujiBus reads. Slots also mapped to SASI targets must not be exposed twice (two caches
   over one image will corrupt it). Use the DBC "which targets are mapped" query so the
   driver skips those units.
   Once the driver is in, `fujinet-lib-experimental/bus/msdos/` works on the Victor as is,
   since it only calls INT F5.
5. **Run the acceptance suite.** `make -C testing victor9k` (or `msdos` once the driver is
   loaded) produces the eight test binaries. Getting them onto the Victor is easy in the
   FujiNet world: mount the 360 KB `fntests.img` read-only in a device slot and copy the
   files across with DOS `COPY`, since `fujinet.sys` presents the slot as a FAT12 drive
   letter and the Victor's DOS reads FAT12 regardless of where the image came from. Run
   `flibtest` first (Fuji device), then `fdsktest`, then `fnettest`, matching the
   Definition of Done order. Remove the video-init and screen assumptions in
   `testing/src/init_video.c` only if they trip on the Victor; today it does nothing for
   msdos.
6. **Tools.** `fmount`, `nget`, `ncopy`, `nput`, `setssid`, `fujitime` are INT F5 clients
   and should run unchanged if they only use DOS console I/O. Anything touching PC BIOS
   video or ports needs a Victor variant.
7. **CONFIG.** Last, per 5.3.

## 7. Milestones (aligned with the guide's ladder)

| | Goal | Proof |
|---|---|---|
| M1 | RP2350 enumerates as CDC | `lsusb` on a laptop |
| M3 | Byte-pipe loopback | Victor `iotest` echoes with a terminal on the CDC port |
| M5 | First FujiBus ACK | `GET_ADAPTERCONFIG_EXTENDED` answered by `fujinet-pc` (RS232), then by the S3 |
| M7 | Boot from FujiNet | Victor boots MS-DOS 3.1 from an image mounted over TNFS via the SASI path; latency measured |
| M7b | Writes and resilience | Writes verified; SD fallback with no S3; survives Victor resets and S3 reboots |
| M8 | `N:` from DOS | `fujinet.sys` loads, `nget` fetches a file, `fmount` mounts a slot |
| M8b | Acceptance | `flibtest`, `fdsktest`, `fnettest`, `fclktest` from `fujinet-lib-experimental/testing` pass on the Victor |
| H1 | Rev B hardware | On-board S3, published to `fujinet-hardware`, platform entry in `fujinet-firmware` |
| M6 | CONFIG on the Victor | Boots into CONFIG, sets WiFi, mounts and boots an image |

CONFIG deliberately comes last: the guide orders it before disk boot only because on
cartridge platforms CONFIG *is* the boot ROM. On the Victor the BIOS boots SASI, so disk
first is the shorter path to a usable machine.

## 8. Risks and questions to settle on Discord

- Confirm: status of PICOBOOT-over-S3 (`intv-combined-flash`); whether an 8088/ISA
  `portio` or PIO exists beyond the guide's design exercise (the proto board has an ISA
  footprint, and the guide's ISA chapter reads like someone intends to build it); whether
  `ACMChannel` will gain a default VID/PID filter; appetite for multi-sector reads; whether
  the byte-pipe register order or `CONTROL` semantics are being standardized; and whether
  a self-framing `bus/victor9k/` in `fujinet-lib-experimental` (MSX style) is welcome
  alongside the INT F5 `bus/msdos/`, or whether they would rather every x86 target go
  through the driver.
- License of this repo. Pulling `fujibus.c` in makes the firmware GPLv3.
- WiFi range inside the Victor chassis.
- VBUS behaviour of the S3 dev board's host port.
- INT F5 availability on Victor MS-DOS.
- Throughput through USB full-speed CDC plus the ESP32 RS232 service loop will be well
  below the SD backend. Fine for boot and daily use; keep SD as the fast local mode.
- `fujiversal`, `pico/intellivision` and the manuals are all under active change; pin the
  `fujinet-firmware` commit you test against and re-sync deliberately. Keeping current has
  been half the battle before, so budget for it.

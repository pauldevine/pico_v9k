# FujiNet Integration Approach for the Victor 9000 DMA Board

Status: proposal, September 2026. No code yet. This is the plan for re-attaching the
working RP2350 DMA/SASI board to FujiNet in the way the FujiNet project now builds
new platforms.

## 0. What this is based on

Read for this document (all current as of 2026-09-03):

- `FujiNetWIFI/fujinet-firmware` master (last commit 2026-09-03), including `lib/bus/rs232`,
  `lib/bus/bus.h`, `lib/device/rs232`, `lib/media/rs232`, `lib/hardware/ACMChannel.*`,
  `pico/intellivision`, `build-platforms/platformio-fujiversal-*.ini`, `include/pinmap/fujiversal-*.h`
- `FujiNetWIFI/fujinet-firmware.wiki`: FEP-004 (The FujiNet Protocol), Intellivision Mailbox
  Protocol, DIY FujiNet for the Intellivision, MSX Technical Overview, MS-DOS BIOS Specification,
  RS232 Quickstart, Definition of Done, Development Guidelines, Software Changes for New Platforms
- `FozzTexx/fujiversal` (RP2040/RP2350 "DBC" firmware for MSX and CoCo) and
  `FozzTexx/fujiversal-pcb-prototype` (Core2350B + ESP32-S3 bring-up board)
- `FujiNetWIFI/fujinet-hardware` (`INTV/FujiNet-INTV-Rev0`, `MSX/Prototype-2`)
- `FujiNetWIFI/fujinet-rs232` (`fujinet.sys`, INT F5, `fujicom`) and `fujinet-lib/msdos`
- This repo's README, Technical Overview, CLAUDE.md, `pico_fujinet/spi.*`, `pico_storage/*`,
  `sasi.c`, the KiCad schematic net list

Not available from the build container: `fujinet.online` (blocked by the egress proxy),
`~/Documents/Victor9k/FujiNet/` and the Discord exports (not in the container),
`mastodon.fozztexx.com`, msx.org. Everything below comes from the repos and the GitHub wiki.
One wiki caveat: the Intellivision mailbox page still describes `rs232Disk::mountROM()`; the
code moved that into `lib/media/rs232/diskTypeROM.cpp` (`MediaTypeROM::push_stream()`) in
August 2026. Where the wiki and the code disagree, this document follows the code.

## 1. Where the FujiNet project is going

### 1.1 "Fujiversal" is real and is the sanctioned path for new platforms

FEP-004 (FozzTexx, October 2025, v1.1 November 2025) is the design document. Its stated
direction:

> Future FujiNet devices will use a Raspberry Pi RP2040 or RP2350 as the physical bus
> interface, with an ESP32 or host computer managing device emulation and network
> communication.

The bus-side MCU is called the DBC (Data Bus Controller) in the MSX docs. The ESP32 side is
the "FujiNet core". The link between them is the FujiBus packet format (section 1.3),
SLIP-framed, over any byte stream. In every shipping instance that byte stream is
USB CDC-ACM with the ESP32-S3 as USB host and the RP2xxx as USB device.

Evidence this is the mainline, not an experiment:

- `fujinet-firmware` has three shipping build targets for it: `fujiversal-rs232`,
  `fujiversal-drivewire` (CoCo) and `fujiversal-intv`. All are `esp32-s3-wroom-1-n16r8`
  with `CONFIG_USB_HOST_ENABLED` and `CONFIG_USB_CDC_ACM_HOST_ENABLED`.
- `lib/hardware/ACMChannel.cpp` is the ESP32-S3 USB host driver. Since May 2026 it takes the
  first CDC-ACM function it finds (no VID/PID hardcoding), and since August 2026 it survives
  the RP2xxx resetting and re-enumerating.
- `pico/intellivision/` in `fujinet-firmware` is a vendored RP2350 cartridge firmware whose
  FujiNet half (`fujibus.c`, `fujibus_usb.c`, `fujinet.c`, `fuji_mailbox.h`) is the reference
  DBC client. Thom Cherryhomes wrote it in August 2026.
- `fujinet-hardware/INTV/FujiNet-INTV-Rev0` is an all-in-one board: RP2354A on the console bus,
  ESP32-S3-WROOM-1-N16R8 as USB host, the two linked on-board over native USB, one external
  USB-C via CP2102N for flashing the S3. Unrouted as of now, but it is the template for
  "production" fujiversal hardware.
- `fujiversal-pcb-prototype` is the bench rig: Waveshare Core2350B + Freenove ESP32-S3 CAM
  (dual USB, microSD), with CoCo and MSX edge adapters. Its bus breakout footprint is an
  8-bit ISA slot, which suggests someone is at least thinking about 8088-class buses.

### 1.2 The firmware refactor that makes this cheap (July to September 2026)

FozzTexx spent the summer decoupling devices from transports:

- `SystemBusBase` (`lib/bus/bus.h`) defines the only contract a device may use:
  `transaction_accept(NO_GET|WILL_GET)`, `transaction_get()`, `transaction_send()`,
  `transaction_success()`, `transaction_error()`. Every bus (SIO, IWM, AdamNet, IEC, Lynx,
  DriveWire, RS232) now inherits it. Devices never touch UARTs.
- Each bus has a packet class (`FujiBusPacket`, `FujiAdamPacket`, `FujiIWMPacket`, ...) that
  satisfies the `FujiPacketLike` concept; `fujiDevice` and its mixins (AppKey, Base64, Hash,
  QR) are shared across all platforms and a test (`tests/check_no_build_ifdefs.py`) forbids
  `BUILD_*` ifdefs inside `lib/device/fujiDevice`.
- `DaisyChain` centralizes device IDs. `fujiDeviceID_t` and `fujiCommandID_t` are `enum class`
  as of last week.

The consequence: the `fujiversal-rs232` build has no idea what computer is on the far end
of the USB link. The Intellivision uses it byte-for-byte unchanged. The Victor can too.

### 1.3 The FujiBus wire format (FEP-004 as implemented)

`lib/bus/rs232/FujiBusPacket.cpp` is authoritative; `pico/intellivision/firmware/src/fujibus.c`
is a plain-C port that is bit-compatible. Summary:

```
SLIP framing:  END 0xC0, ESC 0xDB, ESC_END 0xDC, ESC_ESC 0xDD; frame = END ... END
Header (6 bytes, little-endian):
  u8  device      0x31..0x3F disk, 0x40 printer, 0x45 clock, 0x70 FujiNet,
                  0x71..0x78 network, 0xFF DBC (the RP2xxx itself)
  u8  command     see include/fujiCommandID.h
  u16 length      total decoded length including header
  u8  checksum    8-bit sum with end-around carry, computed with this byte = 0
  u8  descr       first field descriptor (bit 7 = another descriptor byte follows)
Field descriptor low 3 bits -> params:
  0 none | 1..4 = N x u8 | 5 = 1 x u16 | 6 = 2 x u16 | 7 = 1 x u32
Then: params (LE), then optional payload = everything left up to `length`.
Reply: same packet shape, command = 0x06 ACK or 0x15 NAK, payload = reply data.
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
| `R` 0x52 DISK_READ | u32 sector | none | 512 bytes | offset = sector * 512 |
| `P` 0x50 DISK_PUT | u32 sector | 512 bytes | none | write, no verify |
| `W` 0x57 DISK_WRITE | u32 sector | 512 bytes | none | write with verify |
| `S` 0x53 DISK_STATUS | | | 4 bytes | Atari-shaped status |
| `!` / `"` FORMAT | | | | |

Media type `.img` is a raw sector image, 512-byte sectors, 32-bit LBA. That is exactly the
Victor hard-disk image format you already boot from SD. Nothing on the ESP32 has to change
to serve Victor images.

### 1.4 Three DBC styles now exist upstream

| Platform | Host sees | RP2xxx role on the link | Where |
|---|---|---|---|
| MSX | 4 registers at 0xBFFC: GETC, STATUS, PUTC, CONTROL, plus ROM emulation | Transparent byte pipe. The Z80 speaks FujiBus/SLIP itself (fujinet-lib) | `FozzTexx/fujiversal` |
| CoCo | DriveWire on a cartridge ROM | Transparent pipe carrying DriveWire bytes; FujiBus frames to device 0xFF interleaved for ROM push | `fujiversal`, `lib/bus/drivewire` |
| Intellivision | A RAM mailbox at $9C00: SEQ/ACKSEQ, DEVICE, CMD, NPARAM, PARAM_SIZE[8], PARAM_VAL[8], TX[256], RX[512] | Proxy. The RP2350 builds packets, runs the USB transaction, writes the reply back | `pico/intellivision` |

All three keep the ESP32 firmware identical. The choice is purely about what the host CPU
and bus can do. For the Victor the proxy style is the right one (section 2.2).

### 1.5 The MS-DOS side already exists

`fujinet-rs232` is the MS-DOS stack for the ESP32-S3 RS232 FujiNet, all Open Watcom:

- `sys/fujinet.sys`: DOS block device driver. Exposes FujiNet disk slots as drive letters.
  Provides the INT F5 "FujiNet PC BIOS" entry point (`intf5.c`).
- `sys/fujicom.c`: the FujiBus client. `fuji_bus_call(device, cmd, fields, aux1..4, data,
  len, reply, rlen)` builds the packet and SLIP-frames it. Hardware-specific code is confined
  to `portio.asm` (8250 UART, `port_putc`, `port_putbuf_slip`, `port_getbuf_slip_dual`).
- `fmount`, `ncopy`, `nget`, `nput`, `fnshare`, `setssid`, `iss`, `fujiprn.sys`, all built on
  INT F5.
- `fujinet-lib/msdos` and `fujinet-config` (msdos target, replaced on 2026-09-02) sit on INT F5
  as well.

INT F5 register contract (wiki "MS-DOS BIOS Specification", canonical in `sys/intf5.c`):
`DL` direction (0x00 none, 0x40 read, 0x80 write), `DH` field descriptor, `AL` device,
`AH` command, `CL/CH` aux1/aux2, `SI` aux3/aux4, `ES:BX` far buffer, `DI` length.
Returns `AL` = 'C' or 'E', `DX` = reply length.

This matters because the Victor runs MS-DOS. Only the transport under INT F5 has to be
Victor-specific.

### 1.6 Open items in FEP-004 you will bump into

1. Reply packet semantics: settled in code (reply = ACK/NAK packet with payload).
2. Signaling the legacy computer that data is waiting: unsolved in general. The Victor board
   has IR4 wired, so you can solve it locally.
3. Direct ESP32-to-RP2xxx communication: device ID 0xFF (DBC) is the convention. Used today
   for ROM push to cartridges and for the BOOTSEL doorbell on the Intellivision.
4. Behaviour with no RP2xxx present: unsolved upstream. For the Victor this maps to
   "no ESP32 present", handled by keeping the SD backend.

## 2. Recommended architecture for the Victor

```
Victor 9000 8088 bus
   |  (existing PIO register snoop + DMA master, unchanged)
   v
RP2350 (PGA2350) = DMA board + Xebec S1410 emulation + Victor DBC
   |  native USB, RP2350 = CDC-ACM device
   v
ESP32-S3-WROOM-1-N16R8 = USB host, running stock `fujiversal-rs232`
   |
   WiFi / microSD / TNFS / HTTP / N: protocols / web UI
```

### 2.1 Drop the SPI link

The SPI slave transport in `pico_fujinet/spi.c` was your own protocol on both ends. Nothing
upstream speaks it any more and the RS232-over-SPI handshake pin dance was a large part of
the earlier flakiness. USB CDC is what every fujiversal target uses, needs zero GPIOs on the
RP2350 (the PHY is dedicated), and the ESP32 side is maintained by other people. The
`cmdFrame_t` / `fujiCmd.h` device and command numbering you already have carries over; only
the framing and the ack semantics change.

### 2.2 Two host-facing surfaces on the RP2350

**Surface A: transparent SASI disk (no software change on the Victor).**
The BIOS and MS-DOS 3.1 keep talking Xebec to the board exactly as today. Inside the
firmware, SASI Read(6)/Write(6) on target T, LBA L becomes FujiBus `DISK_READ` /
`DISK_PUT` on device `0x31 + T` with a single u32 param L. The RP2350 is the FujiBus client,
the same role the Intellivision cartridge plays. This is a new `STORAGE_BACKEND_FUJINET`
implementation of the `storage_ops_t` interface already in `pico_storage/storage.h`, so
`sasi.c` does not change.

**Surface B: a FujiNet mailbox in the DMA register page, for a DOS driver.**
For CONFIG, N: devices, clock, printer and mounting from DOS, the Victor needs a way to
issue arbitrary FujiBus commands. Copy the Intellivision mailbox shape (sequence-number
interlock, device, command, param count, param sizes and values) but exploit the one thing
this board has that a cartridge does not: it is already an 8088 bus master. Instead of
256/512-byte TX/RX windows, the request block carries a 20-bit physical buffer address and a
16-bit length, and the RP2350 DMAs the payload directly to or from Victor RAM with the
existing `dma_master.pio` burst engine. The request block is exactly the INT F5 register set
(direction, device, command, descriptor, aux1..4, ES:BX linearized, DI), so the Victor's
INT F5 handler becomes a few dozen bytes of register pokes.

Why proxy rather than the MSX transparent-pipe style:

- There would be two originators on one USB link (the SASI path in the RP2350 and the DOS
  driver). The ESP32 RS232 bus is strictly one transaction at a time, and any non-SLIP bytes
  it sees are handed to the modem device. One client living in the RP2350 makes arbitration a
  mutex instead of a protocol.
- A 5 MHz 8088 byte-banging SLIP through a status/data register pair is slow and burns the
  CPU; the DMA engine already exists and is proven.
- The INT F5 contract maps one-to-one onto a mailbox transaction, which means
  `fujinet.sys`, `fujinet-lib/msdos`, `fmount`, `nget`, `ncopy` and CONFIG run above it
  without modification.

**Interrupt.** IR4 is already wired and named `DMA_IRQ_PIN`. Pulse it on ACKSEQ update and
on "network data available" (the RS232 bus polls `rs232_poll_interrupt()` on each N: device,
which on real RS232 hardware toggles RI). That closes FEP-004 open item 2 for this platform.

**Fallback.** When `tud_cdc_connected()` is false, keep serving from the local SD card as
today. That preserves the configuration you have working now and answers FEP-004 open
item 4 for the Victor.

### 2.3 What stays the same

Everything under `pico_victor/` (register snoop PIO, DMA master PIO, cache, IRQ handlers,
bus-release timing) and `sasi.c`. The year of electrical work is not touched by this plan.

## 3. Hardware plan

### 3.1 Phase H0: bench rig, no PCB change

Mirror `fujiversal-pcb-prototype` and the DIY Intellivision guide exactly so other people can
reproduce it:

- Current Victor board as is. Its USB-C connector (DX07S016JA1R1500) carries the RP2350 native
  USB.
- An ESP32-S3-WROOM-1-N16R8 dev board with two USB-C ports (Freenove ESP32-S3 CAM, or the
  "dual Type-C N16R8 DevKitC" clones the wiki lists). Native/OTG port to the Victor board with
  a data cable; UART port to a PC for flashing, monitoring and power.
- microSD on the ESP32-S3 (the fujiversal pinmap: CS 41, SCK 39, MISO 40, MOSI 38). The
  fujiversal SD is the FujiNet's SD; the RP2350's SD remains the offline fallback.
- Status LED: WS2812 on GPIO 48 per the fujiversal pinmaps.

Things to verify on the bench, in order:

1. The S3 host port on the chosen dev board actually supplies VBUS when in host mode. Some
   clones do not; the RP2350 is powered from the Victor anyway, but VBUS presence may be
   needed for enumeration depending on how the PGA2350's VBUS sense is wired.
2. The RP2350 enumerates as CDC-ACM on the S3 (`./build.sh -m` on the S3 shows
   `ACMChannel` opening the device). pico-sdk's default VID:PID is 0x2E8A:000A; the plain
   `fujiversal-rs232` build accepts any CDC-ACM device. Pick a deliberate PID for the Victor
   firmware so it never collides with RP2350 BOOTSEL (0x2E8A:000F), which the INTV build
   explicitly rejects.
3. RP2350 flashing while the S3 owns its USB port: use the SWD pads that are already on the
   board (Pico Debug Probe), or unplug and use BOOTSEL. Do not depend on the
   PICOBOOT-over-ESP32 path yet; the S3-side client is still on the
   `intv-combined-flash` branch, not on master.

### 3.2 Phase H1: Rev B PCB with the ESP32-S3 on board

Template: `fujinet-hardware/INTV/FujiNet-INTV-Rev0` (CERN-OHL-W-2.0). Carry over:

- ESP32-S3-WROOM-1-N16R8 with USB host pins (GPIO19/20) wired directly to the RP2350
  USB D+/D- (90 ohm differential pair, short, series resistors per the RP2350 datasheet).
- One external USB-C via CP2102N (or CH343) plus the DTR/RTS to EN/IO0 auto-program
  circuit for the S3. The RP2350 no longer has its own USB connector.
- RP2350 recovery: two S3 GPIOs driving transistors on RUN and QSPI_SS (the INTV design
  uses GPIO4 and GPIO5), plus the mailbox BOOTSEL doorbell. Keep the SWD header.
- microSD on the S3 (SPI pins as above), WS2812 status LED, WiFi/activity semantics.
- Power: 3.3 V buck (AP63203 in the INTV design) from the slot's +5 V, with bulk
  capacitance for WiFi bursts (INTV estimates ~80 mA average, ~400 mA peak for the S3).
  The Victor slot also offers +12 V and -12 V; a buck from +12 V isolates the S3's burst
  current from the 5 V logic rail if the 5 V rail proves marginal.
- ESD on the external USB-C.

Victor-specific concerns:

- The expansion cards live inside a metal chassis. Plan for ESP32-S3-WROOM-1U-N16R8 (U.FL)
  with an antenna at the rear bracket, or measure RSSI in-chassis with the PCB antenna
  variant before committing.
- GPIO budget on the PGA2350 is nearly spent (0-44 in use, 45 and 46 free). USB costs no
  GPIO. The IR4 line is already routed.
- Licensing: `fujinet-hardware` is CERN-OHL-W-2.0 and requires a link to the FujiNet GitHub
  on the silkscreen. This repo currently has no LICENSE file; pick one before deriving from
  the INTV schematic.

### 3.3 Phase H2: upstream the board

Once Rev B works, add `Victor9000/` to `fujinet-hardware` with a README in the same style as
`INTV/`, and a quickstart page on the wiki.

## 4. RP2350 firmware plan (this repo)

Order of work, each step testable on its own:

1. **USB CDC device.** Add TinyUSB device config with a dedicated CDC interface for FujiBus
   (interface 0, since `ACMChannel::newDevice()` takes the first CDC-ACM IAD it finds).
   Keep debug on the UART; optionally a second CDC for a console later. `tud_task()` runs on
   the core that does not service the bus PIO IRQs. Today storage lives on Core 1 so SDIO
   IRQs land on Core 1's NVIC; the USB stack fits the same place.
2. **FujiBus codec.** Port `pico/intellivision/firmware/src/fujibus.c` and `fujibus_usb.c`
   (plain C, hardware-free, has a desktop self-test). They are GPLv3 as part of
   fujinet-firmware; either accept GPLv3 for this firmware or write the codec fresh from
   FEP-004 (it is about 200 lines) and validate it against the vectors in section 1.3.
3. **`STORAGE_BACKEND_FUJINET`.** Implement `storage_ops_t` on top of the codec:
   `read_sector` = DISK_READ, `write_sector` = DISK_PUT, `is_mounted` = FujiNet STATUS with
   `STATUS_MOUNT_TIME` (the same call `fujinet.sys` uses for media-change detection),
   `sync` = no-op. Capacity is not needed: the Xebec has no READ CAPACITY and the Victor
   keeps geometry in the on-disk label. Replace the `fujinet_read_sector` /
   `fujinet_write_sector` calls in `sasi.c` and `dma.c` with the storage layer.
4. **Backend selection at boot.** Wait a bounded time (the Intellivision waits 3 s) for the
   CDC link; if absent, fall back to SD. Re-check on each SASI command so plugging in the S3
   later switches backends.
5. **Latency and throughput.** A SASI Read(6) with block count N becomes N transactions.
   Measure. Cheap wins: read-ahead of the next few sectors into the RP2350's RAM,
   invalidated on any write or on a mount-time change. A multi-sector `DISK_READ` upstream
   is possible later (the 16-bit length field allows it; `DISK_SECTORBUF_SIZE` on the ESP32
   is 512 today, so it is a small `MediaType` change, worth proposing once the single-sector
   path is solid).
6. **Mailbox registers.** Decode a request block in the EF300 page using the same register
   snoop path as the DMA registers. Suggested layout, modeled on `fuji_mailbox.h`:
   magic 'F' 'N', protocol version, SEQ (Victor writes, skip 0), ACKSEQ (RP2350 writes),
   DEVICE, CMD, DESCR (the FujiBus descriptor byte, so params are passed exactly as INT F5
   receives them), AUX1..AUX4, DIR, BUF_LO/MID/HI (20-bit physical address), LEN_LO/HI,
   ERR, REPLY_CMD, RXLEN_LO/HI, LINK. Verify the chosen offsets are not aliased by the
   original board's decode and are never probed by the BIOS (`SASIDMA.LST`, `BT1HDDVC.ASM`).
7. **Mailbox transaction.** On SEQ change: if DIR is write, DMA `LEN` bytes from the Victor
   into an RP2350 buffer; build and send the packet; wait for the reply (5 s default,
   longer for MOUNT_IMAGE as the Intellivision does); if DIR is read, DMA the reply payload
   to the Victor buffer; write RXLEN, REPLY_CMD, ERR, then ACKSEQ last; pulse IR4. Handle
   segment wrap by working in physical addresses. Cap LEN at what the RP2350 can buffer.
8. **Link arbitration.** One mutex around "send packet, wait reply". SASI-originated and
   mailbox-originated transactions never interleave. Inbound frames addressed to 0xFF (DBC)
   arriving while waiting are dispatched to a DBC handler (NAK anything unknown), exactly
   as `fujibus_set_inbound_handler()` does on the Intellivision.
9. **DBC commands for the Victor.** Reset, version, "which SASI targets are mapped to which
   FujiNet slots", enter BOOTSEL. These go through the same 0xFF device ID convention.
10. **Diagnostics.** Keep the non-blocking UART logger. Never block the bus-servicing core
    on USB.

Keep `pico_fujinet/spi.*` and `fujiCmd.h` around until step 3 works, then delete them.

## 5. ESP32-S3 firmware plan (fujinet-firmware)

### 5.1 Start with stock `fujiversal-rs232`

```
./build.sh -s fujiversal-rs232     # writes platformio.local.ini
./build.sh -cbu                    # clean, build, upload
./build.sh -f                      # web UI filesystem
./build.sh -m                      # monitor
```

Put `fnconfig.ini` with WiFi credentials on the S3's SD, mount a Victor `.img` into device
slot 1 from the web UI (`fujinet.local`), click Mount All. This is the RS232 quickstart
workflow and needs no Victor-specific ESP32 code.

### 5.2 Then a proper platform entry

Per the wiki "Software Changes for New Platforms":

- `build-platforms/platformio-fujiversal-victor9k.ini`: copy of the intv one
  (`build_bus = RS232`, `build_platform = BUILD_RS232`, `-D PINMAP_FUJIVERSAL_VICTOR9K`,
  USB host defines, `esp32-s3-wroom-1-n16r8`).
- `include/pinmap/fujiversal-victor9k.h`: copy of `fujiversal-intv.h` including the two
  RP2350 RUN/BOOTSEL recovery GPIOs.
- `data/webui/config/fujiversal-victor9k.yaml`: copy of `fujiversal-rs232.yaml`.
- `sdkconfig.fujiversal-victor9k`: copy of `sdkconfig.fujiversal-rs232`
  (16 MB flash, octal PSRAM).
- Later, a workflow entry in `.github/workflows` and `autobuild.yml`.

### 5.3 The CONFIG boot problem

The RS232 build boots CONFIG from a PC MS-DOS image (`LOBBY_URL` is
`tnfs://tnfs.fujinet.online/MSDOS/lobby.img`, `IMAGE_EXTENSION` is `.img`) and `CONFIG.EXE`
is built for PC BIOS video. Neither boots on a Victor. Two stages:

1. Now: set `config_enabled=0` in `fnconfig.ini` (the `FUJI_CONFIG_BOOT` command does the
   same at runtime) and keep a Victor MS-DOS 3.1 image in slot 1 with Mount All at startup.
   The Victor boots it over SASI. Configure hosts and slots from the web UI.
2. Later: a `victor9k` target in `fujinet-config` with a Victor screen layer, baked into a
   Victor-bootable image, and a way for the firmware to pick a per-platform boot image.
   The hook is `fujiDevice::insert_boot_device()` / `_lobbyDiskURL`; the cleanest upstream
   form is an `rs232Fuji` subclass or a config key selecting the boot image URL.

### 5.4 Small upstream asks worth filing once the basics work

- Multi-sector `DISK_READ` (payload = N * 512).
- A platform name in `AdapterConfig` / web UI "tweaks" so the UI says Victor 9000.
- Any DBC (0xFF) commands you standardize (reset, version, BOOTSEL) so the Intellivision and
  Victor agree.

Follow the project's guidelines: feature branch off a clean `master`, PRs squash-merged,
install `coding-standard.py --addhook`, no `BUILD_*` ifdefs in `lib/device/fujiDevice`, and
the "Definition of Done" order: Fuji device, disk, then network, then the rest. The wiki is
explicit that you should merge early rather than work in a corner.

## 6. Victor MS-DOS software plan

1. **INT F5 shim.** Confirm INT F5 is free on Victor MS-DOS (the Victor uses INT 41h for the
   SIO and INT FFh for boot; check `ROMNOTES.txt` and `BT1INFO.DOC` in `notes/`). Implement
   the INT F5 register contract as writes to the mailbox and a wait on ACKSEQ (poll first,
   IR4-driven later).
2. **Port `fujinet-rs232/sys` to a `victor9k` build.** Replace `portio.asm` and the packet
   assembly in `fujicom.c` with the mailbox transport; keep `intf5.c`, `commands.c`
   (the DOS block driver), `dispatch.c`, `init.c`, `fujiprn.sys`. It already targets 8088
   real mode with Open Watcom, which is the toolchain in `test/dos_dma_test/makefile.wc`.
3. **Avoid double mounting.** `fujinet.sys` exposes FujiNet slots as DOS drives via FujiBus
   reads. On the Victor, slots that are also mapped to SASI targets must not be exposed
   twice (two caches over one image will corrupt it). Use the DBC "which targets are
   mapped" query from section 4 step 9 so the driver skips those units.
4. **Tools.** `fmount`, `nget`, `ncopy`, `nput`, `setssid`, `fujitime` and `fujinet-lib/msdos`
   are INT F5 clients and should run unchanged if they only use DOS console I/O. Test each;
   anything that touches PC BIOS video or ports needs a Victor variant.
5. **CONFIG.** Last, per section 5.3.

## 7. Milestones

| | Goal | Proof |
|---|---|---|
| M0 | RP2350 enumerates on the S3; one FujiBus round trip | `GET_ADAPTERCONFIG_EXTENDED` reply decoded, matches the web UI |
| M1 | Boot from FujiNet | Victor boots MS-DOS 3.1 from an image mounted over TNFS; sector latency measured |
| M2 | Writes and resilience | Writes verified; SD fallback with no S3; survives Victor resets and S3 reboots |
| M3 | Mailbox and DOS driver | `fujinet.sys` loads, `nget` fetches a file, `fmount` mounts a slot |
| M4 | Rev B hardware | On-board S3, published to `fujinet-hardware`, platform entry in `fujinet-firmware` |
| M5 | CONFIG on the Victor | Boots into CONFIG, sets WiFi, mounts and boots an image |

## 8. Risks and questions to settle on Discord

- The Discord exports and `fujinet.online` were not readable from here. Before starting,
  confirm on Discord: the status of PICOBOOT-over-S3 (`intv-combined-flash`), whether anyone
  has an 8088/ISA fujiversal DBC in progress (the proto board has an ISA footprint), whether
  `ACMChannel` will gain a VID/PID filter by default, and appetite for a multi-sector read.
- License of this repo. Pulling `fujibus.c` in makes the firmware GPLv3.
- WiFi range inside the Victor chassis.
- VBUS behaviour of the S3 dev board's host port.
- INT F5 availability on Victor MS-DOS.
- Effective throughput through USB full-speed CDC plus the ESP32 RS232 service loop. Expect
  it to be well below the SD backend; fine for boot and daily use, so keep the SD fallback
  as the "fast local" mode rather than trying to match it over USB.
- `fujiversal` and `pico/intellivision` are both under active change; pin the
  `fujinet-firmware` commit you test against and re-sync deliberately. Keeping current has
  been half the battle before, so budget for it.

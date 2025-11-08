## Victor 9000 Highlights

- **High-resolution text/graphics** – Soft-font CRT delivers 80×25 text plus an 800×400 bitmap mode by repointing DRAM font tables, outclassing IBM 5150 CGA/MDA limits (`Manuals/technical reference in ascii/CH1.txt:71`, `Manuals/technical reference in ascii/CH10.txt:2`).
- **Variable-speed floppy system** – 96 TPI, GCR-encoded floppies shift spindle speed across 15 zones to reach 600 KB (SS) or 1.2 MB (DS), far above the fixed-speed 5150 drives (`Manuals/technical reference in ascii/CH1.txt:61`, `Boot BIOS ASM source 3.6/BT1FDDVC.ASM:200`, `Manuals/technical reference in ascii/APN.txt:159`).
- **SASI/Xebec mass storage with DMA** – Hard-disk firmware talks to a controller at 0xEF30 with DMA address latches and SCSI-like phases, surpassing IBM XT’s early controller design (`Boot BIOS ASM source 3.6/BT1HDDVC.ASM:69`).
- **Boot firmware diagnostics & flexibility** – ROM prints POST codes over Centronics, supports 512-byte serial bootstrap at 1200 bps, maintains a boot vector and load request block, and spoofs the IBM XT flag for compatibility (`Boot BIOS ASM source 3.6/BT1BASE.ASM:568`, `Boot BIOS ASM source 3.6/BT1BASE.ASM:624`, `Boot BIOS ASM source 3.6/BT1BASE.ASM:669`, `Boot BIOS ASM source 3.6/BT1BASE.ASM:704`, `Boot BIOS ASM source 3.6/BT1INFO.DOC:667`, `Boot BIOS ASM source 3.6/BT1INFO.DOC:701`, `Manuals/technical reference in ascii/ROMNOTES.txt:13`).
- **Rich peripheral set** – Two RS-232C ports, Centronics/IEEE-488 parallel port, dual user ports, and a VIA-driven audio codec come standard, versus the 5150’s sparse onboard I/O (`Manuals/technical reference in ascii/APF.txt:7`, `Manuals/technical reference in ascii/CH5.txt:672`, `Applesauce Images/MS-DOS 3.1 Listings/MS-DOS 3.1 Listings/BELV9000.LST:52`).
- **Loadable BIOS devices** – DOS BIOS stubs redirect PRN/LST to whichever printer driver you load, unlike IBM’s resident handlers, so devices are modular but must be present during boot (`Applesauce Images/MS-DOS 3.1 Listings/MS-DOS 3.1 Listings/MSBIOS.LST:1378`).

### Serial Boot Workflow
1. Boot ROM scans device table and falls back to `SERIAL_BOOT` when nothing else is ready (`Boot BIOS ASM source 3.6/BT1BASE.ASM:568`).
2. Loader resets/programs the μPD7201 SIO at segment E004, enables internal clocks through the keyboard VIA, and sets the 8253 for 1200 bps (`Boot BIOS ASM source 3.6/BT1BASE.ASM:624`, `Boot BIOS ASM source 3.6/BT1BASE.ASM:653`).
3. It requires RING, DCD, and DSR before accepting data to ensure a host is present (`Boot BIOS ASM source 3.6/BT1BASE.ASM:669`).
4. Up to 512 bytes stream into 0000:4000 with per-character timeout; bytes are echoed back for flow control (`Boot BIOS ASM source 3.6/BT1BASE.ASM:704`).
5. Loader patches INT 255 and vectors to the downloaded stub (`Boot BIOS ASM source 3.6/BT1BASE.ASM:704`).
6. Field-service note matches: serial boot demands the control lines and uses 1200 bps (`Boot BIOS ASM source 3.6/BT1INFO.DOC:701`).

### Low-Level Hardware Guide
- **Display** – Allocate a 40 KB aligned buffer, rewrite the screen pointer table, and program the 6845 registers for hi-res (see `Manuals/technical reference in ascii/CH10.txt:11`, `Manuals/technical reference in ascii/CH10.txt:59`, `Manuals/technical reference in ascii/CH10.txt:84`).
- **Serial ports** – SIO interrupt vector is 0x41; configure CR1–CR5 and clock bits via the keyboard VIA (`Manuals/technical reference in ascii/API.txt:35`, `Manuals/technical reference in ascii/API.txt:104`, `Manuals/technical reference in ascii/API.txt:167`). Boot ROM sequence is a working 8‑N‑1 template (`Boot BIOS ASM source 3.6/BT1BASE.ASM:624`).
- **Parallel/Centronics** – Interface uses the “PPORT” 6522; firmware emits early POST faults there with no hardware handshake, so set DDRB carefully and pace /STROBE (`Manuals/technical reference in ascii/ROMNOTES.txt:13`, `Applesauce Images/MS-DOS 3.1 Listings/MS-DOS 3.1 Listings/BELV9000.LST:52`).
- **Audio codec** – 6522 timer drives sample rate, SDA select toggles record/play, volume comes from the VIA shift register (`Manuals/technical reference in ascii/CH5.txt:672`, `Applesauce Images/MS-DOS 3.1 Listings/MS-DOS 3.1 Listings/BELV9000.LST:52`).
- **Boot media metadata** – Track 0 sector 0 stores load addresses, interleave, and 15-zone speed tables; regenerate them when mastering images (`Manuals/technical reference in ascii/APN.txt:120`).

### Suggested Next Steps
1. Capture the 512-byte serial bootstrap and extend it (e.g., add XMODEM) to simplify firmware loads.
2. Build quick utilities that exercise the 6522-controlled display/audio/parallel hardware before running full OS images.
3. Document how Victor’s BIOS services diverge from IBM’s INT interfaces to aid compatibility layers.

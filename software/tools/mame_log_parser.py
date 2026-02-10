#!/usr/bin/env python3
"""
MAME DMA/SASI Boot Log Parser

Parses the MAME Victor 9000 hard disk controller trace logs to extract
SASI command sequences and state transitions. Consolidates polling loops
and presents the protocol interactions in a human-readable format.

Usage:
    python mame_log_parser.py <logfile> [--verbose] [--raw]
"""

import re
import sys
import argparse
from dataclasses import dataclass
from typing import List, Optional, Tuple

# ============================================================================
# Register Definitions (from dma.h)
# ============================================================================

REG_CONTROL = 0x00
REG_DATA    = 0x10
REG_STATUS  = 0x20
REG_ADDR_L  = 0x80
REG_ADDR_M  = 0xA0
REG_ADDR_H  = 0xC0

REG_NAMES = {
    REG_CONTROL: "CONTROL",
    REG_DATA:    "DATA",
    REG_STATUS:  "STATUS",
    REG_ADDR_L:  "ADDR_L",
    REG_ADDR_M:  "ADDR_M",
    REG_ADDR_H:  "ADDR_H",
}

# Control register bits (from dma.h)
CTRL_DMA_ON_VALUE = 0x01  # DMA enable value
CTRL_LOCKOUT      = 0x02  # CPU lockout
CTRL_DMA_STROBE   = 0x04  # Latch strobe
CTRL_DMA_WR_MODE  = 0x08  # Direction: 1 = device -> host (read)
CTRL_SELECT       = 0x10  # Assert SEL/
CTRL_RESET        = 0x20  # Reset device

# Status register bits (SASI bus signals)
STAT_INP = 0x01  # I/O (1 = input to host)
STAT_CTL = 0x02  # C/D (1 = control/status, 0 = data)
STAT_BSY = 0x04  # BUSY
STAT_REQ = 0x08  # REQ (target requesting transfer)
STAT_MSG = 0x10  # MSG (message phase)
STAT_SEL = 0x20  # SEL (selection in progress)
STAT_ACK = 0x40  # ACK (initiator acknowledgment)

# SASI Command Opcodes
SASI_COMMANDS = {
    0x00: "TEST_UNIT_READY",
    0x01: "RECALIBRATE",
    0x03: "REQUEST_SENSE",
    0x04: "FORMAT_UNIT",
    0x08: "READ(6)",
    0x0A: "WRITE(6)",
    0x0B: "SEEK(6)",
    0x0C: "INIT_DRIVE_PARAMS",  # Xebec SET DRIVE PARAMS
    0x15: "MODE_SELECT",
    0x1A: "MODE_SENSE",
    0xE0: "XEBEC_RAM_DIAG",
    0xE3: "XEBEC_DRIVE_DIAG",
    0xE4: "XEBEC_INTERNAL_DIAG",
}

# SASI phases based on status bits
def get_sasi_phase(status: int) -> str:
    """Decode SASI bus phase from status register."""
    if status == 0x00:
        return "BUS_FREE"

    if not (status & STAT_BSY):
        if status & STAT_SEL:
            return "SELECTION"
        return "BUS_FREE"

    # BSY is set - decode phase from CTL, INP, MSG
    ctl = bool(status & STAT_CTL)
    inp = bool(status & STAT_INP)
    msg = bool(status & STAT_MSG)
    req = bool(status & STAT_REQ)

    if msg:
        return "MESSAGE_IN" if inp else "MESSAGE_OUT"
    if ctl:
        return "STATUS" if inp else "COMMAND"
    return "DATA_IN" if inp else "DATA_OUT"


def decode_control(value: int) -> str:
    """Decode control register value to human-readable flags."""
    flags = []
    if value & CTRL_RESET:
        flags.append("RESET")
    if value & CTRL_SELECT:
        flags.append("SELECT")
    if value & CTRL_DMA_WR_MODE:
        flags.append("DIR_IN")
    if value & CTRL_DMA_STROBE:
        flags.append("STROBE")
    if value & CTRL_LOCKOUT:
        flags.append("LOCKOUT")
    if value & CTRL_DMA_ON_VALUE:
        flags.append("DMA_EN")
    return "|".join(flags) if flags else "CLEAR"


def decode_status(value: int) -> str:
    """Decode status register value to human-readable flags."""
    flags = []
    if value & STAT_ACK:
        flags.append("ACK")
    if value & STAT_SEL:
        flags.append("SEL")
    if value & STAT_MSG:
        flags.append("MSG")
    if value & STAT_REQ:
        flags.append("REQ")
    if value & STAT_BSY:
        flags.append("BSY")
    if value & STAT_CTL:
        flags.append("CTL")
    if value & STAT_INP:
        flags.append("INP")
    return "|".join(flags) if flags else "IDLE"


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class LogEntry:
    """Single register access from the log."""
    line_num: int
    is_write: bool
    offset: int
    data: int

    @property
    def reg_name(self) -> str:
        return REG_NAMES.get(self.offset, f"UNK_{self.offset:02X}")


@dataclass
class SASICommand:
    """A complete SASI command transaction."""
    start_line: int
    end_line: int
    dma_address: int
    opcode: int
    command_bytes: List[int]
    status_byte: int
    message_byte: int
    data_phase: bool  # True if there was a data transfer
    data_direction: str  # "IN", "OUT", or "NONE"

    @property
    def command_name(self) -> str:
        return SASI_COMMANDS.get(self.opcode, f"UNKNOWN_0x{self.opcode:02X}")

    def decode_lba(self) -> Optional[int]:
        """Decode LBA from Read/Write command bytes."""
        if len(self.command_bytes) >= 4 and self.opcode in (0x08, 0x0A):
            # Read(6)/Write(6): LBA is in bytes 1-3 (21-bit)
            lba = ((self.command_bytes[1] & 0x1F) << 16) | \
                  (self.command_bytes[2] << 8) | \
                   self.command_bytes[3]
            return lba
        return None

    def decode_block_count(self) -> Optional[int]:
        """Decode block count from Read/Write command."""
        if len(self.command_bytes) >= 5 and self.opcode in (0x08, 0x0A):
            count = self.command_bytes[4]
            return count if count else 256  # 0 means 256 blocks
        return None


@dataclass
class CommandSequence:
    """A sequence of operations forming a logical unit."""
    description: str
    entries: List[LogEntry]
    commands: List[SASICommand]


# ============================================================================
# Parser
# ============================================================================

class MameLogParser:
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.entries: List[LogEntry] = []
        self.commands: List[SASICommand] = []

        # Current state tracking
        self.dma_addr_low = 0
        self.dma_addr_mid = 0
        self.dma_addr_high = 0
        self.last_status = 0
        self.in_command = False
        self.command_bytes: List[int] = []
        self.command_start_line = 0
        self.current_dma_addr = 0

    @property
    def dma_address(self) -> int:
        return (self.dma_addr_high << 16) | (self.dma_addr_mid << 8) | self.dma_addr_low

    def parse_file(self, filename: str) -> List[LogEntry]:
        """Parse the log file into LogEntry objects."""
        pattern = re.compile(r'(Read|Write) offset: ([0-9a-fA-F]+) data: ([0-9a-fA-F]+)')

        with open(filename, 'r') as f:
            for line_num, line in enumerate(f, 1):
                match = pattern.search(line)
                if match:
                    is_write = match.group(1) == 'Write'
                    offset = int(match.group(2), 16)
                    data = int(match.group(3), 16)
                    self.entries.append(LogEntry(line_num, is_write, offset, data))

        return self.entries

    def analyze(self):
        """Analyze parsed entries to extract SASI commands."""
        i = 0
        while i < len(self.entries):
            entry = self.entries[i]

            # Track DMA address writes
            if entry.is_write:
                if entry.offset == REG_ADDR_L:
                    self.dma_addr_low = entry.data
                elif entry.offset == REG_ADDR_M:
                    self.dma_addr_mid = entry.data
                elif entry.offset == REG_ADDR_H:
                    self.dma_addr_high = entry.data
                elif entry.offset == REG_CONTROL:
                    i = self._handle_control_write(i, entry)
                elif entry.offset == REG_DATA:
                    i = self._handle_data_write(i, entry)
            else:  # Read
                if entry.offset == REG_STATUS:
                    i = self._handle_status_read(i, entry)
                elif entry.offset == REG_DATA:
                    i = self._handle_data_read(i, entry)

            i += 1

    def _handle_control_write(self, i: int, entry: LogEntry) -> int:
        """Handle control register write."""
        # Check for command initiation (SELECT bit set)
        if entry.data & CTRL_SELECT:
            self.command_start_line = entry.line_num
            self.current_dma_addr = self.dma_address
            self.command_bytes = []
            self.in_command = False

        # Check for reset
        if entry.data & CTRL_RESET:
            if self.verbose:
                print(f"[{entry.line_num}] DEVICE RESET")

        return i

    def _handle_data_write(self, i: int, entry: LogEntry) -> int:
        """Handle data register write (command byte or data out)."""
        # Look at recent status to determine if we're in command phase
        if self._last_phase() == "COMMAND":
            if not self.in_command:
                self.in_command = True
                if not self.command_start_line:
                    self.command_start_line = entry.line_num
                    self.current_dma_addr = self.dma_address
            self.command_bytes.append(entry.data)
        return i

    def _handle_status_read(self, i: int, entry: LogEntry) -> int:
        """Handle status register read, consolidating polling loops."""
        self.last_status = entry.data
        return i

    def _handle_data_read(self, i: int, entry: LogEntry) -> int:
        """Handle data register read (status/message byte or data in)."""
        phase = get_sasi_phase(self.last_status)

        if phase == "STATUS":
            # This is the status byte from the command
            status_byte = entry.data
            # Look for message byte
            message_byte = 0
            j = i + 1
            while j < len(self.entries):
                next_entry = self.entries[j]
                if not next_entry.is_write and next_entry.offset == REG_STATUS:
                    self.last_status = next_entry.data
                elif not next_entry.is_write and next_entry.offset == REG_DATA:
                    if get_sasi_phase(self.last_status) == "MESSAGE_IN":
                        message_byte = next_entry.data
                        break
                j += 1

            # Record the completed command
            if self.command_bytes:
                cmd = SASICommand(
                    start_line=self.command_start_line,
                    end_line=entry.line_num,
                    dma_address=self.current_dma_addr,
                    opcode=self.command_bytes[0],
                    command_bytes=self.command_bytes.copy(),
                    status_byte=status_byte,
                    message_byte=message_byte,
                    data_phase=self._had_data_phase(),
                    data_direction=self._get_data_direction()
                )
                self.commands.append(cmd)
                self.command_bytes = []
                self.in_command = False

        return i

    def _last_phase(self) -> str:
        """Get the last known SASI phase."""
        return get_sasi_phase(self.last_status)

    def _had_data_phase(self) -> bool:
        """Check if the current command had a data phase."""
        # Look at control register writes for DMA enable
        return False  # Simplified for now

    def _get_data_direction(self) -> str:
        """Get data transfer direction."""
        return "NONE"  # Simplified for now

    def print_summary(self, show_raw: bool = False):
        """Print command summary."""
        print("=" * 80)
        print("MAME SASI COMMAND TRACE ANALYSIS")
        print("=" * 80)
        print(f"Total log entries: {len(self.entries)}")
        print(f"Commands decoded:  {len(self.commands)}")
        print()

        # Command statistics
        cmd_counts = {}
        for cmd in self.commands:
            name = cmd.command_name
            cmd_counts[name] = cmd_counts.get(name, 0) + 1

        print("Command frequency:")
        for name, count in sorted(cmd_counts.items(), key=lambda x: -x[1]):
            print(f"  {name:25s}: {count:5d}")
        print()

        print("-" * 80)
        print("COMMAND SEQUENCE DETAILS")
        print("-" * 80)

        for i, cmd in enumerate(self.commands):
            self._print_command(i, cmd, show_raw)

    def _print_command(self, idx: int, cmd: SASICommand, show_raw: bool):
        """Print a single command with details."""
        print(f"\n[{idx+1}] Lines {cmd.start_line}-{cmd.end_line}: {cmd.command_name}")
        print(f"    DMA Address: 0x{cmd.dma_address:05X}")

        if show_raw:
            cmd_hex = " ".join(f"{b:02X}" for b in cmd.command_bytes)
            print(f"    CDB: [{cmd_hex}]")

        # Decode specific commands
        if cmd.opcode in (0x08, 0x0A):  # Read/Write
            lba = cmd.decode_lba()
            count = cmd.decode_block_count()
            if lba is not None:
                print(f"    LBA: {lba} (0x{lba:05X}), Count: {count} sectors")
        elif cmd.opcode == 0x0B:  # Seek
            if len(cmd.command_bytes) >= 4:
                lba = ((cmd.command_bytes[1] & 0x1F) << 16) | \
                      (cmd.command_bytes[2] << 8) | \
                       cmd.command_bytes[3]
                print(f"    Seek to LBA: {lba} (0x{lba:05X})")
        elif cmd.opcode == 0x0C:  # Init Drive Params
            if len(cmd.command_bytes) >= 5:
                param_len = cmd.command_bytes[4]
                print(f"    Parameter length: {param_len} bytes")
        elif cmd.opcode in (0xE0, 0xE3, 0xE4):  # Xebec diagnostics
            print(f"    Xebec diagnostic command")

        status_str = "GOOD" if cmd.status_byte == 0 else f"CHECK (0x{cmd.status_byte:02X})"
        print(f"    Status: {status_str}, Message: 0x{cmd.message_byte:02X}")

    def print_compact(self):
        """Print a compact one-line-per-command summary."""
        print("=" * 100)
        print("COMPACT COMMAND SUMMARY")
        print("=" * 100)
        print(f"{'#':>4} {'Lines':>12} {'Command':20} {'DMA Addr':>10} {'LBA':>8} {'Count':>6} {'Status':>8}")
        print("-" * 100)

        for i, cmd in enumerate(self.commands):
            lines = f"{cmd.start_line}-{cmd.end_line}"
            lba_str = ""
            count_str = ""

            if cmd.opcode in (0x08, 0x0A):
                lba = cmd.decode_lba()
                count = cmd.decode_block_count()
                if lba is not None:
                    lba_str = str(lba)
                    count_str = str(count)
            elif cmd.opcode == 0x0B:
                if len(cmd.command_bytes) >= 4:
                    lba = ((cmd.command_bytes[1] & 0x1F) << 16) | \
                          (cmd.command_bytes[2] << 8) | \
                           cmd.command_bytes[3]
                    lba_str = str(lba)

            status_str = "GOOD" if cmd.status_byte == 0 else f"ERR:{cmd.status_byte:02X}"

            print(f"{i+1:>4} {lines:>12} {cmd.command_name:20} 0x{cmd.dma_address:05X} {lba_str:>8} {count_str:>6} {status_str:>8}")


def consolidate_polling(entries: List[LogEntry]) -> List[Tuple[LogEntry, int]]:
    """Consolidate repeated status polling into single entries with count."""
    result = []
    i = 0
    while i < len(entries):
        entry = entries[i]
        count = 1

        # Check for repeated identical entries
        while (i + count < len(entries) and
               entries[i + count].is_write == entry.is_write and
               entries[i + count].offset == entry.offset and
               entries[i + count].data == entry.data):
            count += 1

        result.append((entry, count))
        i += count

    return result


def print_protocol_diagram():
    """Print a reference diagram for the SASI protocol."""
    print("""
================================================================================
SASI/DMA PROTOCOL REFERENCE
================================================================================

REGISTER MAP (base 0xEF300):
  0x00 CONTROL (W)   - DMA control, selection, reset
  0x10 DATA    (R/W) - Data bus / command bytes / status
  0x20 STATUS  (R)   - Bus status (REQ/BSY/CTL/INP/MSG/SEL/ACK)
  0x80 ADDR_L  (R/W) - DMA address bits 0-7
  0xA0 ADDR_M  (R/W) - DMA address bits 8-15
  0xC0 ADDR_H  (R/W) - DMA address bits 16-19 (4 bits)

CONTROL REGISTER BITS:
  bit 0 (0x01) - DMA_EN   : DMA enable value
  bit 1 (0x02) - LOCKOUT  : CPU lockout
  bit 2 (0x04) - STROBE   : Latch strobe (must toggle for changes)
  bit 3 (0x08) - DIR_IN   : Direction 1=device->host (read from disk)
  bit 4 (0x10) - SELECT   : Assert SEL/ to select target
  bit 5 (0x20) - RESET    : Reset SASI device

STATUS REGISTER BITS:
  bit 0 (0x01) - INP : I/O direction (1=input to host)
  bit 1 (0x02) - CTL : Control/Data (1=control/status phase)
  bit 2 (0x04) - BSY : Target busy
  bit 3 (0x08) - REQ : Target requesting transfer
  bit 4 (0x10) - MSG : Message phase
  bit 5 (0x20) - SEL : Selection in progress
  bit 6 (0x40) - ACK : Initiator acknowledge

SASI PHASES (decoded from CTL/INP/MSG with BSY=1):
  Status=0x00         -> BUS_FREE
  Status=0x04 (BSY)   -> After selection, waiting for phase
  Status=0x06 (BSY|CTL) -> COMMAND (target ready for cmd byte, REQ not set yet)
  Status=0x0E (REQ|BSY|CTL) -> COMMAND (target requesting next cmd byte)
  Status=0x0F (REQ|BSY|CTL|INP) -> STATUS (target sending status byte)
  Status=0x1F (MSG|REQ|BSY|CTL|INP) -> MESSAGE_IN (completion message)

TYPICAL COMMAND SEQUENCE:
  1. Write target ID to DATA register
  2. Write SELECT|STROBE to CONTROL
  3. Poll STATUS until BSY set (0x04)
  4. Write STROBE only to CONTROL (deassert SELECT)
  5. Poll STATUS until 0x0E (COMMAND phase with REQ)
  6. For each command byte:
     a. Write byte to DATA
     b. Poll STATUS until 0x0E (next byte) or phase change
  7. If READ: Poll STATUS=0x05 (DATA_IN), DMA reads data
  8. Poll STATUS until 0x0F (STATUS phase)
  9. Read DATA (status byte, typically 0x00=GOOD)
  10. Poll STATUS until 0x1F (MESSAGE_IN)
  11. Read DATA (message byte, typically 0x00=COMPLETE)
  12. Bus returns to FREE (status 0x00)
""")


def extract_single_command_trace(entries: List[LogEntry], cmd_num: int, commands: List[SASICommand]) -> List[LogEntry]:
    """Extract all entries for a specific command number."""
    if cmd_num < 1 or cmd_num > len(commands):
        return []

    cmd = commands[cmd_num - 1]
    # Find entries in the line range, with some padding before
    start_line = max(1, cmd.start_line - 10)
    end_line = cmd.end_line + 5

    return [e for e in entries if start_line <= e.line_num <= end_line]


def print_consolidated_trace(entries: List[LogEntry], max_entries: int = 500):
    """Print a consolidated trace showing state transitions."""
    print("=" * 80)
    print("CONSOLIDATED REGISTER TRACE (polling loops collapsed)")
    print("=" * 80)

    consolidated = consolidate_polling(entries)

    # Track state for context
    dma_addr = [0, 0, 0]  # low, mid, high
    last_ctrl = 0
    last_status = 0

    printed = 0
    for entry, count in consolidated:
        if printed >= max_entries:
            print(f"\n... truncated (showing first {max_entries} unique entries)")
            break

        # Build description
        rw = "WR" if entry.is_write else "RD"
        reg = REG_NAMES.get(entry.offset, f"0x{entry.offset:02X}")

        extra = ""
        if entry.offset == REG_CONTROL and entry.is_write:
            extra = f"  [{decode_control(entry.data)}]"
            last_ctrl = entry.data
        elif entry.offset == REG_STATUS and not entry.is_write:
            phase = get_sasi_phase(entry.data)
            flags = decode_status(entry.data)
            extra = f"  [{flags}] -> {phase}"
            last_status = entry.data
        elif entry.offset == REG_DATA:
            if entry.is_write:
                phase = get_sasi_phase(last_status)
                if phase == "COMMAND":
                    cmd_name = SASI_COMMANDS.get(entry.data, "")
                    if cmd_name:
                        extra = f"  <- {cmd_name} opcode"
                    else:
                        extra = f"  <- cmd byte"
            else:
                phase = get_sasi_phase(last_status)
                if phase == "STATUS":
                    extra = f"  -> {'GOOD' if entry.data == 0 else 'CHECK'}"
                elif phase == "MESSAGE_IN":
                    extra = f"  -> msg=0x{entry.data:02X}"
        elif entry.offset in (REG_ADDR_L, REG_ADDR_M, REG_ADDR_H) and entry.is_write:
            idx = {REG_ADDR_L: 0, REG_ADDR_M: 1, REG_ADDR_H: 2}[entry.offset]
            dma_addr[idx] = entry.data
            addr = (dma_addr[2] << 16) | (dma_addr[1] << 8) | dma_addr[0]
            extra = f"  -> DMA=0x{addr:05X}"

        count_str = f" x{count}" if count > 1 else ""
        print(f"{entry.line_num:6d}: {rw} {reg:8s} = 0x{entry.data:02X}{count_str}{extra}")
        printed += 1


def main():
    parser = argparse.ArgumentParser(
        description="Parse MAME Victor 9000 DMA/SASI boot log",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s log.txt --compact           # One-line per command summary
  %(prog)s log.txt --boot-sequence     # First 20 commands with CDB bytes
  %(prog)s log.txt --trace -m 200      # Register trace (200 entries)
  %(prog)s log.txt --protocol          # Show protocol reference
  %(prog)s log.txt --cmd 9 --trace     # Trace for command #9 (first READ)
"""
    )
    parser.add_argument("logfile", nargs='?', help="Log file to parse")
    parser.add_argument("--verbose", "-v", action="store_true",
                       help="Show verbose output")
    parser.add_argument("--raw", "-r", action="store_true",
                       help="Show raw command bytes")
    parser.add_argument("--trace", "-t", action="store_true",
                       help="Show consolidated register trace")
    parser.add_argument("--compact", "-c", action="store_true",
                       help="Show compact one-line-per-command summary")
    parser.add_argument("--max-trace", "-m", type=int, default=500,
                       help="Max trace entries to show (default: 500)")
    parser.add_argument("--boot-sequence", "-b", action="store_true",
                       help="Show just the early boot sequence (first 20 commands)")
    parser.add_argument("--protocol", "-p", action="store_true",
                       help="Show protocol reference diagram")
    parser.add_argument("--cmd", type=int, default=0,
                       help="Show detailed trace for specific command number")

    args = parser.parse_args()

    if args.protocol:
        print_protocol_diagram()
        if not args.logfile:
            return

    if not args.logfile:
        parser.print_help()
        return

    log_parser = MameLogParser(verbose=args.verbose)
    log_parser.parse_file(args.logfile)
    log_parser.analyze()

    if args.cmd > 0:
        # Show trace for specific command
        cmd_entries = extract_single_command_trace(log_parser.entries, args.cmd, log_parser.commands)
        if cmd_entries:
            cmd = log_parser.commands[args.cmd - 1]
            print(f"=" * 80)
            print(f"DETAILED TRACE FOR COMMAND #{args.cmd}: {cmd.command_name}")
            print(f"=" * 80)
            print(f"Lines {cmd.start_line}-{cmd.end_line}, DMA Address: 0x{cmd.dma_address:05X}")
            cmd_hex = " ".join(f"{b:02X}" for b in cmd.command_bytes)
            print(f"CDB: [{cmd_hex}]")
            print()
            print_consolidated_trace(cmd_entries, 200)
        else:
            print(f"Command #{args.cmd} not found")
        return

    if args.trace:
        print_consolidated_trace(log_parser.entries, args.max_trace)
        print()

    if args.compact:
        log_parser.print_compact()
    elif args.boot_sequence:
        # Show early boot with detailed breakdown
        print("=" * 80)
        print("VICTOR 9000 HARD DISK BOOT SEQUENCE ANALYSIS")
        print("=" * 80)
        print()
        print("This shows the initial commands needed to boot from hard disk.")
        print("Your Pico implementation must handle these in order for boot to succeed.")
        print()

        # Limit to first 20 commands
        orig_commands = log_parser.commands
        log_parser.commands = orig_commands[:20]
        log_parser.print_summary(show_raw=True)
        log_parser.commands = orig_commands
    else:
        log_parser.print_summary(show_raw=args.raw)


if __name__ == "__main__":
    main()

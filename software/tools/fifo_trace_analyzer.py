#!/usr/bin/env python3
"""
fifo_trace_analyzer.py
~~~~~~~~~~~~~~~~~~~~~~

Utility script to decode FIFO TRACE lines emitted by the cached DMA handlers.
It ingests `fast_log` output, reconstructs register offsets referenced by each
FIFO word, and summarizes activity for register and DMA operations.

The current protocol uses a 1-bit payload type:
  - 0: read payload
  - 1: write payload

The script also recognizes legacy 2-bit prefetch/commit/write logs so archived
captures remain readable.

Example usage:
    python tools/fifo_trace_analyzer.py logs/pico_debug.log --summary

If no file is supplied the script reads from stdin, which makes it easy to run
directly against a serial log capture:
    cat /path/to/capture.log | python tools/fifo_trace_analyzer.py --limit 40
"""

from __future__ import annotations

import argparse
import re
import sys
from collections import Counter
from typing import Iterable, Optional, Tuple


DMA_REGISTER_BASE = 0xEF300
DMA_REGISTER_LIMIT = DMA_REGISTER_BASE + 0x100

# Trace flags copied from fifo_helpers.h
TRACE_FLAG_ERROR = 0x01
TRACE_FLAG_WRITE = 0x02


class TraceEntry:
    __slots__ = (
        "index",
        "tag",
        "kind",
        "pending_before",
        "pending_after",
        "flags",
        "logged_data",
        "raw",
        "address",
        "offset",
        "masked_offset",
        "decoded_data",
        "in_range",
    )

    def __init__(
        self,
        *,
        index: int,
        tag: int,
        kind: str,
        pending_before: int,
        pending_after: int,
        flags: int,
        logged_data: int,
        raw: int,
        address: Optional[int],
        offset: Optional[int],
        masked_offset: Optional[int],
        decoded_data: Optional[int],
        in_range: bool,
    ) -> None:
        self.index = index
        self.tag = tag
        self.kind = kind
        self.pending_before = pending_before
        self.pending_after = pending_after
        self.flags = flags
        self.logged_data = logged_data
        self.raw = raw
        self.address = address
        self.offset = offset
        self.masked_offset = masked_offset
        self.decoded_data = decoded_data
        self.in_range = in_range

    def describe(self) -> str:
        tag_name = self.kind.upper()
        parts = [
            f"[{self.index:05d}] {tag_name}",
            f"raw=0x{self.raw:08X}",
            f"flags=0x{self.flags:02X}",
            f"pending {self.pending_before}->{self.pending_after}",
        ]

        if self.address is not None:
            parts.append(f"addr=0x{self.address:05X}")
        if self.offset is not None:
            parts.append(f"off=0x{self.offset:05X}")
        if self.masked_offset is not None:
            parts.append(f"masked=0x{self.masked_offset:02X}")
        if not self.in_range and self.address is not None:
            parts.append("OOR")
        if self.decoded_data is not None:
            parts.append(f"data=0x{self.decoded_data:02X}")
        if self.logged_data is not None:
            parts.append(f"log_data=0x{self.logged_data:02X}")
        return " ".join(parts)


def decode_register_address(raw: int, *, shift: int) -> Tuple[int, Optional[int], Optional[int], bool]:
    address = (raw >> shift) & 0xFFFFF
    in_range = DMA_REGISTER_BASE <= address < DMA_REGISTER_LIMIT
    if not in_range:
        return address, None, None, False
    offset = (address - DMA_REGISTER_BASE) & 0xFFFFF
    masked_offset = dma_mask_offset(offset) & 0xFF
    return address, offset, masked_offset, True


def decode_dma_address(raw: int, *, shift: int = 2) -> int:
    return (raw >> shift) & 0xFFFFF


def parse_trace_lines(lines: Iterable[str]) -> Iterable[TraceEntry]:
    pattern = re.compile(
        r"FIFO TRACE tag=(?P<tag>\d+)\s+"
        r"before=(?P<before>\d+)\s+"
        r"after=(?P<after>\d+)\s+"
        r"flags=0x(?P<flags>[0-9a-fA-F]+)\s+"
        r"data=0x(?P<data>[0-9a-fA-F]+)\s+"
        r"raw=0x(?P<raw>[0-9a-fA-F]+)"
    )

    for index, line in enumerate(lines):
        match = pattern.search(line)
        if not match:
            continue

        tag = int(match.group("tag"))
        pending_before = int(match.group("before"))
        pending_after = int(match.group("after"))
        flags = int(match.group("flags"), 16)
        logged_data = int(match.group("data"), 16)
        raw = int(match.group("raw"), 16)

        address: Optional[int] = None
        offset: Optional[int] = None
        masked_offset: Optional[int] = None
        decoded_data: Optional[int] = None
        in_range = False
        kind = "unknown"

        if tag == 0:
            # New protocol register read: [type=0][address bits 30:11]
            reg_address, reg_offset, reg_masked, reg_in_range = decode_register_address(raw, shift=11)
            if reg_in_range:
                address = reg_address
                offset = reg_offset
                masked_offset = reg_masked
                in_range = True
                kind = "reg_read"
            else:
                # Legacy prefetch decode fallback: [tag bits 31:30][address bits 29:10]
                legacy_addr, legacy_off, legacy_masked, legacy_in_range = decode_register_address(raw, shift=10)
                if legacy_in_range:
                    address = legacy_addr
                    offset = legacy_off
                    masked_offset = legacy_masked
                    in_range = True
                    kind = "reg_prefetch_legacy"
                else:
                    # DMA path payloads also use tag 0/1 but with different layout.
                    address = decode_dma_address(raw, shift=2)
                    decoded_data = (raw >> 22) & 0xFF
                    in_range = True
                    kind = "dma_read"
        elif tag == 1:
            # New protocol register write: [type=1][data bits 30:23][address bits 22:3]
            write_address = decode_dma_address(raw, shift=3)
            write_data = (raw >> 23) & 0xFF
            if DMA_REGISTER_BASE <= write_address < DMA_REGISTER_LIMIT:
                address = write_address
                decoded_data = write_data
                in_range = True
                offset = (write_address - DMA_REGISTER_BASE) & 0xFFFFF
                masked_offset = dma_mask_offset(offset) & 0xFF
                kind = "reg_write"
            else:
                # Legacy commit fallback: [tag bits 31:30][address bits 29:10]
                legacy_addr, legacy_off, legacy_masked, legacy_in_range = decode_register_address(raw, shift=10)
                if legacy_in_range:
                    address = legacy_addr
                    offset = legacy_off
                    masked_offset = legacy_masked
                    in_range = True
                    kind = "reg_commit_legacy"
                else:
                    address = decode_dma_address(raw, shift=2)
                    decoded_data = (raw >> 22) & 0xFF
                    in_range = True
                    kind = "dma_write"
        elif tag == 2:
            # Legacy write: [tag bits 31:30][data bits 29:22][address bits 21:2]
            address = decode_dma_address(raw, shift=2)
            decoded_data = (raw >> 22) & 0xFF
            in_range = DMA_REGISTER_BASE <= address < DMA_REGISTER_LIMIT
            if in_range:
                offset = (address - DMA_REGISTER_BASE) & 0xFFFFF
                masked_offset = dma_mask_offset(offset) & 0xFF
                kind = "reg_write_legacy"
            else:
                kind = "unknown_legacy_tag2"

        yield TraceEntry(
            index=index,
            tag=tag,
            kind=kind,
            pending_before=pending_before,
            pending_after=pending_after,
            flags=flags,
            logged_data=logged_data,
            raw=raw,
            address=address,
            offset=offset,
            masked_offset=masked_offset,
            decoded_data=decoded_data,
            in_range=in_range,
        )


def dma_mask_offset(offset: int) -> int:
    offset &= 0xFFFFF  # Clamp to 20-bit
    if offset >= 0x80:
        return offset & ~0x1F
    return offset & ~0x0F


def validate_sequence(entries: Iterable[TraceEntry]) -> Tuple[Counter, Counter, int]:
    summary = Counter()
    anomalies = Counter()
    processed = 0

    for entry in entries:
        processed += 1

        if entry.flags & TRACE_FLAG_ERROR:
            anomalies["explicit-error-flagged"] += 1

        if entry.kind in ("reg_read", "reg_prefetch_legacy", "reg_commit_legacy"):
            summary[(entry.kind, entry.masked_offset)] += 1
        elif entry.kind in ("reg_write", "reg_write_legacy"):
            if not entry.in_range:
                anomalies["reg_write-out-of-range"] += 1
            else:
                summary[(entry.kind, entry.masked_offset)] += 1
            if (
                entry.decoded_data is not None
                and entry.logged_data is not None
                and entry.decoded_data != entry.logged_data
            ):
                anomalies["reg_write-data-mismatch"] += 1
        elif entry.kind in ("dma_read", "dma_write"):
            summary[(entry.kind, entry.address)] += 1
            if (
                entry.decoded_data is not None
                and entry.logged_data is not None
                and entry.decoded_data != entry.logged_data
            ):
                anomalies[f"{entry.kind}-data-mismatch"] += 1
        elif entry.kind.startswith("unknown_legacy"):
            anomalies["legacy-unknown"] += 1
        else:
            anomalies["unknown-kind"] += 1

    return summary, anomalies, processed


def print_summary(summary: Counter, limit: Optional[int] = None) -> None:
    if not summary:
        print("No trace entries parsed.")
        return

    print("Event summary (type @ offset -> count):")
    rows = [
        (count, event, offset)
        for (event, offset), count in summary.items()
        if offset is not None
    ]
    rows.sort(reverse=True)

    for idx, (count, event, offset) in enumerate(rows, start=1):
        if limit is not None and idx > limit:
            break
        if event.startswith("dma_"):
            offset_str = f"0x{offset:05X}"
        else:
            offset_str = f"0x{offset:02X}"
        print(f"  {event:<12} @ {offset_str} : {count}")


def main(argv: Optional[Iterable[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Decode FIFO TRACE entries.")
    parser.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="Path to the log file (defaults to stdin if omitted).",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=None,
        help="Stop after printing N decoded entries.",
    )
    parser.add_argument(
        "--summary",
        action="store_true",
        help="Print a per-offset summary instead of individual entries.",
    )

    args = parser.parse_args(argv)

    if args.logfile:
        try:
            source = open(args.logfile, "r", encoding="utf-8")
        except OSError as exc:
            parser.error(f"Failed to open {args.logfile!r}: {exc}")
    else:
        source = sys.stdin

    with source:
        entries = list(parse_trace_lines(source))

    summary, anomalies, processed = validate_sequence(entries)

    if args.summary:
        print_summary(summary)
        if anomalies:
            print("\nAnomalies:")
            for key, count in sorted(anomalies.items()):
                print(f"  {key}: {count}")
        print(f"\nProcessed {processed} traced FIFO entries.")
        return 0

    limit = args.limit if args.limit is not None else len(entries)
    for idx, entry in enumerate(entries, start=1):
        if idx > limit:
            break
        print(entry.describe())

    if anomalies:
        print("\nAnomalies detected:")
        for key, count in sorted(anomalies.items()):
            print(f"  {key}: {count}")

    print(f"\nProcessed {processed} traced FIFO entries.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

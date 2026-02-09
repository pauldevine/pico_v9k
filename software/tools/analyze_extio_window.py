#!/usr/bin/env python3
"""
Stream analysis of large DSLogic CSV captures.

Focuses on EXTIO activity within a limited time window and how RD/WR pulses
align with EXTIO. Designed for very large CSV files (streamed, no full load).

Example:
  python3 tools/analyze_extio_window.py "/path/to/capture.csv" --window-us 120
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class Segment:
    start: float
    end: float
    samples: int
    rd_low: bool
    wr_low: bool

    @property
    def duration(self) -> float:
        return max(0.0, self.end - self.start)


@dataclass
class PulseStats:
    total: int = 0
    extio_active_at_fall: int = 0
    extio_overlap: int = 0
    lead_sum_s: float = 0.0
    lead_count: int = 0
    missed: int = 0


def iter_rows(path: Path) -> Iterable[List[str]]:
    header_seen = False
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith(";"):
                continue
            if not header_seen:
                header_seen = True
                continue
            line = line.strip()
            if not line:
                continue
            yield line.split(",")


def read_header(path: Path) -> Tuple[Dict[str, int], List[str]]:
    header_line: Optional[str] = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith(";"):
                continue
            header_line = line.strip()
            break
    if header_line is None:
        raise ValueError("No CSV header line found.")
    columns = [col.strip() for col in header_line.split(",")]
    return {name: idx for idx, name in enumerate(columns)}, columns


def format_us(value_s: float) -> str:
    return f"{value_s * 1e6:.3f} us"


def analyze_window(
    path: Path,
    time_col: str,
    extio_col: str,
    rd_col: str,
    wr_col: str,
    window_us: float,
    active_low: bool,
    idle_samples: int,
    start_mode: str,
    auto_flip: bool,
    dump_segments: int,
    dump_pulses: int,
) -> int:
    col_map, columns = read_header(path)

    for required in (time_col, extio_col, rd_col, wr_col):
        if required not in col_map:
            raise ValueError(f"Column {required!r} not found in header.")

    time_idx = col_map[time_col]
    extio_idx = col_map[extio_col]
    rd_idx = col_map[rd_col]
    wr_idx = col_map[wr_col]

    # Determine idle level from the first idle_samples rows.
    idle_count = {0: 0, 1: 0}
    start_time: Optional[float] = None
    idle_done = False

    # EXTIO tracking
    extio_active_value: Optional[int] = None
    extio_idle_value: Optional[int] = None
    extio_active = False
    extio_fall_time: Optional[float] = None
    extio_segments: List[Segment] = []
    seg_start = 0.0
    seg_samples = 0
    seg_rd_low = False
    seg_wr_low = False

    # RD/WR pulse tracking
    rd_active = False
    wr_active = False
    rd_start = 0.0
    wr_start = 0.0
    rd_overlap = False
    wr_overlap = False
    rd_stats = PulseStats()
    wr_stats = PulseStats()

    rd_pulse_offsets: List[float] = []
    wr_pulse_offsets: List[float] = []

    # Window start/end
    window_start: Optional[float] = None
    window_end: Optional[float] = None
    window_triggered = False

    rows_processed = 0

    for row in iter_rows(path):
        rows_processed += 1
        try:
            t = float(row[time_idx])
        except (ValueError, IndexError):
            continue

        if start_time is None:
            start_time = t

        if not idle_done:
            try:
                extio_val = int(row[extio_idx])
            except (ValueError, IndexError):
                extio_val = None
            if extio_val in (0, 1):
                idle_count[extio_val] += 1
            if rows_processed >= idle_samples:
                extio_idle_value = 1 if idle_count[1] >= idle_count[0] else 0
                extio_active_value = 0 if active_low else 1
                if auto_flip and extio_idle_value == extio_active_value:
                    extio_active_value = 1 - extio_active_value
                idle_done = True
                if start_mode == "file-start":
                    window_start = start_time
                    window_end = window_start + (window_us * 1e-6)
                    window_triggered = True

        if idle_done:
            if start_mode == "first-extio" and not window_triggered:
                try:
                    extio_val = int(row[extio_idx])
                except (ValueError, IndexError):
                    extio_val = None
                if extio_val == extio_active_value:
                    window_start = t
                    window_end = window_start + (window_us * 1e-6)
                    window_triggered = True

            if window_triggered and window_end is not None and t > window_end:
                break

        if not window_triggered:
            continue

        try:
            extio_val = int(row[extio_idx])
            rd_val = int(row[rd_idx])
            wr_val = int(row[wr_idx])
        except (ValueError, IndexError):
            continue

        # EXTIO segment tracking
        if extio_val == extio_active_value:
            if not extio_active:
                extio_active = True
                seg_start = t
                seg_samples = 0
                seg_rd_low = False
                seg_wr_low = False
                extio_fall_time = t
            seg_samples += 1
            if rd_val == 0:
                seg_rd_low = True
            if wr_val == 0:
                seg_wr_low = True
        elif extio_active:
            extio_segments.append(
                Segment(
                    start=seg_start,
                    end=t,
                    samples=seg_samples,
                    rd_low=seg_rd_low,
                    wr_low=seg_wr_low,
                )
            )
            extio_active = False
            extio_fall_time = None

        # RD pulse tracking
        if rd_val == 0 and not rd_active:
            rd_active = True
            rd_start = t
            rd_overlap = extio_active
            rd_stats.total += 1
            if extio_active:
                rd_stats.extio_active_at_fall += 1
                if extio_fall_time is not None:
                    rd_stats.lead_sum_s += (t - extio_fall_time)
                    rd_stats.lead_count += 1
                    if len(rd_pulse_offsets) < dump_pulses:
                        rd_pulse_offsets.append(t - extio_fall_time)
            else:
                rd_stats.missed += 1
        elif rd_val == 1 and rd_active:
            rd_active = False
            if rd_overlap or extio_active:
                rd_stats.extio_overlap += 1

        # WR pulse tracking
        if wr_val == 0 and not wr_active:
            wr_active = True
            wr_start = t
            wr_overlap = extio_active
            wr_stats.total += 1
            if extio_active:
                wr_stats.extio_active_at_fall += 1
                if extio_fall_time is not None:
                    wr_stats.lead_sum_s += (t - extio_fall_time)
                    wr_stats.lead_count += 1
                    if len(wr_pulse_offsets) < dump_pulses:
                        wr_pulse_offsets.append(t - extio_fall_time)
            else:
                wr_stats.missed += 1
        elif wr_val == 1 and wr_active:
            wr_active = False
            if wr_overlap or extio_active:
                wr_stats.extio_overlap += 1

    # Close segment if file ended mid-active.
    if extio_active and window_end is not None:
        extio_segments.append(
            Segment(
                start=seg_start,
                end=window_end,
                samples=seg_samples,
                rd_low=seg_rd_low,
                wr_low=seg_wr_low,
            )
        )

    if window_start is None:
        print(f"File: {path}")
        print("No EXTIO activity found in the scanned window.")
        return 0

    print(f"File: {path}")
    print(f"Columns: {', '.join(columns)}")
    print(f"Window start: {format_us(window_start)}")
    print(f"Window length: {window_us:.1f} us")
    print(f"Rows processed: {rows_processed}")
    print(f"EXTIO idle={extio_idle_value} active={extio_active_value} (active_low={active_low})")
    print(f"EXTIO segments: {len(extio_segments)}")

    if extio_segments:
        durations = [seg.duration for seg in extio_segments if seg.samples]
        if durations:
            avg = sum(durations) / len(durations)
            print(
                "EXTIO segment duration (min/avg/max): "
                f"{format_us(min(durations))} / {format_us(avg)} / {format_us(max(durations))}"
            )

        seg_read = sum(1 for seg in extio_segments if seg.rd_low and not seg.wr_low)
        seg_write = sum(1 for seg in extio_segments if seg.wr_low and not seg.rd_low)
        seg_both = sum(1 for seg in extio_segments if seg.wr_low and seg.rd_low)
        seg_none = len(extio_segments) - seg_read - seg_write - seg_both
        print(
            "EXTIO segments: "
            f"read={seg_read} write={seg_write} both={seg_both} none={seg_none}"
        )

    def report_pulse(name: str, stats: PulseStats, offsets: List[float]) -> None:
        if stats.total == 0:
            print(f"{name} pulses: 0")
            return
        lead_avg = (stats.lead_sum_s / stats.lead_count) if stats.lead_count else 0.0
        print(
            f"{name} pulses: {stats.total} "
            f"(extio_at_fall={stats.extio_active_at_fall}, "
            f"overlap={stats.extio_overlap}, missed={stats.missed})"
        )
        if stats.lead_count:
            print(f"{name} lead avg (EXTIO fall -> {name} fall): {format_us(lead_avg)}")
        if offsets:
            preview = ", ".join(format_us(v) for v in offsets)
            print(f"{name} lead samples: {preview}")

    report_pulse("RD", rd_stats, rd_pulse_offsets)
    report_pulse("WR", wr_stats, wr_pulse_offsets)

    if dump_segments > 0 and extio_segments:
        print("\nFirst EXTIO segments:")
        for idx, seg in enumerate(extio_segments[:dump_segments], start=1):
            print(
                f"{idx:03d} start={format_us(seg.start)} dur={format_us(seg.duration)} "
                f"rd_low={int(seg.rd_low)} wr_low={int(seg.wr_low)} samples={seg.samples}"
            )

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze EXTIO alignment in DSLogic CSV.")
    parser.add_argument("csv_path", type=Path, help="Path to DSLogic CSV export.")
    parser.add_argument("--time-col", default="Time(s)", help="Time column name.")
    parser.add_argument("--extio", default="EXTIO", help="EXTIO column name.")
    parser.add_argument("--rd", default="RD", help="RD column name.")
    parser.add_argument("--wr", default="WR", help="WR column name.")
    parser.add_argument("--window-us", type=float, default=120.0, help="Window size in microseconds.")
    parser.add_argument(
        "--active-high",
        action="store_true",
        help="Treat EXTIO as active-high (default: active-low).",
    )
    parser.add_argument(
        "--idle-samples",
        type=int,
        default=2000,
        help="Samples to determine EXTIO idle state.",
    )
    parser.add_argument(
        "--auto-flip",
        action="store_true",
        help="Auto-flip EXTIO polarity if idle matches active level.",
    )
    parser.add_argument(
        "--start-mode",
        choices=("file-start", "first-extio"),
        default="file-start",
        help="Window start: file-start or first-extio.",
    )
    parser.add_argument("--dump-segments", type=int, default=0, help="Print first N EXTIO segments.")
    parser.add_argument("--dump-pulses", type=int, default=5, help="Print first N lead samples.")
    args = parser.parse_args()

    return analyze_window(
        path=args.csv_path,
        time_col=args.time_col,
        extio_col=args.extio,
        rd_col=args.rd,
        wr_col=args.wr,
        window_us=args.window_us,
        active_low=not args.active_high,
        idle_samples=args.idle_samples,
        start_mode=args.start_mode,
        auto_flip=args.auto_flip,
        dump_segments=args.dump_segments,
        dump_pulses=args.dump_pulses,
    )


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""
Analyze DSLogic CSV captures for EXTIO activity and basic bus patterns.

Example:
  python3 tools/analyze_logic_csv.py "/Users/pauldevine/projects/fujinet-project/bus captures/DSLogic U3Pro16-la-260110-150803.csv"
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class CaptureMeta:
    sample_rate: Optional[str] = None
    sample_count: Optional[str] = None
    channels: Optional[str] = None


@dataclass
class Segment:
    start_time: float
    end_time: float
    samples: int
    rd_low: bool
    wr_low: bool

    @property
    def duration_s(self) -> float:
        return max(0.0, self.end_time - self.start_time)


def read_header(path: Path) -> Tuple[CaptureMeta, List[str]]:
    meta = CaptureMeta()
    header: Optional[str] = None
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith(";"):
                line = line.strip()
                if line.startswith("; Sample rate:"):
                    meta.sample_rate = line.split(":", 1)[1].strip()
                elif line.startswith("; Sample count:"):
                    meta.sample_count = line.split(":", 1)[1].strip()
                elif line.startswith("; Channels"):
                    parts = line.split(":", 1)
                    meta.channels = parts[1].strip() if len(parts) > 1 else line.lstrip(";").strip()
                continue
            header = line.strip()
            break
    if header is None:
        raise ValueError("No CSV header line found.")
    columns = [col.strip() for col in header.split(",")]
    return meta, columns


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


def format_us(value_s: float) -> str:
    return f"{value_s * 1e6:.3f} us"


def format_count(value: Optional[int]) -> str:
    return "n/a" if value is None else str(value)


def analyze_capture(
    path: Path,
    time_col: str,
    extio_col: str,
    rd_col: Optional[str],
    wr_col: Optional[str],
    window_us: Optional[float],
    active_low: bool,
    dump_segments: int,
) -> int:
    meta, columns = read_header(path)
    column_map = {name: idx for idx, name in enumerate(columns)}

    if time_col not in column_map:
        raise ValueError(f"Time column {time_col!r} not found in header.")
    if extio_col not in column_map:
        raise ValueError(f"EXTIO column {extio_col!r} not found in header.")

    rd_idx = column_map.get(rd_col) if rd_col else None
    wr_idx = column_map.get(wr_col) if wr_col else None

    extio_idx = column_map[extio_col]
    time_idx = column_map[time_col]

    # Pass 1: determine idle value by counting EXTIO values.
    extio_counts = Counter()
    total_samples = 0
    for row in iter_rows(path):
        total_samples += 1
        try:
            extio_counts[int(row[extio_idx])] += 1
        except (ValueError, IndexError):
            continue

    idle_value = 1 if extio_counts[1] >= extio_counts[0] else 0
    active_value = 0 if active_low else 1
    flipped_active = False
    if idle_value == active_value:
        active_value = 1 - active_value
        flipped_active = True

    # Pass 2: analyze transitions and windows.
    segments: List[Segment] = []
    in_active = False
    seg_start = 0.0
    seg_samples = 0
    seg_rd_low = False
    seg_wr_low = False

    first_active_time: Optional[float] = None
    last_active_time: Optional[float] = None

    active_samples = 0
    active_rd_low_samples = 0
    active_wr_low_samples = 0

    edge_counts_window: Dict[str, int] = defaultdict(int)
    prev_values: Dict[str, Optional[int]] = {name: None for name in columns}
    window_end: Optional[float] = None

    for row in iter_rows(path):
        try:
            time_s = float(row[time_idx])
        except (ValueError, IndexError):
            continue

        try:
            extio = int(row[extio_idx])
        except (ValueError, IndexError):
            continue

        rd_val = None
        wr_val = None
        if rd_idx is not None and rd_idx < len(row):
            try:
                rd_val = int(row[rd_idx])
            except ValueError:
                rd_val = None
        if wr_idx is not None and wr_idx < len(row):
            try:
                wr_val = int(row[wr_idx])
            except ValueError:
                wr_val = None

        if extio == active_value:
            if not in_active:
                in_active = True
                seg_start = time_s
                seg_samples = 0
                seg_rd_low = False
                seg_wr_low = False
                if first_active_time is None:
                    first_active_time = time_s
                    if window_us is not None:
                        window_end = first_active_time + (window_us * 1e-6)
            seg_samples += 1
            if rd_val == 0:
                seg_rd_low = True
                active_rd_low_samples += 1
            if wr_val == 0:
                seg_wr_low = True
                active_wr_low_samples += 1
            active_samples += 1
            last_active_time = time_s
        elif in_active:
            segments.append(
                Segment(
                    start_time=seg_start,
                    end_time=time_s,
                    samples=seg_samples,
                    rd_low=seg_rd_low,
                    wr_low=seg_wr_low,
                )
            )
            in_active = False

        # Edge counts within the window.
        if window_end is None or time_s <= window_end:
            for name, idx in column_map.items():
                if idx >= len(row):
                    continue
                try:
                    value = int(row[idx])
                except ValueError:
                    continue
                prev = prev_values.get(name)
                if prev is not None and value != prev:
                    edge_counts_window[name] += 1
                prev_values[name] = value

    if in_active:
        segments.append(
            Segment(
                start_time=seg_start,
                end_time=last_active_time or seg_start,
                samples=seg_samples,
                rd_low=seg_rd_low,
                wr_low=seg_wr_low,
            )
        )

    # Summaries.
    segment_durations = [seg.duration_s for seg in segments if seg.samples > 0]
    total_active_time = sum(segment_durations)

    seg_class = Counter()
    for seg in segments:
        if seg.rd_low and not seg.wr_low:
            seg_class["read"] += 1
        elif seg.wr_low and not seg.rd_low:
            seg_class["write"] += 1
        elif seg.rd_low and seg.wr_low:
            seg_class["both"] += 1
        else:
            seg_class["none"] += 1

    print(f"File: {path}")
    print(f"Columns: {', '.join(columns)}")
    print(f"Samples (meta): {meta.sample_count or 'n/a'}")
    print(f"Sample rate (meta): {meta.sample_rate or 'n/a'}")
    print(f"EXTIO idle={idle_value} active={active_value} (active_low={active_low}, flipped={flipped_active})")
    print(f"EXTIO counts: 0={extio_counts.get(0, 0)} 1={extio_counts.get(1, 0)} total={total_samples}")

    if first_active_time is None:
        print("No EXTIO activity detected.")
        return 0

    print(f"First EXTIO activity: {format_us(first_active_time)}")
    print(f"Last EXTIO activity: {format_us(last_active_time or first_active_time)}")
    if last_active_time is not None:
        print(f"EXTIO activity span: {format_us(last_active_time - first_active_time)}")

    print(f"EXTIO segments: {len(segments)}")
    if segment_durations:
        min_d = min(segment_durations)
        max_d = max(segment_durations)
        avg_d = total_active_time / len(segment_durations)
        print(
            "Segment duration (min/avg/max): "
            f"{format_us(min_d)} / {format_us(avg_d)} / {format_us(max_d)}"
        )
        print(f"Total active time: {format_us(total_active_time)}")

    print(
        "Segment classification: "
        f"read={seg_class['read']} write={seg_class['write']} "
        f"both={seg_class['both']} none={seg_class['none']}"
    )

    if active_samples:
        rd_pct = (active_rd_low_samples / active_samples) * 100.0
        wr_pct = (active_wr_low_samples / active_samples) * 100.0
        print(f"Active sample RD low: {rd_pct:.1f}%")
        print(f"Active sample WR low: {wr_pct:.1f}%")

    if window_end is not None:
        print(f"Edge counts within first {window_us:.1f} us from activity start:")
        for name in columns:
            if name == time_col:
                continue
            count = edge_counts_window.get(name, 0)
            print(f"  {name}: {count}")

    if dump_segments > 0:
        print("\nFirst EXTIO segments:")
        for idx, seg in enumerate(segments[:dump_segments], start=1):
            print(
                f"{idx:03d} start={format_us(seg.start_time)} "
                f"dur={format_us(seg.duration_s)} "
                f"rd_low={int(seg.rd_low)} wr_low={int(seg.wr_low)} "
                f"samples={seg.samples}"
            )

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze DSLogic CSV captures.")
    parser.add_argument("csv_path", type=Path, help="Path to DSLogic CSV export.")
    parser.add_argument("--time-col", default="Time(s)", help="Time column name.")
    parser.add_argument("--extio", default="EXTIO", help="EXTIO column name.")
    parser.add_argument("--rd", default="RD", help="RD column name.")
    parser.add_argument("--wr", default="WR", help="WR column name.")
    parser.add_argument("--window-us", type=float, default=100.0, help="Window size in microseconds.")
    parser.add_argument(
        "--active-high",
        action="store_true",
        help="Treat EXTIO as active-high (default: active-low).",
    )
    parser.add_argument(
        "--dump-segments",
        type=int,
        default=0,
        help="Print the first N EXTIO active segments.",
    )

    args = parser.parse_args()
    return analyze_capture(
        path=args.csv_path,
        time_col=args.time_col,
        extio_col=args.extio,
        rd_col=args.rd,
        wr_col=args.wr,
        window_us=args.window_us,
        active_low=not args.active_high,
        dump_segments=args.dump_segments,
    )


if __name__ == "__main__":
    raise SystemExit(main())

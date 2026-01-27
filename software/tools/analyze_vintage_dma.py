#!/usr/bin/env python3
"""
Analyze vintage DMA board logic analyzer capture to understand IR4 interrupt behavior.
"""

import sys
import csv
from collections import defaultdict

def analyze_vintage_capture(filename, max_lines=50_000_000):
    """Analyze the vintage DMA board capture for IR4 patterns."""

    # Track IR4 transitions
    ir4_transitions = []
    ir4_high_durations = []
    ir4_low_durations = []

    # Track EXTIO (register access) transitions
    extio_transitions = []

    # Track RD/WR during EXTIO low
    register_accesses = []

    # State tracking
    prev_ir4 = None
    prev_extio = None
    prev_rd = None
    prev_wr = None
    ir4_transition_time = None
    extio_start_time = None

    # Column indices (will be set from header)
    col_idx = {}

    print(f"Analyzing {filename}...")

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        line_count = 0

        for row in reader:
            # Skip comment lines
            if not row or row[0].startswith(';'):
                continue

            # Parse header
            if row[0] == 'Time(s)':
                for i, name in enumerate(row):
                    col_idx[name.strip()] = i  # Strip whitespace from column names
                print(f"Columns: {list(col_idx.keys())}")
                continue

            line_count += 1
            if line_count > max_lines:
                print(f"Stopped at {max_lines} lines")
                break

            if line_count % 10_000_000 == 0:
                print(f"  Processed {line_count/1_000_000:.0f}M lines...")

            try:
                time = float(row[col_idx['Time(s)']])
                ir4 = int(row[col_idx['IR4']])
                extio = int(row[col_idx['EXTIO']])
                rd = int(row[col_idx['RD']])
                wr = int(row[col_idx['WR']])
                ale = int(row[col_idx['ALE']])

                # Get address bits if available
                a19 = int(row[col_idx['A19']]) if 'A19' in col_idx else 0
                a18 = int(row[col_idx['A18']]) if 'A18' in col_idx else 0
                a17 = int(row[col_idx['A17']]) if 'A17' in col_idx else 0
                bd0 = int(row[col_idx['BD0']]) if 'BD0' in col_idx else 0

            except (ValueError, KeyError) as e:
                continue

            # Track IR4 transitions
            if prev_ir4 is not None and ir4 != prev_ir4:
                ir4_transitions.append({
                    'time': time,
                    'edge': 'rising' if ir4 == 1 else 'falling',
                    'extio': extio,
                    'rd': rd,
                    'wr': wr
                })

                # Calculate duration since last transition
                if ir4_transition_time is not None:
                    duration = time - ir4_transition_time
                    if prev_ir4 == 1:  # Was high, now low
                        ir4_high_durations.append(duration)
                    else:  # Was low, now high
                        ir4_low_durations.append(duration)

                ir4_transition_time = time

            # Track EXTIO transitions (register accesses)
            if prev_extio is not None and extio != prev_extio:
                if extio == 0:  # EXTIO asserted (active low)
                    extio_start_time = time
                else:  # EXTIO deasserted
                    if extio_start_time is not None:
                        duration = time - extio_start_time
                        extio_transitions.append({
                            'start': extio_start_time,
                            'end': time,
                            'duration': duration
                        })

            # Track register accesses when EXTIO is low
            if extio == 0:
                if prev_rd == 1 and rd == 0:  # RD falling edge
                    register_accesses.append({
                        'time': time,
                        'type': 'READ',
                        'ir4': ir4,
                        'a19': a19, 'a18': a18, 'a17': a17
                    })
                if prev_wr == 1 and wr == 0:  # WR falling edge
                    register_accesses.append({
                        'time': time,
                        'type': 'WRITE',
                        'ir4': ir4,
                        'a19': a19, 'a18': a18, 'a17': a17
                    })

            prev_ir4 = ir4
            prev_extio = extio
            prev_rd = rd
            prev_wr = wr

    print(f"\nProcessed {line_count} lines total")

    # Report results
    print(f"\n=== IR4 Analysis ===")
    print(f"Total IR4 transitions: {len(ir4_transitions)}")

    if ir4_transitions:
        rising = sum(1 for t in ir4_transitions if t['edge'] == 'rising')
        falling = sum(1 for t in ir4_transitions if t['edge'] == 'falling')
        print(f"  Rising edges: {rising}")
        print(f"  Falling edges: {falling}")

        print(f"\nFirst 20 IR4 transitions:")
        for i, t in enumerate(ir4_transitions[:20]):
            print(f"  {i+1}. {t['time']:.9f}s: {t['edge']:8s} EXTIO={t['extio']} RD={t['rd']} WR={t['wr']}")

    if ir4_high_durations:
        print(f"\nIR4 HIGH durations:")
        print(f"  Min: {min(ir4_high_durations)*1e6:.2f} µs")
        print(f"  Max: {max(ir4_high_durations)*1e6:.2f} µs")
        print(f"  Avg: {sum(ir4_high_durations)/len(ir4_high_durations)*1e6:.2f} µs")

        # Show distribution
        ranges = [(0, 10), (10, 50), (50, 100), (100, 500), (500, 1000), (1000, 10000)]
        print(f"  Distribution (µs):")
        for lo, hi in ranges:
            count = sum(1 for d in ir4_high_durations if lo <= d*1e6 < hi)
            if count > 0:
                print(f"    {lo}-{hi}: {count}")

    print(f"\n=== EXTIO (Register Access) Analysis ===")
    print(f"Total EXTIO pulses: {len(extio_transitions)}")

    if extio_transitions:
        durations = [t['duration'] for t in extio_transitions]
        print(f"  Duration range: {min(durations)*1e9:.0f} - {max(durations)*1e9:.0f} ns")
        print(f"  Avg duration: {sum(durations)/len(durations)*1e9:.0f} ns")

    print(f"\n=== Register Accesses ===")
    print(f"Total register accesses: {len(register_accesses)}")

    reads = [a for a in register_accesses if a['type'] == 'READ']
    writes = [a for a in register_accesses if a['type'] == 'WRITE']
    print(f"  Reads: {len(reads)}")
    print(f"  Writes: {len(writes)}")

    # Show accesses around IR4 transitions
    print(f"\nRegister accesses near IR4 transitions:")
    for i, ir4_t in enumerate(ir4_transitions[:10]):
        ir4_time = ir4_t['time']
        nearby = [a for a in register_accesses
                  if abs(a['time'] - ir4_time) < 0.001]  # Within 1ms
        if nearby:
            print(f"  IR4 {ir4_t['edge']} at {ir4_time:.9f}s:")
            for a in nearby[:5]:
                delta = (a['time'] - ir4_time) * 1e6
                print(f"    {a['type']} at {delta:+.1f}µs (IR4={a['ir4']})")

    # Check: Is IR4 ever high when EXTIO is low (register access in progress)?
    ir4_during_extio = sum(1 for a in register_accesses if a['ir4'] == 1)
    print(f"\nRegister accesses while IR4=1: {ir4_during_extio}/{len(register_accesses)}")

    return ir4_transitions, extio_transitions, register_accesses

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_vintage_dma.py <csv_file>")
        sys.exit(1)

    analyze_vintage_capture(sys.argv[1])

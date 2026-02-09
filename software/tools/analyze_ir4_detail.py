#!/usr/bin/env python3
"""
Detailed analysis of IR4 transitions on vintage DMA board.
Focus on what happens immediately before and after each transition.
"""

import sys
import csv

def analyze_ir4_detail(filename, max_transitions=50):
    """Analyze IR4 transitions in detail."""

    col_idx = {}
    samples = []
    ir4_transitions = []

    print(f"Analyzing {filename}...")
    print("Loading samples around IR4 transitions...")

    # First pass: find all IR4 transition times
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        prev_ir4 = None

        for row in reader:
            if not row or row[0].startswith(';'):
                continue

            if row[0] == 'Time(s)':
                for i, name in enumerate(row):
                    col_idx[name.strip()] = i
                continue

            try:
                time = float(row[col_idx['Time(s)']])
                ir4 = int(row[col_idx['IR4']])
            except (ValueError, KeyError):
                continue

            if prev_ir4 is not None and ir4 != prev_ir4:
                ir4_transitions.append(time)
                if len(ir4_transitions) >= max_transitions * 2:
                    break

            prev_ir4 = ir4

    print(f"Found {len(ir4_transitions)} IR4 transitions")

    # Second pass: collect samples around each transition
    window_ns = 50000  # 50µs before and after
    sample_rate = 125e6  # 125 MHz

    transition_details = []

    with open(filename, 'r') as f:
        reader = csv.reader(f)
        current_transition_idx = 0
        collecting = False
        collect_start = None
        collect_end = None
        current_samples = []

        for row in reader:
            if not row or row[0].startswith(';'):
                continue

            if row[0] == 'Time(s)':
                continue

            if current_transition_idx >= len(ir4_transitions):
                break

            try:
                time = float(row[col_idx['Time(s)']])
                ir4 = int(row[col_idx['IR4']])
                extio = int(row[col_idx['EXTIO']])
                rd = int(row[col_idx['RD']])
                wr = int(row[col_idx['WR']])
                ale = int(row[col_idx['ALE']])
                den = int(row[col_idx['DEN']])
                dtr = int(row[col_idx['DT/R']])
            except (ValueError, KeyError):
                continue

            target_time = ir4_transitions[current_transition_idx]
            window_s = window_ns / 1e9

            # Check if we should start collecting
            if not collecting and time >= target_time - window_s:
                collecting = True
                collect_start = target_time - window_s
                collect_end = target_time + window_s
                current_samples = []

            # Collect samples
            if collecting:
                current_samples.append({
                    'time': time,
                    'ir4': ir4,
                    'extio': extio,
                    'rd': rd,
                    'wr': wr,
                    'ale': ale,
                    'den': den,
                    'dtr': dtr
                })

                if time >= collect_end:
                    transition_details.append({
                        'transition_time': target_time,
                        'samples': current_samples
                    })
                    collecting = False
                    current_transition_idx += 1

    # Analyze each transition
    print(f"\n=== Detailed IR4 Transition Analysis ===\n")

    for i, detail in enumerate(transition_details[:20]):
        trans_time = detail['transition_time']
        samples = detail['samples']

        # Find the exact transition sample
        trans_sample = None
        prev_sample = None
        for s in samples:
            if prev_sample is not None:
                if prev_sample['ir4'] != s['ir4']:
                    trans_sample = s
                    break
            prev_sample = s

        if not trans_sample:
            continue

        edge = "RISING" if trans_sample['ir4'] == 1 else "FALLING"

        print(f"--- Transition #{i+1}: IR4 {edge} at {trans_time:.9f}s ---")

        # Find EXTIO pulses (register accesses) near this transition
        extio_pulses = []
        prev_extio = None
        extio_start = None
        for s in samples:
            if prev_extio is not None:
                if prev_extio == 1 and s['extio'] == 0:
                    extio_start = s['time']
                elif prev_extio == 0 and s['extio'] == 1:
                    if extio_start is not None:
                        # Collect RD/WR state during EXTIO low
                        access_type = "READ" if any(ss['rd'] == 0 for ss in samples
                                                     if extio_start <= ss['time'] <= s['time']) else "WRITE"
                        extio_pulses.append({
                            'start': extio_start,
                            'end': s['time'],
                            'type': access_type,
                            'delta_us': (extio_start - trans_time) * 1e6
                        })
            prev_extio = s['extio']

        # Show register accesses near the transition
        nearby_pulses = [p for p in extio_pulses if abs(p['delta_us']) < 30]
        for p in nearby_pulses:
            print(f"  {p['type']:5s} at {p['delta_us']:+7.2f}µs (EXTIO pulse {(p['end']-p['start'])*1e9:.0f}ns)")

        # Show what was happening right at the transition
        print(f"  At transition: EXTIO={trans_sample['extio']} RD={trans_sample['rd']} WR={trans_sample['wr']}")

        # For falling edges, check if it coincides with a read
        if edge == "FALLING":
            if trans_sample['extio'] == 0 and trans_sample['rd'] == 0:
                print(f"  >>> IR4 cleared during READ cycle!")
            elif trans_sample['extio'] == 0 and trans_sample['wr'] == 0:
                print(f"  >>> IR4 cleared during WRITE cycle!")

        print()

    # Summary statistics
    print("\n=== Summary ===")
    rising_during_extio = 0
    falling_during_extio = 0
    falling_during_read = 0

    for detail in transition_details:
        samples = detail['samples']
        prev_sample = None
        for s in samples:
            if prev_sample is not None and prev_sample['ir4'] != s['ir4']:
                if s['ir4'] == 1:  # Rising
                    if s['extio'] == 0:
                        rising_during_extio += 1
                else:  # Falling
                    if s['extio'] == 0:
                        falling_during_extio += 1
                        if s['rd'] == 0:
                            falling_during_read += 1
            prev_sample = s

    print(f"IR4 RISING during EXTIO=0: {rising_during_extio}")
    print(f"IR4 FALLING during EXTIO=0: {falling_during_extio}")
    print(f"IR4 FALLING during READ (EXTIO=0, RD=0): {falling_during_read}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python analyze_ir4_detail.py <csv_file>")
        sys.exit(1)

    analyze_ir4_detail(sys.argv[1])

#!/usr/bin/env python3
"""
Analyze logic analyzer capture of DMA transfers to debug bit corruption.

This script parses DSView CSV exports and identifies:
- DMA write cycles (ALE high for address, WR low for data)
- Data bus values during T1 (address) and T2-T3 (data) phases
- Patterns in bit corruption
- Cycles where BD0-BD7 are all zero during write phase
- Double-write and address increment anomalies
"""

import csv
import sys
from dataclasses import dataclass, field
from typing import List, Optional
from collections import Counter

@dataclass
class DmaCycle:
    """Represents one DMA bus cycle"""
    cycle_num: int
    t1_start_time: float
    t1_address_byte: int  # BD0-BD7 during ALE (A0-A7)
    t2_start_time: float
    t2_data_byte: int     # BD0-BD7 during WR low
    t2_all_zeros: bool    # Flag if data was all zeros
    wr_duration_ns: float
    data_samples: List[int] = field(default_factory=list)

def get_row_value(row: dict, key: str) -> str:
    """Get value from row, trying with and without leading space"""
    if key in row:
        return row[key]
    if f' {key}' in row:
        return row[f' {key}']
    raise KeyError(f"Column '{key}' not found (tried with/without leading space)")

def parse_bd_value(row: dict) -> int:
    """Extract 8-bit value from BD0-BD7 columns"""
    value = 0
    for i in range(8):
        bit = int(get_row_value(row, f'BD{i}'))
        value |= (bit << i)
    return value

def analyze_capture(filename: str, max_cycles: int = 100) -> List[DmaCycle]:
    """Parse CSV and extract DMA write cycles (HOLD=0, HLDA=1)"""

    cycles = []
    cycle_num = 0

    # State machine
    state = 'IDLE'
    ale_rise_time = None
    ale_address = None
    wr_fall_time = None
    wr_data_samples = []

    prev_ale = 0
    prev_wr = 1

    print(f"Analyzing {filename}...")
    print(f"Looking for Pico DMA write cycles (HOLD=0, HLDA=1, ALE then WR)")
    print()

    line_count = 0
    skipped_non_dma = 0

    with open(filename, 'r') as f:
        # Skip comment lines starting with ;
        reader = csv.DictReader(
            (line for line in f if not line.startswith(';'))
        )

        for row in reader:
            line_count += 1
            if line_count % 1000000 == 0:
                print(f"  Processed {line_count:,} samples, found {cycle_num} cycles...")

            try:
                time = float(get_row_value(row, 'Time(s)'))
                ale = int(get_row_value(row, 'ALE'))
                wr = int(get_row_value(row, 'WR'))
                hold = int(get_row_value(row, 'HOLD'))
                hlda = int(get_row_value(row, 'HLDA'))
                bd_value = parse_bd_value(row)

                # Only look at Pico DMA cycles (HOLD=0 means Pico is requesting bus, HLDA=1 means granted)
                if hold != 0 or hlda != 1:
                    prev_ale = ale
                    prev_wr = wr
                    skipped_non_dma += 1
                    continue

                # Detect ALE rising edge (T1 start - address latch)
                if ale == 1 and prev_ale == 0:
                    if state == 'IDLE':
                        state = 'T1_ALE_HIGH'
                        ale_rise_time = time
                        ale_address = bd_value

                # Detect ALE falling edge (end of T1)
                if ale == 0 and prev_ale == 1:
                    if state == 'T1_ALE_HIGH':
                        state = 'WAIT_WR'

                # Detect WR falling edge (T2 start - write data)
                if wr == 0 and prev_wr == 1:
                    if state == 'WAIT_WR':
                        state = 'T2_WR_LOW'
                        wr_fall_time = time
                        wr_data_samples = [bd_value]

                # Collect samples while WR is low
                if state == 'T2_WR_LOW' and wr == 0:
                    wr_data_samples.append(bd_value)

                # Detect WR rising edge (end of write cycle)
                if wr == 1 and prev_wr == 0:
                    if state == 'T2_WR_LOW' and wr_data_samples:
                        # Use the most common value during WR low
                        data_counts = Counter(wr_data_samples)
                        most_common_data = data_counts.most_common(1)[0][0]

                        wr_rise_time = time
                        wr_duration = (wr_rise_time - wr_fall_time) * 1e9  # ns

                        all_zeros = all(d == 0 for d in wr_data_samples)

                        cycle = DmaCycle(
                            cycle_num=cycle_num,
                            t1_start_time=ale_rise_time,
                            t1_address_byte=ale_address,
                            t2_start_time=wr_fall_time,
                            t2_data_byte=most_common_data,
                            t2_all_zeros=all_zeros,
                            wr_duration_ns=wr_duration,
                            data_samples=wr_data_samples.copy()
                        )
                        cycles.append(cycle)
                        cycle_num += 1

                        if cycle_num >= max_cycles:
                            print(f"  Reached {max_cycles} cycles, stopping.")
                            break

                        state = 'IDLE'

                prev_ale = ale
                prev_wr = wr

            except (ValueError, KeyError) as e:
                continue

    print(f"  Total samples processed: {line_count:,}")
    print(f"  Samples skipped (not Pico DMA): {skipped_non_dma:,}")
    return cycles

def print_cycle_analysis(cycles: List[DmaCycle], expected_data: List[int] = None):
    """Print detailed analysis of DMA cycles"""

    print()
    print("=" * 110)
    print("DMA WRITE CYCLE ANALYSIS")
    print("=" * 110)
    print()
    print(f"{'Cyc':>4} {'Addr':>6} {'Data':>6} {'Data(bin)':>12} {'Zero?':>6} {'WR(ns)':>8} {'#Samp':>6} Notes")
    print("-" * 110)

    zero_cycles = []
    nonzero_cycles = []
    prev_addr = None
    duplicate_addr_count = 0
    addr_increment_issues = 0

    for i, c in enumerate(cycles):
        addr_hex = f"0x{c.t1_address_byte:02X}"
        data_hex = f"0x{c.t2_data_byte:02X}"
        data_bin = f"{c.t2_data_byte:08b}"
        all_zero = "YES" if c.t2_all_zeros else "no"

        notes = []
        if c.t2_all_zeros:
            notes.append("ALL ZEROS!")
            zero_cycles.append(c)
        else:
            nonzero_cycles.append(c)

        # Check for duplicate addresses
        if prev_addr is not None:
            if c.t1_address_byte == prev_addr:
                notes.append("DUP_ADDR")
                duplicate_addr_count += 1
            elif c.t1_address_byte != prev_addr + 1 and c.t1_address_byte != (prev_addr + 1) & 0xFF:
                diff = c.t1_address_byte - prev_addr
                notes.append(f"ADDR+{diff}")
                if diff != 1:
                    addr_increment_issues += 1

        # Show unique values seen during WR
        unique_vals = sorted(set(c.data_samples))
        if len(unique_vals) > 1:
            notes.append(f"varied: {', '.join(f'0x{v:02X}' for v in unique_vals[:4])}")

        notes_str = " | ".join(notes) if notes else ""
        print(f"{c.cycle_num:>4} {addr_hex:>6} {data_hex:>6} {data_bin:>12} {all_zero:>6} {c.wr_duration_ns:>8.1f} {len(c.data_samples):>6} {notes_str}")

        prev_addr = c.t1_address_byte

    print()
    print("=" * 110)
    print("SUMMARY")
    print("=" * 110)
    print(f"Total cycles analyzed:     {len(cycles)}")
    print(f"Cycles with data:          {len(nonzero_cycles)}")
    print(f"Cycles with all zeros:     {len(zero_cycles)}")
    print(f"Duplicate address writes:  {duplicate_addr_count}")
    print(f"Address increment issues:  {addr_increment_issues}")

    # Analyze address pattern
    addresses = [c.t1_address_byte for c in cycles]
    unique_addresses = sorted(set(addresses))
    print()
    print("Address pattern analysis:")
    print(f"  Unique addresses seen: {len(unique_addresses)}")
    if len(unique_addresses) >= 2:
        # Check if addresses are all even
        all_even = all(a % 2 == 0 for a in unique_addresses)
        all_odd = all(a % 2 == 1 for a in unique_addresses)
        if all_even:
            print(f"  WARNING: All addresses are EVEN - possible word-alignment issue!")
        elif all_odd:
            print(f"  WARNING: All addresses are ODD")

        # Check increment pattern
        increments = [addresses[i+1] - addresses[i] for i in range(len(addresses)-1) if addresses[i+1] != addresses[i]]
        if increments:
            common_inc = Counter(increments).most_common(1)[0]
            print(f"  Most common address increment: {common_inc[0]} ({common_inc[1]} occurrences)")

    # Analyze data for the pairs if we have duplicates
    if duplicate_addr_count > 0:
        print()
        print("Duplicate address data comparison:")
        i = 0
        pair_count = 0
        while i < len(cycles) - 1 and pair_count < 10:
            if cycles[i].t1_address_byte == cycles[i+1].t1_address_byte:
                d1 = cycles[i].t2_data_byte
                d2 = cycles[i+1].t2_data_byte
                addr = cycles[i].t1_address_byte
                match = "SAME" if d1 == d2 else "DIFFERENT"
                print(f"  Addr 0x{addr:02X}: data1=0x{d1:02X}, data2=0x{d2:02X} ({match})")
                pair_count += 1
                i += 2
            else:
                i += 1

    if expected_data:
        print()
        print("=" * 110)
        print("COMPARISON WITH EXPECTED DATA")
        print("=" * 110)
        print(f"{'Cyc':>4} {'Expected':>10} {'Actual':>10} {'XOR':>10} {'Match':>8}")
        print("-" * 110)

        mismatches = []
        for i, c in enumerate(cycles):
            if i < len(expected_data):
                exp = expected_data[i]
                act = c.t2_data_byte
                xor_val = exp ^ act
                match = "OK" if exp == act else "MISMATCH"

                exp_hex = f"0x{exp:02X}"
                act_hex = f"0x{act:02X}"
                xor_hex = f"0x{xor_val:02X}" if xor_val else "-"

                if exp != act:
                    mismatches.append((i, exp, act, xor_val))

                print(f"{i:>4} {exp_hex:>10} {act_hex:>10} {xor_hex:>10} {match:>8}")

        if mismatches:
            print()
            print("Bit error analysis:")
            bit_errors = [0] * 8
            for _, exp, act, xor_val in mismatches:
                for bit in range(8):
                    if xor_val & (1 << bit):
                        bit_errors[bit] += 1

            for bit in range(8):
                if bit_errors[bit] > 0:
                    bar = "#" * bit_errors[bit]
                    print(f"  Bit {bit} (BD{bit}): {bit_errors[bit]:>3} errors {bar}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_dma_capture.py <capture.csv> [max_cycles]")
        print()
        print("Example:")
        print("  python analyze_dma_capture.py '~/bus captures/first cycle.csv' 50")
        sys.exit(1)

    filename = sys.argv[1]
    max_cycles = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    # Expected data for LBA 0 (first 8 bytes)
    # From dma_verify.c expected_first8 for LBA 0: 02 00 01 00 74 61 6E 64
    expected_lba0 = [0x02, 0x00, 0x01, 0x00, 0x74, 0x61, 0x6E, 0x64]

    cycles = analyze_capture(filename, max_cycles)

    if not cycles:
        print("No DMA write cycles found in capture!")
        print("Make sure the capture includes ALE and WR signals during Pico DMA transfer (HOLD=0, HLDA=1).")
        sys.exit(1)

    # Use expected data for comparison if we have enough cycles
    print_cycle_analysis(cycles, expected_lba0 if len(cycles) >= 8 else None)

if __name__ == '__main__':
    main()

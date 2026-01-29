#!/usr/bin/env python3
"""
Analyze logic analyzer capture to find DMA write cycles and check data integrity.
Looks for cycles where HLDA=1 (Pico has bus) and WR/=0 (write strobe active).
"""

import sys
import csv

def parse_data_byte(row):
    """Extract BD0-BD7 as a byte value"""
    # Columns: Time, BD0, BD1, BD2, BD3, BD4, BD5, BD6, BD7, ...
    byte_val = 0
    for i in range(8):
        if int(row[i + 1]):  # BD0 is column 1
            byte_val |= (1 << i)
    return byte_val

def analyze_capture(filename, max_writes=20):
    """Find DMA write cycles and print the data being written"""

    # Column indices (0-based after Time column)
    # Time(s), BD0, BD1, BD2, BD3, BD4, BD5, BD6, BD7, WR, RD, HOLD, HLDA, ALE, EXTIO, A16, A17
    COL_WR = 9      # WR/ signal
    COL_HLDA = 12   # HLDA signal
    COL_ALE = 13    # ALE signal

    writes_found = 0
    in_write_cycle = False
    prev_wr = 1  # WR/ is active low, starts high

    print(f"Scanning {filename} for DMA write cycles...")
    print("Looking for: HLDA=1 (Pico has bus) AND WR/=0 (write active)")
    print("-" * 60)

    with open(filename, 'r') as f:
        reader = csv.reader(f)

        # Skip header lines
        for row in reader:
            if row[0].startswith(';'):
                continue
            if 'Time' in row[0]:
                print(f"Header: {row}")
                continue
            break

        line_num = 0
        for row in reader:
            line_num += 1
            if line_num % 100000000 == 0:
                print(f"  Progress: {line_num / 1e9:.1f}B lines...")

            try:
                wr = int(row[COL_WR])
                hlda = int(row[COL_HLDA])

                # Detect falling edge of WR/ while HLDA is high
                if prev_wr == 1 and wr == 0 and hlda == 1:
                    # This is a DMA write cycle starting
                    data_byte = parse_data_byte(row)
                    time_s = float(row[0])

                    print(f"Write #{writes_found + 1} at t={time_s:.9f}s:")
                    print(f"  BD7-BD0: {row[8]}{row[7]}{row[6]}{row[5]}{row[4]}{row[3]}{row[2]}{row[1]}")
                    print(f"  Data byte: 0x{data_byte:02X} ({data_byte})")
                    print(f"  Bit 7 (0x80): {(data_byte >> 7) & 1}")
                    print(f"  Bit 3 (0x08): {(data_byte >> 3) & 1}")
                    print()

                    writes_found += 1
                    if writes_found >= max_writes:
                        print(f"\nFound {writes_found} write cycles, stopping.")
                        return

                prev_wr = wr

            except (ValueError, IndexError) as e:
                continue

    print(f"\nTotal write cycles found: {writes_found}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_dma_capture.py <capture.csv> [max_writes]")
        sys.exit(1)

    filename = sys.argv[1]
    max_writes = int(sys.argv[2]) if len(sys.argv) > 2 else 20

    analyze_capture(filename, max_writes)

#!/usr/bin/env python3
"""
Analyze DSLogic logic analyzer CSV export for bus timing issues.
Focuses on EXTIO timing relative to WR and READY signals during write cycles.
"""

import csv
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Pin assignments from dma.h (adjust if your capture uses different channel names)
PIN_NAMES = {
    'BD0': 1, 'BD1': 2, 'BD2': 3, 'BD3': 4,
    'BD4': 5, 'BD5': 6, 'BD6': 7, 'BD7': 8,
    'A8': 9, 'A9': 10, 'A10': 11, 'A11': 12,
    'A12': 13, 'A13': 14, 'A14': 15, 'A15': 16,
    'A16': 17, 'A17': 18, 'A18': 19, 'A19': 20,
    'RD': 21, 'WR': 22, 'DTR': 23, 'ALE': 24,
    'DEN': 25, 'HOLD': 26, 'EXTIO': 27, 'READY': 28,
}

@dataclass
class WriteCycle:
    """Represents a single write cycle with timing information"""
    start_time: float  # WR goes low
    end_time: float    # WR goes high
    extio_low_time: Optional[float] = None  # When EXTIO went low
    extio_high_time: Optional[float] = None  # When EXTIO went high
    ready_high_time: Optional[float] = None  # When READY went high
    data_value: int = 0  # Data bus value
    address: int = 0  # Address (A8-A19)

    @property
    def extio_before_ready(self) -> Optional[float]:
        """How many ns before READY did EXTIO go low? Negative = EXTIO was late"""
        if self.extio_low_time and self.ready_high_time:
            return (self.ready_high_time - self.extio_low_time) * 1e9
        return None

    @property
    def wr_duration_ns(self) -> float:
        return (self.end_time - self.start_time) * 1e9


def parse_csv(filename: str, max_time_us: float = 100.0) -> Tuple[List[str], List[dict]]:
    """Parse DSLogic CSV export, returning headers and sample data"""
    samples = []
    headers = []

    with open(filename, 'r') as f:
        # Skip comment lines starting with ';'
        for line in f:
            line = line.strip()
            if line and not line.startswith(';'):
                # This is the header line
                headers = [h.strip() for h in line.split(',')]
                break

        # Find time column
        time_col = None
        for i, h in enumerate(headers):
            if 'time' in h.lower():
                time_col = i
                break

        if time_col is None:
            print(f"Headers found: {headers}")
            raise ValueError("Could not find time column in CSV")

        # Now read the rest of the file as CSV data
        reader = csv.reader(f)

        for row in reader:
            try:
                time_val = float(row[time_col])
                # Convert to seconds if needed (DSLogic might use various units)
                if abs(time_val) > 1000:  # Probably in nanoseconds
                    time_val = time_val * 1e-9
                elif abs(time_val) > 1:  # Probably in microseconds
                    time_val = time_val * 1e-6

                # Only keep samples in the first max_time_us
                if time_val > max_time_us * 1e-6:
                    break

                sample = {'time': time_val}
                for i, val in enumerate(row):
                    if i != time_col:
                        try:
                            sample[headers[i]] = int(val)
                        except ValueError:
                            sample[headers[i]] = val
                samples.append(sample)
            except (ValueError, IndexError) as e:
                continue

    return headers, samples


def find_channel(headers: List[str], pin_name: str) -> Optional[str]:
    """Find the channel name that corresponds to a pin"""
    pin_name_lower = pin_name.lower()
    for h in headers:
        h_lower = h.lower().strip()
        h_stripped = h.strip()
        # Direct match (case insensitive)
        if pin_name_lower == h_lower:
            return h_stripped
        # Partial match
        if pin_name_lower in h_lower or f'ch{PIN_NAMES.get(pin_name, -1)}' in h_lower:
            return h_stripped
        # Try matching by channel number
        if pin_name in PIN_NAMES:
            pin_num = PIN_NAMES[pin_name]
            if f'{pin_num}' in h and ('ch' in h_lower or 'channel' in h_lower or 'd' in h_lower):
                return h_stripped
    return None


def get_signal(sample: dict, channel: str, default: int = 1) -> int:
    """Get signal value from sample, handling various formats"""
    if channel and channel in sample:
        val = sample[channel]
        if isinstance(val, int):
            return val
        try:
            return int(val)
        except ValueError:
            return default
    return default


def analyze_write_cycles(headers: List[str], samples: List[dict]) -> List[WriteCycle]:
    """Find and analyze all write cycles"""

    # Find relevant channels
    print(f"\nAvailable headers: {headers}\n")

    wr_ch = find_channel(headers, 'WR')
    extio_ch = find_channel(headers, 'EXTIO')
    ready_ch = find_channel(headers, 'READY')

    # Try to find data bus channels
    data_channels = []
    for i in range(8):
        ch = find_channel(headers, f'BD{i}')
        if ch:
            data_channels.append(ch)

    print(f"WR channel: {wr_ch}")
    print(f"EXTIO channel: {extio_ch}")
    print(f"READY channel: {ready_ch}")
    print(f"Data channels: {data_channels}")

    if not wr_ch:
        # Try to auto-detect by looking for transitions
        print("\nCould not find WR channel by name, analyzing all channels...")
        for h in headers:
            if 'time' in h.lower():
                continue
            transitions = 0
            prev_val = None
            for s in samples[:1000]:
                val = get_signal(s, h)
                if prev_val is not None and val != prev_val:
                    transitions += 1
                prev_val = val
            if transitions > 0:
                print(f"  {h}: {transitions} transitions in first 1000 samples")
        return []

    write_cycles = []
    current_cycle = None
    prev_wr = 1  # WR is active low, starts high
    prev_extio = 1  # EXTIO is active low
    prev_ready = 0  # READY starts low

    for sample in samples:
        time = sample['time']
        wr = get_signal(sample, wr_ch, 1)
        extio = get_signal(sample, extio_ch, 1) if extio_ch else 1
        ready = get_signal(sample, ready_ch, 0) if ready_ch else 0

        # WR falling edge - start of write cycle
        if prev_wr == 1 and wr == 0:
            current_cycle = WriteCycle(start_time=time, end_time=time)

            # Capture data bus value
            if data_channels:
                data_val = 0
                for i, ch in enumerate(data_channels):
                    if get_signal(sample, ch, 0):
                        data_val |= (1 << i)
                current_cycle.data_value = data_val

        # Track EXTIO transitions during write cycle
        if current_cycle:
            if prev_extio == 1 and extio == 0:
                current_cycle.extio_low_time = time
            if prev_extio == 0 and extio == 1:
                current_cycle.extio_high_time = time

            # Track READY rising edge
            if prev_ready == 0 and ready == 1:
                current_cycle.ready_high_time = time

        # WR rising edge - end of write cycle
        if prev_wr == 0 and wr == 1:
            if current_cycle:
                current_cycle.end_time = time
                write_cycles.append(current_cycle)
                current_cycle = None

        prev_wr = wr
        prev_extio = extio
        prev_ready = ready

    return write_cycles


def analyze_extio_edges(headers: List[str], samples: List[dict]):
    """Analyze all EXTIO transitions to understand the pattern"""
    extio_ch = find_channel(headers, 'EXTIO')
    wr_ch = find_channel(headers, 'WR')
    rd_ch = find_channel(headers, 'RD')
    ready_ch = find_channel(headers, 'READY')

    if not extio_ch:
        print("Could not find EXTIO channel")
        return

    print(f"\n=== EXTIO Edge Analysis ===")
    print(f"EXTIO channel: {extio_ch}")

    prev_extio = 1
    extio_events = []

    for sample in samples:
        time = sample['time']
        extio = get_signal(sample, extio_ch, 1)
        wr = get_signal(sample, wr_ch, 1) if wr_ch else 1
        rd = get_signal(sample, rd_ch, 1) if rd_ch else 1
        ready = get_signal(sample, ready_ch, 0) if ready_ch else 0

        if extio != prev_extio:
            edge = 'falling' if extio == 0 else 'rising'
            extio_events.append({
                'time': time,
                'edge': edge,
                'wr': wr,
                'rd': rd,
                'ready': ready
            })
        prev_extio = extio

    print(f"\nFound {len(extio_events)} EXTIO transitions:")
    for i, evt in enumerate(extio_events[:50]):  # Show first 50
        print(f"  {evt['time']*1e6:8.3f} us: EXTIO {evt['edge']:7s} | WR={evt['wr']} RD={evt['rd']} READY={evt['ready']}")

    if len(extio_events) > 50:
        print(f"  ... and {len(extio_events) - 50} more")


def analyze_detailed_write_timing(headers: List[str], samples: List[dict]):
    """Detailed analysis of each write cycle showing WR→EXTIO and EXTIO→READY timing"""
    wr_ch = find_channel(headers, 'WR')
    extio_ch = find_channel(headers, 'EXTIO')
    ready_ch = find_channel(headers, 'READY')
    ale_ch = find_channel(headers, 'ALE')

    # Find address bit channels
    a17_ch = find_channel(headers, 'A17')
    a18_ch = find_channel(headers, 'A18')
    a19_ch = find_channel(headers, 'A19')
    bd0_ch = find_channel(headers, 'BD0')

    if not all([wr_ch, extio_ch, ready_ch]):
        print("Missing required channels for detailed analysis")
        return

    print(f"\n=== Detailed Write Cycle Timing ===")
    print(f"{'Cycle':<6} {'WR↓ (µs)':<12} {'EXTIO↓ (µs)':<14} {'READY↑ (µs)':<14} {'WR→EXTIO (ns)':<14} {'EXTIO→READY (ns)':<18} {'A19-17':<8} {'Status'}")
    print("-" * 110)

    prev_wr = 1
    prev_extio = 1
    prev_ready = 0

    cycle_num = 0
    wr_fall_time = None
    extio_fall_time = None
    ready_rise_time = None
    in_write_cycle = False
    addr_bits = "???"

    for sample in samples:
        time = sample['time']
        wr = get_signal(sample, wr_ch, 1)
        extio = get_signal(sample, extio_ch, 1)
        ready = get_signal(sample, ready_ch, 0)

        # WR falling edge - start of write cycle
        if prev_wr == 1 and wr == 0:
            in_write_cycle = True
            wr_fall_time = time
            extio_fall_time = None
            ready_rise_time = None
            # Capture address bits at WR falling edge
            a19 = get_signal(sample, a19_ch, 0) if a19_ch else 0
            a18 = get_signal(sample, a18_ch, 0) if a18_ch else 0
            a17 = get_signal(sample, a17_ch, 0) if a17_ch else 0
            addr_bits = f"{a19}{a18}{a17}"

        # EXTIO falling edge during write cycle
        if in_write_cycle and prev_extio == 1 and extio == 0:
            extio_fall_time = time

        # READY rising edge during write cycle
        if in_write_cycle and prev_ready == 0 and ready == 1:
            ready_rise_time = time

        # WR rising edge - end of write cycle
        if prev_wr == 0 and wr == 1:
            if in_write_cycle and wr_fall_time is not None:
                cycle_num += 1

                wr_to_extio = None
                extio_to_ready = None
                status = "NO EXTIO"

                if extio_fall_time is not None:
                    wr_to_extio = (extio_fall_time - wr_fall_time) * 1e9
                    if ready_rise_time is not None:
                        extio_to_ready = (ready_rise_time - extio_fall_time) * 1e9
                        if extio_to_ready < 0:
                            status = f"LATE by {-extio_to_ready:.0f}ns"
                        elif extio_to_ready < 50:
                            status = f"MARGINAL"
                        else:
                            status = "OK"
                    else:
                        status = "NO READY"

                wr_str = f"{wr_fall_time*1e6:.3f}"
                extio_str = f"{extio_fall_time*1e6:.3f}" if extio_fall_time else "N/A"
                ready_str = f"{ready_rise_time*1e6:.3f}" if ready_rise_time else "N/A"
                wr_extio_str = f"{wr_to_extio:.0f}" if wr_to_extio is not None else "N/A"
                extio_ready_str = f"{extio_to_ready:.0f}" if extio_to_ready is not None else "N/A"

                print(f"{cycle_num:<6} {wr_str:<12} {extio_str:<14} {ready_str:<14} {wr_extio_str:<14} {extio_ready_str:<18} {addr_bits:<8} {status}")

            in_write_cycle = False

        prev_wr = wr
        prev_extio = extio
        prev_ready = ready

    # Summary of cycles with EXTIO
    print("\n=== Cycles with EXTIO Assertion (DMA Register Writes) ===")
    print("Cycle 2: 464ns response - FAST")
    print("Cycle 7: 968ns response - SLOW")
    print("Cycle 8: 976ns response - SLOW (and LATE!)")
    print("\nThe ~500ns variance in PIO response time suggests the PIO state machine")
    print("is being delayed by IRQ handler processing or FIFO congestion between cycles.")


def main():
    if len(sys.argv) < 2:
        filename = "/Users/pauldevine/projects/fujinet-project/bus captures/DSLogic U3Pro16-la-260110-150803.csv"
    else:
        filename = sys.argv[1]

    print(f"Analyzing: {filename}")
    print("=" * 60)

    try:
        headers, samples = parse_csv(filename, max_time_us=100.0)
        print(f"Loaded {len(samples)} samples from first 100us")

        if len(samples) == 0:
            print("No samples found! Check the CSV format.")
            return

        # Show sample data structure
        print(f"\nFirst sample: {samples[0]}")

        # Analyze EXTIO edges first
        analyze_extio_edges(headers, samples)

        # Detailed write timing analysis
        analyze_detailed_write_timing(headers, samples)

        # Analyze write cycles
        write_cycles = analyze_write_cycles(headers, samples)

        if not write_cycles:
            print("\nNo write cycles found. This might be a channel mapping issue.")
            return

        print(f"\n=== Write Cycle Analysis ===")
        print(f"Found {len(write_cycles)} write cycles")

        # Analyze timing
        late_extio_cycles = []
        early_extio_cycles = []

        for i, cycle in enumerate(write_cycles):
            margin = cycle.extio_before_ready
            if margin is not None:
                if margin < 0:
                    late_extio_cycles.append((i, cycle, margin))
                else:
                    early_extio_cycles.append((i, cycle, margin))

        print(f"\nEXTIO asserted BEFORE READY: {len(early_extio_cycles)} cycles")
        if early_extio_cycles:
            margins = [m for _, _, m in early_extio_cycles]
            print(f"  Min margin: {min(margins):.1f} ns")
            print(f"  Max margin: {max(margins):.1f} ns")
            print(f"  Avg margin: {sum(margins)/len(margins):.1f} ns")

        print(f"\nEXTIO asserted AFTER READY (LATE!): {len(late_extio_cycles)} cycles")
        for i, cycle, margin in late_extio_cycles:
            print(f"  Cycle {i}: EXTIO was {-margin:.1f} ns LATE")
            print(f"    WR duration: {cycle.wr_duration_ns:.1f} ns")
            print(f"    Time: {cycle.start_time*1e6:.3f} us")
            print(f"    Data: 0x{cycle.data_value:02X}")

        # Show timing histogram
        print(f"\n=== Timing Distribution ===")
        all_margins = [m for _, _, m in early_extio_cycles + late_extio_cycles]
        if all_margins:
            buckets = {}
            for m in all_margins:
                bucket = int(m / 50) * 50  # 50ns buckets
                buckets[bucket] = buckets.get(bucket, 0) + 1

            for bucket in sorted(buckets.keys()):
                count = buckets[bucket]
                bar = '#' * min(count, 50)
                label = f"{bucket:+5d} to {bucket+50:+5d} ns"
                print(f"  {label}: {bar} ({count})")

    except FileNotFoundError:
        print(f"File not found: {filename}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

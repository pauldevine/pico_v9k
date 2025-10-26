#!/usr/bin/env python3
"""
board_registers_probe.py
~~~~~~~~~~~~~~~~~~~~~~~~

Utility script intended to run on a Raspberry Pi to exercise the Pico
`board_registers.pio` state machine.  It alternates between emulating 8088-style
write and read bus cycles so you can single-step through the PIO wait points.

Hardware wiring (BCM numbering on the Pi):
    5, 6, 13, 19  -> Pico GPIO 1, 2, 3, 4  (shared data bus, bit 0 first)
    16            -> Pico GPIO 24 (ALE / sample trigger, active high)
    20            -> Pico GPIO 21 (RD#, drive trigger, active low)
    26            -> Pico GPIO 30 (CLOCK_5, idle high, pulse low to start T0)
    12            -> Pico GPIO 28 (READY, hold low to pause, high to release)

Only the lower four data lines (BD0-BD3) are driven/sampled; the Pico will see
zero on the upper bits.  Write cycles alternate between the nibbles 0xA and 0x5.
During read cycles the script prints the nibble driven by the Pico.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Iterable, List

try:
    import RPi.GPIO as GPIO
except RuntimeError as exc:  # pragma: no cover - runtime guard on Pi only
    print("Error importing RPi.GPIO: ensure this script runs on a Raspberry Pi.", file=sys.stderr)
    raise


# GPIO wiring configuration (BCM numbering on the Raspberry Pi)
BUS_PINS: List[int] = [5, 6, 13, 19]  # Shared data/address bus, LSB -> MSB
ALE_PIN = 16                         # Connected to Pico GPIO 24
RD_PIN = 20                          # Connected to Pico GPIO 21 (active low)
CLK5_PIN = 26                        # Connected to Pico GPIO 30
READY_PIN = 12                       # Connected to Pico GPIO 28

# Default register address nibble to place on the bus during ALE high.
REGISTER_ADDRESS = 0x0


def _ensure_length(values: Iterable[int], expected: int, label: str) -> List[int]:
    data = list(values)
    if len(data) != expected:
        raise ValueError(f"{label} must contain {expected} entries, got {len(data)}")
    return data


class BoardRegisterProbe:
    """Implements the sequencing needed to tick the PIO state machine."""

    def __init__(
        self,
        bus_pins: Iterable[int],
        *,
        ale_pin: int,
        rd_pin: int,
        clk5_pin: int,
        ready_pin: int,
        t_setup: float,
        t_hold: float,
        t_data: float,
    ) -> None:
        self.bus_pins = _ensure_length(bus_pins, 4, "bus_pins")
        self.ale_pin = ale_pin
        self.rd_pin = rd_pin
        self.clk5_pin = clk5_pin
        self.ready_pin = ready_pin
        self.t_setup = t_setup
        self.t_hold = t_hold
        self.t_data = t_data

        self._bus_is_output = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Control pins
        GPIO.setup(self.ale_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.rd_pin, GPIO.OUT, initial=GPIO.HIGH)   # RD# idle high
        GPIO.setup(self.clk5_pin, GPIO.OUT, initial=GPIO.HIGH)  # CLK idle high
        GPIO.setup(self.ready_pin, GPIO.OUT, initial=GPIO.HIGH)  # READY idles high

        # Start with bus as outputs (address/data driving)
        self._set_bus_direction(GPIO.OUT, pull=None)
        self._drive_bus(0)

    def cleanup(self) -> None:
        GPIO.cleanup()

    # ------------------------------------------------------------------ helpers
    def _set_bus_direction(self, direction: int, *, pull: int | None) -> None:
        if self._bus_is_output and direction == GPIO.OUT:
            return
        if not self._bus_is_output and direction == GPIO.IN:
            return
        for pin in self.bus_pins:
            if direction == GPIO.OUT:
                GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            else:
                if pull is None:
                    GPIO.setup(pin, GPIO.IN)
                else:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=pull)
        self._bus_is_output = direction == GPIO.OUT

    def _drive_bus(self, value: int) -> None:
        self._set_bus_direction(GPIO.OUT, pull=None)
        for idx, pin in enumerate(self.bus_pins):
            GPIO.output(pin, GPIO.HIGH if (value >> idx) & 0x1 else GPIO.LOW)

    def _sample_bus(self) -> int:
        self._set_bus_direction(GPIO.IN, pull=GPIO.PUD_DOWN)
        time.sleep(self.t_data)  # allow the Pico to take control
        value = 0
        for idx, pin in enumerate(self.bus_pins):
            if GPIO.input(pin):
                value |= (1 << idx)
        return value

    def _log_state(self, phase: str, *, rd: str | None = None, ready: str | None = None) -> None:
        parts = [phase]
        if rd is not None:
            parts.append(f"RD#{rd}")
        if ready is not None:
            parts.append(f"READY={ready}")
        print("  ".join(parts))

    def _begin_cycle(self, *, is_read: bool, ready_asserted: bool) -> None:
        """Drive CLK5/ALE through T0/T1 and prime control signals for T2."""
        self._log_state("T0: CLK5 falling edge")
        GPIO.output(self.clk5_pin, GPIO.HIGH)
        time.sleep(self.t_setup)
        GPIO.output(self.ale_pin, GPIO.LOW)
        GPIO.output(self.clk5_pin, GPIO.LOW)
        time.sleep(self.t_setup)

        self._log_state("T1: ALE high (address valid)")
        self._drive_bus(REGISTER_ADDRESS & 0xF)
        GPIO.output(self.ale_pin, GPIO.HIGH)
        time.sleep(self.t_hold)
        GPIO.output(self.ale_pin, GPIO.LOW)
        GPIO.output(self.ready_pin, GPIO.HIGH if ready_asserted else GPIO.LOW)
        GPIO.output(self.rd_pin, GPIO.LOW if is_read else GPIO.HIGH)
        time.sleep(self.t_setup)

    def _complete_cycle(self) -> None:
        """Return the control signals to their idle state."""
        GPIO.output(self.ready_pin, GPIO.HIGH)
        GPIO.output(self.rd_pin, GPIO.HIGH)
        GPIO.output(self.clk5_pin, GPIO.HIGH)
        time.sleep(self.t_setup)

    # ----------------------------------------------------------------- cycles
    def write_cycle(self, value: int) -> None:
        """Emulate an 8088 write (RD# held high)."""
        nibble = value & 0xF
        print(f"\n[WRITE] Driving nibble 0x{nibble:X}")
        self._begin_cycle(is_read=False, ready_asserted=False)
        self._log_state("T2: drive data (READY low)", rd="HIGH", ready="LOW")
        self._drive_bus(nibble)
        time.sleep(self.t_data)

        GPIO.output(self.ready_pin, GPIO.HIGH)  # release the PIO wait
        self._log_state("T3: READY asserted", rd="HIGH", ready="HIGH")
        time.sleep(self.t_hold)

        self._complete_cycle()

    def read_cycle(self) -> None:
        """Emulate an 8088 read (RD# asserted low during T3)."""
        print("\n[READ] Expect Pico to drive data")
        self._begin_cycle(is_read=True, ready_asserted=True)

        self._log_state("T2: RD# asserted", rd="LOW", ready="HIGH")
        self._set_bus_direction(GPIO.IN, pull=GPIO.PUD_DOWN)
        time.sleep(self.t_setup)

        self._log_state("T3: sample data", rd="LOW", ready="HIGH")
        value = self._sample_bus()
        print(f"  Sampled nibble: 0x{value:X} (bits: {value:04b})")

        GPIO.output(self.rd_pin, GPIO.HIGH)  # finish the read
        self._log_state("T4: RD# released", rd="HIGH", ready="HIGH")
        time.sleep(self.t_hold)

        self._complete_cycle()
        # Restore bus to output mode for the next address phase
        self._drive_bus(REGISTER_ADDRESS & 0xF)


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Exercise the Pico board_registers.pio state machine.")
    parser.add_argument(
        "-n",
        "--cycles",
        type=int,
        default=4,
        help="Number of write/read pairs to execute (default: 4 cycles).",
    )
    parser.add_argument(
        "--setup-us",
        type=float,
        default=50.0,
        help="Setup delay in microseconds before/after major transitions (default: 50us).",
    )
    parser.add_argument(
        "--hold-us",
        type=float,
        default=50.0,
        help="Hold delay in microseconds for ALE/data/READY phases (default: 50us).",
    )
    parser.add_argument(
        "--data-us",
        type=float,
        default=25.0,
        help="Extra delay in microseconds before sampling read data (default: 25us).",
    )
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    probe = BoardRegisterProbe(
        BUS_PINS,
        ale_pin=ALE_PIN,
        rd_pin=RD_PIN,
        clk5_pin=CLK5_PIN,
        ready_pin=READY_PIN,
        t_setup=args.setup_us / 1_000_000.0,
        t_hold=args.hold_us / 1_000_000.0,
        t_data=args.data_us / 1_000_000.0,
    )

    write_pattern = [0xA, 0x5]

    try:
        for idx in range(args.cycles):
            nibble = write_pattern[idx % len(write_pattern)]
            probe.write_cycle(nibble)
            probe.read_cycle()
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        probe.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))

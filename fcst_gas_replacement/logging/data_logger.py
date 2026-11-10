#!/usr/bin/env python3
"""Log FCST MFC setpoints and indicated-flow readings to a whitespace DAT file.

The logged flow values are converted from the FCST 16-bit register scale to
sccm using each MFC's configured full-scale value. No gas-specific CF
correction is applied to either Setpoint or Indicated Flow.

Run from the project root:
    python3 -m data.data_logger --interval 1.0

The logger performs a read-only identity/status check once at startup, then
reads only Setpoint and Indicated Flow from each active MFC on every cycle.
It never sends WRITE commands.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Protocol

from config.mfc_profiles import MFC_PROFILES, MFCProfile
from mfc.mfc_manager import MFCManager, raw_to_sccm


DEFAULT_OUTPUT_FILE = Path("data/mfc_flow.dat")
DEFAULT_INTERVAL_SECONDS = 1.0


class FlowReadDriver(Protocol):
    """Minimum FCST driver interface required by this logger."""

    def read_setpoint_raw(self, mac_id: int) -> int: ...

    def read_indicated_flow_raw(self, mac_id: int) -> int: ...


@dataclass(frozen=True, slots=True)
class MFCFlowReading:
    """One MFC's Setpoint and Indicated Flow, expressed as sccm-equivalent."""

    setpoint_sccm: float
    indicated_flow_sccm: float


class MFCDataLogger:
    """Append one row per sampling cycle to a whitespace-separated DAT file.

    Columns are:
        UnixTime[s],
        Cir01_Setpoint[sccm], Cir01_IndicatedFlow[sccm],
        Cir02_Setpoint[sccm], Cir02_IndicatedFlow[sccm],
        Exh01_Setpoint[sccm], Exh01_IndicatedFlow[sccm]

    Setpoint is the flow command currently stored in the MFC.
    Indicated Flow is the value read from the MFC's Indicated Flow register.
    Both are converted linearly using the MFC full-scale value, but neither
    value is CF-corrected.
    """

    def __init__(
        self,
        driver: FlowReadDriver,
        *,
        output_file: str | Path = DEFAULT_OUTPUT_FILE,
        interval_seconds: float = DEFAULT_INTERVAL_SECONDS,
        profiles: Mapping[str, MFCProfile] = MFC_PROFILES,
        verify_identity: bool = True,
    ) -> None:
        if not math.isfinite(interval_seconds) or interval_seconds <= 0:
            raise ValueError("interval_seconds must be a finite positive number.")

        active_profiles = {
            name: profile for name, profile in profiles.items() if profile.active
        }
        if not active_profiles:
            raise ValueError("No active MFC profiles are configured.")

        self.driver = driver
        self.output_file = Path(output_file)
        self.interval_seconds = float(interval_seconds)
        self.profiles = active_profiles
        self.verify_identity = verify_identity

    def verify_devices(self) -> None:
        """Verify the serial number and MAC ID of each configured MFC once."""
        if not self.verify_identity:
            return

        manager = MFCManager(self.driver, profiles=dict(self.profiles))
        statuses = manager.read_all_status()
        for name, status in statuses.items():
            print(
                f"[{name}] identity verified: serial={status.serial_number}, "
                f"MAC=0x{status.mac_id:02X}"
            )

    def _columns(self) -> list[str]:
        columns = ["UnixTime_s"]
        for name in self.profiles:
            columns.extend((
                f"{name}_Setpoint_sccm",
                f"{name}_IndicatedFlow_sccm",
            ))
        return columns

    def _ensure_output_file(self) -> None:
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        if not self.output_file.exists() or self.output_file.stat().st_size == 0:
            with self.output_file.open("a", encoding="utf-8") as stream:
                stream.write("# " + " ".join(self._columns()) + "\n")
                stream.flush()

    def write_sample(self) -> dict[str, MFCFlowReading | None]:
        """Read all active MFCs once and append one row to the DAT file.

        If one register read fails, the corresponding value is written as
        ``nan`` and the error is printed to stderr. The remaining registers
        and MFCs are still attempted, so one communication fault does not
        silently discard the other readings in that cycle.
        """
        timestamp = time.time()
        row_values: list[str] = [f"{timestamp:.3f}"]
        results: dict[str, MFCFlowReading | None] = {}

        for name, profile in self.profiles.items():
            setpoint_value = math.nan
            indicated_value = math.nan
            setpoint_ok = False
            indicated_ok = False

            try:
                setpoint_raw = self.driver.read_setpoint_raw(profile.mac_id)
                setpoint_value = raw_to_sccm(
                    setpoint_raw, profile.full_scale_sccm
                )
                setpoint_ok = True
            except Exception as exc:  # keep recording the other channels
                print(
                    f"[{name}] Setpoint read failed: {exc}",
                    file=sys.stderr,
                )

            try:
                indicated_raw = self.driver.read_indicated_flow_raw(profile.mac_id)
                indicated_value = raw_to_sccm(
                    indicated_raw, profile.full_scale_sccm
                )
                indicated_ok = True
            except Exception as exc:  # keep recording the other channels
                print(
                    f"[{name}] Indicated Flow read failed: {exc}",
                    file=sys.stderr,
                )

            row_values.extend((
                f"{setpoint_value:.3f}" if setpoint_ok else "nan",
                f"{indicated_value:.3f}" if indicated_ok else "nan",
            ))

            results[name] = (
                MFCFlowReading(setpoint_value, indicated_value)
                if setpoint_ok and indicated_ok
                else None
            )

        self._ensure_output_file()
        with self.output_file.open("a", encoding="utf-8") as stream:
            stream.write(" ".join(row_values) + "\n")
            stream.flush()

        return results

    def run(
        self,
        *,
        duration_seconds: float | None = None,
        max_samples: int | None = None,
    ) -> int:
        """Run the logger at the configured interval until stopped.

        Returns the number of sampling cycles written. Use Ctrl+C to stop a
        continuous run. If acquisition takes longer than the requested
        interval, the logger reports an overrun and starts the next cycle
        immediately; it does not create catch-up bursts.
        """
        if duration_seconds is not None and (
            not math.isfinite(duration_seconds) or duration_seconds <= 0
        ):
            raise ValueError("duration_seconds must be a finite positive number.")
        if max_samples is not None and max_samples <= 0:
            raise ValueError("max_samples must be a positive integer.")

        self.verify_devices()
        self._ensure_output_file()

        print(f"Output file : {self.output_file}")
        print(f"Interval    : {self.interval_seconds:g} s")
        print("Logging Setpoint and Indicated Flow. CF correction is disabled.")
        print("Press Ctrl+C to stop.")

        start_monotonic = time.monotonic()
        next_sample = start_monotonic
        sample_count = 0

        while True:
            if duration_seconds is not None:
                if time.monotonic() - start_monotonic >= duration_seconds:
                    break
            if max_samples is not None and sample_count >= max_samples:
                break

            cycle_start = time.monotonic()
            self.write_sample()
            sample_count += 1

            next_sample += self.interval_seconds
            now = time.monotonic()
            delay = next_sample - now
            if delay > 0:
                time.sleep(delay)
            else:
                print(
                    f"[WARN] Sampling cycle {sample_count} exceeded the "
                    f"{self.interval_seconds:g} s interval "
                    f"(cycle time: {now - cycle_start:.3f} s).",
                    file=sys.stderr,
                )
                # Avoid rapid catch-up cycles after an overrun.
                next_sample = time.monotonic()

        print(f"Logging stopped. Samples written: {sample_count}")
        return sample_count


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Log FCST MFC Setpoint and Indicated Flow values to a DAT file "
            "without gas-specific CF correction."
        )
    )
    parser.add_argument(
        "--port",
        default=None,
        help="RS-485 serial port (default: auto-detect B&B ULINX USOPTL4).",
    )
    parser.add_argument(
        "--output",
        default=str(DEFAULT_OUTPUT_FILE),
        help=f"Output DAT file (default: {DEFAULT_OUTPUT_FILE}).",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=DEFAULT_INTERVAL_SECONDS,
        help="Sampling interval in seconds (default: 1.0).",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Stop after this many seconds (default: run until Ctrl+C).",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=None,
        help="Stop after this many samples (useful for a short test).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print RS-485 TX/RX frames (debugging only).",
    )
    return parser


def main() -> int:
    parser = _build_argument_parser()
    args = parser.parse_args()

    try:
        from mfc.fcst_driver import FCSTDriver

        with FCSTDriver(port=args.port, verbose=args.verbose) as driver:
            logger = MFCDataLogger(
                driver,
                output_file=args.output,
                interval_seconds=args.interval,
            )
            logger.run(
                duration_seconds=args.duration,
                max_samples=args.samples,
            )
    except KeyboardInterrupt:
        print("\nInterrupted by user. Logger stopped.")
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

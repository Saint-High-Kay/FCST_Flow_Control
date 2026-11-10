#!/usr/bin/env python3
"""CDC pressure feedback decision module for the COMET gas-circulation system.

This module reads the existing SlowMonitor GL840 log file.  It does not
communicate with the GL840 and it does not write to any MFC.

The pressure controller returns a proposed *total mixture flow* in sccm.
The caller is responsible for converting that target into individual MFC
setpoints (for example, through gas_replacement.py and MFCManager).

Run a one-shot dry run from the project root:
    python3 control/pressure_controller.py --current-flow-sccm 200

The MFC flow-range constants below are based on the currently configured
hardware and Fujikin conversion factors.  Keep them synchronized with
config/mfc_profiles.py if those profiles are changed.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


DEFAULT_GL840_FILE = Path("/home/cdc/SlowMonitor/dat/gl840.dat")
DEFAULT_MAX_DATA_AGE_S = 120.0

# CDC gas mixture: He : i-C4H10 = 90 : 10.
HE_FRACTION = 0.90
IC4H10_FRACTION = 0.10

# Fujikin conversion factors supplied for the current MFC calibration.
# Cir01 and Cir02 are calibrated for the 90:10 mixture but carry pure He or
# pure i-C4H10, respectively.  Exh01 is calibrated for the 90:10 mixture.
MIXTURE_CF = 0.946
HE_CF = 1.428
IC4H10_CF = 0.210
EXHAUST_MIXTURE_CF = 0.946

# Positive-flow limits are 2 % to 100 % of full scale.
# Values are the MFC setpoint-equivalent flow ranges in sccm.
CIR01_FS_SCCM = 5000.0
CIR01_MIN_SCCM = 100.0
CIR02_FS_SCCM = 400.0
CIR02_MIN_SCCM = 8.0
EXH01_FS_SCCM = 1000.0
EXH01_MIN_SCCM = 20.0


def _setpoint_fraction(component_fraction: float, component_cf: float) -> float:
    """MFC setpoint flow / total target-mixture flow."""
    return component_fraction * MIXTURE_CF / component_cf


# Setpoint-equivalent MFC flow for each sccm of total 90:10 mixture.
_CIR01_PER_TOTAL = _setpoint_fraction(HE_FRACTION, HE_CF)
_CIR02_PER_TOTAL = _setpoint_fraction(IC4H10_FRACTION, IC4H10_CF)
_EXH01_PER_TOTAL = MIXTURE_CF / EXHAUST_MIXTURE_CF

# The simultaneous operating envelope is the intersection of all 3 MFC ranges.
# A total flow of exactly zero is also a valid dedicated MFC stop command.
MIN_ACTIVE_TOTAL_FLOW_SCCM = max(
    CIR01_MIN_SCCM / _CIR01_PER_TOTAL,
    CIR02_MIN_SCCM / _CIR02_PER_TOTAL,
    EXH01_MIN_SCCM / _EXH01_PER_TOTAL,
)
MAX_TOTAL_FLOW_SCCM = min(
    CIR01_FS_SCCM / _CIR01_PER_TOTAL,
    CIR02_FS_SCCM / _CIR02_PER_TOTAL,
    EXH01_FS_SCCM / _EXH01_PER_TOTAL,
)


class PressureDataError(RuntimeError):
    """Raised when the GL840 log is missing, stale, incomplete, or invalid."""


class FlowRangeError(ValueError):
    """Raised when the supplied current flow is outside the supported range."""


@dataclass(frozen=True)
class PressureReading:
    timestamp: float
    cdc_pressure_pa: float
    buffer_pressure_pa: float
    age_s: float


@dataclass(frozen=True)
class PressureDecision:
    reading: PressureReading
    current_total_flow_sccm: float
    proposed_total_flow_sccm: float
    action: str
    reason: str
    flow_limit_reached: bool


class PressureController:
    """Read CDC pressure and calculate a safe next total-mixture-flow target.

    No MFC write operation is performed by this class.

    Initial feedback policy (intentionally configurable):
      CDC pressure > 120 Pa : reduce current total flow by 10 %
      CDC pressure > 100 Pa : reduce current total flow by 2 %
      CDC pressure <   0 Pa : increase current total flow by 20 %
      CDC pressure <  20 Pa : increase current total flow by 10 %
      CDC pressure <  50 Pa : increase current total flow by 5 %
      otherwise             : hold current total flow

    The pressure bands follow the structure of the previous feedback_control.py.
    The percentage steps are provisional starting values for the new MFCs and
    must be evaluated using dry-run logs before enabling physical MFC writes.
    """

    def __init__(
        self,
        data_file: Path | str = DEFAULT_GL840_FILE,
        max_data_age_s: float = DEFAULT_MAX_DATA_AGE_S,
        target_pressure_pa: float = 100.0,
        high_pressure_threshold_pa: float = 120.0,
        severe_low_pressure_pa: float = 0.0,
        low_pressure_threshold_pa: float = 20.0,
        moderate_low_pressure_pa: float = 50.0,
        high_pressure_decrease_ratio: float = 0.10,
        mild_high_pressure_decrease_ratio: float = 0.02,
        severe_low_pressure_increase_ratio: float = 0.20,
        low_pressure_increase_ratio: float = 0.10,
        moderate_low_pressure_increase_ratio: float = 0.05,
    ) -> None:
        self.data_file = Path(data_file)
        self.max_data_age_s = float(max_data_age_s)
        self.target_pressure_pa = float(target_pressure_pa)
        self.high_pressure_threshold_pa = float(high_pressure_threshold_pa)
        self.severe_low_pressure_pa = float(severe_low_pressure_pa)
        self.low_pressure_threshold_pa = float(low_pressure_threshold_pa)
        self.moderate_low_pressure_pa = float(moderate_low_pressure_pa)
        self.high_pressure_decrease_ratio = float(high_pressure_decrease_ratio)
        self.mild_high_pressure_decrease_ratio = float(mild_high_pressure_decrease_ratio)
        self.severe_low_pressure_increase_ratio = float(severe_low_pressure_increase_ratio)
        self.low_pressure_increase_ratio = float(low_pressure_increase_ratio)
        self.moderate_low_pressure_increase_ratio = float(moderate_low_pressure_increase_ratio)

        if self.max_data_age_s <= 0:
            raise ValueError("max_data_age_s must be positive")
        if not (0 < MIN_ACTIVE_TOTAL_FLOW_SCCM < MAX_TOTAL_FLOW_SCCM):
            raise RuntimeError("Calculated MFC flow range is invalid")
        for name, ratio in (
            ("high_pressure_decrease_ratio", self.high_pressure_decrease_ratio),
            ("mild_high_pressure_decrease_ratio", self.mild_high_pressure_decrease_ratio),
            ("severe_low_pressure_increase_ratio", self.severe_low_pressure_increase_ratio),
            ("low_pressure_increase_ratio", self.low_pressure_increase_ratio),
            ("moderate_low_pressure_increase_ratio", self.moderate_low_pressure_increase_ratio),
        ):
            if not (0.0 <= ratio < 1.0) and "increase" not in name:
                raise ValueError(f"{name} must be in the range [0, 1)")
            if "increase" in name and ratio < 0.0:
                raise ValueError(f"{name} must be non-negative")

    def read_latest_pressure(self, now: Optional[float] = None) -> PressureReading:
        """Read the latest complete GL840 row.

        gl840.dat columns used here (0-based):
          0: Unix timestamp, 7: CDC differential pressure [Pa],
          8: buffer-tank differential pressure [Pa].
        """
        if not self.data_file.is_file():
            raise PressureDataError(f"GL840 data file does not exist: {self.data_file}")

        try:
            with self.data_file.open("rb") as stream:
                stream.seek(0, 2)
                file_size = stream.tell()
                if file_size == 0:
                    raise PressureDataError("GL840 data file is empty")
                read_size = min(file_size, 8192)
                stream.seek(-read_size, 2)
                raw = stream.read()
        except OSError as exc:
            raise PressureDataError(f"Cannot read GL840 data file: {exc}") from exc

        # The logger appends newline-terminated records.  Ignore a possible
        # partially-written final record rather than controlling from it.
        if not raw.endswith((b"\n", b"\r")):
            raw = raw.rsplit(b"\n", 1)[0] if b"\n" in raw else b""
        lines = [line.strip() for line in raw.splitlines() if line.strip()]
        if not lines:
            raise PressureDataError("No complete GL840 data row is available")

        try:
            fields = lines[-1].decode("utf-8").split()
            if len(fields) <= 8:
                raise ValueError(f"expected at least 9 columns, got {len(fields)}")
            timestamp = float(fields[0])
            cdc_pressure = float(fields[7])
            buffer_pressure = float(fields[8])
        except (UnicodeDecodeError, ValueError) as exc:
            raise PressureDataError(f"Cannot parse the latest GL840 row: {exc}") from exc

        if not all(math.isfinite(value) for value in (timestamp, cdc_pressure, buffer_pressure)):
            raise PressureDataError("Latest GL840 row contains a non-finite value")

        current_time = time.time() if now is None else float(now)
        age_s = current_time - timestamp
        if age_s < -5.0:
            raise PressureDataError(
                f"GL840 timestamp is {abs(age_s):.1f} s in the future; check clock synchronization"
            )
        if age_s > self.max_data_age_s:
            raise PressureDataError(
                f"GL840 data is stale: {age_s:.1f} s old "
                f"(limit: {self.max_data_age_s:.1f} s)"
            )

        return PressureReading(
            timestamp=timestamp,
            cdc_pressure_pa=cdc_pressure,
            buffer_pressure_pa=buffer_pressure,
            age_s=max(0.0, age_s),
        )

    @staticmethod
    def validate_total_flow(total_flow_sccm: float, *, allow_stop: bool = True) -> float:
        """Validate a total 90:10 mixture flow against all three MFC ranges."""
        flow = float(total_flow_sccm)
        if not math.isfinite(flow):
            raise FlowRangeError("Current total flow must be finite")
        if allow_stop and math.isclose(flow, 0.0, abs_tol=1e-9):
            return 0.0
        if flow < MIN_ACTIVE_TOTAL_FLOW_SCCM - 1e-9:
            raise FlowRangeError(
                f"Positive total flow must be 0 (stop) or at least "
                f"{MIN_ACTIVE_TOTAL_FLOW_SCCM:.2f} sccm"
            )
        if flow > MAX_TOTAL_FLOW_SCCM + 1e-9:
            raise FlowRangeError(
                f"Total flow exceeds the three-MFC limit of {MAX_TOTAL_FLOW_SCCM:.2f} sccm"
            )
        return flow

    def calculate_decision(
        self,
        reading: PressureReading,
        current_total_flow_sccm: float,
    ) -> PressureDecision:
        """Calculate the next total flow without writing to an MFC."""
        current = self.validate_total_flow(current_total_flow_sccm)
        pressure = reading.cdc_pressure_pa

        if current == 0.0:
            proposed = 0.0
            action = "HOLD_STOPPED"
            reason = "Current flow is stopped; automatic restart is intentionally disabled."
        elif pressure > self.high_pressure_threshold_pa:
            proposed = current * (1.0 - self.high_pressure_decrease_ratio)
            action = "DECREASE"
            reason = (
                f"CDC pressure is above {self.high_pressure_threshold_pa:.1f} Pa; "
                f"reduce total flow by {self.high_pressure_decrease_ratio:.1%}."
            )
        elif pressure > self.target_pressure_pa:
            proposed = current * (1.0 - self.mild_high_pressure_decrease_ratio)
            action = "DECREASE"
            reason = (
                f"CDC pressure is above the {self.target_pressure_pa:.1f} Pa target; "
                f"apply a small {self.mild_high_pressure_decrease_ratio:.1%} reduction."
            )
        elif pressure < self.severe_low_pressure_pa:
            proposed = current * (1.0 + self.severe_low_pressure_increase_ratio)
            action = "INCREASE"
            reason = "CDC pressure is below 0 Pa; increase total flow more strongly."
        elif pressure < self.low_pressure_threshold_pa:
            proposed = current * (1.0 + self.low_pressure_increase_ratio)
            action = "INCREASE"
            reason = (
                f"CDC pressure is below {self.low_pressure_threshold_pa:.1f} Pa; "
                f"increase total flow by {self.low_pressure_increase_ratio:.1%}."
            )
        elif pressure < self.moderate_low_pressure_pa:
            proposed = current * (1.0 + self.moderate_low_pressure_increase_ratio)
            action = "INCREASE"
            reason = (
                f"CDC pressure is below {self.moderate_low_pressure_pa:.1f} Pa; "
                f"increase total flow by {self.moderate_low_pressure_increase_ratio:.1%}."
            )
        else:
            proposed = current
            action = "HOLD"
            reason = "CDC pressure is within the 50–100 Pa hold band."

        # Keep all positive commands inside the common operating envelope.
        # If the controller asks for less than the minimum positive flow, it
        # holds the minimum; it does not silently issue a zero/stop command.
        proposed_unbounded = proposed
        if proposed > 0.0:
            proposed = min(max(proposed, MIN_ACTIVE_TOTAL_FLOW_SCCM), MAX_TOTAL_FLOW_SCCM)
        proposed = round(proposed, 2)
        limit_reached = not math.isclose(proposed, proposed_unbounded, rel_tol=0.0, abs_tol=0.005)
        if limit_reached:
            if proposed_unbounded < MIN_ACTIVE_TOTAL_FLOW_SCCM:
                reason += (
                    f" Minimum active flow reached ({MIN_ACTIVE_TOTAL_FLOW_SCCM:.2f} sccm); "
                    "automatic stop is not issued."
                )
                action = "MIN_FLOW_LIMIT"
            elif proposed_unbounded > MAX_TOTAL_FLOW_SCCM:
                reason += f" Maximum flow reached ({MAX_TOTAL_FLOW_SCCM:.2f} sccm)."
                action = "MAX_FLOW_LIMIT"

        return PressureDecision(
            reading=reading,
            current_total_flow_sccm=current,
            proposed_total_flow_sccm=proposed,
            action=action,
            reason=reason,
            flow_limit_reached=limit_reached,
        )

    def evaluate(self, current_total_flow_sccm: float) -> PressureDecision:
        """Read the latest pressure and calculate a proposed flow."""
        reading = self.read_latest_pressure()
        return self.calculate_decision(reading, current_total_flow_sccm)


def _print_report(decision: PressureDecision) -> None:
    reading = decision.reading
    print("=== CDC pressure feedback dry run ===")
    print(f"GL840 timestamp       : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(reading.timestamp))}")
    print(f"Data age              : {reading.age_s:.1f} s")
    print(f"CDC pressure          : {reading.cdc_pressure_pa:.2f} Pa")
    print(f"Buffer-tank pressure  : {reading.buffer_pressure_pa:.2f} Pa")
    print()
    print("=== Common MFC operating range ===")
    print(f"Minimum active flow   : {MIN_ACTIVE_TOTAL_FLOW_SCCM:.2f} sccm")
    print(f"Maximum total flow    : {MAX_TOTAL_FLOW_SCCM:.2f} sccm")
    print("Zero flow             : supported as a dedicated stop command")
    print()
    print("=== Pressure-control decision ===")
    print(f"Current total flow    : {decision.current_total_flow_sccm:.2f} sccm")
    print(f"Proposed total flow   : {decision.proposed_total_flow_sccm:.2f} sccm")
    print(f"Action                : {decision.action}")
    print(f"Flow limit reached    : {'YES' if decision.flow_limit_reached else 'NO'}")
    print(f"Reason                : {decision.reason}")
    print()
    print("DRY RUN ONLY: no MFC communication or write command was performed.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read the latest GL840 pressure record and calculate a dry-run flow target."
    )
    parser.add_argument(
        "--current-flow-sccm",
        type=float,
        required=True,
        help="Current commanded total 90:10 mixture flow in sccm (not an indicated flow reading).",
    )
    parser.add_argument(
        "--data-file",
        type=Path,
        default=DEFAULT_GL840_FILE,
        help=f"GL840 data file (default: {DEFAULT_GL840_FILE})",
    )
    parser.add_argument(
        "--max-data-age",
        type=float,
        default=DEFAULT_MAX_DATA_AGE_S,
        help=f"Maximum permitted data age in seconds (default: {DEFAULT_MAX_DATA_AGE_S:g})",
    )
    args = parser.parse_args()

    try:
        controller = PressureController(
            data_file=args.data_file,
            max_data_age_s=args.max_data_age,
        )
        decision = controller.evaluate(args.current_flow_sccm)
    except (PressureDataError, FlowRangeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    _print_report(decision)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Convert a total He:i-C4H10 mixture-flow target into individual MFC setpoints.

This module performs calculations only.  It does not open an RS-485 port and
never writes to an MFC.

Gas composition:
    He : i-C4H10 = 90 : 10

The three MFCs are calibrated for the 90:10 mixture.  Cir01 carries pure He,
Cir02 carries pure i-C4H10, and Exh01 carries the mixed gas.  The Fujikin CF
conversion is therefore applied when calculating the supply-MFC setpoints.

All public flow values are expressed in sccm.  The resulting setpoint values
are intended to be passed to MFCManager.set_flow_sccm() by flow_controller.py.
The lower-level MFC driver remains responsible for raw-register conversion and
quantization.

Run a calculation-only check from the project root:
    python3 gas/gas_replacement.py --total-flow-sccm 200
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Tuple


# ---------------------------------------------------------------------------
# Gas composition and Fujikin conversion factors
# ---------------------------------------------------------------------------
HE_FRACTION = 0.90
IC4H10_FRACTION = 0.10

# CF values supplied by Fujikin for the relevant gases.
MIXTURE_CF = 0.946
HE_CF = 1.428
IC4H10_CF = 0.210


# ---------------------------------------------------------------------------
# Active MFC configuration (flow values are MFC setpoint-equivalent sccm)
# Keep these values synchronized with config/mfc_profiles.py.
# The positive setpoint range is 2–100 % FS.
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class MFCFlowLimit:
    name: str
    role: str
    mac_id: int
    serial_number: str
    full_scale_sccm: float
    minimum_sccm: float
    calibration_cf: float
    flowing_gas_cf: float
    actual_flow_fraction: float

    @property
    def setpoint_per_actual_gas_flow(self) -> float:
        """MFC setpoint sccm per sccm of the gas physically passing through it."""
        return self.calibration_cf / self.flowing_gas_cf

    @property
    def setpoint_per_total_mixture(self) -> float:
        """MFC setpoint sccm required per sccm of total 90:10 mixture."""
        return self.actual_flow_fraction * self.setpoint_per_actual_gas_flow

    @property
    def minimum_total_mixture_sccm(self) -> float:
        return self.minimum_sccm / self.setpoint_per_total_mixture

    @property
    def maximum_total_mixture_sccm(self) -> float:
        return self.full_scale_sccm / self.setpoint_per_total_mixture


MFC_FLOW_LIMITS: Tuple[MFCFlowLimit, ...] = (
    MFCFlowLimit(
        name="Cir01",
        role="He supply",
        mac_id=0x21,
        serial_number="AT18ZQW-00",
        full_scale_sccm=5000.0,
        minimum_sccm=100.0,
        calibration_cf=MIXTURE_CF,
        flowing_gas_cf=HE_CF,
        actual_flow_fraction=HE_FRACTION,
    ),
    MFCFlowLimit(
        name="Cir02",
        role="i-C4H10 supply",
        mac_id=0x22,
        serial_number="AT1MSM9-00",
        full_scale_sccm=400.0,
        minimum_sccm=8.0,
        calibration_cf=MIXTURE_CF,
        flowing_gas_cf=IC4H10_CF,
        actual_flow_fraction=IC4H10_FRACTION,
    ),
    MFCFlowLimit(
        name="Exh01",
        role="90:10 mixture exhaust",
        mac_id=0x24,
        serial_number="AT1MSM6-00",
        full_scale_sccm=1000.0,
        minimum_sccm=20.0,
        calibration_cf=MIXTURE_CF,
        flowing_gas_cf=MIXTURE_CF,
        actual_flow_fraction=1.0,
    ),
)


def _intersection_limits() -> tuple[float, float]:
    min_total = max(item.minimum_total_mixture_sccm for item in MFC_FLOW_LIMITS)
    max_total = min(item.maximum_total_mixture_sccm for item in MFC_FLOW_LIMITS)
    if not (math.isfinite(min_total) and math.isfinite(max_total) and min_total < max_total):
        raise RuntimeError("The configured MFC flow ranges do not overlap")
    return min_total, max_total


MIN_ACTIVE_TOTAL_FLOW_SCCM, MAX_TOTAL_FLOW_SCCM = _intersection_limits()


class GasReplacementError(ValueError):
    """Raised when a requested mixture flow cannot be represented by the MFCs."""


@dataclass(frozen=True)
class MFCSetpoint:
    """Calculated setpoint for one MFC.

    target_actual_flow_sccm:
        Desired physical component flow (pure gas for Cir01/Cir02, mixture for
        Exh01).
    setpoint_sccm:
        Setpoint value in the MFC's mixture-calibrated sccm scale.  This is the
        value to pass to MFCManager.set_flow_sccm().
    """

    name: str
    role: str
    mac_id: int
    serial_number: str
    target_actual_flow_sccm: float
    setpoint_sccm: float
    full_scale_sccm: float
    minimum_sccm: float
    percent_of_full_scale: float

    @property
    def setpoint_slm(self) -> float:
        return self.setpoint_sccm / 1000.0


@dataclass(frozen=True)
class GasReplacementPlan:
    """Complete three-MFC plan for a requested total mixture flow."""

    total_mixture_flow_sccm: float
    helium_actual_flow_sccm: float
    isobutane_actual_flow_sccm: float
    mfc_setpoints: Tuple[MFCSetpoint, ...]
    stopped: bool = False

    def by_name(self) -> dict[str, MFCSetpoint]:
        return {item.name: item for item in self.mfc_setpoints}

    def setpoint_map_sccm(self) -> dict[str, float]:
        """Return {MFC profile name: setpoint sccm} for the flow layer."""
        return {item.name: item.setpoint_sccm for item in self.mfc_setpoints}


def validate_total_flow_sccm(
    total_flow_sccm: float,
    *,
    allow_stop: bool = True,
) -> float:
    """Validate total 90:10 mixture flow against all three MFCs.

    Zero is a dedicated stop command.  For a positive flow, all three MFCs
    must remain within their 2–100 % FS setpoint ranges.  This function does
    not silently clamp an invalid request.
    """
    try:
        flow = float(total_flow_sccm)
    except (TypeError, ValueError) as exc:
        raise GasReplacementError("Total mixture flow must be a number") from exc

    if not math.isfinite(flow):
        raise GasReplacementError("Total mixture flow must be finite")

    if math.isclose(flow, 0.0, rel_tol=0.0, abs_tol=1e-9):
        if allow_stop:
            return 0.0
        raise GasReplacementError("Zero flow (stop) is not allowed for this operation")

    if flow < 0.0:
        raise GasReplacementError("Negative total mixture flow is not supported")

    if flow < MIN_ACTIVE_TOTAL_FLOW_SCCM - 1e-9:
        raise GasReplacementError(
            f"Positive total flow must be at least {MIN_ACTIVE_TOTAL_FLOW_SCCM:.2f} sccm "
            "so that all three MFCs remain above their 2 % FS minimums"
        )

    if flow > MAX_TOTAL_FLOW_SCCM + 1e-9:
        raise GasReplacementError(
            f"Total flow exceeds the common three-MFC maximum "
            f"({MAX_TOTAL_FLOW_SCCM:.2f} sccm)"
        )

    return flow


def calculate_gas_replacement_plan(
    total_mixture_flow_sccm: float,
    *,
    allow_stop: bool = True,
) -> GasReplacementPlan:
    """Calculate Cir01/Cir02/Exh01 setpoints for a total mixture-flow target.

    CF conversion:
        MFC setpoint = desired actual gas flow * CF(calibration gas) / CF(flowing gas)

    The supply targets are split 90:10.  Exh01 receives the total mixture flow.
    The result is calculation-only; no MFC operation is performed.
    """
    total = validate_total_flow_sccm(total_mixture_flow_sccm, allow_stop=allow_stop)
    if total == 0.0:
        stopped_setpoints = tuple(
            MFCSetpoint(
                name=profile.name,
                role=profile.role,
                mac_id=profile.mac_id,
                serial_number=profile.serial_number,
                target_actual_flow_sccm=0.0,
                setpoint_sccm=0.0,
                full_scale_sccm=profile.full_scale_sccm,
                minimum_sccm=profile.minimum_sccm,
                percent_of_full_scale=0.0,
            )
            for profile in MFC_FLOW_LIMITS
        )
        return GasReplacementPlan(
            total_mixture_flow_sccm=0.0,
            helium_actual_flow_sccm=0.0,
            isobutane_actual_flow_sccm=0.0,
            mfc_setpoints=stopped_setpoints,
            stopped=True,
        )

    actual_flows = {
        "Cir01": total * HE_FRACTION,
        "Cir02": total * IC4H10_FRACTION,
        "Exh01": total,
    }

    setpoints = []
    for profile in MFC_FLOW_LIMITS:
        actual_flow = actual_flows[profile.name]
        setpoint = actual_flow * profile.setpoint_per_actual_gas_flow
        percent_fs = 100.0 * setpoint / profile.full_scale_sccm

        # Guard against floating-point boundary drift and keep the validation
        # close to the actual values passed to MFCManager.
        if setpoint < profile.minimum_sccm - 1e-7:
            raise GasReplacementError(
                f"{profile.name} calculated setpoint {setpoint:.6f} sccm is below "
                f"its minimum {profile.minimum_sccm:.3f} sccm"
            )
        if setpoint > profile.full_scale_sccm + 1e-7:
            raise GasReplacementError(
                f"{profile.name} calculated setpoint {setpoint:.6f} sccm exceeds "
                f"its full scale {profile.full_scale_sccm:.3f} sccm"
            )

        setpoints.append(
            MFCSetpoint(
                name=profile.name,
                role=profile.role,
                mac_id=profile.mac_id,
                serial_number=profile.serial_number,
                target_actual_flow_sccm=actual_flow,
                setpoint_sccm=setpoint,
                full_scale_sccm=profile.full_scale_sccm,
                minimum_sccm=profile.minimum_sccm,
                percent_of_full_scale=percent_fs,
            )
        )

    return GasReplacementPlan(
        total_mixture_flow_sccm=total,
        helium_actual_flow_sccm=actual_flows["Cir01"],
        isobutane_actual_flow_sccm=actual_flows["Cir02"],
        mfc_setpoints=tuple(setpoints),
        stopped=False,
    )


def format_plan(plan: GasReplacementPlan) -> str:
    """Format a calculation plan for terminal display."""
    lines = [
        "=== Gas replacement setpoint calculation (DRY RUN) ===",
        f"Target total mixture flow : {plan.total_mixture_flow_sccm:.3f} sccm",
        f"Target He flow             : {plan.helium_actual_flow_sccm:.3f} sccm",
        f"Target i-C4H10 flow        : {plan.isobutane_actual_flow_sccm:.3f} sccm",
        "",
        f"{'MFC':<8} {'MAC':>5} {'Target gas flow':>18} {'MFC setpoint':>16} {'FS':>9} {'%FS':>8}",
        "-" * 74,
    ]
    for item in plan.mfc_setpoints:
        lines.append(
            f"{item.name:<8} 0x{item.mac_id:02X} "
            f"{item.target_actual_flow_sccm:>14.3f} "
            f"{item.setpoint_sccm:>13.3f} "
            f"{item.full_scale_sccm:>8.1f} "
            f"{item.percent_of_full_scale:>7.2f}%"
        )
    lines.extend(
        [
            "",
            f"Common positive-flow range : {MIN_ACTIVE_TOTAL_FLOW_SCCM:.2f}–{MAX_TOTAL_FLOW_SCCM:.2f} sccm",
            "No MFC communication or write command was performed.",
        ]
    )
    if plan.stopped:
        lines.append("STOP PLAN: all three MFC setpoints are zero (dedicated stop command).")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Calculate 90:10 He:i-C4H10 setpoints for Cir01, Cir02 and Exh01."
    )
    parser.add_argument(
        "--total-flow-sccm",
        type=float,
        required=True,
        help="Target total 90:10 mixture flow in sccm; 0 requests a stop plan.",
    )
    parser.add_argument(
        "--disallow-stop",
        action="store_true",
        help="Reject a zero-flow target instead of generating a stop plan.",
    )
    args = parser.parse_args()

    try:
        plan = calculate_gas_replacement_plan(
            args.total_flow_sccm,
            allow_stop=not args.disallow_stop,
        )
    except (GasReplacementError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    print(format_plan(plan))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

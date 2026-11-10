#!/usr/bin/env python3
"""Coordinate flow-setpoint updates for Cir01, Cir02 and Exh01.

This module connects ``gas/gas_replacement.py`` to ``mfc/mfc_manager.py``.
It does not implement the FCST serial protocol itself.

All application-facing flow values are in sccm on the MFC setpoint scale.
The gas-replacement module performs the CF conversion and returns one
setpoint for each active MFC.  MFCManager performs raw-register conversion,
Digital-mode switching, Freeze Follow handling, identity checks and
write/read-back verification.

Safety properties
-----------------
* Planning performs READ operations only; no MFC setting is changed.
* All three MFCs are preflighted before the first WRITE is attempted.
* Applying a plan requires both ``apply=True`` (or a direct call to
  ``apply_plan``) and an explicit ``confirmed=True`` argument.
* A failed write stops the sequence.  This module does not attempt an
  automatic rollback because the physical state after a communication error
  may be uncertain.

Recommended command order
-------------------------
* Increasing total flow: Exh01 -> Cir01 -> Cir02
* Decreasing total flow or stopping: Cir01 -> Cir02 -> Exh01

The caller must provide ``current_total_flow_sccm``.  This is the previous
commanded total mixture-flow target tracked by the top-level controller, not
an instantaneous flowmeter reading.  It is used only to select the safe
transition order.

Run a read-only preflight from the project root:
    python3 -m control.flow_controller \\
        --current-total-flow-sccm 0 --target-total-flow-sccm 200

To permit writes, add ``--apply``.  The program will display the plan and
require the operator to type APPLY explicitly.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Any, Mapping, Protocol

from gas.gas_replacement import (
    GasReplacementError,
    GasReplacementPlan,
    MFCSetpoint,
    calculate_gas_replacement_plan,
)


MFC_NAMES: tuple[str, ...] = ("Cir01", "Cir02", "Exh01")
INCREASE_ORDER: tuple[str, ...] = ("Exh01", "Cir01", "Cir02")
DECREASE_ORDER: tuple[str, ...] = ("Cir01", "Cir02", "Exh01")
FLOW_DIRECTION_TOLERANCE_SCCM = 0.01


class MFCManagerLike(Protocol):
    """Subset of MFCManager used by this module."""

    def get_profile(self, name: str) -> Any: ...
    def validate_flow(self, name: str, flow_sccm: float) -> float: ...
    def plan_flow(self, name: str, flow_sccm: float) -> Any: ...
    def set_flow_sccm(
        self,
        name: str,
        flow_sccm: float,
        *,
        apply: bool = False,
        confirmed: bool = False,
    ) -> Any: ...


class FlowControllerError(RuntimeError):
    """Base exception for flow-controller planning and execution errors."""


class FlowPreflightError(FlowControllerError):
    """At least one MFC failed validation before any write was attempted."""


class FlowApplicationError(FlowControllerError):
    """A sequential MFC update failed; the system may be partially updated."""

    def __init__(
        self,
        message: str,
        *,
        applied_mfcs: tuple[str, ...],
        failed_mfc: str,
        not_attempted_mfcs: tuple[str, ...],
    ) -> None:
        super().__init__(message)
        self.applied_mfcs = applied_mfcs
        self.failed_mfc = failed_mfc
        self.not_attempted_mfcs = not_attempted_mfcs


@dataclass(frozen=True, slots=True)
class MFCPreflight:
    """Read-only validation result for one MFC."""

    name: str
    requested_setpoint_sccm: float
    manager_plan: Any


@dataclass(frozen=True, slots=True)
class FlowControllerPlan:
    """Complete, read-only plan for a three-MFC flow transition."""

    current_total_flow_sccm: float
    target_total_flow_sccm: float
    direction: str
    operation_order: tuple[str, ...]
    gas_plan: GasReplacementPlan
    preflight: tuple[MFCPreflight, ...]

    @property
    def is_noop(self) -> bool:
        return len(self.operation_order) == 0

    def setpoint_map_sccm(self) -> dict[str, float]:
        return self.gas_plan.setpoint_map_sccm()


@dataclass(frozen=True, slots=True)
class MFCWriteResult:
    """Verified result returned by MFCManager after one MFC update."""

    name: str
    requested_setpoint_sccm: float
    status: Any


@dataclass(frozen=True, slots=True)
class FlowExecutionResult:
    """Summary of a completed three-MFC update."""

    plan: FlowControllerPlan
    updated_mfcs: tuple[MFCWriteResult, ...]

    @property
    def applied_names(self) -> tuple[str, ...]:
        return tuple(item.name for item in self.updated_mfcs)


def _normalise_nonnegative_flow(value: float, label: str) -> float:
    try:
        flow = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a numeric value in sccm.") from exc
    if not math.isfinite(flow):
        raise ValueError(f"{label} must be finite.")
    if flow < 0.0:
        raise ValueError(f"{label} must not be negative.")
    return flow


def _validate_profile_matches_setpoint(
    manager: MFCManagerLike,
    item: MFCSetpoint,
) -> None:
    """Detect drift between gas_replacement.py and mfc_profiles.py."""
    profile = manager.get_profile(item.name)
    checks = (
        ("MAC ID", int(profile.mac_id), int(item.mac_id)),
        ("serial number", str(profile.serial_number), str(item.serial_number)),
    )
    for label, actual, expected in checks:
        if actual != expected:
            raise ValueError(
                f"{item.name}: {label} mismatch between gas_replacement.py "
                f"and mfc_profiles.py (manager={actual!r}, gas plan={expected!r})."
            )

    numeric_checks = (
        ("full scale", float(profile.full_scale_sccm), float(item.full_scale_sccm)),
        ("minimum flow", float(profile.min_flow_sccm), float(item.minimum_sccm)),
    )
    for label, actual, expected in numeric_checks:
        if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError(
                f"{item.name}: {label} mismatch between gas_replacement.py "
                f"and mfc_profiles.py (manager={actual:g}, gas plan={expected:g})."
            )


def format_flow_plan(plan: FlowControllerPlan) -> str:
    """Return a readable summary of the planned MFC setpoints and order."""
    lines = [
        "=== FCST flow-controller plan (READ-ONLY PREFLIGHT) ===",
        f"Current commanded total flow : {plan.current_total_flow_sccm:.3f} sccm",
        f"Target total mixture flow   : {plan.target_total_flow_sccm:.3f} sccm",
        f"Transition direction        : {plan.direction}",
        "",
        f"{'MFC':<8} {'MAC':>5} {'Setpoint':>14} {'Quantized':>14} {'%FS':>8} {'Mode':>9}",
        "-" * 68,
    ]

    preflight_by_name = {item.name: item.manager_plan for item in plan.preflight}
    for item in plan.gas_plan.mfc_setpoints:
        manager_plan = preflight_by_name[item.name]
        mode = getattr(manager_plan, "current_control_mode", None)
        mode_text = {0x01: "Digital", 0x02: "Analog"}.get(mode, f"0x{mode:02X}" if mode is not None else "unknown")
        quantized = getattr(manager_plan, "quantized_flow_sccm", item.setpoint_sccm)
        lines.append(
            f"{item.name:<8} 0x{item.mac_id:02X} "
            f"{item.setpoint_sccm:>11.3f} "
            f"{quantized:>11.3f} "
            f"{item.percent_of_full_scale:>7.2f}% "
            f"{mode_text:>9}"
        )

    lines.extend(("", "MFC write order: " + (" -> ".join(plan.operation_order) if plan.operation_order else "No write (target unchanged).")))
    lines.append("All three MFCs passed read-only preflight; no WRITE has been sent.")
    if plan.gas_plan.stopped:
        lines.append("STOP PLAN: all three MFCs will receive the dedicated zero-flow command.")
    return "\n".join(lines)


class FlowController:
    """Coordinate gas-flow calculations and sequential MFC setpoint updates.

    Parameters
    ----------
    manager:
        An initialized ``MFCManager`` instance.  The serial connection is
        managed by the caller (normally the top-level controller).
    """

    def __init__(self, manager: MFCManagerLike) -> None:
        self.manager = manager

    def plan_total_flow_sccm(
        self,
        target_total_flow_sccm: float,
        *,
        current_total_flow_sccm: float,
        force: bool = False,
    ) -> FlowControllerPlan:
        """Calculate and preflight a three-MFC transition without writing.

        ``current_total_flow_sccm`` must be the previous commanded total
        mixture flow tracked by the caller.  It is not read from the MFCs and
        should not be replaced by an instantaneous indicated-flow value.

        All MFCs are checked before a plan is returned.  If any identity,
        range, mode-state or communication check fails, this method raises
        ``FlowPreflightError`` and no MFC write has been performed.
        """
        current = _normalise_nonnegative_flow(
            current_total_flow_sccm, "Current total flow"
        )
        try:
            gas_plan = calculate_gas_replacement_plan(target_total_flow_sccm)
        except (GasReplacementError, ValueError) as exc:
            raise FlowControllerError(f"Invalid target flow: {exc}") from exc

        target = gas_plan.total_mixture_flow_sccm
        difference = target - current
        if difference > FLOW_DIRECTION_TOLERANCE_SCCM:
            direction = "increase"
            order = INCREASE_ORDER
        elif difference < -FLOW_DIRECTION_TOLERANCE_SCCM:
            direction = "decrease"
            order = DECREASE_ORDER
        elif force:
            direction = "reapply"
            order = DECREASE_ORDER if target == 0.0 else INCREASE_ORDER
        else:
            direction = "hold"
            order = ()

        preflight_results: list[MFCPreflight] = []
        setpoints_by_name = gas_plan.by_name()
        missing = set(MFC_NAMES) - set(setpoints_by_name)
        extra = set(setpoints_by_name) - set(MFC_NAMES)
        if missing or extra:
            raise FlowControllerError(
                f"Gas-replacement plan MFC names mismatch; missing={sorted(missing)}, "
                f"unexpected={sorted(extra)}."
            )

        # Always preflight in a stable order before allowing any write.
        for name in MFC_NAMES:
            item = setpoints_by_name[name]
            try:
                _validate_profile_matches_setpoint(self.manager, item)
                self.manager.validate_flow(name, item.setpoint_sccm)
                manager_plan = self.manager.plan_flow(name, item.setpoint_sccm)
            except Exception as exc:
                raise FlowPreflightError(
                    f"Preflight failed for {name}: {exc}. "
                    "No MFC writes have been performed."
                ) from exc

            # Ensure the manager planned the same setpoint that the gas module
            # calculated.  The manager's upward raw quantization may differ by
            # a small amount; requested_flow_sccm itself must match exactly.
            planned_request = float(manager_plan.requested_flow_sccm)
            if not math.isclose(
                planned_request, item.setpoint_sccm, rel_tol=0.0, abs_tol=1e-8
            ):
                raise FlowPreflightError(
                    f"{name}: MFCManager preflight target ({planned_request:g} sccm) "
                    f"does not match gas-replacement target ({item.setpoint_sccm:g} sccm). "
                    "No MFC writes have been performed."
                )

            preflight_results.append(
                MFCPreflight(
                    name=name,
                    requested_setpoint_sccm=item.setpoint_sccm,
                    manager_plan=manager_plan,
                )
            )

        return FlowControllerPlan(
            current_total_flow_sccm=current,
            target_total_flow_sccm=target,
            direction=direction,
            operation_order=tuple(order),
            gas_plan=gas_plan,
            preflight=tuple(preflight_results),
        )

    def apply_plan(
        self,
        plan: FlowControllerPlan,
        *,
        confirmed: bool = False,
    ) -> FlowExecutionResult:
        """Apply a previously preflighted plan in the selected safe order.

        A no-op plan returns immediately and sends no writes.  Otherwise,
        ``confirmed=True`` is mandatory.  The top-level controller should
        set this only after displaying the plan and receiving operator consent.
        """
        if plan.is_noop:
            return FlowExecutionResult(plan=plan, updated_mfcs=())
        if not confirmed:
            raise FlowControllerError(
                "MFC writes blocked: apply_plan requires confirmed=True after "
                "the operator has reviewed the target flow and transition order."
            )

        setpoints = plan.gas_plan.setpoint_map_sccm()
        updated: list[MFCWriteResult] = []

        for index, name in enumerate(plan.operation_order):
            try:
                status = self.manager.set_flow_sccm(
                    name,
                    setpoints[name],
                    apply=True,
                    confirmed=True,
                )
            except Exception as exc:
                remaining = plan.operation_order[index + 1 :]
                applied_names = tuple(item.name for item in updated)
                message = (
                    f"MFC update failed at {name}: {exc}. "
                    f"Successfully updated: {', '.join(applied_names) if applied_names else 'none'}. "
                    f"Not attempted: {', '.join(remaining) if remaining else 'none'}. "
                    "The system may be partially updated. No automatic rollback was attempted; "
                    "read all MFC states and assess the gas path before retrying."
                )
                raise FlowApplicationError(
                    message,
                    applied_mfcs=applied_names,
                    failed_mfc=name,
                    not_attempted_mfcs=tuple(remaining),
                ) from exc

            updated.append(
                MFCWriteResult(
                    name=name,
                    requested_setpoint_sccm=setpoints[name],
                    status=status,
                )
            )

        return FlowExecutionResult(plan=plan, updated_mfcs=tuple(updated))

    def set_total_flow_sccm(
        self,
        target_total_flow_sccm: float,
        *,
        current_total_flow_sccm: float,
        apply: bool = False,
        confirmed: bool = False,
        force: bool = False,
    ) -> FlowControllerPlan | FlowExecutionResult:
        """Plan a target flow, optionally applying it after confirmation."""
        plan = self.plan_total_flow_sccm(
            target_total_flow_sccm,
            current_total_flow_sccm=current_total_flow_sccm,
            force=force,
        )
        if not apply:
            return plan
        return self.apply_plan(plan, confirmed=confirmed)


def _main() -> int:
    parser = argparse.ArgumentParser(
        description="Preflight or apply a three-MFC gas-flow transition."
    )
    parser.add_argument(
        "--current-total-flow-sccm",
        type=float,
        required=True,
        help="Previous commanded total mixture flow in sccm (0 if stopped).",
    )
    parser.add_argument(
        "--target-total-flow-sccm",
        type=float,
        required=True,
        help="New total 90:10 He:i-C4H10 mixture-flow target in sccm (0=stop).",
    )
    parser.add_argument("--port", help="Explicit RS-485 serial port, e.g. /dev/ttyUSB1.")
    parser.add_argument("--verbose", action="store_true", help="Print TX/RX frames.")
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Permit MFC writes after an additional interactive APPLY confirmation.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Reapply a target even if it equals the current commanded total flow.",
    )
    args = parser.parse_args()

    try:
        # Imports are kept inside the CLI entry point so this module can be
        # imported and unit-tested without opening a serial port.
        from mfc.fcst_driver import FCSTDriver
        from mfc.mfc_manager import MFCManager

        with FCSTDriver(port=args.port, verbose=args.verbose) as driver:
            manager = MFCManager(driver)
            controller = FlowController(manager)
            plan = controller.plan_total_flow_sccm(
                args.target_total_flow_sccm,
                current_total_flow_sccm=args.current_total_flow_sccm,
                force=args.force,
            )
            print(format_flow_plan(plan))

            if not args.apply:
                print("\nDRY RUN: no MFC WRITE command was sent.")
                return 0
            if plan.is_noop:
                print("\nNo update required; no MFC WRITE command was sent.")
                return 0

            print("\nWARNING: This will change the setpoints of three MFCs.")
            confirmation = input("Type APPLY to continue: ").strip()
            if confirmation != "APPLY":
                print("Aborted by operator. No MFC WRITE command was sent.")
                return 0

            result = controller.apply_plan(plan, confirmed=True)
            print("\nMFC setpoint updates completed:")
            for item in result.updated_mfcs:
                print(f"  {item.name}: {item.requested_setpoint_sccm:.3f} sccm (verified)")
            return 0

    except KeyboardInterrupt:
        print("\nInterrupted by operator.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(_main())

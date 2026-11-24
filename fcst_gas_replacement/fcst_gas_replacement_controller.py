#!/usr/bin/env python3
"""Top-level controller for COMET CDC gas replacement and pressure feedback.

The controller integrates:
    control/pressure_controller.py  : read GL840 log and decide total flow
    gas/gas_replacement.py           : calculate Cir01/Cir02 feedback setpoints
    control/flow_controller.py      : preflight and apply supply-MFC setpoints
    logging/data_logger.py          : log and monitor Setpoint/Indicated Flow
    mfc/fcst_driver.py              : shared RS-485 connection
    mfc/mfc_manager.py              : MFC identity, state, and write verification

Safety model:
* Default operation is DRY RUN. No MFC WRITE is issued unless --apply is set.
* --apply requires the operator to type "ENABLE FEEDBACK" once at startup.
* Pressure feedback adjusts Cir01 and Cir02 only. Exh01 is never changed by
  pressure feedback; its Setpoint is preserved.
* After a pressure-feedback Setpoint change, the changed supply MFCs are
  excluded from deviation judgement for 5 seconds. Five valid 1-second log
  samples are then collected before another pressure-feedback update is allowed.
* All three MFCs are monitored for Setpoint/Indicated Flow deviation. A
  deviation greater than 2.7% for five consecutive log samples triggers an
  emergency zero-Setpoint sequence: Cir01 -> Cir02 -> Exh01.
* The Cir01/Cir02 CF-corrected indicated-flow ratio is also monitored. It must
  remain in [8.5, 9.5), i.e. round to 9:1. Five consecutive out-of-range samples
  trigger the same emergency stop.
* If the GL840 record is older than 120 seconds, APPLY mode restores the initial
  supply-flow target (Cir01 -> Cir02) and exits. Exh01 is not changed. DRY RUN
  displays the restoration plan without writing it.
* Other missing/invalid GL840 data and non-flow-monitoring errors stop the
  controller without automatic restoration or stopping.

Run from the project root:
    python3 fcst_gas_replacement_controller.py --current-flow-sccm 200

Enable automatic MFC writes only after checking the dry-run output:
    python3 fcst_gas_replacement_controller.py --current-flow-sccm 200 --apply
"""
from __future__ import annotations

import argparse
import importlib.util
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from control.flow_controller import (
    FlowApplicationError,
    FlowController,
    FlowControllerPlan,
    FlowPreflightError,
    format_flow_plan,
)
from control.pressure_controller import (
    DEFAULT_GL840_FILE,
    FlowRangeError,
    PressureController,
    PressureDataError,
    PressureDataStaleError,
)
from gas.gas_replacement import (
    HE_CF,
    IC4H10_CF,
    calculate_gas_replacement_plan,
)
from mfc.mfc_manager import MFCManager, raw_to_sccm, sccm_to_raw


DEFAULT_CONTROL_INTERVAL_SECONDS = 5.0
DEFAULT_LOG_INTERVAL_SECONDS = 1.0
DEFAULT_FLOW_LOG_FILE = Path("data/mfc_flow.dat")

# Flow deviation is relative to each MFC's current Setpoint.  A zero Setpoint
# is a dedicated stop command and is excluded from relative-deviation checks.
FLOW_DEVIATION_LIMIT_RATIO = 0.027
FLOW_DEVIATION_CONSECUTIVE_SAMPLES = 5
SETPOINT_SETTLING_SECONDS = 5.0
MONITORED_MFC_NAMES: tuple[str, ...] = ("Cir01", "Cir02", "Exh01")
EMERGENCY_STOP_ORDER: tuple[str, ...] = ("Cir01", "Cir02", "Exh01")

# The ratio is calculated from the indicated flows corrected from mixture-
# calibrated MFC scale to the gas physically flowing through each supply MFC.
# The common mixture CF cancels in the ratio.
MIXTURE_RATIO_MIN = 8.5
MIXTURE_RATIO_MAX_EXCLUSIVE = 9.5


def _load_data_logger_class() -> type:
    """Load data_logger.py from data/ or logging/ without importing stdlib logging.

    The preferred project location is data/data_logger.py.  The logging/
    fallback supports the original directory name while loading the file by
    its path, avoiding a collision with Python's standard-library logging
    package.
    """
    project_root = Path(__file__).resolve().parent
    candidates = (
        project_root / "data" / "data_logger.py",
        project_root / "logging" / "data_logger.py",
    )
    for logger_path in candidates:
        if not logger_path.is_file():
            continue
        module_name = "_fcst_gas_replacement_data_logger"
        spec = importlib.util.spec_from_file_location(module_name, logger_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(
                f"Could not load the MFC data logger module: {logger_path}"
            )
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        try:
            spec.loader.exec_module(module)
        except Exception:
            sys.modules.pop(module_name, None)
            raise
        logger_class = getattr(module, "MFCDataLogger", None)
        if logger_class is None:
            raise RuntimeError(
                f"MFCDataLogger class was not found in {logger_path}"
            )
        return logger_class

    raise RuntimeError(
        "data_logger.py was not found. Put it in data/data_logger.py "
        "(recommended) or logging/data_logger.py."
    )


class ControllerStartupError(RuntimeError):
    """Raised when startup checks do not permit feedback control to begin."""


class FlowDeviationError(RuntimeError):
    """Raised after persistent flow deviation and the emergency-stop attempt."""


@dataclass(frozen=True, slots=True)
class CycleResult:
    cycle_number: int
    current_total_flow_sccm: float
    target_total_flow_sccm: float
    action: str
    applied: bool
    plan: FlowControllerPlan


class GasReplacementFeedbackController:
    """Orchestrate pressure feedback, MFC flow monitoring, and data logging.

    The FCSTDriver instance is shared by MFCManager and MFCDataLogger. All
    serial transactions are performed sequentially on this one connection.
    """

    def __init__(
        self,
        driver: Any,
        manager: MFCManager,
        *,
        current_total_flow_sccm: float,
        pressure_file: str | Path = DEFAULT_GL840_FILE,
        flow_log_file: str | Path = DEFAULT_FLOW_LOG_FILE,
        control_interval_seconds: float = DEFAULT_CONTROL_INTERVAL_SECONDS,
        log_interval_seconds: float = DEFAULT_LOG_INTERVAL_SECONDS,
    ) -> None:
        if not math.isfinite(control_interval_seconds) or control_interval_seconds <= 0:
            raise ValueError("control_interval_seconds must be finite and positive")
        if not math.isfinite(log_interval_seconds) or log_interval_seconds <= 0:
            raise ValueError("log_interval_seconds must be finite and positive")

        self.current_total_flow_sccm = PressureController.validate_total_flow(
            current_total_flow_sccm
        )
        # Preserve the startup value separately.  A stale GL840 log restores
        # this initial supply target, not the most recent feedback target.
        self.initial_total_flow_sccm = self.current_total_flow_sccm
        self.manager = manager
        self.pressure_controller = PressureController(data_file=pressure_file)
        self.flow_controller = FlowController(manager)
        # Identity has already been checked by startup read_all_status().
        # The logger uses this same serial driver and only issues READ commands.
        logger_class = _load_data_logger_class()
        self.data_logger = logger_class(
            driver,
            output_file=flow_log_file,
            interval_seconds=log_interval_seconds,
            verify_identity=False,
        )
        self.control_interval_seconds = float(control_interval_seconds)
        self.log_interval_seconds = float(log_interval_seconds)
        self._cycle_number = 0
        self._log_sample_number = 0

        self._flow_deviation_counts = {
            name: 0 for name in MONITORED_MFC_NAMES
        }
        self._ratio_deviation_count = 0
        self._settled_until = {name: 0.0 for name in MONITORED_MFC_NAMES}
        # A new pressure-feedback target is not issued until the changed
        # supply MFCs have settled and five post-settling log samples exist.
        self._feedback_hold_pending = False
        self._post_settle_valid_samples = 0

    def verify_startup(self, *, applying: bool) -> None:
        """Verify all MFC identities and supply flow against digital setpoints.

        An Analog-mode supply MFC is reported but its digital Setpoint register
        is not used to validate the declared current flow, because that register
        may not represent the active analog input command. Exh01 is read and
        displayed for identity/status monitoring, but its Setpoint is not
        compared with the supply-flow reference.
        """
        statuses = self.manager.read_all_status()
        missing = {"Cir01", "Cir02", "Exh01"} - set(statuses)
        if missing:
            raise ControllerStartupError(
                "The following configured MFCs were not returned by read_all_status(): "
                + ", ".join(sorted(missing))
            )

        print("=== MFC startup status (identity verified) ===")
        for name in MONITORED_MFC_NAMES:
            status = statuses[name]
            mode_text = {0x01: "Digital", 0x02: "Analog"}.get(
                status.current_control_mode,
                f"unknown (0x{status.current_control_mode:02X})",
            )
            print(
                f"{name}: serial={status.serial_number}, MAC=0x{status.mac_id:02X}, "
                f"mode={mode_text}, setpoint={status.setpoint_sccm:.3f} sccm, "
                f"indicated={status.indicated_flow_sccm_equivalent:.3f} sccm"
            )

        # Exh01 remains outside pressure-feedback setpoint validation.
        expected_plan = calculate_gas_replacement_plan(
            self.current_total_flow_sccm,
            include_exhaust=False,
        )
        expected_by_name = expected_plan.by_name()
        mismatches: list[str] = []
        analog_names: list[str] = []

        for name in ("Cir01", "Cir02"):
            status = statuses[name]
            if status.current_control_mode != 0x01:  # not Digital
                analog_names.append(name)
                continue

            profile = self.manager.get_profile(name)
            expected_item = expected_by_name[name]
            expected_raw = sccm_to_raw(
                expected_item.setpoint_sccm, profile.full_scale_sccm
            )
            expected_quantized = raw_to_sccm(
                expected_raw, profile.full_scale_sccm
            )
            if not math.isclose(
                status.setpoint_sccm,
                expected_quantized,
                rel_tol=0.0,
                abs_tol=1e-6,
            ):
                mismatches.append(
                    f"{name}: digital setpoint is {status.setpoint_sccm:.3f} sccm, "
                    f"but --current-flow-sccm {self.current_total_flow_sccm:.3f} "
                    f"expects {expected_quantized:.3f} sccm"
                )

        if analog_names:
            print(
                "[WARN] Digital Setpoint readback cannot verify the active command "
                "for Analog-mode supply MFC(s): " + ", ".join(analog_names)
            )
        if mismatches:
            detail = "\n  - ".join(mismatches)
            message = (
                "The declared current total flow does not match the current "
                "Digital-mode supply MFC Setpoints:\n  - " + detail
            )
            if applying:
                raise ControllerStartupError(
                    message + "\nAutomatic MFC writes are blocked. Check the actual "
                    "supply setpoints and restart with the correct --current-flow-sccm."
                )
            print("[WARN] " + message.replace("\n", "\n       "))
        else:
            print(
                f"Declared current supply flow: {self.current_total_flow_sccm:.3f} sccm"
            )

        print("Startup identity check completed.\n")

    def _run_control_cycle(self, *, apply: bool) -> CycleResult:
        """Read pressure, calculate a target, preflight, and optionally apply."""
        self._cycle_number += 1
        decision = self.pressure_controller.evaluate(
            self.current_total_flow_sccm
        )

        print("\n" + "=" * 72)
        print(f"CONTROL CYCLE {self._cycle_number}")
        print("=" * 72)
        print(
            f"GL840 data age       : {decision.reading.age_s:.2f} s\n"
            f"CDC pressure         : {decision.reading.cdc_pressure_pa:.2f} Pa\n"
            f"Buffer-tank pressure : {decision.reading.buffer_pressure_pa:.2f} Pa\n"
            f"Control action       : {decision.action}\n"
            f"Reason               : {decision.reason}\n"
            f"Current total flow   : {decision.current_total_flow_sccm:.2f} sccm\n"
            f"Proposed total flow  : {decision.proposed_total_flow_sccm:.2f} sccm"
        )

        # The pressure-feedback plan contains Cir01 and Cir02 only.
        # Exh01 is not calculated, preflighted, or written by this feedback path.
        plan = self.flow_controller.plan_total_flow_sccm(
            decision.proposed_total_flow_sccm,
            current_total_flow_sccm=self.current_total_flow_sccm,
            include_exhaust=False,
        )
        print()
        print(format_flow_plan(plan))

        applied = False
        if plan.is_noop:
            print("No MFC setpoint change is required.")
        elif apply:
            result = self.flow_controller.apply_plan(plan, confirmed=True)
            self.current_total_flow_sccm = plan.target_total_flow_sccm
            applied = True
            print("\nSupply MFC setpoint updates completed and verified:")
            for item in result.updated_mfcs:
                print(
                    f"  {item.name}: {item.requested_setpoint_sccm:.3f} sccm"
                )

            # Start a fresh settling period only for MFCs actually written.
            # Exh01 is not in this plan and therefore remains continuously
            # monitored without a settling reset.
            settled_until = time.monotonic() + SETPOINT_SETTLING_SECONDS
            for item in result.updated_mfcs:
                self._settled_until[item.name] = settled_until
                self._flow_deviation_counts[item.name] = 0
            if {item.name for item in result.updated_mfcs} & {"Cir01", "Cir02"}:
                self._ratio_deviation_count = 0
                self._feedback_hold_pending = True
                self._post_settle_valid_samples = 0
                print(
                    f"[INFO] Supply-flow deviation checks will resume after "
                    f"{SETPOINT_SETTLING_SECONDS:.1f} s settling and "
                    f"{FLOW_DEVIATION_CONSECUTIVE_SAMPLES} valid log samples."
                )
        else:
            print("\nDRY RUN: proposed supply-MFC setpoints were not written.")
            print(
                "The current-flow reference is unchanged because no physical "
                "MFC update was made."
            )

        return CycleResult(
            cycle_number=self._cycle_number,
            current_total_flow_sccm=decision.current_total_flow_sccm,
            target_total_flow_sccm=decision.proposed_total_flow_sccm,
            action=decision.action,
            applied=applied,
            plan=plan,
        )

    def _recover_initial_flow_after_stale_gl840(
        self,
        *,
        apply: bool,
        stale_error: PressureDataStaleError,
    ) -> None:
        """Restore the startup supply target after GL840 data exceeds 120 s."""
        print(f"\n[GL840 STALE] {stale_error}")
        print(
            "The controller will return Cir01 and Cir02 to the initial "
            f"supply-flow target ({self.initial_total_flow_sccm:.2f} sccm), "
            "then exit. Exh01 Setpoint will remain unchanged."
        )

        plan = self.flow_controller.plan_total_flow_sccm(
            self.initial_total_flow_sccm,
            current_total_flow_sccm=self.current_total_flow_sccm,
            include_exhaust=False,
        )
        print()
        print(format_flow_plan(plan))

        if plan.is_noop:
            print("Supply flow is already at the initial target; no Setpoint write is needed.")
        elif apply:
            result = self.flow_controller.apply_plan(plan, confirmed=True)
            self.current_total_flow_sccm = self.initial_total_flow_sccm
            print("\nInitial supply-flow target restored:")
            for item in result.updated_mfcs:
                print(f"  {item.name}: {item.requested_setpoint_sccm:.3f} sccm")
        else:
            print("DRY RUN: restoration plan displayed; no MFC WRITE was sent.")

        print("GL840 is stale. Pressure feedback is terminated.")

    def _trigger_emergency_stop(
        self,
        alarm_reasons: list[str],
        *,
        apply: bool,
    ) -> None:
        """Stop all three MFCs, attempting every device in the required order."""
        details = "\n  - ".join(alarm_reasons)
        print(
            "\n[FLOW ALARM] Persistent flow deviation detected:\n  - " + details,
            file=sys.stderr,
        )

        if not apply:
            raise FlowDeviationError(
                "Flow deviation reached the alarm threshold in DRY RUN. "
                "No MFC stop WRITE was sent because --apply is not enabled. "
                "The controller is exiting."
            )

        print(
            "[EMERGENCY STOP] Sending zero-Setpoint commands in order: "
            "Cir01 -> Cir02 -> Exh01.",
            file=sys.stderr,
        )
        successes: list[str] = []
        failures: list[str] = []

        # Do not stop the sequence after one failure. A failed/uncertain
        # transaction for one MFC must not prevent attempts on the others.
        for name in EMERGENCY_STOP_ORDER:
            try:
                status = self.manager.set_flow_sccm(
                    name,
                    0.0,
                    apply=True,
                    confirmed=True,
                )
                successes.append(name)
                print(
                    f"[EMERGENCY STOP] {name}: zero Setpoint verified "
                    f"({status.setpoint_sccm:.3f} sccm).",
                    file=sys.stderr,
                )
            except Exception as exc:
                failures.append(f"{name}: {type(exc).__name__}: {exc}")
                print(
                    f"[EMERGENCY STOP][FAILED] {name}: {type(exc).__name__}: {exc}",
                    file=sys.stderr,
                )

        summary = (
            "Emergency stop sequence completed. "
            f"Verified zero Setpoint: {', '.join(successes) if successes else 'none'}."
        )
        if failures:
            summary += " Stop-command failures: " + " | ".join(failures)
        summary += " No restoration to the initial flow will be attempted after a flow alarm."
        raise FlowDeviationError(summary)

    def _check_flow_deviations(
        self,
        readings: Mapping[str, Any],
        *,
        apply: bool,
        sample_monotonic: float,
    ) -> None:
        """Check each MFC deviation and the CF-corrected supply-flow ratio."""
        alarm_reasons: list[str] = []
        valid_readings: dict[str, Any] = {}

        for name in MONITORED_MFC_NAMES:
            reading = readings.get(name)
            if reading is None:
                # The logger has already recorded nan and reported the read
                # failure. An incomplete sample breaks consecutiveness but is
                # not itself classified as a measured flow deviation.
                self._flow_deviation_counts[name] = 0
                continue

            setpoint = float(reading.setpoint_sccm)
            indicated = float(reading.indicated_flow_sccm)
            if not (math.isfinite(setpoint) and math.isfinite(indicated)):
                self._flow_deviation_counts[name] = 0
                continue
            valid_readings[name] = reading

            if setpoint <= 0.0:
                # Relative error is undefined at zero flow. No comparison is
                # made while this MFC's dedicated stop Setpoint is active.
                self._flow_deviation_counts[name] = 0
                continue

            if sample_monotonic < self._settled_until[name]:
                self._flow_deviation_counts[name] = 0
                continue

            deviation_percent = (indicated - setpoint) / setpoint * 100.0
            if abs(deviation_percent) > FLOW_DEVIATION_LIMIT_RATIO * 100.0:
                self._flow_deviation_counts[name] += 1
                count = self._flow_deviation_counts[name]
                print(
                    f"[FLOW WARN] {name}: Setpoint={setpoint:.3f} sccm, "
                    f"Indicated Flow={indicated:.3f} sccm, "
                    f"deviation={deviation_percent:+.2f}% "
                    f"({count}/{FLOW_DEVIATION_CONSECUTIVE_SAMPLES}).",
                    file=sys.stderr,
                )
                if count >= FLOW_DEVIATION_CONSECUTIVE_SAMPLES:
                    alarm_reasons.append(
                        f"{name} Setpoint/Indicated Flow deviation "
                        f"{deviation_percent:+.2f}% exceeded ±2.7% for "
                        f"{FLOW_DEVIATION_CONSECUTIVE_SAMPLES} consecutive samples"
                    )
            else:
                self._flow_deviation_counts[name] = 0

        # The supply ratio is calculated from Indicated Flow, not raw Setpoint
        # values. For the current CF model, R = 1.428/0.210 * Cir01/Cir02.
        cir01 = valid_readings.get("Cir01")
        cir02 = valid_readings.get("Cir02")
        supply_settled = sample_monotonic >= max(
            self._settled_until["Cir01"], self._settled_until["Cir02"]
        )
        if cir01 is not None and cir02 is not None and supply_settled:
            cir01_setpoint = float(cir01.setpoint_sccm)
            cir02_setpoint = float(cir02.setpoint_sccm)

            if cir01_setpoint <= 0.0 and cir02_setpoint <= 0.0:
                # Both supplies are intentionally stopped; a mixture ratio is
                # undefined and no composition alarm is evaluated.
                self._ratio_deviation_count = 0
                ratio = None
            elif cir01_setpoint <= 0.0 or cir02_setpoint <= 0.0:
                # One component is stopped while the other is flowing. This is
                # not a 9:1 mixture and must be treated as a ratio deviation.
                ratio = 0.0 if cir01_setpoint <= 0.0 else math.inf
            else:
                he_actual = float(cir01.indicated_flow_sccm) * HE_CF
                isobutane_actual = float(cir02.indicated_flow_sccm) * IC4H10_CF
                ratio = (
                    he_actual / isobutane_actual
                    if isobutane_actual > 0.0
                    else math.inf
                )

            if ratio is not None and not (
                MIXTURE_RATIO_MIN <= ratio < MIXTURE_RATIO_MAX_EXCLUSIVE
            ):
                self._ratio_deviation_count += 1
                print(
                    f"[MIXTURE RATIO WARN] CF-corrected He:i-C4H10="
                    f"{ratio:.5f}:1; required range is [8.5, 9.5) "
                    f"({self._ratio_deviation_count}/"
                    f"{FLOW_DEVIATION_CONSECUTIVE_SAMPLES}).",
                    file=sys.stderr,
                )
                if self._ratio_deviation_count >= FLOW_DEVIATION_CONSECUTIVE_SAMPLES:
                    alarm_reasons.append(
                        f"CF-corrected He:i-C4H10 ratio {ratio:.5f}:1 is outside "
                        "[8.5, 9.5), so it no longer rounds to 9:1"
                    )
            else:
                self._ratio_deviation_count = 0
        else:
            self._ratio_deviation_count = 0

        # Count actual valid 1-second supply log samples after the 5-second
        # settling period. Do not allow a new feedback write before these five
        # checks have been obtained; otherwise a 5-second feedback cycle could
        # repeatedly reset the deviation counter before it reaches five.
        if self._feedback_hold_pending and supply_settled:
            if cir01 is not None and cir02 is not None:
                self._post_settle_valid_samples += 1
                if self._post_settle_valid_samples >= FLOW_DEVIATION_CONSECUTIVE_SAMPLES:
                    self._feedback_hold_pending = False
                    print(
                        "[INFO] Five valid post-settling supply-flow samples "
                        "were collected; pressure feedback may resume."
                    )

        if alarm_reasons:
            self._trigger_emergency_stop(alarm_reasons, apply=apply)

    def _write_log_sample(self, *, apply: bool) -> None:
        start = time.monotonic()
        readings = self.data_logger.write_sample()
        sample_finished = time.monotonic()
        self._log_sample_number += 1
        self._check_flow_deviations(
            readings,
            apply=apply,
            sample_monotonic=sample_finished,
        )
        elapsed = time.monotonic() - start
        if elapsed > self.log_interval_seconds:
            print(
                f"[WARN] MFC log sample {self._log_sample_number} took "
                f"{elapsed:.3f} s (requested interval: "
                f"{self.log_interval_seconds:.3f} s).",
                file=sys.stderr,
            )

    def _run_control_cycle_or_recover_stale(
        self,
        *,
        apply: bool,
    ) -> bool:
        """Run one pressure cycle; return False after stale-log recovery/exit."""
        try:
            self._run_control_cycle(apply=apply)
        except PressureDataStaleError as exc:
            self._recover_initial_flow_after_stale_gl840(
                apply=apply,
                stale_error=exc,
            )
            return False
        return True

    def run(
        self,
        *,
        apply: bool,
        once: bool = False,
        max_cycles: int | None = None,
    ) -> int:
        """Run pressure feedback and best-effort 1-second flow monitoring."""
        if max_cycles is not None and max_cycles <= 0:
            raise ValueError("max_cycles must be a positive integer")

        print("=== COMET CDC gas-replacement controller ===")
        print(f"Mode              : {'APPLY (automatic MFC writes enabled)' if apply else 'DRY RUN'}")
        print(f"Control interval  : {self.control_interval_seconds:.3f} s")
        print(f"MFC log interval  : {self.log_interval_seconds:.3f} s")
        print(f"Initial flow ref. : {self.initial_total_flow_sccm:.3f} sccm")
        print(f"Pressure file     : {self.pressure_controller.data_file}")
        print(f"MFC log file      : {self.data_logger.output_file}")
        print("Flow-deviation limit: ±2.7% for Cir01, Cir02, and Exh01")
        print(
            "Mixture-ratio guard: CF-corrected He:i-C4H10 must remain in "
            "[8.5, 9.5) (rounds to 9:1)."
        )
        print("Deviation alarm: five consecutive valid MFC log samples.")
        print("\nThe MFC logger and controller share one RS-485 connection and run sequentially.")
        print("A 1-second log interval is best-effort; control transactions can delay samples.")

        if once:
            self._write_log_sample(apply=apply)
            if not self._run_control_cycle_or_recover_stale(apply=apply):
                return 2

            # If this one-shot cycle changed a supply Setpoint, keep acquiring
            # the existing 1-second logger samples until the 5-second settling
            # period and five valid post-settling samples have completed. Do not
            # issue another pressure-feedback cycle in --once mode.
            while apply and self._feedback_hold_pending:
                time.sleep(self.log_interval_seconds)
                self._write_log_sample(apply=apply)

            print("\nOne control cycle and its post-settling flow checks completed.")
            return 0

        next_log = time.monotonic()
        next_control = next_log
        completed_cycles = 0
        stop_after_validation = False

        while True:
            now = time.monotonic()
            did_work = False

            # Log first when both jobs are due. Serial access is kept sequential.
            if now >= next_log:
                self._write_log_sample(apply=apply)
                did_work = True
                next_log += self.log_interval_seconds
                after_log = time.monotonic()
                if next_log <= after_log:
                    missed = int((after_log - next_log) // self.log_interval_seconds) + 1
                    next_log += missed * self.log_interval_seconds

            now = time.monotonic()
            if (
                now >= next_control
                and not self._feedback_hold_pending
                and not stop_after_validation
            ):
                if not self._run_control_cycle_or_recover_stale(apply=apply):
                    return 2
                completed_cycles += 1
                did_work = True
                if max_cycles is not None and completed_cycles >= max_cycles:
                    # Do not leave immediately after changing a Setpoint. Keep
                    # logging until its settling and five valid supply samples
                    # have been checked, but do not start another feedback cycle.
                    if self._feedback_hold_pending:
                        stop_after_validation = True
                    else:
                        break
                next_control += self.control_interval_seconds
                after_control = time.monotonic()
                if next_control <= after_control:
                    missed = int((after_control - next_control) // self.control_interval_seconds) + 1
                    next_control += missed * self.control_interval_seconds

            if stop_after_validation and not self._feedback_hold_pending:
                break

            if not did_work:
                # If a new supply target is settling/being verified, the
                # pressure cycle may already be overdue. Wait for the next log
                # sample rather than busy-looping on that overdue control time.
                sleep_until = (
                    next_log
                    if self._feedback_hold_pending
                    else min(next_log, next_control)
                )
                time.sleep(max(0.0, min(0.2, sleep_until - time.monotonic())))

        print(
            f"\nController stopped. Control cycles: {completed_cycles}; "
            f"MFC log samples: {self._log_sample_number}."
        )
        return 0


def _confirm_live_operation() -> bool:
    print("\n" + "!" * 72)
    print("WARNING: AUTOMATIC MFC WRITES ARE ENABLED.")
    print("Pressure feedback will adjust Cir01 and Cir02 setpoints only.")
    print("Exh01 Setpoint is not changed by pressure feedback.")
    print("All three MFCs are monitored for ±2.7% Setpoint/Indicated Flow deviation.")
    print("Persistent deviation or a non-9:1 rounded supply ratio triggers zero Setpoints")
    print("in order Cir01 -> Cir02 -> Exh01.")
    print("Confirm that the gas lines, exhaust path, bubbler, and initial flow are ready.")
    print("!" * 72)
    try:
        answer = input('Type "ENABLE FEEDBACK" to start: ').strip()
    except EOFError:
        return False
    return answer == "ENABLE FEEDBACK"


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Run COMET CDC gas-replacement pressure feedback. Default mode is "
            "read-only DRY RUN; use --apply to enable MFC writes."
        )
    )
    parser.add_argument(
        "--current-flow-sccm",
        type=float,
        required=True,
        help=(
            "Currently commanded total 90:10 mixture flow in sccm; use 0 if "
            "the system is stopped. This value determines increase/decrease order "
            "and is preserved as the restoration target for a stale GL840 log."
        ),
    )
    parser.add_argument(
        "--pressure-file",
        type=Path,
        default=DEFAULT_GL840_FILE,
        help=f"GL840 data file (default: {DEFAULT_GL840_FILE}).",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=DEFAULT_FLOW_LOG_FILE,
        help=f"MFC flow log file (default: {DEFAULT_FLOW_LOG_FILE}).",
    )
    parser.add_argument(
        "--control-interval",
        type=float,
        default=DEFAULT_CONTROL_INTERVAL_SECONDS,
        help=f"Pressure-feedback interval in seconds (default: {DEFAULT_CONTROL_INTERVAL_SECONDS:g}).",
    )
    parser.add_argument(
        "--log-interval",
        type=float,
        default=DEFAULT_LOG_INTERVAL_SECONDS,
        help=f"MFC logging interval in seconds (default: {DEFAULT_LOG_INTERVAL_SECONDS:g}).",
    )
    parser.add_argument(
        "--max-cycles",
        type=int,
        default=None,
        help="Stop after this many pressure-control cycles (default: continuous).",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help=(
            "Run one pressure-control cycle, then exit after any applied supply "
            "Setpoint change has completed settling and post-settling checks."
        ),
    )
    parser.add_argument(
        "--port",
        default=None,
        help="RS-485 serial port (default: auto-detect B&B ULINX USOPTL4).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print low-level RS-485 TX/RX frames (debugging).",
    )
    parser.add_argument(
        "--apply",
        action="store_true",
        help="Enable automatic MFC writes after explicit startup confirmation.",
    )
    return parser


def main() -> int:
    args = _build_argument_parser().parse_args()

    try:
        current_flow = PressureController.validate_total_flow(
            args.current_flow_sccm
        )
        if args.control_interval <= 0 or not math.isfinite(args.control_interval):
            raise ValueError("--control-interval must be finite and positive")
        if args.log_interval <= 0 or not math.isfinite(args.log_interval):
            raise ValueError("--log-interval must be finite and positive")
        if args.max_cycles is not None and args.max_cycles <= 0:
            raise ValueError("--max-cycles must be a positive integer")

        from mfc.fcst_driver import FCSTDriver

        with FCSTDriver(port=args.port, verbose=args.verbose) as driver:
            manager = MFCManager(driver)
            controller = GasReplacementFeedbackController(
                driver,
                manager,
                current_total_flow_sccm=current_flow,
                pressure_file=args.pressure_file,
                flow_log_file=args.log_file,
                control_interval_seconds=args.control_interval,
                log_interval_seconds=args.log_interval,
            )
            controller.verify_startup(applying=args.apply)

            if args.apply:
                if not _confirm_live_operation():
                    print("Confirmation not received. No MFC writes were sent.")
                    return 0
            else:
                print("DRY RUN selected: no MFC WRITE commands will be sent.\n")

            return controller.run(
                apply=args.apply,
                once=args.once,
                max_cycles=args.max_cycles,
            )

    except KeyboardInterrupt:
        print("\nInterrupted by operator. Controller stopped.", file=sys.stderr)
        print(
            "If interruption occurred during a multi-MFC update, the MFCs may "
            "be partially updated. Read all three MFC states before restarting.",
            file=sys.stderr,
        )
        return 130
    except FlowDeviationError as exc:
        print(f"\n[FLOW CONTROL STOP] {exc}", file=sys.stderr)
        print(
            "Do not restore the initial flow after this alarm. Verify all three "
            "MFC Setpoints, the gas cylinders, and the supply/exhaust lines before restarting.",
            file=sys.stderr,
        )
        return 2
    except (PressureDataError, FlowRangeError, FlowPreflightError,
            FlowApplicationError, ControllerStartupError) as exc:
        print(f"\n[CONTROL STOP] {exc}", file=sys.stderr)
        print(
            "No further feedback actions will be issued. Check the CDC pressure, "
            "all three MFC states, and the gas path before restarting.",
            file=sys.stderr,
        )
        return 2
    except Exception as exc:
        print(f"\n[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        print(
            "Controller exited. Confirm the hardware state before retrying.",
            file=sys.stderr,
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

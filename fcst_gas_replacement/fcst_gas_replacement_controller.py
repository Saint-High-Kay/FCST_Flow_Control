#!/usr/bin/env python3
"""Top-level controller for COMET CDC gas replacement and pressure feedback.

The controller integrates:
    control/pressure_controller.py  : read GL840 log and decide total flow
    gas/gas_replacement.py           : calculate Cir01/Cir02/Exh01 setpoints
    control/flow_controller.py      : preflight and apply the MFC setpoints
    data_logger.py                  : log Setpoint and Indicated Flow values
    mfc/fcst_driver.py              : shared RS-485 connection
    mfc/mfc_manager.py              : MFC identity, mode, and write verification

Safety model:
* Default operation is DRY RUN. No MFC WRITE is issued unless --apply is set.
* --apply requires the operator to type "ENABLE FEEDBACK" once at startup.
* The declared current total flow must be accurate; it determines the MFC
  transition order. Digital-mode setpoints are checked against that declaration.
* A stale/invalid GL840 record or an MFC preflight/write error stops the
  controller. No automatic stop or rollback is attempted on an error.
* Starting from 0 sccm does not automatically restart the gas flow. Start the
  initial flow separately, then launch this feedback controller with that flow
  specified as --current-flow-sccm.

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
from typing import Any

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
)
from gas.gas_replacement import calculate_gas_replacement_plan
from mfc.mfc_manager import MFCManager, raw_to_sccm, sccm_to_raw


DEFAULT_CONTROL_INTERVAL_SECONDS = 5.0
DEFAULT_LOG_INTERVAL_SECONDS = 1.0
DEFAULT_FLOW_LOG_FILE = Path("data/mfc_flow.dat")


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


@dataclass(frozen=True, slots=True)
class CycleResult:
    cycle_number: int
    current_total_flow_sccm: float
    target_total_flow_sccm: float
    action: str
    applied: bool
    plan: FlowControllerPlan


class GasReplacementFeedbackController:
    """Orchestrate pressure feedback, MFC flow updates, and flow logging.

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
        self.manager = manager
        self.pressure_controller = PressureController(data_file=pressure_file)
        self.flow_controller = FlowController(manager)
        # Identity has already been checked by the startup read_all_status().
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

    def verify_startup(self, *, applying: bool) -> None:
        """Verify MFC identities and check declared flow against digital SPs.

        An Analog-mode MFC is reported but its digital Setpoint register is not
        used to validate the declared current flow, because that register may
        not represent the active analog input command.
        """
        statuses = self.manager.read_all_status()
        missing = {"Cir01", "Cir02", "Exh01"} - set(statuses)
        if missing:
            raise ControllerStartupError(
                "The following configured MFCs were not returned by read_all_status(): "
                + ", ".join(sorted(missing))
            )

        print("=== MFC startup status (identity verified) ===")
        for name in ("Cir01", "Cir02", "Exh01"):
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

        expected_plan = calculate_gas_replacement_plan(
            self.current_total_flow_sccm
        )
        expected_by_name = expected_plan.by_name()
        mismatches: list[str] = []
        analog_names: list[str] = []

        for name in ("Cir01", "Cir02", "Exh01"):
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
                "for Analog-mode MFC(s): " + ", ".join(analog_names)
            )
        if mismatches:
            detail = "\n  - ".join(mismatches)
            message = (
                "The declared current total flow does not match the current "
                "Digital-mode MFC Setpoints:\n  - " + detail
            )
            if applying:
                raise ControllerStartupError(
                    message + "\nAutomatic MFC writes are blocked. Check the actual "
                    "setpoints and restart with the correct --current-flow-sccm."
                )
            print("[WARN] " + message.replace("\n", "\n       "))
        else:
            print(
                f"Declared current total flow: {self.current_total_flow_sccm:.3f} sccm"
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

        plan = self.flow_controller.plan_total_flow_sccm(
            decision.proposed_total_flow_sccm,
            current_total_flow_sccm=self.current_total_flow_sccm,
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
            print("\nMFC setpoint updates completed and verified:")
            for item in result.updated_mfcs:
                print(
                    f"  {item.name}: {item.requested_setpoint_sccm:.3f} sccm"
                )
        else:
            print("\nDRY RUN: proposed MFC setpoints were not written.")
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

    def _write_log_sample(self) -> None:
        start = time.monotonic()
        self.data_logger.write_sample()
        elapsed = time.monotonic() - start
        self._log_sample_number += 1
        if elapsed > self.log_interval_seconds:
            print(
                f"[WARN] MFC log sample {self._log_sample_number} took "
                f"{elapsed:.3f} s (requested interval: "
                f"{self.log_interval_seconds:.3f} s).",
                file=sys.stderr,
            )

    def run(
        self,
        *,
        apply: bool,
        once: bool = False,
        max_cycles: int | None = None,
    ) -> int:
        """Run the feedback loop and best-effort periodic MFC flow logging."""
        if max_cycles is not None and max_cycles <= 0:
            raise ValueError("max_cycles must be a positive integer")

        print("=== COMET CDC gas-replacement controller ===")
        print(f"Mode              : {'APPLY (automatic MFC writes enabled)' if apply else 'DRY RUN'}")
        print(f"Control interval  : {self.control_interval_seconds:.3f} s")
        print(f"MFC log interval  : {self.log_interval_seconds:.3f} s")
        print(f"Initial flow ref. : {self.current_total_flow_sccm:.3f} sccm")
        print(f"Pressure file     : {self.pressure_controller.data_file}")
        print(f"MFC log file      : {self.data_logger.output_file}")
        print("\nThe MFC logger and controller share one RS-485 connection and run sequentially.")
        print("A 1-second log interval is best-effort; control transactions can delay samples.")

        if once:
            self._write_log_sample()
            self._run_control_cycle(apply=apply)
            print("\nOne control cycle completed.")
            return 0

        next_log = time.monotonic()
        next_control = next_log
        completed_cycles = 0

        while True:
            now = time.monotonic()
            did_work = False

            # Log first when both jobs are due. Serial access is kept sequential.
            if now >= next_log:
                self._write_log_sample()
                did_work = True
                next_log += self.log_interval_seconds
                after_log = time.monotonic()
                if next_log <= after_log:
                    missed = int((after_log - next_log) // self.log_interval_seconds) + 1
                    next_log += missed * self.log_interval_seconds

            now = time.monotonic()
            if now >= next_control:
                self._run_control_cycle(apply=apply)
                completed_cycles += 1
                did_work = True
                if max_cycles is not None and completed_cycles >= max_cycles:
                    break
                next_control += self.control_interval_seconds
                after_control = time.monotonic()
                if next_control <= after_control:
                    missed = int((after_control - next_control) // self.control_interval_seconds) + 1
                    next_control += missed * self.control_interval_seconds

            if not did_work:
                sleep_until = min(next_log, next_control)
                time.sleep(max(0.0, min(0.2, sleep_until - time.monotonic())))

        print(
            f"\nController stopped. Control cycles: {completed_cycles}; "
            f"MFC log samples: {self._log_sample_number}."
        )
        return 0


def _confirm_live_operation() -> bool:
    print("\n" + "!" * 72)
    print("WARNING: AUTOMATIC MFC WRITES ARE ENABLED.")
    print("The controller will change Cir01, Cir02 and Exh01 setpoints as feedback runs.")
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
            "the system is stopped. This value determines increase/decrease order."
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
        help="Run one log sample and one pressure-control cycle, then exit.",
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

#!/usr/bin/env python3
"""Gas-replacement flow controller for two FUJIKIN FCST MFCs.

Uses the verified low-level FCST serial helpers from
fcst_flow_control_user_friendly.py (must be in the same directory).

The MFCs are calibrated for He:isobutane = 90:10 mixture. Pure-gas actual
flows are estimated from the supplied relative CF values:
    Q_actual = Q_setpoint * CF_pure_gas / CF_mixture
To produce a requested actual mixed-gas total at 90:10, the per-MFC setpoints
are calculated inversely. All writes are disabled unless --apply is supplied
and an explicit confirmation string is typed.

IMPORTANT: sequentially setting two MFCs creates a temporary composition
transient. The script chooses the order to avoid temporarily exceeding the
10% isobutane target when ramping up/down, but it does not make updates atomic.
The downstream flowmeter must still be checked by the operator; it is not
queried electronically by this script.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

try:
    from fcst_flow_control_user_friendly import (
        BAUDRATE, TIMEOUT_SECONDS, ZERO_RAW, DIGITAL_MODE, ANALOG_MODE,
        FREEZE_OFF, FREEZE_ON, SERIAL_NUMBER, CONFIGURED_MAC, CONTROL_MODE,
        DEFAULT_CONTROL_MODE, FREEZE_FOLLOW, SETPOINT, FILTERED_SETPOINT,
        INDICATED_FLOW, Device, DEVICES, TARGETS, detect_default_port,
        read_item, write_item, raw_to_flow, flow_to_raw,
    )
except ImportError as exc:
    print("ERROR: place fcst_gas_replacement_control.py in the same directory as")
    print("       fcst_flow_control_user_friendly.py.")
    raise

# Fujikin CF values supplied by the user.
CF_N2 = 1.000
CF_HE = 1.428
CF_ISOBUTANE = 0.210
CF_MIX = 0.946
HE_FRACTION = 0.90
ISOBUTANE_FRACTION = 0.10

# Use the established friendly-name profile: Cir01 is the 5 SLM He branch;
# Cir02 is the 400 sccm isobutane branch. Both devices are mix-calibrated.
HE_TARGET = "Cir01"
ISOBUTANE_TARGET = "Cir02"


@dataclass
class MFCState:
    target: str
    label: str
    serial: str
    mac: int
    device: Device
    current_mode: int
    default_mode: int
    freeze: int
    setpoint_raw: int
    filtered_raw: int
    indicated_raw: int
    transition_needed: bool


@dataclass
class FlowPlan:
    total_target_sccm: float
    actual_he_target_sccm: float
    actual_iso_target_sccm: float
    he_setpoint_slm: float
    iso_setpoint_sccm: float
    he_raw: int
    iso_raw: int
    he_encoded_actual_sccm: float
    iso_encoded_actual_sccm: float
    encoded_total_actual_sccm: float


def actual_gas_flow_from_mix_calibrated_setpoint(setpoint_sccm: float, gas_cf: float) -> float:
    """Convert mix-calibrated nominal setpoint to estimated pure-gas flow, sccm."""
    return setpoint_sccm * gas_cf / CF_MIX


def mixture_target_to_setpoints(total_sccm: float) -> FlowPlan:
    if not math.isfinite(total_sccm) or total_sccm < 0:
        raise ValueError("Total flow must be a finite, non-negative value in sccm.")

    he_dev = DEVICES[TARGETS[HE_TARGET]["serial"]]       # F.S. 5 SLM
    iso_dev = DEVICES[TARGETS[ISOBUTANE_TARGET]["serial"]]  # F.S. 400 sccm

    if total_sccm == 0:
        he_slm = 0.0
        iso_sccm = 0.0
        he_raw = flow_to_raw(he_slm, he_dev)
        iso_raw = flow_to_raw(iso_sccm, iso_dev)
        return FlowPlan(0.0, 0.0, 0.0, he_slm, iso_sccm, he_raw, iso_raw,
                        0.0, 0.0, 0.0)

    actual_he = total_sccm * HE_FRACTION
    actual_iso = total_sccm * ISOBUTANE_FRACTION

    # Inverse CF relation: setpoint = desired actual gas flow * CF_mix / CF_gas.
    he_setpoint_sccm = actual_he * CF_MIX / CF_HE
    he_setpoint_slm = he_setpoint_sccm / 1000.0
    iso_setpoint_sccm = actual_iso * CF_MIX / CF_ISOBUTANE

    # Validate both operating ranges BEFORE opening the serial port.
    he_raw = flow_to_raw(he_setpoint_slm, he_dev)
    iso_raw = flow_to_raw(iso_setpoint_sccm, iso_dev)

    # Setpoint register quantization: calculate what the encoded values imply.
    he_encoded_setpoint_sccm = raw_to_flow(he_raw, he_dev) * 1000.0
    iso_encoded_setpoint_sccm = raw_to_flow(iso_raw, iso_dev)
    he_encoded_actual = actual_gas_flow_from_mix_calibrated_setpoint(he_encoded_setpoint_sccm, CF_HE)
    iso_encoded_actual = actual_gas_flow_from_mix_calibrated_setpoint(iso_encoded_setpoint_sccm, CF_ISOBUTANE)
    return FlowPlan(total_sccm, actual_he, actual_iso, he_setpoint_slm,
                    iso_setpoint_sccm, he_raw, iso_raw, he_encoded_actual,
                    iso_encoded_actual, he_encoded_actual + iso_encoded_actual)


def read_state(ser_obj, target: str) -> Optional[MFCState]:
    profile = TARGETS[target]
    label = profile["label"]
    expected_serial = profile["serial"]
    mac = int(profile["mac"])
    dev = DEVICES[expected_serial]

    print("\n" + "=" * 72)
    print(f"PREFLIGHT {label}: expected serial={expected_serial}; MAC=0x{mac:02X}; F.S.={dev.full_scale:g} {dev.unit}")
    serial_no = read_item(ser_obj, mac, f"{label} Serial Number", SERIAL_NUMBER, string=True)
    configured_mac = read_item(ser_obj, mac, f"{label} Configured MAC ID", CONFIGURED_MAC)
    if serial_no != expected_serial or configured_mac != mac:
        print(f"ABORT: identity mismatch for {label}; expected {expected_serial}/0x{mac:02X}, got {serial_no!r}/0x{configured_mac if configured_mac is not None else 'None'}.")
        return None

    mode = read_item(ser_obj, mac, f"{label} Current Control Mode", CONTROL_MODE)
    default_mode = read_item(ser_obj, mac, f"{label} Default Control Mode", DEFAULT_CONTROL_MODE)
    freeze = read_item(ser_obj, mac, f"{label} Freeze Follow", FREEZE_FOLLOW)
    setpoint_raw = read_item(ser_obj, mac, f"{label} Setpoint Register", SETPOINT)
    filtered_raw = read_item(ser_obj, mac, f"{label} Filtered Setpoint", FILTERED_SETPOINT)
    indicated_raw = read_item(ser_obj, mac, f"{label} Indicated Flow (internal MFC value)", INDICATED_FLOW)
    if any(v is None for v in (mode, default_mode, freeze, setpoint_raw, filtered_raw, indicated_raw)):
        print(f"ABORT: one or more status reads failed for {label}.")
        return None
    if mode not in (ANALOG_MODE, DIGITAL_MODE) or freeze not in (FREEZE_OFF, FREEZE_ON):
        print(f"ABORT: unsupported control state for {label}: mode={mode}, Freeze Follow={freeze}.")
        return None

    if mode == ANALOG_MODE:
        if freeze != FREEZE_OFF or setpoint_raw != ZERO_RAW or filtered_raw != ZERO_RAW:
            print(f"ABORT: {label} is in Analog mode, but does not have the expected zero/off transition state.")
            print("No writes should be made until the current MFC state is inspected.")
            return None
        transition_needed = True
    elif freeze == FREEZE_OFF:
        if setpoint_raw != ZERO_RAW or filtered_raw != ZERO_RAW:
            print(f"ABORT: {label} is Digital with Freeze Follow=0 but active setpoint registers are not both zero.")
            return None
        transition_needed = False
    else:
        transition_needed = False

    return MFCState(target, label, expected_serial, mac, dev, mode, default_mode,
                    freeze, setpoint_raw, filtered_raw, indicated_raw, transition_needed)


def show_plan(plan: FlowPlan) -> None:
    print("\n" + "=" * 72)
    print("FCST GAS-REPLACEMENT FLOW CONTROL (He:isobutane = 90:10 actual-flow ratio)")
    print(f"CF values: N2={CF_N2:.3f}; He={CF_HE:.3f}; isobutane={CF_ISOBUTANE:.3f}; mix={CF_MIX:.3f}")
    print(f"Requested actual mixed-gas total: {plan.total_target_sccm:g} sccm")
    if plan.total_target_sccm == 0:
        print("STOP REQUEST: both Cir01 (He) and Cir02 (isobutane) will receive zero-flow commands.")
    else:
        print(f"Desired actual component flows: He={plan.actual_he_target_sccm:.3f} sccm; isobutane={plan.actual_iso_target_sccm:.3f} sccm")
        print("Calculated MFC setpoints (mixed-gas-calibrated units):")
        print(f"  Cir01 / pure He / F.S. 5 SLM: {plan.he_setpoint_slm:.6f} SLM; raw=0x{plan.he_raw:04X}")
        print(f"  Cir02 / pure isobutane / F.S. 400 sccm: {plan.iso_setpoint_sccm:.3f} sccm; raw=0x{plan.iso_raw:04X}")
        print("CF-corrected actual flows implied by quantized setpoints:")
        print(f"  He: {plan.he_encoded_actual_sccm:.3f} sccm")
        print(f"  Isobutane: {plan.iso_encoded_actual_sccm:.3f} sccm")
        print(f"  Total estimate: {plan.encoded_total_actual_sccm:.3f} sccm (rounding/CF estimate; not an independent meter reading)")
        if plan.encoded_total_actual_sccm > 0:
            iso_percent = 100.0 * plan.iso_encoded_actual_sccm / plan.encoded_total_actual_sccm
            print(f"  Estimated composition: He {100.0-iso_percent:.3f}% / isobutane {iso_percent:.3f}%")


def execute_one(ser_obj, state: MFCState, target_raw: int, settle_seconds: float) -> bool:
    label, mac = state.label, state.mac
    # If changing control source, only the zero/off preflight state is accepted.
    if state.transition_needed:
        if not write_item(ser_obj, mac, f"{label}: WRITE Current Control Mode = Digital", CONTROL_MODE, DIGITAL_MODE):
            print(f"ABORT: {label} Digital-mode write uncertain; inspect both MFCs before retrying.")
            return False
        time.sleep(settle_seconds)
        mode_after = read_item(ser_obj, mac, f"{label}: Verify Current Control Mode", CONTROL_MODE)
        freeze_after = read_item(ser_obj, mac, f"{label}: Verify Freeze Follow", FREEZE_FOLLOW)
        if mode_after != DIGITAL_MODE or freeze_after != FREEZE_OFF:
            print(f"ABORT: {label} did not enter expected Digital/Freeze Follow=0 state.")
            return False

    if state.freeze == FREEZE_OFF:
        if not write_item(ser_obj, mac, f"{label}: Buffer Setpoint while Freeze Follow=0", SETPOINT, target_raw):
            print(f"ABORT: {label} Setpoint write uncertain. Inspect both MFCs before retrying.")
            return False
        time.sleep(settle_seconds)
        sp_check = read_item(ser_obj, mac, f"{label}: Check active Setpoint while buffered", SETPOINT)
        filt_check = read_item(ser_obj, mac, f"{label}: Check active Filtered Setpoint while buffered", FILTERED_SETPOINT)
        if sp_check != state.setpoint_raw or filt_check != state.filtered_raw:
            print(f"ABORT: {label} active setpoint registers changed unexpectedly while Freeze Follow=0.")
            print("No Freeze Follow release was sent. Inspect both MFCs before retrying.")
            return False
        if not write_item(ser_obj, mac, f"{label}: Apply buffered Setpoint (Freeze Follow=1)", FREEZE_FOLLOW, FREEZE_ON):
            print(f"ABORT: {label} Freeze Follow write uncertain; inspect both MFCs before retrying.")
            return False
        time.sleep(settle_seconds)
    else:
        if not write_item(ser_obj, mac, f"{label}: WRITE Setpoint (Digital + Freeze Follow=1)", SETPOINT, target_raw):
            print(f"ABORT: {label} Setpoint write uncertain; inspect both MFCs before retrying.")
            return False
        time.sleep(settle_seconds)

    # Per-MFC readback confirmation.
    mode_final = read_item(ser_obj, mac, f"{label}: Final Control Mode", CONTROL_MODE)
    freeze_final = read_item(ser_obj, mac, f"{label}: Final Freeze Follow", FREEZE_FOLLOW)
    sp_final = read_item(ser_obj, mac, f"{label}: Final Setpoint", SETPOINT)
    filter_final = read_item(ser_obj, mac, f"{label}: Final Filtered Setpoint", FILTERED_SETPOINT)
    if mode_final != DIGITAL_MODE or freeze_final != FREEZE_ON or sp_final != target_raw or filter_final != target_raw:
        print(f"FAIL: {label} final readback did not match the target. Do not rerun blindly.")
        return False
    print(f"SUCCESS: {label} setpoint verified at 0x{target_raw:04X}.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Set actual He:isobutane=90:10 mixed-gas total flow using CF values and two FCST MFCs."
    )
    parser.add_argument("--total-flow-sccm", type=float,
                        help="Target actual mixed-gas total flow in sccm (0 commands both MFCs to stop).")
    parser.add_argument("--port", help="RS-485/USB adapter port; a unique USOPTL4 by-id port is auto-detected if omitted.")
    parser.add_argument("--apply", action="store_true", help="Write both MFC setpoints; dry-run is the default.")
    parser.add_argument("--settle-seconds", type=float, default=1.0,
                        help="Wait after each write before verification (0-10 seconds; default 1).")
    parser.add_argument("--list", action="store_true", help="Show fixed MFC/CF assignment and exit.")
    args = parser.parse_args()

    if args.list:
        print("Gas-replacement MFC map:")
        print("  He:        Cir01 / AT18ZQW-00 / MAC 0x21 / F.S. 5 SLM / CF 1.428")
        print("  Isobutane: Cir02 / AT1MSM9-00 / MAC 0x22 / F.S. 400 sccm / CF 0.210")
        print("  Calibration gas: He:isobutane=90:10 mixture / CF 0.946")
        print("  Positive target is valid only if BOTH calculated MFC setpoints lie within their control ranges.")
        return 0

    if args.total_flow_sccm is None:
        try:
            args.total_flow_sccm = float(input("Enter target ACTUAL mixed-gas total flow in sccm (0 = stop both): ").strip())
        except (ValueError, EOFError):
            print("ERROR: enter a numeric total flow in sccm.")
            return 2
    if not math.isfinite(args.total_flow_sccm) or args.total_flow_sccm < 0:
        print("ERROR: total flow must be a finite, non-negative number of sccm.")
        print("No serial port was opened and no MFC commands were sent.")
        return 2
    if not 0 <= args.settle_seconds <= 10:
        print("ERROR: --settle-seconds must be between 0 and 10 seconds.")
        return 2

    try:
        plan = mixture_target_to_setpoints(args.total_flow_sccm)
    except ValueError as exc:
        print("=" * 72)
        print("FCST GAS-REPLACEMENT FLOW CONTROL")
        print(f"Requested actual mixed-gas total: {args.total_flow_sccm:g} sccm")
        print("ERROR: could not produce the requested ratio within both MFC operating ranges.")
        print("Reason:", exc)
        print("No serial port was opened and no MFC commands were sent; existing settings remain unchanged.")
        return 2

    if args.port is None:
        args.port = detect_default_port()
        if args.port is None:
            print("ERROR: could not uniquely detect a USOPTL4 serial adapter.")
            print("Specify --port /dev/serial/by-id/<USOPTL4-device-name>.")
            return 2
    show_plan(plan)
    print(f"Port: {args.port}; serial: {BAUDRATE} bps, 8N1")
    print("Target profiles are fixed internally; serial number and MAC ID are checked before writes.")
    print("Default Control Mode will not be changed.")
    print("Mode: APPLY (MFC settings will be changed)" if args.apply else "Mode: DRY RUN (no writes will be sent)")
    print("Positive target validation: both settings are individually checked against the 2-100% FS MFC ranges.")
    if serial is None:
        print("ERROR: pySerial is not installed in this Python environment.")
        print("The CF calculation above is complete, but no MFC communication occurred.")
        return 2

    try:
        with serial.Serial(args.port, BAUDRATE, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                           timeout=TIMEOUT_SECONDS, write_timeout=1.0) as ser_obj:
            time.sleep(0.1)
            # Read and validate BOTH devices before any write is allowed.
            he_state = read_state(ser_obj, HE_TARGET)
            if he_state is None:
                print("ABORT: He MFC preflight failed. No writes sent.")
                return 1
            iso_state = read_state(ser_obj, ISOBUTANE_TARGET)
            if iso_state is None:
                print("ABORT: isobutane MFC preflight failed. No writes sent.")
                return 1

            print("\n" + "=" * 72)
            print("BOTH MFC PREFLIGHTS PASSED")
            print(f"Cir01 / He: current={raw_to_flow(he_state.indicated_raw, he_state.device)*1000.0*CF_HE/CF_MIX:.3f} sccm estimated actual He flow")
            print(f"Cir02 / isobutane: current={raw_to_flow(iso_state.indicated_raw, iso_state.device)*CF_ISOBUTANE/CF_MIX:.3f} sccm estimated actual isobutane flow")
            current_indicated_total = (raw_to_flow(he_state.indicated_raw, he_state.device)*1000.0*CF_HE/CF_MIX +
                                       raw_to_flow(iso_state.indicated_raw, iso_state.device)*CF_ISOBUTANE/CF_MIX)
            current_setpoint_total = (raw_to_flow(he_state.setpoint_raw, he_state.device)*1000.0*CF_HE/CF_MIX +
                                      raw_to_flow(iso_state.setpoint_raw, iso_state.device)*CF_ISOBUTANE/CF_MIX)
            print(f"Current CF-corrected internal indicated-flow estimate: {current_indicated_total:.3f} sccm (not an independent flowmeter reading)")
            print(f"Current CF-corrected setpoint-implied total: {current_setpoint_total:.3f} sccm")
            print("Planned target setpoints:")
            print(f"  Cir01 / He: {plan.he_setpoint_slm:g} SLM, raw=0x{plan.he_raw:04X}")
            print(f"  Cir02 / isobutane: {plan.iso_setpoint_sccm:g} sccm, raw=0x{plan.iso_raw:04X}")
            if not args.apply:
                print("\nDRY RUN complete. No settings were changed.")
                print("Review the computed CF values, physical gas routing and vent conditions before adding --apply.")
                return 0

            # Avoid deliberately increasing isobutane before He is ready. If the currently
            # commanded isobutane component exceeds the target component, reduce it first;
            # otherwise set He first and add isobutane last. This cannot eliminate transient
            # composition changes, and it uses setpoint-implied flows rather than a sensor.
            current_iso_setpoint_actual = (raw_to_flow(iso_state.setpoint_raw, iso_state.device) *
                                           CF_ISOBUTANE / CF_MIX)
            if current_iso_setpoint_actual > plan.actual_iso_target_sccm:
                write_order = [(ISOBUTANE_TARGET, iso_state, plan.iso_raw),
                               (HE_TARGET, he_state, plan.he_raw)]
                order_text = "Cir02 (reduce isobutane to target first), then Cir01 (He)"
            else:
                write_order = [(HE_TARGET, he_state, plan.he_raw),
                               (ISOBUTANE_TARGET, iso_state, plan.iso_raw)]
                order_text = "Cir01 (He first), then Cir02 (isobutane)"
            print("\nApply order:", order_text)
            print("WARNING: the two setpoints cannot be applied atomically. A temporary composition/total-flow transient will occur.")
            print("Confirm both gas paths, downstream pressure, ventilation, exhaust and the temporary transient are safe.")
            if args.total_flow_sccm == 0:
                confirmation = "STOP BOTH CIRCULATION MFCS"
            else:
                confirmation = f"SET MIX TO {args.total_flow_sccm:g} sccm HE:ISOBUTANE 90:10"
            typed = input(f'Type exactly "{confirmation}" to continue: ').strip()
            if typed != confirmation:
                print("ABORT: confirmation did not match. No writes sent.")
                return 1

            for target, state, target_raw in write_order:
                print(f"\nApplying {state.label} ({'He' if target == HE_TARGET else 'isobutane'})...")
                if not execute_one(ser_obj, state, target_raw, args.settle_seconds):
                    print("\nSTOP: at least one MFC could not be verified. Do not rerun blindly.")
                    print("Because another MFC may already have been updated, read both MFC states before further commands.")
                    print("If a safe stop is required, issue an explicit zero-flow request after assessing the gas path.")
                    return 1

            # Final verification on both devices, including actual-flow estimates from internal readings.
            he_flow_raw = read_item(ser_obj, he_state.mac, "Cir01 Final Indicated Flow", INDICATED_FLOW)
            iso_flow_raw = read_item(ser_obj, iso_state.mac, "Cir02 Final Indicated Flow", INDICATED_FLOW)
            he_sp_final = read_item(ser_obj, he_state.mac, "Cir01 Final Setpoint", SETPOINT)
            iso_sp_final = read_item(ser_obj, iso_state.mac, "Cir02 Final Setpoint", SETPOINT)
            he_filt_final = read_item(ser_obj, he_state.mac, "Cir01 Final Filtered Setpoint", FILTERED_SETPOINT)
            iso_filt_final = read_item(ser_obj, iso_state.mac, "Cir02 Final Filtered Setpoint", FILTERED_SETPOINT)
            expected_final_ok = (he_sp_final == plan.he_raw and he_filt_final == plan.he_raw and
                                 iso_sp_final == plan.iso_raw and iso_filt_final == plan.iso_raw)
            if he_flow_raw is None or iso_flow_raw is None:
                print("WARNING: at least one internal indicated-flow read failed; external verification is required.")
            else:
                he_nominal = raw_to_flow(he_flow_raw, he_state.device) * 1000.0
                iso_nominal = raw_to_flow(iso_flow_raw, iso_state.device)
                he_actual_est = actual_gas_flow_from_mix_calibrated_setpoint(he_nominal, CF_HE)
                iso_actual_est = actual_gas_flow_from_mix_calibrated_setpoint(iso_nominal, CF_ISOBUTANE)
                estimated_total = he_actual_est + iso_actual_est
                print("\n" + "=" * 72)
                print("FINAL CF-CORRECTED INTERNAL FLOW ESTIMATE")
                print(f"He: nominal indicated={he_nominal:.3f} sccm-equivalent; estimated actual={he_actual_est:.3f} sccm")
                print(f"Isobutane: nominal indicated={iso_nominal:.3f} sccm-equivalent; estimated actual={iso_actual_est:.3f} sccm")
                print(f"Total estimated actual flow: {estimated_total:.3f} sccm; target={plan.total_target_sccm:.3f} sccm")
                if estimated_total > 0:
                    iso_percent = iso_actual_est / estimated_total * 100.0
                    print(f"Estimated composition: He={100.0-iso_percent:.3f}%; isobutane={iso_percent:.3f}%")
                print("This estimate is based on MFC internal readings and supplied CF values, not an independent measurement.")
                print("Verify the final total and ratio with the downstream flowmeter / suitable measurement method.")
            if expected_final_ok:
                print("SUCCESS: both setpoint and Filtered Setpoint registers match their calculated targets.")
                print("Default Control Mode values were not changed.")
                return 0
            print("FAIL: final setpoint readback does not match the calculated targets. Inspect both MFCs before retrying.")
            return 1
    except (serial.SerialException, PermissionError, OSError) as exc:
        print("ERROR opening/accessing serial port or communicating with the MFC:", exc)
        print("If a write was already sent, inspect both MFCs' mode, Freeze Follow and setpoint registers before retrying.")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

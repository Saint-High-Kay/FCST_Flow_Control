#!/usr/bin/env python3
"""Addressed FCST MFC flow control: enable Digital mode if needed, then set flow.

User-facing circulation names (sorted by full-scale, largest first):
  Cir01 -> AT18ZQW-00, MAC 0x21, F.S. 5 SLM; --setpoint is in SLM
  Cir02 -> AT1MSM9-00, MAC 0x22, F.S. 400 sccm; --setpoint is in sccm
The operator selects Cir01/Cir02; serial number and MAC are verified internally.

Safety behavior:
  * An addressed MAC ID (0x21..0x9F) and expected serial number are required.
  * Dry-run is the default; no writes occur without --apply and typed confirmation.
  * Exact numeric zero is a separate zero-flow command, encoded as 0x4000.
  * Positive requests outside 2..100% FS are rejected before opening the serial port.
  * If Analog mode is active, the script only changes over when Freeze Follow=0 and
    both Setpoint and Filtered Setpoint are at the zero code 0x4000.
  * Default Control Mode is read only and never changed.
  * When Freeze Follow=0, the manual says the new Setpoint is buffered in RAM,
    while the visible Setpoint/Filtered Setpoint retain the previously active value.
    The verification therefore checks that the visible values remain unchanged
    before Freeze Follow is set to 1.
  * Actual hardware execution of this corrected combined script has not been verified.
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, Union

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

BAUDRATE = 38400
TIMEOUT_SECONDS = 1.0
STX = 0x02
READ_COMMAND = 0x80
WRITE_COMMAND = 0x81
ZERO_RAW = 0x4000
MAX_RAW = 0xC000
MIN_CONTROL_PERCENT_FS = 2.0
DIGITAL_MODE = 0x01
ANALOG_MODE = 0x02
FREEZE_OFF = 0x00
FREEZE_ON = 0x01

# (class ID, instance ID, attribute ID, data length)
SERIAL_NUMBER = (0x64, 0x01, 0x07, 7)  # class 0x64, instance 1, attribute 7
CONFIGURED_MAC = (0x03, 0x01, 0x01, 1)
CONTROL_MODE = (0x69, 0x01, 0x03, 1)
DEFAULT_CONTROL_MODE = (0x69, 0x01, 0x04, 1)
FREEZE_FOLLOW = (0x69, 0x01, 0x05, 1)
SETPOINT = (0x69, 0x01, 0xA4, 2)
FILTERED_SETPOINT = (0x6A, 0x01, 0xA6, 2)
INDICATED_FLOW = (0x6A, 0x01, 0xA9, 2)


@dataclass(frozen=True)
class Device:
    serial_number: str
    full_scale: float
    unit: str


DEVICES = {
    "AT18ZQW-00": Device("AT18ZQW-00", 5.0, "SLM"),
    "AT1MSM9-00": Device("AT1MSM9-00", 400.0, "sccm"),
}

# Operator-friendly aliases. Keep this mapping synchronized with the physical labels.
# Cir01 is the highest-F.S. unit; Cir02 is the lower-F.S. unit.
TARGETS = {
    "Cir01": {"label": "Cir 01", "serial": "AT18ZQW-00", "mac": 0x21},
    "Cir02": {"label": "Cir 02", "serial": "AT1MSM9-00", "mac": 0x22},
}


def print_pickup_table() -> None:
    print("\nAvailable MFCs (ordered by full-scale, largest first):")
    print("  Selection | F.S.     | Flow unit | Positive control range")
    print("  ----------+----------+-----------+-----------------------")
    for key, profile in TARGETS.items():
        dev = DEVICES[profile["serial"]]
        minimum = dev.full_scale * MIN_CONTROL_PERCENT_FS / 100.0
        print(f"  {key:<9} | {dev.full_scale:>6g} {dev.unit:<4} | {dev.unit:<9} | {minimum:g}-{dev.full_scale:g} {dev.unit}")
    print("  Note: serial number and MAC ID are checked automatically; do not rewire based on the names alone.")


def detect_default_port() -> Optional[str]:
    candidates = sorted(str(p) for p in Path("/dev/serial/by-id").glob("*USOPTL4*"))
    if len(candidates) == 1:
        return candidates[0]
    return None


def make_read_request(mac: int, item: Tuple[int, int, int, int]) -> bytes:
    cls, inst, attr, _size = item
    length = 0x03
    checksum = (STX + READ_COMMAND + length + cls + inst + attr) & 0xFF
    return bytes([mac, STX, READ_COMMAND, length, cls, inst, attr, 0x00, checksum])


def make_write_request(mac: int, item: Tuple[int, int, int, int], value: int) -> bytes:
    cls, inst, attr, size = item
    if size not in (1, 2):
        raise ValueError("Only UINT8 and UINT16 writes are supported.")
    if not 0 <= value < (1 << (8 * size)):
        raise ValueError(f"Value {value} does not fit UINT{8 * size}.")
    payload = value.to_bytes(size, "little", signed=False)
    length = 3 + size
    checksum = (STX + WRITE_COMMAND + length + cls + inst + attr + sum(payload)) & 0xFF
    return bytes([mac, STX, WRITE_COMMAND, length, cls, inst, attr]) + payload + bytes([0x00, checksum])


def parse_read_response(rx: bytes, item: Tuple[int, int, int, int], string: bool = False):
    cls, inst, attr, expected_size = item
    marker = bytes([0x00, STX, READ_COMMAND])
    pos = rx.find(marker)
    if pos < 0:
        if b"\x16" in rx:
            return None, "NAK (0x16): MFC rejected the request."
        return None, "No FCST response frame found."
    if len(rx) < pos + 9:
        return None, "Response frame is incomplete."
    packet_len = rx[pos + 3]
    got = tuple(rx[pos + 4:pos + 7])
    if got != (cls, inst, attr):
        return None, f"Response class/instance/attribute mismatch: {got!r}."
    data_len = packet_len - 3
    if data_len < 0 or (not string and data_len != expected_size):
        return None, f"Unexpected payload size {data_len}."
    end = pos + 7 + data_len
    if len(rx) < end + 2:
        return None, "Response frame is incomplete."
    data, pad, checksum = rx[pos + 7:end], rx[end], rx[end + 1]
    calculated = sum(rx[pos + 1:end]) & 0xFF
    if pad != 0x00:
        return None, f"Unexpected PAD byte 0x{pad:02X}."
    if checksum != calculated:
        return None, f"Checksum mismatch (received 0x{checksum:02X}, expected 0x{calculated:02X})."
    return data, "OK"


def read_item(ser_obj, mac: int, label: str, item: Tuple[int, int, int, int], string: bool = False):
    tx = make_read_request(mac, item)
    ser_obj.reset_input_buffer()
    ser_obj.write(tx)
    ser_obj.flush()
    rx = ser_obj.read(128)
    print(f"\n[{label}] TX: {tx.hex(' ').upper()}")
    print("RX:", rx.hex(" ").upper() if rx else "<timeout>")
    if not rx:
        print("ERROR: read timed out.")
        return None
    data, message = parse_read_response(rx, item, string=string)
    if data is None:
        print("ERROR:", message)
        return None
    if string:
        value = data.decode("ascii", errors="replace").strip("\x00 \r\n")
        print("Value:", value)
        return value
    value = int.from_bytes(data, "little", signed=False)
    print(f"Value: 0x{value:0{len(data)*2}X} ({value})")
    return value


def write_item(ser_obj, mac: int, label: str, item: Tuple[int, int, int, int], value: int) -> bool:
    tx = make_write_request(mac, item, value)
    print(f"\n[{label}] TX: {tx.hex(' ').upper()}")
    ser_obj.reset_input_buffer()
    ser_obj.write(tx)
    ser_obj.flush()
    ack1 = ser_obj.read(1)
    ack2 = ser_obj.read(1)
    print("ACK 1:", ack1.hex(" ").upper() if ack1 else "<timeout>")
    print("ACK 2:", ack2.hex(" ").upper() if ack2 else "<timeout>")
    if ack1 != b"\x06" or ack2 != b"\x06":
        print("ERROR: expected two ACK (0x06) bytes; write result may be uncertain.")
        if ack1 == b"\x16" or ack2 == b"\x16":
            print("MFC returned NAK (0x16).")
        return False
    return True


def raw_to_flow(raw: int, device: Device) -> float:
    return ((raw - ZERO_RAW) / 32768.0) * device.full_scale


def flow_to_raw(flow: float, device: Device) -> int:
    if not math.isfinite(flow) or flow < 0:
        raise ValueError("Flow must be a finite non-negative number.")
    if flow == 0.0:
        return ZERO_RAW
    minimum_flow = device.full_scale * MIN_CONTROL_PERCENT_FS / 100.0
    if flow < minimum_flow or flow > device.full_scale:
        raise ValueError(
            f"Positive flow must be within {minimum_flow:g} to {device.full_scale:g} {device.unit} "
            f"(2-100% FS); received {flow:g} {device.unit}."
        )
    raw_float = ZERO_RAW + (flow / device.full_scale) * 32768.0
    raw = int(raw_float + 0.5)
    min_raw = math.ceil(ZERO_RAW + (MIN_CONTROL_PERCENT_FS / 100.0) * 32768.0)
    raw = max(min_raw, min(raw, MAX_RAW))
    return raw


def main() -> int:
    p = argparse.ArgumentParser(
        description=("User-friendly FCST flow control: select Cir01/Cir02, verify the mapped MFC, "
                     "enable Digital mode if needed, then set and verify the flow setpoint.")
    )
    p.add_argument("--port", help="RS-485/USB adapter serial port. If omitted, one USOPTL4 by-id port is auto-detected.")
    p.add_argument("--target", choices=sorted(TARGETS), help="User-facing MFC name: Cir01 (5 SLM) or Cir02 (400 sccm)")
    p.add_argument("--setpoint", type=float, help="Target flow; unit is selected automatically (SLM for Cir01, sccm for Cir02); 0 means zero flow")
    p.add_argument("--list", action="store_true", help="Show the MFC selection table and exit")
    p.add_argument("--apply", action="store_true", help="Perform writes; default is read-only dry-run")
    p.add_argument("--settle-seconds", type=float, default=1.0,
                   help="Wait after writes before verification (0-10 seconds; default 1.0)")
    args = p.parse_args()

    if args.list:
        print_pickup_table()
        return 0

    print_pickup_table()
    if args.target is None:
        choices = list(TARGETS)
        try:
            selection = input("Select MFC by name (Cir01/Cir02) or number (1/2): ").strip()
        except EOFError:
            print("ERROR: no selection received. Use --target Cir01 or --target Cir02.")
            return 2
        if selection in ("1", "2"):
            args.target = choices[int(selection) - 1]
        elif selection in TARGETS:
            args.target = selection
        else:
            print("ERROR: unknown MFC selection. Choose Cir01 or Cir02.")
            return 2

    target_profile = TARGETS[args.target]
    target_label = target_profile["label"]
    expected_serial = target_profile["serial"]
    mac = int(target_profile["mac"])
    device = DEVICES[expected_serial]

    if args.setpoint is None:
        try:
            raw_value = input(f"Enter target flow for {target_label} in {device.unit} (0 = stop): ").strip()
            args.setpoint = float(raw_value)
        except (ValueError, EOFError):
            print(f"ERROR: enter a numeric flow in {device.unit}.")
            return 2

    if args.port is None:
        args.port = detect_default_port()
        if args.port is None:
            print("ERROR: could not uniquely detect a USOPTL4 serial adapter.")
            print("Specify it explicitly with --port /dev/serial/by-id/<USOPTL4-device-name>.")
            return 2

    try:
        # MAC ID is intentionally fixed in the internal profile, not entered by the operator.
        if not 0x21 <= mac <= 0x9F:
            raise ValueError
    except ValueError:
        print("ERROR: internal Cir-name mapping has an invalid MAC ID. Check the TARGETS table in the script.")
        return 2
    if not 0 <= args.settle_seconds <= 10:
        print("ERROR: --settle-seconds must be between 0 and 10 seconds.")
        return 2
    if not math.isfinite(args.setpoint):
        print("ERROR: --setpoint must be a finite number.")
        return 2
    if args.setpoint < 0:
        print("ERROR: negative flow is invalid. Use 0 for zero flow.")
        return 2

    device = DEVICES[expected_serial]
    try:
        target_raw = flow_to_raw(args.setpoint, device)
    except ValueError as exc:
        requested = args.setpoint
        minimum = device.full_scale * MIN_CONTROL_PERCENT_FS / 100.0
        print("=" * 72)
        print("FCST ADDRESSED DIGITAL FLOW CONTROL")
        print(f"Target: {target_label}; F.S.: {device.full_scale:g} {device.unit}; requested flow: {requested:g} {device.unit}")
        print("ERROR:", exc)
        if requested > 0 and requested < minimum:
            print(f"Reason: positive request is below 2% FS ({minimum:g} {device.unit}).")
        elif requested > device.full_scale:
            print("Reason: requested flow exceeds 100% FS.")
        print("No serial port was opened and no command was sent; existing MFC settings remain unchanged.")
        return 2

    encoded_target = raw_to_flow(target_raw, device)
    target_percent = (encoded_target / device.full_scale) * 100.0
    is_zero_command = args.setpoint == 0.0

    print("=" * 72)
    print("FCST USER-FRIENDLY DIGITAL FLOW CONTROL")
    print("Process: select circulation MFC -> verify hardware identity -> set flow -> read back")
    print(f"Port: {args.port}; serial: {BAUDRATE} bps, 8N1")
    print(f"Selected target: {target_label} ({args.target})")
    print(f"Mapped hardware: F.S. {device.full_scale:g} {device.unit}; internal identity check enabled")
    print(f"Requested flow: {args.setpoint:g} {device.unit}; encoded raw: 0x{target_raw:04X}; decoded target: {encoded_target:.6f} {device.unit} ({target_percent:.3f}% FS)")
    print("Serial number and MAC ID are fixed in the profile and verified automatically.")
    print("Default Control Mode is read only and will not be changed.")
    print("Mode: APPLY (MFC settings will be changed)" if args.apply else "Mode: DRY RUN (no writes will be sent)")
    print("Zero-flow exception: exact numeric 0 is allowed and encoded as 0x4000." if is_zero_command else
          f"Positive target must be within {device.full_scale * 0.02:g}-{device.full_scale:g} {device.unit} (2-100% FS).")

    if serial is None:
        print("ERROR: pySerial is not installed in this Python environment.")
        return 2

    try:
        with serial.Serial(args.port, BAUDRATE, bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                           timeout=TIMEOUT_SECONDS, write_timeout=1.0) as ser_obj:
            time.sleep(0.1)
            serial_no = read_item(ser_obj, mac, "Serial Number", SERIAL_NUMBER, string=True)
            configured_mac = read_item(ser_obj, mac, "Configured MAC ID", CONFIGURED_MAC)
            if serial_no != expected_serial:
                print(f"\nABORT: {target_label} is mapped to serial {expected_serial!r}, but received {serial_no!r}. No writes sent.")
                return 1
            if configured_mac != mac:
                print(f"ABORT: {target_label} is mapped to MAC 0x{mac:02X}, but MFC reports {configured_mac!r}. No writes sent.")
                return 1

            mode = read_item(ser_obj, mac, "Current Control Mode", CONTROL_MODE)
            default_mode = read_item(ser_obj, mac, "Default Control Mode", DEFAULT_CONTROL_MODE)
            freeze = read_item(ser_obj, mac, "Freeze Follow", FREEZE_FOLLOW)
            setpoint_before = read_item(ser_obj, mac, "Setpoint Register", SETPOINT)
            filtered_before = read_item(ser_obj, mac, "Filtered Setpoint", FILTERED_SETPOINT)
            flow_before = read_item(ser_obj, mac, "Indicated Flow (internal MFC value)", INDICATED_FLOW)
            if any(v is None for v in (mode, default_mode, freeze, setpoint_before, filtered_before, flow_before)):
                print("\nABORT: one or more status reads failed. No writes sent.")
                return 1
            if mode not in (ANALOG_MODE, DIGITAL_MODE):
                print(f"ABORT: unsupported Current Control Mode 0x{mode:02X}. No writes sent.")
                return 1
            if freeze not in (FREEZE_OFF, FREEZE_ON):
                print(f"ABORT: unexpected Freeze Follow value 0x{freeze:02X}. No writes sent.")
                return 1

            # Safe transition policy: only switch from Analog when no nonzero digital target is buffered/applied.
            if mode == ANALOG_MODE:
                if freeze != FREEZE_OFF or setpoint_before != ZERO_RAW or filtered_before != ZERO_RAW:
                    print("ABORT: MFC is in Analog mode but the state is not the expected safe transition state:")
                    print(f"  mode=0x{mode:02X}; Freeze Follow=0x{freeze:02X}; Setpoint=0x{setpoint_before:04X}; Filtered=0x{filtered_before:04X}")
                    print("No writes sent. Inspect this MFC's state before changing the active control source.")
                    return 1
                transition_needed = True
            elif freeze == FREEZE_OFF:
                if setpoint_before != ZERO_RAW or filtered_before != ZERO_RAW:
                    print("ABORT: Digital mode with Freeze Follow=0 but current setpoint registers are not both zero.")
                    print("No writes sent; inspect the buffered/applied setpoint before proceeding.")
                    return 1
                transition_needed = False
            else:
                transition_needed = False  # Digital + Freeze Follow=1; normal setpoint write path.

            plan = []
            if transition_needed:
                plan.append("write Current Control Mode=Digital (0x01)")
            if freeze == FREEZE_OFF:
                plan.append(f"buffer Setpoint=0x{target_raw:04X} while Freeze Follow=0")
                plan.append("write Freeze Follow=0x01 to apply buffered setpoint")
            else:
                plan.append(f"write Setpoint=0x{target_raw:04X} directly (Digital + Freeze Follow=1)")
            print("\n--- Preflight summary ---")
            print(f"Target: {target_label}; verified hardware identity: {serial_no}, MAC 0x{mac:02X}; F.S.: {device.full_scale:g} {device.unit}")
            print(f"Current Mode: 0x{mode:02X}; Default Mode: 0x{default_mode:02X}; Freeze Follow: 0x{freeze:02X}")
            print(f"Setpoint: 0x{setpoint_before:04X} ({raw_to_flow(setpoint_before, device):.6f} {device.unit}); Filtered: 0x{filtered_before:04X} ({raw_to_flow(filtered_before, device):.6f} {device.unit})")
            print("Planned write sequence:")
            for n, action in enumerate(plan, 1):
                print(f"  {n}. {action}")
            print("Default Control Mode will remain unchanged.")
            if not args.apply:
                print("\nDRY RUN complete. No settings were changed.")
                print("Review the target identity, setpoint, gas path and outlet conditions before adding --apply.")
                return 0

            flow_txt = f"{args.setpoint:g} {device.unit}"
            confirmation = f"SET {args.target} TO {flow_txt}"
            print("\nWARNING: this operation may change actual gas flow and the active control source.")
            print("Confirm the gas path, outlet/ventilation, downstream pressure and target flow are safe.")
            typed = input(f'Type exactly "{confirmation}" to continue: ').strip()
            if typed != confirmation:
                print("ABORT: confirmation did not match. No writes sent.")
                return 1

            # Step 1: transition from Analog to Digital only after the preflight and typed confirmation.
            if transition_needed:
                if not write_item(ser_obj, mac, "WRITE Current Control Mode = Digital (0x01)", CONTROL_MODE, DIGITAL_MODE):
                    print("ABORT: mode write ACK pair missing; state may be uncertain. Read status before retrying.")
                    return 1
                time.sleep(args.settle_seconds)
                mode_after_switch = read_item(ser_obj, mac, "Verify Current Control Mode = Digital", CONTROL_MODE)
                freeze_after_switch = read_item(ser_obj, mac, "Verify Freeze Follow remains 0x00", FREEZE_FOLLOW)
                if mode_after_switch != DIGITAL_MODE or freeze_after_switch != FREEZE_OFF:
                    print("ABORT: mode/Freeze Follow did not match expectations. No further writes sent.")
                    return 1

            # Step 2 / 3: buffer target then release Freeze Follow, or direct setpoint write when freeze is already on.
            if freeze == FREEZE_OFF:
                if not write_item(ser_obj, mac, "WRITE Setpoint (buffered with Freeze Follow=0)", SETPOINT, target_raw):
                    print("ABORT: Setpoint ACK pair missing; mode may now be Digital. Read status before retrying.")
                    return 1
                time.sleep(args.settle_seconds)
                sp_buffer_check = read_item(ser_obj, mac, "Check Setpoint while Freeze Follow=0", SETPOINT)
                filtered_buffer_check = read_item(ser_obj, mac, "Check Filtered Setpoint while Freeze Follow=0", FILTERED_SETPOINT)
                # FUJIKIN FCST command manual, section 8.7.3: when Freeze Follow=0,
                # a new Setpoint is stored in RAM, but the active Setpoint remains unchanged.
                # Therefore the readable registers are expected to remain at their pre-write
                # values here; they are not expected to show target_raw until Freeze Follow=1.
                if (sp_buffer_check != setpoint_before or
                        filtered_buffer_check != filtered_before):
                    print("ABORT: active Setpoint/Filtered Setpoint changed unexpectedly while Freeze Follow=0.")
                    print(f"  Expected active Setpoint to remain 0x{setpoint_before:04X}; received {('None' if sp_buffer_check is None else f'0x{sp_buffer_check:04X}') }.")
                    print(f"  Expected Filtered Setpoint to remain 0x{filtered_before:04X}; received {('None' if filtered_buffer_check is None else f'0x{filtered_buffer_check:04X}') }.")
                    print("No Freeze Follow write was sent. Read current status and evaluate the MFC state before retrying.")
                    return 1
                if not write_item(ser_obj, mac, "WRITE Freeze Follow = 0x01 (apply buffered setpoint)", FREEZE_FOLLOW, FREEZE_ON):
                    print("ERROR: Freeze Follow ACK pair missing; the MFC may remain in Digital mode with a buffered target.")
                    print("Read status before retrying or issuing another flow command.")
                    return 1
                time.sleep(args.settle_seconds)
            else:
                if not write_item(ser_obj, mac, "WRITE Setpoint (Digital mode; Freeze Follow=1)", SETPOINT, target_raw):
                    print("ABORT: Setpoint ACK pair missing; flow state may be uncertain. Read status before retrying.")
                    return 1
                time.sleep(args.settle_seconds)

            # Final verification.
            mode_final = read_item(ser_obj, mac, "Final Current Control Mode", CONTROL_MODE)
            freeze_final = read_item(ser_obj, mac, "Final Freeze Follow", FREEZE_FOLLOW)
            setpoint_final = read_item(ser_obj, mac, "Final Setpoint Register", SETPOINT)
            filtered_final = read_item(ser_obj, mac, "Final Filtered Setpoint", FILTERED_SETPOINT)
            default_final = read_item(ser_obj, mac, "Final Default Control Mode", DEFAULT_CONTROL_MODE)
            flow_final = read_item(ser_obj, mac, "Final Indicated Flow (internal MFC value)", INDICATED_FLOW)
            print("\n--- Final result ---")
            print(f"Target: {target_label}; verified serial: {serial_no}; MAC: 0x{mac:02X}; unit: {device.unit}; F.S.: {device.full_scale:g} {device.unit}")
            print(f"Requested: {args.setpoint:g} {device.unit}; encoded target: 0x{target_raw:04X} = {encoded_target:.6f} {device.unit}")
            print(f"Current Mode: {mode_final}; Freeze Follow: {freeze_final}; Setpoint: {setpoint_final}; Filtered Setpoint: {filtered_final}; Default Mode: {default_final}")
            if flow_final is not None:
                print(f"Indicated Flow raw: 0x{flow_final:04X}; display-equivalent {raw_to_flow(flow_final, device):.6f} {device.unit} (internal value only, not an independent flowmeter measurement).")
            if (mode_final == DIGITAL_MODE and freeze_final == FREEZE_ON and
                    setpoint_final == target_raw and filtered_final == target_raw and
                    default_final == default_mode):
                print("SUCCESS: Digital mode, Freeze Follow and both setpoint registers were verified.")
                print(f"Default Control Mode remains 0x{default_final:02X}; it was not changed by this program.")
                return 0
            print("FAIL: final readback did not fully match the target. Do not rerun blindly; inspect the current state first.")
            return 1
    except (serial.SerialException, PermissionError, OSError) as exc:
        print("ERROR opening/accessing serial port or communicating with the MFC:", exc)
        print("If any write was already sent, read back the current mode, Freeze Follow, setpoint and flow before retrying.")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

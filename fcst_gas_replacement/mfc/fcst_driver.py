#!/usr/bin/env python3
"""Low-level RS-485 driver for Fujikin FCST MFCs.

This module implements the interface expected by ``mfc/mfc_manager.py``.
It communicates with multiple MFCs on one RS-485 bus by addressing each
unit through its configured MAC ID.

Configured system addresses (defined in config/mfc_profiles.py):
    Cir01 : MAC 0x21, serial AT18ZQW-00
    Cir02 : MAC 0x22, serial AT1MSM9-00
    Exh01 : MAC 0x24, serial AT1MSM6-00

The driver is intentionally device-name agnostic. Exh01 needs no special
protocol branch: it is handled in exactly the same way as Cir01 and Cir02,
using MAC ID 0x24. Device identity and full-scale information belong in the
profile table, not in this transport layer.

Protocol settings used here:
    Baud rate : 38400 bps
    Format    : 8 data bits, no parity, 1 stop bit
    READ      : 0x80
    WRITE     : 0x81
    STX       : 0x02
    READ reply: 0x00 0x02 0x80 ...
    WRITE ack : two consecutive 0x06 bytes

No communication with physical hardware is performed at import time.
"""
from __future__ import annotations

import glob
import os
import time
from dataclasses import dataclass
from typing import Any

try:
    import serial as _serial  # type: ignore[import-not-found]
except ImportError:  # pySerial is only required when opening a real port.
    _serial = None


BAUDRATE = 38400
TIMEOUT_SECONDS = 1.0
WRITE_TIMEOUT_SECONDS = 1.0
INTER_COMMAND_DELAY_SECONDS = 0.03

STX = 0x02
READ_COMMAND = 0x80
WRITE_COMMAND = 0x81
READ_RESPONSE_MARKER = bytes((0x00, STX, READ_COMMAND))
ACK = 0x06
NAK = 0x16

# (class ID, instance ID, attribute ID, expected data size in bytes)
SERIAL_NUMBER = (0x64, 0x01, 0x07, None)  # Serial-number length is read from frame.
CONFIGURED_MAC_ID = (0x03, 0x01, 0x01, 1)
CONTROL_MODE = (0x69, 0x01, 0x03, 1)
DEFAULT_CONTROL_MODE = (0x69, 0x01, 0x04, 1)
FREEZE_FOLLOW = (0x69, 0x01, 0x05, 1)
SETPOINT = (0x69, 0x01, 0xA4, 2)
FILTERED_SETPOINT = (0x6A, 0x01, 0xA6, 2)
INDICATED_FLOW = (0x6A, 0x01, 0xA9, 2)


class FCSTDriverError(RuntimeError):
    """Base class for serial/protocol errors raised by FCSTDriver."""


class FCSTPortError(FCSTDriverError):
    """The serial port could not be selected or opened."""


class FCSTTimeoutError(FCSTDriverError):
    """The MFC did not return a complete response before the timeout."""


class FCSTProtocolError(FCSTDriverError):
    """The MFC response was malformed or failed validation."""


class FCSTNAKError(FCSTDriverError):
    """The MFC rejected a command with NAK (0x16)."""


class FCSTWriteError(FCSTDriverError):
    """A WRITE command did not return the expected two ACK bytes."""


@dataclass(frozen=True, slots=True)
class Register:
    """Description of one FCST attribute."""

    class_id: int
    instance_id: int
    attribute_id: int
    size: int | None


def detect_default_port() -> str:
    """Return the unique B&B ULINX USOPTL4 by-id path.

    A by-id path is preferred over /dev/ttyUSBn because the latter may change
    when USB devices are reconnected. If detection is ambiguous, the caller
    must pass ``port=...`` explicitly.
    """
    candidates = sorted(
        path for path in glob.glob("/dev/serial/by-id/*USOPTL4*")
        if os.path.exists(path)
    )
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise FCSTPortError(
            "No B&B ULINX USOPTL4 adapter was found under "
            "/dev/serial/by-id/. Specify the port explicitly, for example "
            "port='/dev/ttyUSB1'."
        )
    raise FCSTPortError(
        "More than one USOPTL4 adapter was found. Specify the intended "
        "serial port explicitly: " + ", ".join(candidates)
    )


def _validate_mac_id(mac_id: int) -> int:
    if not isinstance(mac_id, int):
        raise TypeError("MAC ID must be an integer.")
    # 0xFF is retained for one-to-one commissioning; normal bus operation
    # uses the addressed range 0x21-0x9F.
    if not (0x21 <= mac_id <= 0x9F or mac_id == 0xFF):
        raise ValueError(
            f"Unsupported MAC ID 0x{mac_id:02X}; expected 0x21-0x9F or 0xFF."
        )
    return mac_id


def _make_read_request(mac_id: int, register: Register) -> bytes:
    mac = _validate_mac_id(mac_id)
    length = 0x03
    checksum = (
        STX + READ_COMMAND + length + register.class_id
        + register.instance_id + register.attribute_id
    ) & 0xFF
    return bytes((
        mac, STX, READ_COMMAND, length,
        register.class_id, register.instance_id, register.attribute_id,
        0x00, checksum,
    ))


def _make_write_request(mac_id: int, register: Register, value: int) -> bytes:
    mac = _validate_mac_id(mac_id)
    if register.size not in (1, 2):
        raise ValueError("WRITE supports only one- or two-byte unsigned values.")
    if not isinstance(value, int) or not (0 <= value < (1 << (8 * register.size))):
        raise ValueError(
            f"Value must fit in an unsigned {register.size * 8}-bit integer."
        )

    payload = value.to_bytes(register.size, byteorder="little", signed=False)
    length = 3 + register.size
    checksum = (
        STX + WRITE_COMMAND + length + register.class_id
        + register.instance_id + register.attribute_id + sum(payload)
    ) & 0xFF
    return (
        bytes((
            mac, STX, WRITE_COMMAND, length,
            register.class_id, register.instance_id, register.attribute_id,
        ))
        + payload
        + bytes((0x00, checksum))
    )


class FCSTDriver:
    """Serial transport and FCST attribute access for multiple MFCs.

    Parameters
    ----------
    port:
        Serial port path. When omitted, a unique ``*USOPTL4*`` path under
        ``/dev/serial/by-id`` is selected automatically.
    baudrate:
        FCST baud rate. Defaults to 38400 bps.
    timeout:
        Per-read serial timeout in seconds.
    write_timeout:
        Serial write timeout in seconds.
    verbose:
        Print TX/RX frames when True. Defaults to False to keep controller
        output readable; set True for communication debugging.
    serial_instance:
        Optional already-created serial-like object. Intended for testing;
        normal operation should use ``port`` and let this class open pySerial.

    Example
    -------
        with FCSTDriver(verbose=True) as driver:
            manager = MFCManager(driver)
            print(manager.read_all_status())  # read-only

    The driver does not change any MFC setting on construction or open.
    WRITE methods are called only by the higher-level manager after its
    explicit apply/confirmation checks.
    """

    def __init__(
        self,
        port: str | None = None,
        *,
        baudrate: int = BAUDRATE,
        timeout: float = TIMEOUT_SECONDS,
        write_timeout: float = WRITE_TIMEOUT_SECONDS,
        verbose: bool = False,
        serial_instance: Any | None = None,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.verbose = verbose
        self._serial = serial_instance
        self._owns_serial = serial_instance is None
        self._last_transaction_at = 0.0

    @property
    def is_open(self) -> bool:
        return self._serial is not None and bool(
            getattr(self._serial, "is_open", True)
        )

    def open(self) -> "FCSTDriver":
        """Open the configured serial port; no MFC commands are sent."""
        if self._serial is not None:
            if not bool(getattr(self._serial, "is_open", True)):
                try:
                    self._serial.open()
                except Exception as exc:  # serial-like objects may vary
                    raise FCSTPortError(f"Could not open serial port: {exc}") from exc
            return self

        if _serial is None:
            raise FCSTPortError(
                "pySerial is not installed. Install it with: python3 -m pip install pyserial"
            )

        selected_port = self.port or detect_default_port()
        try:
            self._serial = _serial.Serial(
                port=selected_port,
                baudrate=self.baudrate,
                bytesize=_serial.EIGHTBITS,
                parity=_serial.PARITY_NONE,
                stopbits=_serial.STOPBITS_ONE,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
        except Exception as exc:
            raise FCSTPortError(
                f"Could not open serial port {selected_port!r}: {exc}"
            ) from exc
        self.port = selected_port
        # Give the USB-RS485 adapter a brief settling interval after opening.
        time.sleep(0.1)
        return self

    def close(self) -> None:
        """Close the serial port if this driver opened it."""
        if self._serial is None:
            return
        if self._owns_serial:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def __enter__(self) -> "FCSTDriver":
        return self.open()

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.close()

    def _ensure_open(self) -> Any:
        if self._serial is None or not bool(getattr(self._serial, "is_open", True)):
            self.open()
        return self._serial

    def _wait_between_commands(self) -> None:
        remaining = INTER_COMMAND_DELAY_SECONDS - (
            time.monotonic() - self._last_transaction_at
        )
        if remaining > 0:
            time.sleep(remaining)

    def _read_exact(self, count: int) -> bytes:
        """Read exactly count bytes or raise a timeout error."""
        ser = self._ensure_open()
        result = bytearray()
        while len(result) < count:
            chunk = ser.read(count - len(result))
            if not chunk:
                raise FCSTTimeoutError(
                    f"Timed out while receiving a response: got "
                    f"{len(result)} of {count} expected bytes."
                )
            result.extend(chunk)
        return bytes(result)

    def _receive_read_frame(self) -> bytes:
        """Find and read one complete FCST READ response frame."""
        ser = self._ensure_open()
        prefix = bytearray()
        while True:
            byte = ser.read(1)
            if not byte:
                raise FCSTTimeoutError(
                    "Timed out waiting for FCST READ response. Received bytes: "
                    + (prefix.hex(" ").upper() if prefix else "<none>")
                )
            prefix.extend(byte)

            marker_pos = prefix.find(READ_RESPONSE_MARKER)
            if marker_pos >= 0:
                # The request/response stream may begin with an ACK (0x06).
                # Bytes before the marker are kept only for diagnostic output.
                if NAK in prefix[:marker_pos]:
                    raise FCSTNAKError(
                        "MFC returned NAK (0x16): "
                        + prefix.hex(" ").upper()
                    )
                marker = bytes(prefix[marker_pos:marker_pos + 3])
                length_byte = self._read_exact(1)
                packet_length = length_byte[0]
                if packet_length < 3:
                    raise FCSTProtocolError(
                        f"Invalid FCST packet length: {packet_length}."
                    )
                data_length = packet_length - 3
                # After packet length: class, instance, attribute, payload,
                # PAD, checksum = 5 + data_length bytes.
                tail = self._read_exact(5 + data_length)
                frame = marker + length_byte + tail
                if self.verbose:
                    print("RX:", (bytes(prefix) + length_byte + tail).hex(" ").upper())
                return frame

            if NAK in prefix:
                raise FCSTNAKError(
                    "MFC returned NAK (0x16): " + prefix.hex(" ").upper()
                )
            # A malformed/noisy stream should not grow without bound.
            if len(prefix) > 64:
                raise FCSTProtocolError(
                    "Could not find FCST READ response marker in received data: "
                    + prefix.hex(" ").upper()
                )

    @staticmethod
    def _parse_read_frame(
        frame: bytes,
        register: Register,
    ) -> bytes:
        """Validate the frame header, payload length, PAD and checksum."""
        if len(frame) < 9:
            raise FCSTProtocolError("FCST READ response frame is too short.")
        if frame[:3] != READ_RESPONSE_MARKER:
            raise FCSTProtocolError("FCST READ response marker is invalid.")

        packet_length = frame[3]
        data_length = packet_length - 3
        expected_frame_length = 9 + data_length
        if len(frame) != expected_frame_length:
            raise FCSTProtocolError(
                f"Frame length mismatch: received {len(frame)} bytes; "
                f"expected {expected_frame_length}."
            )

        received_register = (frame[4], frame[5], frame[6])
        requested_register = (
            register.class_id,
            register.instance_id,
            register.attribute_id,
        )
        if received_register != requested_register:
            raise FCSTProtocolError(
                "Response register mismatch: received "
                f"{received_register!r}; requested {requested_register!r}."
            )
        if register.size is not None and data_length != register.size:
            raise FCSTProtocolError(
                f"Unexpected payload size for register "
                f"0x{register.class_id:02X}/0x{register.instance_id:02X}/"
                f"0x{register.attribute_id:02X}: received {data_length}, "
                f"expected {register.size}."
            )

        data_end = 7 + data_length
        payload = frame[7:data_end]
        pad = frame[data_end]
        received_checksum = frame[data_end + 1]
        calculated_checksum = sum(frame[1:data_end]) & 0xFF

        if pad != 0x00:
            raise FCSTProtocolError(f"Unexpected PAD byte 0x{pad:02X}.")
        if received_checksum != calculated_checksum:
            raise FCSTProtocolError(
                f"Checksum mismatch: received 0x{received_checksum:02X}; "
                f"calculated 0x{calculated_checksum:02X}."
            )
        return payload

    def _read_register(self, mac_id: int, register_tuple: tuple[int, int, int, int | None]) -> bytes:
        """Send one addressed READ request and return its validated payload."""
        ser = self._ensure_open()
        register = Register(*register_tuple)
        request = _make_read_request(mac_id, register)

        self._wait_between_commands()
        ser.reset_input_buffer()
        if self.verbose:
            print("TX:", request.hex(" ").upper())
        written = ser.write(request)
        if written is not None and written != len(request):
            raise FCSTDriverError(
                f"Incomplete serial write: sent {written} of {len(request)} bytes."
            )
        ser.flush()
        self._last_transaction_at = time.monotonic()

        frame = self._receive_read_frame()
        return self._parse_read_frame(frame, register)

    def _write_register(
        self,
        mac_id: int,
        register_tuple: tuple[int, int, int, int | None],
        value: int,
    ) -> None:
        """Send one addressed WRITE request and require the ACK pair."""
        ser = self._ensure_open()
        register = Register(*register_tuple)
        request = _make_write_request(mac_id, register, value)

        self._wait_between_commands()
        ser.reset_input_buffer()
        if self.verbose:
            print("TX:", request.hex(" ").upper())
        written = ser.write(request)
        if written is not None and written != len(request):
            raise FCSTWriteError(
                f"Incomplete serial write: sent {written} of {len(request)} bytes."
            )
        ser.flush()
        self._last_transaction_at = time.monotonic()

        ack1 = ser.read(1)
        if ack1 == bytes((NAK,)):
            raise FCSTNAKError("MFC returned NAK (0x16) to WRITE request.")
        if ack1 != bytes((ACK,)):
            raise FCSTWriteError(
                "WRITE response did not begin with ACK (0x06); received "
                + (ack1.hex(" ").upper() if ack1 else "<timeout>")
                + ". The write result may be uncertain; do not retry blindly."
            )

        ack2 = ser.read(1)
        if ack2 != bytes((ACK,)):
            if ack2 == bytes((NAK,)):
                raise FCSTNAKError(
                    "MFC returned ACK followed by NAK (0x06 0x16)."
                )
            raise FCSTWriteError(
                "Second WRITE ACK was not received; got "
                + (ack2.hex(" ").upper() if ack2 else "<timeout>")
                + ". The write result may be uncertain; read the MFC state "
                "before attempting another write."
            )
        if self.verbose:
            print("RX: 06 06 (ACK, ACK)")

    @staticmethod
    def _decode_uint(payload: bytes) -> int:
        return int.from_bytes(payload, byteorder="little", signed=False)

    # ------------------------------------------------------------------
    # Read methods required by MFCManager.FCSTDriverProtocol
    # ------------------------------------------------------------------
    def read_serial_number(self, mac_id: int) -> str:
        payload = self._read_register(mac_id, SERIAL_NUMBER)
        value = payload.decode("ascii", errors="strict").strip("\x00 \r\n")
        if not value:
            raise FCSTProtocolError("Serial-number register returned an empty string.")
        return value

    def read_configured_mac_id(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, CONFIGURED_MAC_ID))

    def read_control_mode(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, CONTROL_MODE))

    def read_default_control_mode(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, DEFAULT_CONTROL_MODE))

    def read_freeze_follow(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, FREEZE_FOLLOW))

    def read_setpoint_raw(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, SETPOINT))

    def read_filtered_setpoint_raw(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, FILTERED_SETPOINT))

    def read_indicated_flow_raw(self, mac_id: int) -> int:
        return self._decode_uint(self._read_register(mac_id, INDICATED_FLOW))

    # ------------------------------------------------------------------
    # Write methods required by MFCManager.FCSTDriverProtocol
    # ------------------------------------------------------------------
    def write_control_mode(self, mac_id: int, mode: int) -> None:
        if mode not in (0x01, 0x02):
            raise ValueError("Control mode must be 0x01 (Digital) or 0x02 (Analog).")
        self._write_register(mac_id, CONTROL_MODE, mode)

    def write_freeze_follow(self, mac_id: int, value: int) -> None:
        if value not in (0x00, 0x01):
            raise ValueError("Freeze Follow must be 0x00 or 0x01.")
        self._write_register(mac_id, FREEZE_FOLLOW, value)

    def write_setpoint_raw(self, mac_id: int, raw_value: int) -> None:
        if not (0x4000 <= raw_value <= 0xC000):
            raise ValueError(
                f"Setpoint raw value must be in 0x4000-0xC000; "
                f"received 0x{raw_value:04X}."
            )
        self._write_register(mac_id, SETPOINT, raw_value)


__all__ = [
    "FCSTDriver",
    "FCSTDriverError",
    "FCSTPortError",
    "FCSTTimeoutError",
    "FCSTProtocolError",
    "FCSTNAKError",
    "FCSTWriteError",
    "detect_default_port",
]

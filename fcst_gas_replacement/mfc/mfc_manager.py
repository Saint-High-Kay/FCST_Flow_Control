"""High-level manager for multiple FCST MFCs.

This module deliberately does not implement serial framing itself.
It expects a low-level fcst_driver object with the methods declared by
FCSTDriverProtocol. The corresponding mfc/fcst_driver.py will implement
that protocol and remain responsible for packet framing, checksums, ACK/NAK,
timeouts, and the physical serial connection.

All public flow values in this module are in sccm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

from config.mfc_profiles import MFCProfile, MFC_PROFILES, get_profile


DIGITAL_MODE = 0x01
ANALOG_MODE = 0x02
FREEZE_FOLLOW_BUFFERED = 0x00
FREEZE_FOLLOW_APPLIED = 0x01

SETPOINT_ZERO_RAW = 0x4000
SETPOINT_FULL_SCALE_SPAN = 0x8000
SETPOINT_MAX_RAW = 0xC000


class MFCError(RuntimeError):
    """Base class for MFC management errors."""


class MFCIdentityError(MFCError):
    """The connected device did not match its configured profile."""


class MFCOutOfRangeError(MFCError, ValueError):
    """Requested flow is outside the configured MFC operating range."""


class MFCVerificationError(MFCError):
    """A write was acknowledged but read-back verification failed."""


class MFCStateError(MFCError):
    """The MFC state is not safe or valid for the requested operation."""


@runtime_checkable
class FCSTDriverProtocol(Protocol):
    """Required low-level interface to be implemented by fcst_driver.py.

    Each method takes a MAC ID as its first argument.
    Read methods return decoded Python values; setpoint/flow registers return
    unsigned 16-bit raw values. Write methods must raise an exception if the
    FCST response is NAK, malformed, timed out, or otherwise unsuccessful.
    """

    def read_serial_number(self, mac_id: int) -> str: ...
    def read_configured_mac_id(self, mac_id: int) -> int: ...
    def read_control_mode(self, mac_id: int) -> int: ...
    def read_default_control_mode(self, mac_id: int) -> int: ...
    def read_freeze_follow(self, mac_id: int) -> int: ...
    def read_setpoint_raw(self, mac_id: int) -> int: ...
    def read_filtered_setpoint_raw(self, mac_id: int) -> int: ...
    def read_indicated_flow_raw(self, mac_id: int) -> int: ...

    def write_control_mode(self, mac_id: int, mode: int) -> None: ...
    def write_freeze_follow(self, mac_id: int, value: int) -> None: ...
    def write_setpoint_raw(self, mac_id: int, raw_value: int) -> None: ...


@dataclass(frozen=True, slots=True)
class MFCStatus:
    profile_name: str
    serial_number: str
    mac_id: int
    current_control_mode: int
    default_control_mode: int
    freeze_follow: int
    setpoint_raw: int
    filtered_setpoint_raw: int
    indicated_flow_raw: int
    setpoint_sccm: float
    filtered_setpoint_sccm: float
    indicated_flow_sccm_equivalent: float


@dataclass(frozen=True, slots=True)
class SetpointPlan:
    profile_name: str
    serial_number: str
    mac_id: int
    requested_flow_sccm: float
    target_raw: int
    quantized_flow_sccm: float
    current_control_mode: int
    current_freeze_follow: int
    needs_digital_mode: bool
    needs_apply_freeze_follow: bool
    default_control_mode: int


def raw_to_sccm(raw_value: int, full_scale_sccm: float) -> float:
    """Convert FCST setpoint/indicated-flow raw value to sccm-equivalent."""
    if not (SETPOINT_ZERO_RAW <= raw_value <= SETPOINT_MAX_RAW):
        raise ValueError(
            f"Raw setpoint 0x{raw_value:04X} is outside "
            "0x4000-0xC000."
        )
    fraction = (raw_value - SETPOINT_ZERO_RAW) / SETPOINT_FULL_SCALE_SPAN
    return fraction * full_scale_sccm


def sccm_to_raw(flow_sccm: float, full_scale_sccm: float) -> int:
    """Encode sccm to FCST raw setpoint.

    Positive flows are rounded upward to avoid quantizing a requested
    minimum operating value below its lower limit. Zero is the dedicated
    stop command and always maps to 0x4000.
    """
    if not math.isfinite(flow_sccm):
        raise ValueError("Flow must be a finite number.")
    if flow_sccm < 0:
        raise ValueError("Negative flow is not allowed.")
    if full_scale_sccm <= 0:
        raise ValueError("Full scale must be positive.")
    if flow_sccm == 0:
        return SETPOINT_ZERO_RAW

    increment = math.ceil(
        flow_sccm / full_scale_sccm * SETPOINT_FULL_SCALE_SPAN
    )
    raw = SETPOINT_ZERO_RAW + increment
    if raw > SETPOINT_MAX_RAW:
        raise ValueError("Encoded setpoint exceeds full-scale raw value.")
    return raw


def _validate_raw(raw: int, label: str) -> None:
    if not isinstance(raw, int) or not (SETPOINT_ZERO_RAW <= raw <= SETPOINT_MAX_RAW):
        raise MFCVerificationError(
            f"{label} returned invalid raw value: {raw!r}."
        )


class MFCManager:
    """Manage configured FCST MFCs through a compatible low-level driver.

    Example:
        manager = MFCManager(driver)
        status = manager.read_status("Cir01")
        plan = manager.plan_flow("Exh01", 250.0)
        final_status = manager.set_flow_sccm(
            "Exh01", 250.0, apply=True, confirmed=True
        )

    `confirmed=True` is intended to be passed only after the top-level
    controller has displayed the target, gas path, and safety warning and
    received the operator's explicit confirmation.
    """

    def __init__(
        self,
        driver: FCSTDriverProtocol,
        profiles: dict[str, MFCProfile] | None = None,
    ) -> None:
        self.driver = driver
        self.profiles = profiles if profiles is not None else MFC_PROFILES

    def get_profile(self, name: str) -> MFCProfile:
        # Use local profile mapping, while still supporting aliases.
        from config.mfc_profiles import _normalise_name  # package-local helper

        key = _normalise_name(name)
        for canonical, profile in self.profiles.items():
            if _normalise_name(canonical) == key:
                return profile
        valid = ", ".join(self.profiles)
        raise KeyError(f"Unknown MFC {name!r}. Available: {valid}")

    def _verify_identity(self, profile: MFCProfile) -> tuple[str, int]:
        serial = self.driver.read_serial_number(profile.mac_id)
        configured_mac = self.driver.read_configured_mac_id(profile.mac_id)

        if serial != profile.serial_number:
            raise MFCIdentityError(
                f"{profile.name}: serial mismatch at MAC 0x{profile.mac_id:02X}; "
                f"expected {profile.serial_number}, received {serial!r}."
            )
        if configured_mac != profile.mac_id:
            raise MFCIdentityError(
                f"{profile.name}: MAC mismatch; expected "
                f"0x{profile.mac_id:02X}, received 0x{configured_mac:02X}."
            )
        return serial, configured_mac

    def read_status(self, name: str, verify_identity: bool = True) -> MFCStatus:
        """Read a profile's identity, control state, and flow registers."""
        profile = self.get_profile(name)
        if verify_identity:
            serial, mac_id = self._verify_identity(profile)
        else:
            serial, mac_id = (
                profile.serial_number,
                profile.mac_id,
            )

        mode = self.driver.read_control_mode(mac_id)
        default_mode = self.driver.read_default_control_mode(mac_id)
        freeze = self.driver.read_freeze_follow(mac_id)
        setpoint = self.driver.read_setpoint_raw(mac_id)
        filtered = self.driver.read_filtered_setpoint_raw(mac_id)
        indicated = self.driver.read_indicated_flow_raw(mac_id)

        _validate_raw(setpoint, "Setpoint")
        _validate_raw(filtered, "Filtered Setpoint")
        _validate_raw(indicated, "Indicated Flow")

        if mode not in (DIGITAL_MODE, ANALOG_MODE):
            raise MFCStateError(
                f"{profile.name}: unsupported control mode 0x{mode:02X}."
            )
        if freeze not in (
            FREEZE_FOLLOW_BUFFERED,
            FREEZE_FOLLOW_APPLIED,
        ):
            raise MFCStateError(
                f"{profile.name}: unsupported Freeze Follow value 0x{freeze:02X}."
            )

        return MFCStatus(
            profile_name=profile.name,
            serial_number=serial,
            mac_id=mac_id,
            current_control_mode=mode,
            default_control_mode=default_mode,
            freeze_follow=freeze,
            setpoint_raw=setpoint,
            filtered_setpoint_raw=filtered,
            indicated_flow_raw=indicated,
            setpoint_sccm=raw_to_sccm(
                setpoint, profile.full_scale_sccm
            ),
            filtered_setpoint_sccm=raw_to_sccm(
                filtered, profile.full_scale_sccm
            ),
            indicated_flow_sccm_equivalent=raw_to_sccm(
                indicated, profile.full_scale_sccm
            ),
        )

    def read_all_status(self) -> dict[str, MFCStatus]:
        """Read all active configured profiles in insertion order."""
        return {
            name: self.read_status(name)
            for name, profile in self.profiles.items()
            if profile.active
        }

    def validate_flow(self, name: str, flow_sccm: float) -> float:
        """Validate and return a normalized sccm flow value.

        Zero is a dedicated stop instruction. Positive values must be within
        the profile's configured 2-100% F.S. range.
        """
        profile = self.get_profile(name)
        try:
            flow = float(flow_sccm)
        except (TypeError, ValueError) as exc:
            raise ValueError("Flow must be numeric and expressed in sccm.") from exc

        if not math.isfinite(flow):
            raise ValueError("Flow must be finite.")
        if flow < 0:
            raise MFCOutOfRangeError("Negative flow is not allowed.")
        if flow == 0:
            return 0.0
        if not (profile.min_flow_sccm <= flow <= profile.max_flow_sccm):
            raise MFCOutOfRangeError(
                f"{profile.name}: requested {flow:g} sccm is outside "
                f"the positive operating range "
                f"{profile.min_flow_sccm:g}-{profile.max_flow_sccm:g} sccm. "
                "No write has been performed."
            )
        return flow

    def plan_flow(self, name: str, flow_sccm: float) -> SetpointPlan:
        """Read current state and prepare a no-write setpoint plan."""
        profile = self.get_profile(name)
        flow = self.validate_flow(name, flow_sccm)
        serial, mac_id = self._verify_identity(profile)

        mode = self.driver.read_control_mode(mac_id)
        default_mode = self.driver.read_default_control_mode(mac_id)
        freeze = self.driver.read_freeze_follow(mac_id)

        if mode not in (DIGITAL_MODE, ANALOG_MODE):
            raise MFCStateError(
                f"{profile.name}: unsupported control mode 0x{mode:02X}."
            )
        if freeze not in (FREEZE_FOLLOW_BUFFERED, FREEZE_FOLLOW_APPLIED):
            raise MFCStateError(
                f"{profile.name}: unsupported Freeze Follow 0x{freeze:02X}."
            )

        raw = sccm_to_raw(flow, profile.full_scale_sccm)
        quantized = raw_to_sccm(raw, profile.full_scale_sccm)

        return SetpointPlan(
            profile_name=profile.name,
            serial_number=serial,
            mac_id=mac_id,
            requested_flow_sccm=flow,
            target_raw=raw,
            quantized_flow_sccm=quantized,
            current_control_mode=mode,
            current_freeze_follow=freeze,
            needs_digital_mode=(mode != DIGITAL_MODE),
            needs_apply_freeze_follow=(freeze != FREEZE_FOLLOW_APPLIED),
            default_control_mode=default_mode,
        )

    def set_flow_sccm(
        self,
        name: str,
        flow_sccm: float,
        *,
        apply: bool = False,
        confirmed: bool = False,
    ) -> MFCStatus | SetpointPlan:
        """Set one MFC's flow, or return a dry-run plan.

        No MFC write is sent unless both apply=True and confirmed=True.
        The default control mode is never modified.

        If Freeze Follow is already 1, the setpoint is written directly.
        If Freeze Follow is 0, the setpoint is buffered first and then
        Freeze Follow is set to 1 to apply it.
        """
        plan = self.plan_flow(name, flow_sccm)
        if not apply:
            return plan
        if not confirmed:
            raise MFCStateError(
                "Write blocked: confirmed=True is required after an "
                "operator safety confirmation."
            )

        mac_id = plan.mac_id
        profile = self.get_profile(name)

        # Re-check identity immediately before any writes.
        serial, verified_mac = self._verify_identity(profile)
        if serial != plan.serial_number or verified_mac != plan.mac_id:
            raise MFCIdentityError(
                f"{profile.name}: identity changed after preflight; no write."
            )

        current_mode = self.driver.read_control_mode(mac_id)
        freeze = self.driver.read_freeze_follow(mac_id)
        default_before = self.driver.read_default_control_mode(mac_id)

        if default_before != plan.default_control_mode:
            raise MFCStateError(
                f"{profile.name}: Default Control Mode changed after preflight."
            )

        if current_mode != DIGITAL_MODE:
            self.driver.write_control_mode(mac_id, DIGITAL_MODE)
            mode_check = self.driver.read_control_mode(mac_id)
            if mode_check != DIGITAL_MODE:
                raise MFCVerificationError(
                    f"{profile.name}: Digital mode could not be verified."
                )

        # The mode switch must not silently alter Freeze Follow.
        freeze = self.driver.read_freeze_follow(mac_id)
        if freeze == FREEZE_FOLLOW_APPLIED:
            self.driver.write_setpoint_raw(mac_id, plan.target_raw)
        elif freeze == FREEZE_FOLLOW_BUFFERED:
            old_setpoint = self.driver.read_setpoint_raw(mac_id)
            old_filtered = self.driver.read_filtered_setpoint_raw(mac_id)

            self.driver.write_setpoint_raw(mac_id, plan.target_raw)

            # With Freeze Follow=0 the active Setpoint registers are expected
            # to retain their previous values until Freeze Follow is released.
            buffered_setpoint = self.driver.read_setpoint_raw(mac_id)
            buffered_filtered = self.driver.read_filtered_setpoint_raw(mac_id)

            if (
                buffered_setpoint != old_setpoint
                or buffered_filtered != old_filtered
            ):
                raise MFCVerificationError(
                    f"{profile.name}: active registers changed while "
                    "Freeze Follow=0. Freeze Follow was not released."
                )

            self.driver.write_freeze_follow(
                mac_id, FREEZE_FOLLOW_APPLIED
            )
        else:
            raise MFCStateError(
                f"{profile.name}: invalid Freeze Follow value 0x{freeze:02X}."
            )

        # Final verification.
        final_mode = self.driver.read_control_mode(mac_id)
        final_freeze = self.driver.read_freeze_follow(mac_id)
        final_setpoint = self.driver.read_setpoint_raw(mac_id)
        final_filtered = self.driver.read_filtered_setpoint_raw(mac_id)
        final_default = self.driver.read_default_control_mode(mac_id)

        if final_mode != DIGITAL_MODE:
            raise MFCVerificationError(
                f"{profile.name}: final control mode is not Digital."
            )
        if final_freeze != FREEZE_FOLLOW_APPLIED:
            raise MFCVerificationError(
                f"{profile.name}: Freeze Follow is not applied."
            )
        if final_setpoint != plan.target_raw:
            raise MFCVerificationError(
                f"{profile.name}: Setpoint mismatch; expected "
                f"0x{plan.target_raw:04X}, received 0x{final_setpoint:04X}."
            )
        if final_filtered != plan.target_raw:
            raise MFCVerificationError(
                f"{profile.name}: Filtered Setpoint mismatch; expected "
                f"0x{plan.target_raw:04X}, received 0x{final_filtered:04X}."
            )
        if final_default != plan.default_control_mode:
            raise MFCVerificationError(
                f"{profile.name}: Default Control Mode unexpectedly changed."
            )

        return self.read_status(name, verify_identity=True)

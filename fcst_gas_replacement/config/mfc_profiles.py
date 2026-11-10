"""MFC profile definitions for the COMET CDC gas-replacement system.

All flow values exposed to the upper-level application are in sccm.
The native_unit field documents the engineering unit convention used by
the respective FCST model; conversion to the FCST raw setpoint is performed
using full_scale_sccm, so the manager does not mix SLM and sccm numerically.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True, slots=True)
class MFCProfile:
    """Immutable identity and operating profile for one FCST MFC."""

    name: str
    role: str
    serial_number: str
    mac_id: int
    full_scale_sccm: float
    min_flow_sccm: float
    max_flow_sccm: float
    calibration_gas: str
    calibration_cf: float
    native_unit: str
    active: bool = True

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("MFC name must not be empty.")
        if not self.serial_number.strip():
            raise ValueError(f"{self.name}: serial number must not be empty.")
        if not (0x21 <= self.mac_id <= 0x9F):
            raise ValueError(
                f"{self.name}: MAC ID 0x{self.mac_id:02X} is outside "
                "the supported profile range 0x21-0x9F."
            )
        if self.full_scale_sccm <= 0:
            raise ValueError(f"{self.name}: full_scale_sccm must be positive.")
        if self.min_flow_sccm <= 0:
            raise ValueError(f"{self.name}: min_flow_sccm must be positive.")
        if self.max_flow_sccm < self.min_flow_sccm:
            raise ValueError(
                f"{self.name}: max_flow_sccm must be >= min_flow_sccm."
            )
        if self.max_flow_sccm > self.full_scale_sccm:
            raise ValueError(
                f"{self.name}: max_flow_sccm cannot exceed full scale."
            )
        if self.calibration_cf <= 0:
            raise ValueError(f"{self.name}: calibration_cf must be positive.")
        if self.native_unit not in {"SLM", "sccm"}:
            raise ValueError(
                f"{self.name}: native_unit must be 'SLM' or 'sccm'."
            )

    @property
    def positive_flow_range(self) -> tuple[float, float]:
        return self.min_flow_sccm, self.max_flow_sccm


# Active MFCs. All application-facing flow values are expressed in sccm.
MFC_PROFILES: dict[str, MFCProfile] = {
    "Cir01": MFCProfile(
        name="Cir01",
        role="He supply",
        serial_number="AT18ZQW-00",
        mac_id=0x21,
        full_scale_sccm=5000.0,   # 5 SLM
        min_flow_sccm=100.0,      # 2% F.S.
        max_flow_sccm=5000.0,
        calibration_gas="He:i-C4H10=90:10",
        calibration_cf=0.946,
        native_unit="SLM",
    ),
    "Cir02": MFCProfile(
        name="Cir02",
        role="isobutane supply",
        serial_number="AT1MSM9-00",
        mac_id=0x22,
        full_scale_sccm=400.0,
        min_flow_sccm=8.0,        # 2% F.S.
        max_flow_sccm=400.0,
        calibration_gas="He:i-C4H10=90:10",
        calibration_cf=0.946,
        native_unit="sccm",
    ),
    "Exh01": MFCProfile(
        name="Exh01",
        role="CDC exhaust",
        serial_number="AT1MSM6-00",
        mac_id=0x24,
        full_scale_sccm=1000.0,   # 1 SLM
        min_flow_sccm=20.0,       # 2% F.S.; provisional until verified on device
        max_flow_sccm=1000.0,
        calibration_gas="He:i-C4H10=90:10",
        calibration_cf=0.946,
        native_unit="SLM",
    ),
}

# MAC ID 0x23 is intentionally reserved for a future Cir03.
RESERVED_MAC_IDS: Mapping[int, str] = {
    0x23: "Cir03 (future circulation MFC; not configured)",
}


def _normalise_name(name: str) -> str:
    """Accept labels such as 'Cir 01', 'Cir01', and 'cir01'."""
    return "".join(ch for ch in name.lower() if ch.isalnum())


_PROFILE_LOOKUP: dict[str, str] = {}
for _canonical in MFC_PROFILES:
    _PROFILE_LOOKUP[_normalise_name(_canonical)] = _canonical
    _PROFILE_LOOKUP[_normalise_name(_canonical[:3] + " " + _canonical[3:])] = _canonical


def get_profile(name: str) -> MFCProfile:
    """Return a configured profile by canonical name or user-friendly alias."""
    key = _normalise_name(name)
    canonical = _PROFILE_LOOKUP.get(key)
    if canonical is None:
        valid = ", ".join(MFC_PROFILES)
        raise KeyError(f"Unknown MFC profile {name!r}. Available: {valid}")
    return MFC_PROFILES[canonical]


def validate_profile_table(
    profiles: Mapping[str, MFCProfile] = MFC_PROFILES,
) -> None:
    """Check unique MAC IDs and serial numbers in the active profile table."""
    mac_ids: dict[int, str] = {}
    serials: dict[str, str] = {}

    for key, profile in profiles.items():
        if key != profile.name:
            raise ValueError(
                f"Profile key {key!r} does not match name {profile.name!r}."
            )
        if not profile.active:
            continue
        if profile.mac_id in mac_ids:
            raise ValueError(
                f"Duplicate MAC ID 0x{profile.mac_id:02X}: "
                f"{mac_ids[profile.mac_id]} and {profile.name}."
            )
        if profile.serial_number in serials:
            raise ValueError(
                f"Duplicate serial number {profile.serial_number}: "
                f"{serials[profile.serial_number]} and {profile.name}."
            )
        if profile.mac_id in RESERVED_MAC_IDS:
            raise ValueError(
                f"{profile.name} uses reserved MAC ID 0x{profile.mac_id:02X}."
            )
        mac_ids[profile.mac_id] = profile.name
        serials[profile.serial_number] = profile.name


validate_profile_table()

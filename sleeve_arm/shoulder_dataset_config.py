from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from sleeve_arm.config import SensorConfig, load_sensor_config


MOTIONS = ("forward", "lateral", "backward")


@dataclass(frozen=True, slots=True)
class FlexSelectionConfig:
    channels: tuple[int, int, int]
    interpolation: str


@dataclass(frozen=True, slots=True)
class ImuRoleConfig:
    torso_imu: str
    upper_arm_imu: str


@dataclass(frozen=True, slots=True)
class DatasetCollectionConfig:
    output_hz: float
    status_hz: float
    flush_interval_s: float
    poll_interval_ms: float


@dataclass(frozen=True, slots=True)
class DatasetSyncConfig:
    max_skew_ms: float
    buffer_duration_ms: float
    alignment_delay_ms: float


@dataclass(frozen=True, slots=True)
class NeutralCalibrationConfig:
    neutral_duration_s: float
    countdown_s: int
    startup_timeout_s: float


@dataclass(frozen=True, slots=True)
class BodyFrameConfig:
    forward_axis: tuple[float, float, float]
    lateral_axis: tuple[float, float, float]
    up_axis: tuple[float, float, float]
    upper_arm_axis_neutral: tuple[float, float, float]
    side: str


@dataclass(frozen=True, slots=True)
class DirectionLabelConfig:
    min_amplitude_deg: float
    max_direction_error_deg: float


@dataclass(frozen=True, slots=True)
class ShoulderDatasetConfig:
    path: Path
    sensor_config_path: Path
    sensors: SensorConfig
    flex: FlexSelectionConfig
    imu_roles: ImuRoleConfig
    collection: DatasetCollectionConfig
    sync: DatasetSyncConfig
    calibration: NeutralCalibrationConfig
    body_frame: BodyFrameConfig
    label: DirectionLabelConfig


def _mapping(raw: dict[str, Any], name: str) -> dict[str, Any]:
    value = raw.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"shoulder dataset config requires a {name} mapping")
    return value


def _positive_finite(name: str, value: object) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return result


def _unit_vector(name: str, value: object) -> tuple[float, float, float]:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"body_frame.{name} must contain three finite values")
    norm = float(np.linalg.norm(array))
    if norm <= 1e-12:
        raise ValueError(f"body_frame.{name} must be nonzero")
    return tuple(float(item) for item in array / norm)


def load_shoulder_dataset_config(path: str | Path) -> ShoulderDatasetConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"invalid shoulder dataset config: {config_path}")

    sensor_value = raw.get("sensor_config")
    if not sensor_value:
        raise ValueError("sensor_config is required")
    sensor_path = Path(str(sensor_value)).expanduser()
    if not sensor_path.is_absolute():
        sensor_path = config_path.parent / sensor_path
    sensor_path = sensor_path.resolve()
    sensors = load_sensor_config(sensor_path)

    flex_raw = _mapping(raw, "flex")
    channels_raw = flex_raw.get("channels")
    if not isinstance(channels_raw, list) or len(channels_raw) != 3:
        raise ValueError("flex.channels must select exactly three zero-based channels")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not math.isfinite(float(item))
        or float(item) != int(item)
        for item in channels_raw
    ):
        raise ValueError("flex.channels must contain integer indices")
    channels = tuple(int(item) for item in channels_raw)
    if len(set(channels)) != 3:
        raise ValueError("flex.channels must contain three distinct channels")
    expected_fields = sensors.sleeve.expected_fields
    if expected_fields is None:
        raise ValueError("sleeve expected_fields is required for flex channel validation")
    if any(channel < 0 or channel >= expected_fields for channel in channels):
        raise ValueError(
            f"flex channel index must be in 0..{expected_fields - 1} for this sleeve"
        )
    interpolation = str(flex_raw.get("interpolation", "linear")).lower()
    if interpolation not in ("linear", "nearest"):
        raise ValueError("flex.interpolation must be linear or nearest")
    flex = FlexSelectionConfig(channels, interpolation)  # type: ignore[arg-type]

    roles_raw = _mapping(raw, "imu_roles")
    roles = ImuRoleConfig(
        torso_imu=str(roles_raw.get("torso_imu", "")),
        upper_arm_imu=str(roles_raw.get("upper_arm_imu", "")),
    )
    if roles.torso_imu not in ("imu1", "imu2") or roles.upper_arm_imu not in ("imu1", "imu2"):
        raise ValueError("imu_roles values must be imu1 or imu2")
    if roles.torso_imu == roles.upper_arm_imu:
        raise ValueError("torso_imu and upper_arm_imu must be different sources")
    endpoints = {"imu1": sensors.imu1, "imu2": sensors.imu2}
    if not sensors.sleeve.enabled:
        raise ValueError("shoulder collection requires the sleeve source to be enabled")
    for role in (roles.torso_imu, roles.upper_arm_imu):
        if not endpoints[role].enabled:
            raise ValueError(f"shoulder collection requires {role} to be enabled")

    collection_raw = _mapping(raw, "collection")
    collection = DatasetCollectionConfig(
        output_hz=_positive_finite("collection.output_hz", collection_raw.get("output_hz", 60)),
        status_hz=_positive_finite("collection.status_hz", collection_raw.get("status_hz", 1)),
        flush_interval_s=_positive_finite(
            "collection.flush_interval_s", collection_raw.get("flush_interval_s", 1)
        ),
        poll_interval_ms=_positive_finite(
            "collection.poll_interval_ms", collection_raw.get("poll_interval_ms", 2)
        ),
    )

    sync_raw = _mapping(raw, "sync")
    sync = DatasetSyncConfig(
        max_skew_ms=_positive_finite("sync.max_skew_ms", sync_raw.get("max_skew_ms", 20)),
        buffer_duration_ms=_positive_finite(
            "sync.buffer_duration_ms", sync_raw.get("buffer_duration_ms", 1000)
        ),
        alignment_delay_ms=_positive_finite(
            "sync.alignment_delay_ms", sync_raw.get("alignment_delay_ms", 25)
        ),
    )
    if sync.alignment_delay_ms < sync.max_skew_ms:
        raise ValueError("sync.alignment_delay_ms must be at least max_skew_ms")
    if sync.buffer_duration_ms <= sync.alignment_delay_ms + sync.max_skew_ms:
        raise ValueError("sync.buffer_duration_ms is too short for delay plus skew")

    calibration_raw = _mapping(raw, "calibration")
    countdown_s = int(calibration_raw.get("countdown_s", 3))
    if countdown_s < 0:
        raise ValueError("calibration.countdown_s must be non-negative")
    calibration = NeutralCalibrationConfig(
        neutral_duration_s=_positive_finite(
            "calibration.neutral_duration_s",
            calibration_raw.get("neutral_duration_s", 3),
        ),
        countdown_s=countdown_s,
        startup_timeout_s=_positive_finite(
            "calibration.startup_timeout_s",
            calibration_raw.get("startup_timeout_s", 10),
        ),
    )

    body_raw = _mapping(raw, "body_frame")
    body = BodyFrameConfig(
        forward_axis=_unit_vector("forward_axis", body_raw.get("forward_axis")),
        lateral_axis=_unit_vector("lateral_axis", body_raw.get("lateral_axis")),
        up_axis=_unit_vector("up_axis", body_raw.get("up_axis")),
        upper_arm_axis_neutral=_unit_vector(
            "upper_arm_axis_neutral", body_raw.get("upper_arm_axis_neutral")
        ),
        side=str(body_raw.get("side", "right")).lower(),
    )
    forward = np.asarray(body.forward_axis)
    lateral = np.asarray(body.lateral_axis)
    up = np.asarray(body.up_axis)
    neutral = np.asarray(body.upper_arm_axis_neutral)
    if max(abs(float(np.dot(forward, lateral))), abs(float(np.dot(forward, up))), abs(float(np.dot(lateral, up)))) > 1e-6:
        raise ValueError("body_frame forward/lateral/up axes must be orthogonal")
    if float(np.dot(np.cross(forward, lateral), up)) < 1.0 - 1e-6:
        raise ValueError("body_frame axes must form a right-handed coordinate frame")
    if max(abs(float(np.dot(neutral, forward))), abs(float(np.dot(neutral, lateral)))) > 1e-6:
        raise ValueError("upper_arm_axis_neutral must be perpendicular to forward and lateral")
    if body.side != "right":
        raise ValueError("this dataset is fixed to the right shoulder; body_frame.side must be right")

    label_raw = _mapping(raw, "label")
    minimum = float(label_raw.get("min_amplitude_deg", 5))
    maximum_error = float(label_raw.get("max_direction_error_deg", 30))
    if not math.isfinite(minimum) or minimum < 0.0:
        raise ValueError("label.min_amplitude_deg must be finite and non-negative")
    if not math.isfinite(maximum_error) or not 0.0 < maximum_error < 90.0:
        raise ValueError("label.max_direction_error_deg must be in (0, 90)")

    return ShoulderDatasetConfig(
        path=config_path,
        sensor_config_path=sensor_path,
        sensors=sensors,
        flex=flex,
        imu_roles=roles,
        collection=collection,
        sync=sync,
        calibration=calibration,
        body_frame=body,
        label=DirectionLabelConfig(minimum, maximum_error),
    )

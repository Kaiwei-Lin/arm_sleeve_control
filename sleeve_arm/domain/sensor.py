from __future__ import annotations

import math
from dataclasses import dataclass


def _finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


@dataclass(frozen=True, slots=True)
class SleeveFrame:
    timestamp: float
    channels: tuple[float, ...]
    sequence_id: int | None = None
    host_timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp", _finite("timestamp", self.timestamp))
        channels = tuple(_finite("channel", value) for value in self.channels)
        if not channels:
            raise ValueError("SleeveFrame requires at least one channel")
        object.__setattr__(self, "channels", channels)
        if self.sequence_id is not None:
            object.__setattr__(self, "sequence_id", int(self.sequence_id))
        host_timestamp_ns = (
            round(self.timestamp * 1_000_000_000)
            if self.host_timestamp_ns is None
            else int(self.host_timestamp_ns)
        )
        if host_timestamp_ns < 0:
            raise ValueError("host_timestamp_ns must be non-negative")
        object.__setattr__(self, "host_timestamp_ns", host_timestamp_ns)


@dataclass(frozen=True, slots=True)
class ImuFrame:
    timestamp: float
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float
    quat_w: float | None = None
    quat_x: float | None = None
    quat_y: float | None = None
    quat_z: float | None = None
    sequence_id: int | None = None
    device_timestamp_us: int | None = None
    host_timestamp_ns: int | None = None

    def __post_init__(self) -> None:
        for name in ("timestamp", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z"):
            object.__setattr__(self, name, _finite(name, getattr(self, name)))
        quaternion = (self.quat_w, self.quat_x, self.quat_y, self.quat_z)
        if any(value is not None for value in quaternion):
            if not all(value is not None for value in quaternion):
                raise ValueError("quaternion fields must be all present or all absent")
            for name in ("quat_w", "quat_x", "quat_y", "quat_z"):
                object.__setattr__(self, name, _finite(name, getattr(self, name)))
        if self.sequence_id is not None:
            object.__setattr__(self, "sequence_id", int(self.sequence_id))
        if self.device_timestamp_us is not None:
            object.__setattr__(self, "device_timestamp_us", int(self.device_timestamp_us))
        host_timestamp_ns = (
            round(self.timestamp * 1_000_000_000)
            if self.host_timestamp_ns is None
            else int(self.host_timestamp_ns)
        )
        if host_timestamp_ns < 0:
            raise ValueError("host_timestamp_ns must be non-negative")
        object.__setattr__(self, "host_timestamp_ns", host_timestamp_ns)


@dataclass(frozen=True, slots=True)
class SensorSample:
    timestamp: float
    sleeve: SleeveFrame
    imu1: ImuFrame | None = None
    imu2: ImuFrame | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "timestamp", _finite("timestamp", self.timestamp))

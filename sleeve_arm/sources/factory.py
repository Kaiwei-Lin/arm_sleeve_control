from __future__ import annotations

from sleeve_arm.config import SensorConfig, SensorEndpointConfig
from sleeve_arm.sources.base import ImuSource, SleeveSource
from sleeve_arm.sources.imu import Imu770SerialSource
from sleeve_arm.sources.sleeve import SerialSleeveSource


def create_sleeve_source(config: SensorConfig) -> SleeveSource:
    endpoint = config.sleeve
    _require_serial_settings("sleeve", endpoint)
    if endpoint.backend != "serial_ascii":
        raise ValueError(f"unsupported sleeve backend: {endpoint.backend}")
    return SerialSleeveSource(
        endpoint.port or "", endpoint.baudrate or 0, endpoint.timeout_s,
        endpoint.delimiter or ";", endpoint.expected_fields or 11,
    )


def create_imu_source(name: str, endpoint: SensorEndpointConfig) -> ImuSource | None:
    if not endpoint.enabled:
        return None
    _require_serial_settings(name, endpoint)
    if endpoint.backend != "imu770_serial":
        raise ValueError(f"unsupported {name} backend: {endpoint.backend}")
    return Imu770SerialSource(endpoint.port or "", endpoint.baudrate or 0, endpoint.timeout_s)


def _require_serial_settings(name: str, endpoint: SensorEndpointConfig) -> None:
    if not endpoint.port or not endpoint.baudrate:
        raise ValueError(f"{name} serial port and baudrate must be configured")

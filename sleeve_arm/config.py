from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from sleeve_arm.domain.joint import JOINT_NAMES


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "robot.yaml"


@dataclass(frozen=True)
class NetworkConfig:
    device_id: int
    local_ip: str
    local_port: int
    remote_ip: str
    remote_port: int
    fast_mode: int
    status: str


@dataclass(frozen=True)
class SafetyConfig:
    stable_feedback_samples: int
    stable_feedback_interval_s: float
    monitor_interval_s: float


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    can_id: int
    direction: int
    direction_status: str
    zero_position: float | None
    min_position: float | None
    max_position: float | None
    max_velocity: float | None
    max_current: float | None
    max_tracking_error: float | None
    max_position_step: float | None


@dataclass(frozen=True)
class RobotConfig:
    network: NetworkConfig
    safety: SafetyConfig
    joints: dict[str, JointConfig]


def _optional_float(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    return None if value is None else float(value)


def _positive_optional(name: str, value: float | None) -> None:
    if value is not None and value <= 0:
        raise ValueError(f"{name} must be positive or null")


def load_robot_config(path: str | Path = DEFAULT_CONFIG_PATH) -> RobotConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"invalid robot config: {config_path}")

    network_raw = raw.get("network")
    safety_raw = raw.get("safety")
    joints_raw = raw.get("joints")
    if not all(isinstance(item, dict) for item in (network_raw, safety_raw, joints_raw)):
        raise ValueError("robot config requires network, safety, and joints mappings")

    network = NetworkConfig(
        device_id=int(network_raw["device_id"]),
        local_ip=str(network_raw["local_ip"]),
        local_port=int(network_raw["local_port"]),
        remote_ip=str(network_raw["remote_ip"]),
        remote_port=int(network_raw["remote_port"]),
        fast_mode=int(network_raw["fast_mode"]),
        status=str(network_raw["status"]),
    )
    if not 0 <= network.device_id <= 0xFFFF:
        raise ValueError("network.device_id must fit uint16")
    for label, port in (("local_port", network.local_port), ("remote_port", network.remote_port)):
        if not 1 <= port <= 65535:
            raise ValueError(f"network.{label} must be in 1..65535")
    if network.fast_mode <= 0:
        raise ValueError("network.fast_mode must be positive")

    safety = SafetyConfig(
        stable_feedback_samples=int(safety_raw["stable_feedback_samples"]),
        stable_feedback_interval_s=float(safety_raw["stable_feedback_interval_s"]),
        monitor_interval_s=float(safety_raw["monitor_interval_s"]),
    )
    if safety.stable_feedback_samples < 2:
        raise ValueError("safety.stable_feedback_samples must be at least 2")
    if safety.stable_feedback_interval_s < 0 or safety.monitor_interval_s <= 0:
        raise ValueError("feedback intervals must be non-negative/positive")

    if set(joints_raw) != set(JOINT_NAMES):
        raise ValueError(f"joints must be exactly: {', '.join(JOINT_NAMES)}")

    joints: dict[str, JointConfig] = {}
    identities: set[tuple[int, int]] = set()
    for name in JOINT_NAMES:
        item = joints_raw[name]
        if not isinstance(item, dict):
            raise ValueError(f"joints.{name} must be a mapping")
        joint = JointConfig(
            name=name,
            motor_id=int(item["motor_id"]),
            can_id=int(item["can_id"]),
            direction=int(item["direction"]),
            direction_status=str(item["direction_status"]),
            zero_position=_optional_float(item, "zero_position"),
            min_position=_optional_float(item, "min_position"),
            max_position=_optional_float(item, "max_position"),
            max_velocity=_optional_float(item, "max_velocity"),
            max_current=_optional_float(item, "max_current"),
            max_tracking_error=_optional_float(item, "max_tracking_error"),
            max_position_step=_optional_float(item, "max_position_step"),
        )
        if not 0 <= joint.motor_id <= 0xFFFF or not 0 <= joint.can_id <= 0xFFFF:
            raise ValueError(f"{name}: motor_id and can_id must fit uint16")
        if joint.direction not in (-1, 1):
            raise ValueError(f"{name}: direction must be -1 or 1")
        if joint.direction_status not in ("NEEDS_HARDWARE_VALIDATION", "VALIDATED"):
            raise ValueError(f"{name}: invalid direction_status")
        if (
            joint.min_position is not None
            and joint.max_position is not None
            and joint.min_position > joint.max_position
        ):
            raise ValueError(f"{name}: min_position exceeds max_position")
        _positive_optional(f"{name}.max_velocity", joint.max_velocity)
        _positive_optional(f"{name}.max_current", joint.max_current)
        _positive_optional(f"{name}.max_tracking_error", joint.max_tracking_error)
        _positive_optional(f"{name}.max_position_step", joint.max_position_step)
        identity = (joint.motor_id, joint.can_id)
        if identity in identities:
            raise ValueError(f"duplicate motor/CAN mapping: {identity}")
        identities.add(identity)
        joints[name] = joint

    return RobotConfig(network=network, safety=safety, joints=joints)


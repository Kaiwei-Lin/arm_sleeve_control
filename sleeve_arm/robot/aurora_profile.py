"""Explicit deployment contract. No robot geometry is inferred from SDK examples."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml

from sleeve_arm.config import SafetyConfig
from sleeve_arm.control.parameters import ControlConfig, JointSafetyParameters

API_FAMILY = "aurora-configure-lease-v1"
SDK_VERSION = "1.0.1"
# These names are part of SDK 3d4e02e's groupResource(), not inferred robot layout.
GROUP_RESOURCES = {
    "left_manipulator": "LEFT_MANIPULATOR",
    "right_manipulator": "RIGHT_MANIPULATOR",
    "left_end_effector": "LEFT_END_EFFECTOR",
    "right_end_effector": "RIGHT_END_EFFECTOR",
}


def finite(value, label: str, *, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f"{label} must be finite" + (" and positive" if positive else ""))
    return value


@dataclass(frozen=True)
class AuroraJoint:
    name: str
    index: int
    sign: int | None
    zero: float | None
    limits: JointSafetyParameters
    verified: bool = False
    open_position: float | None = None
    closed_position: float | None = None
    kind: str = "arm"

    def to_sdk(self, value: float) -> float:
        if self.sign not in (-1, 1) or self.zero is None:
            raise ValueError(f"{self.name}: direction/zero not calibrated")
        return self.zero + self.sign * value

    def from_sdk(self, value: float) -> float:
        if self.sign not in (-1, 1) or self.zero is None:
            raise ValueError(f"{self.name}: direction/zero not calibrated")
        return self.sign * (value - self.zero)


@dataclass(frozen=True)
class AuroraGroup:
    name: str
    side: str
    part: str
    resource: str
    count: int | None
    motor_mode: str | None
    joints: tuple[AuroraJoint, ...]
    capabilities: tuple[str, ...]
    verified: bool = False


@dataclass(frozen=True)
class AuroraRobotProfile:
    api_family: str
    sdk_version: str
    robot_type: str | None
    hardware_type: str | None
    end_effector_type: str | None
    connection: dict
    groups: tuple[AuroraGroup, ...]
    allowed_fsm: tuple[int, ...]
    feedback_timeout_s: float
    endpoint_timeout_s: float
    control_period_s: float
    arrival_tolerance_rad: float
    arrival_timeout_s: float
    stop_policy: str | None
    verified: bool
    simulated: bool = False
    verification_note: str = ""

    def validate(self, *, execute: bool = False, simulation: bool = False) -> None:
        if type(self.verified) is not bool or type(self.simulated) is not bool:
            raise ValueError("verified/simulated must be booleans")
        if self.api_family != API_FAMILY or self.sdk_version != SDK_VERSION:
            raise ValueError(f"unsupported api_family/version: {self.api_family}/{self.sdk_version}; "
                             f"supported {API_FAMILY}/{SDK_VERSION}; no legacy fallback")
        allowed_options = {"domain_id", "namespace_name", "topic_prefix", "dds_mode",
                           "is_ros_compatible", "use_namespace", "use_discovery_server",
                           "service_timeout_ms", "source_name"}
        if set(self.connection) - allowed_options:
            raise ValueError("unsupported DDS connection fields")
        for required in ("domain_id", "source_name", "topic_prefix", "dds_mode", "use_namespace"):
            if required not in self.connection:
                raise ValueError(f"connection.{required} is required")
        if not re.fullmatch(r"[A-Za-z0-9._-]{1,64}", self.connection["source_name"]):
            raise ValueError("invalid connection.source_name")
        for field, maximum in (("domain_id", 2**32 - 1), ("dds_mode", 3)):
            val = self.connection[field]
            if type(val) is not int or not 0 <= val <= maximum:
                raise ValueError(f"invalid connection.{field}")
        for field in ("use_namespace", "is_ros_compatible", "use_discovery_server"):
            if field in self.connection and type(self.connection[field]) is not bool:
                raise ValueError(f"connection.{field} must be boolean")
        if not isinstance(self.connection["topic_prefix"], str):
            raise ValueError("connection.topic_prefix must be a string")
        if "namespace_name" in self.connection and not isinstance(self.connection["namespace_name"], str):
            raise ValueError("connection.namespace_name must be a string")
        if "service_timeout_ms" in self.connection:
            finite(self.connection["service_timeout_ms"], "service_timeout_ms", positive=True)
        for field in ("feedback_timeout_s", "endpoint_timeout_s", "control_period_s",
                      "arrival_tolerance_rad", "arrival_timeout_s"):
            finite(getattr(self, field), field, positive=True)
        if self.control_period_s > self.feedback_timeout_s:
            raise ValueError("control period exceeds feedback timeout")
        if any(type(v) is not int or v < 0 for v in self.allowed_fsm):
            raise ValueError("allowed_fsm must contain nonnegative integer IDs")
        names, slots = set(), set()
        for group in self.groups:
            if type(group.verified) is not bool:
                raise ValueError("group verified must be boolean")
            if group.name in names or (group.side, group.part) in slots:
                raise ValueError("duplicate group name or side/part")
            names.add(group.name)
            slots.add((group.side, group.part))
            if group.side not in ("left", "right") or group.part not in ("arm", "hand"):
                raise ValueError("invalid group side/part")
            expected = f"{group.side}_" + ("manipulator" if group.part == "arm" else "end_effector")
            if group.name != expected or GROUP_RESOURCES.get(group.name) != group.resource:
                raise ValueError(f"{group.name}: SDK group/resource/side mismatch")
            if group.count is not None and (type(group.count) is not int or group.count <= 0):
                raise ValueError(f"{group.name}: invalid count")
            if group.motor_mode not in (None, "PD", "POSITION"):
                raise ValueError("only calibrated PD/POSITION joint commands are supported")
            if set(group.capabilities) - {"joint_position", "wrist_orientation", "hand_joints", "hand_closure"}:
                raise ValueError("unsupported capability (Cartesian/IK is not implemented)")
            indices, joint_names = set(), set()
            for joint in group.joints:
                if type(joint.verified) is not bool or joint.kind not in ("arm", "wrist", "finger"):
                    raise ValueError("invalid joint verified/kind")
                if (group.part == "hand") != (joint.kind == "finger"):
                    raise ValueError("joint kind does not match group part")
                if not re.fullmatch(r"[a-z][a-z0-9_]*", joint.name):
                    raise ValueError("invalid semantic joint name")
                if joint.name in joint_names or joint.index in indices:
                    raise ValueError("duplicate semantic joint or group index")
                indices.add(joint.index)
                joint_names.add(joint.name)
                if type(joint.index) is not int or joint.index < 0 or (group.count is not None and joint.index >= group.count):
                    raise ValueError(f"{joint.name}: index out of range")
                if joint.sign is not None and (type(joint.sign) is not int or joint.sign not in (-1, 1)):
                    raise ValueError(f"{joint.name}: sign must be -1 or 1")
                for val in (joint.zero, joint.open_position, joint.closed_position):
                    if val is not None:
                        finite(val, joint.name)
                limits = joint.limits
                for field in ("min_position", "max_position", "max_velocity", "max_current",
                              "max_tracking_error", "max_position_step"):
                    val = getattr(limits, field)
                    if val is not None:
                        finite(val, f"{joint.name}.{field}", positive=field not in ("min_position", "max_position"))
                if limits.min_position is not None and limits.max_position is not None:
                    if limits.min_position >= limits.max_position:
                        raise ValueError(f"{joint.name}: invalid limits")
                    for val in (joint.open_position, joint.closed_position):
                        if val is not None and not limits.min_position <= val <= limits.max_position:
                            raise ValueError(f"{joint.name}: closure calibration outside limits")
            if execute:
                if not group.verified or not group.count or indices != set(range(group.count)):
                    raise ValueError(f"{group.name}: complete verified mapping for every slot is required")
                if group.motor_mode is None or "joint_position" not in group.capabilities:
                    raise ValueError(f"{group.name}: missing position command capability/mode")
                if group.part == "hand" and (not self.end_effector_type or "hand_joints" not in group.capabilities):
                    raise ValueError("hand requires verified end_effector_type and hand_joints capability")
                for joint in group.joints:
                    limits = joint.limits
                    if (not joint.verified or joint.sign is None or joint.zero is None or
                        any(getattr(limits, f) is None for f in ("min_position", "max_position", "max_velocity",
                                                               "max_tracking_error", "max_position_step"))):
                        raise ValueError(f"{joint.name}: missing verified calibration/safety limits")
                    if limits.max_current is not None or limits.require_motor_error:
                        raise ValueError("current/per-motor error safety requires unavailable feedback")
                    if "hand_closure" in group.capabilities and (joint.open_position is None or joint.closed_position is None):
                        raise ValueError("hand closure requires every joint's open/closed calibration")
        if execute:
            if not self.verified or not self.verification_note or not self.robot_type or not self.groups or not self.allowed_fsm:
                raise ValueError("unverified profile: robot identity, FSM, groups and verification note required")
            if self.simulated and not simulation:
                raise ValueError("simulated profile is forbidden for real execute")
            if self.stop_policy != "stop_publishing_release":
                raise ValueError("verified stop_policy=stop_publishing_release is required")

    def selected_groups(self, sides: tuple[str, ...], parts: tuple[str, ...]) -> tuple[AuroraGroup, ...]:
        if not sides or len(set(sides)) != len(sides) or any(s not in ("left", "right") for s in sides):
            raise ValueError("explicit unique left/right side(s) required")
        if not parts or len(set(parts)) != len(parts) or any(p not in ("arm", "hand") for p in parts):
            raise ValueError("explicit arm/hand part(s) required")
        selected = tuple(g for g in self.groups if g.side in sides and g.part in parts)
        if len(selected) != len(sides) * len(parts):
            raise ValueError("profile lacks requested side/part capability")
        return selected

    def control_config(self, sides: tuple[str, ...], parts: tuple[str, ...]) -> ControlConfig:
        from dataclasses import replace
        joints = {}
        for group in self.selected_groups(sides, parts):
            for joint in group.joints:
                name = f"{group.side}.{joint.name}" if len(sides) > 1 else joint.name
                if name in joints:
                    raise ValueError("ambiguous semantic name across selected groups")
                joints[name] = replace(joint.limits, name=name)
        return ControlConfig(SafetyConfig(2, self.control_period_s, self.control_period_s), joints)


def load_aurora_profile(path: str | Path) -> AuroraRobotProfile:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("profile must be a YAML mapping")
    groups = []
    for g in raw.pop("groups", []):
        joints = []
        for j in g.pop("joints", []):
            limits = JointSafetyParameters(name=j["name"], **j.pop("limits", {}))
            joints.append(AuroraJoint(limits=limits, **j))
        g["capabilities"] = tuple(g.get("capabilities", ()))
        groups.append(AuroraGroup(joints=tuple(joints), **g))
    raw["allowed_fsm"] = tuple(raw.get("allowed_fsm", ()))
    profile = AuroraRobotProfile(groups=tuple(groups), **raw)
    if type(profile.verified) is not bool or type(profile.simulated) is not bool:
        raise ValueError("verified/simulated must be YAML booleans")
    if any(type(g.verified) is not bool or any(type(j.verified) is not bool for j in g.joints) for g in groups):
        raise ValueError("group/joint verified must be YAML booleans")
    profile.validate()
    return profile

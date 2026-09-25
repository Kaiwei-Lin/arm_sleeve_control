"""Deployment facts for Aurora 0.1.8; unknown values remain unknown."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, replace
from pathlib import Path

import yaml
from sleeve_arm.config import SafetyConfig
from sleeve_arm.control.parameters import ControlConfig, JointSafetyParameters

API_FAMILY = 'aurora-python-dds-0.1.8'
SDK_VERSION = '0.1.8'


def finite(value, label, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{label} must be a finite number')
    value = float(value)
    if not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(f'{label} must be finite' + (' and positive' if positive else ''))
    return value


@dataclass(frozen=True)
class AuroraJoint:
    name: str
    index: int | None
    sign: int | None
    zero: float | None
    limits: JointSafetyParameters
    verified: bool = False
    kind: str = 'arm'

    def to_sdk(self, value):
        self.require_transform()
        return self.zero + self.sign * finite(value, self.name)

    def from_sdk(self, value):
        self.require_transform()
        return self.sign * (finite(value, self.name) - self.zero)

    def require_transform(self):
        if self.index is None or self.sign not in (-1, 1) or self.zero is None:
            raise ValueError(f'{self.name}: index/direction/zero not calibrated')


@dataclass(frozen=True)
class AuroraGroup:
    name: str | None
    side: str
    count: int | None
    joints: tuple[AuroraJoint, ...]
    # SDK-coordinate bounds for EVERY slot, including slots without semantic names.
    sdk_position_limits: tuple[tuple[float, float], ...] | None = None
    max_tracking_error: float | None = None
    part: str = 'arm'
    capabilities: tuple[str, ...] = ('joint_position',)
    verified: bool = False

    def check_vector(self, vector):
        if not self.count or len(vector) != self.count:
            raise ValueError(f'{self.name}: vector dimension mismatch/unverified DOF')
        for i, value in enumerate(vector):
            finite(value, f'{self.name}[{i}]')
            if self.sdk_position_limits is not None:
                low, high = self.sdk_position_limits[i]
                if not low <= value <= high:
                    raise ValueError(f'{self.name}[{i}]: SDK position outside limits')
        for joint in self.joints:
            if joint.index is not None and joint.sign is not None and joint.zero is not None:
                value = joint.from_sdk(vector[joint.index])
                low, high = joint.limits.min_position, joint.limits.max_position
                if (low is not None and value < low) or (high is not None and value > high):
                    raise ValueError(f'{joint.name}: semantic position outside limits')


@dataclass(frozen=True)
class AuroraRobotProfile:
    api_family: str
    sdk_version: str
    connection: dict
    groups: tuple[AuroraGroup, ...]
    allowed_fsm: tuple[int, ...] = ()
    control_hz: float | None = None
    feedback_timeout_s: float = 0.25
    endpoint_timeout_s: float = 5.0
    arrival_tolerance_rad: float | None = None
    arrival_timeout_s: float = 3.0
    stop_policy: str | None = None
    verified: bool = False
    simulated: bool = False
    verification_note: str = ''
    robot_type: str | None = None
    authority_verified: bool = False

    @property
    def control_period_s(self):
        return 1 / finite(self.control_hz, 'control_hz', positive=True)

    def selected_groups(self, sides, parts=('arm',)):
        if not sides or len(set(sides)) != len(sides) or set(sides) - {'left', 'right'}:
            raise ValueError('explicit unique left/right side(s) required')
        if not parts or len(set(parts)) != len(parts) or set(parts) - {'arm', 'hand'}:
            raise ValueError('invalid part')
        selected = tuple(g for g in self.groups if g.side in sides and g.part in parts)
        if len(selected) != len(sides) * len(parts):
            raise ValueError('unsupported side/part capability')
        return selected

    def validate(self, *, execute=False, simulation=False, groups=None):
        if self.api_family != API_FAMILY or self.sdk_version != SDK_VERSION:
            raise ValueError(f'unsupported api_family/version {self.api_family}/{self.sdk_version}; only 0.1.8, no fallback')
        for name in ('verified', 'simulated', 'authority_verified'):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f'{name} must be boolean')
        if set(self.connection) - {'domain_id', 'robot_name', 'namespace', 'is_ros_compatible'}:
            raise ValueError('unsupported 0.1.8 connection option')
        domain = self.connection.get('domain_id')
        if domain is not None and (type(domain) is not int or not 0 <= domain < 2**32):
            raise ValueError('domain_id must be uint32 or null')
        for name in ('robot_name', 'namespace'):
            if self.connection.get(name) is not None and not isinstance(self.connection[name], str):
                raise ValueError(f'{name} must be string or null')
        ros = self.connection.get('is_ros_compatible')
        if ros is not None and type(ros) is not bool:
            raise ValueError('is_ros_compatible must be boolean or null')
        for name in ('feedback_timeout_s', 'endpoint_timeout_s', 'arrival_timeout_s'):
            if finite(getattr(self, name), name, positive=True) > 60:
                raise ValueError(f'{name} must be <= 60 seconds')
        if self.control_hz is not None:
            if not 1 <= finite(self.control_hz, 'control_hz') <= 500:
                raise ValueError('control_hz must be in [1, 500]')
        if self.arrival_tolerance_rad is not None:
            finite(self.arrival_tolerance_rad, 'arrival tolerance', positive=True)
        if any(type(v) is not int or v < 0 for v in self.allowed_fsm):
            raise ValueError('allowed_fsm must be nonnegative integer IDs')
        names, slots = set(), set()
        for group in self.groups:
            if group.side not in ('left', 'right') or group.part not in ('arm', 'hand'):
                raise ValueError('invalid side/part')
            if (group.side, group.part) in slots or (group.name is not None and group.name in names):
                raise ValueError('duplicate group/side/part')
            slots.add((group.side, group.part)); names.add(group.name)
            if group.name is not None and (not isinstance(group.name, str) or not group.name.strip()):
                raise ValueError('group name must be nonempty or null')
            if type(group.verified) is not bool:
                raise ValueError('group verified must be boolean')
            if group.count is not None and (type(group.count) is not int or group.count <= 0):
                raise ValueError('DOF must be positive integer or null')
            if set(group.capabilities) - {'joint_position', 'wrist_orientation', 'hand_joints'}:
                raise ValueError('unsupported capability')
            indices, joint_names = set(), set()
            for joint in group.joints:
                if not re.fullmatch(r'[a-z][a-z0-9_]*', joint.name) or joint.name in joint_names:
                    raise ValueError('invalid/duplicate joint name')
                joint_names.add(joint.name)
                if type(joint.verified) is not bool:
                    raise ValueError('joint verified must be boolean')
                if joint.kind not in ('arm', 'wrist', 'finger') or ((joint.kind == 'finger') != (group.part == 'hand')):
                    raise ValueError('physical joint kind does not match group part')
                if joint.index is not None:
                    if (type(joint.index) is not int or joint.index < 0 or joint.index in indices
                            or (group.count is not None and joint.index >= group.count)):
                        raise ValueError('duplicate or invalid joint index')
                    indices.add(joint.index)
                if joint.sign is not None and (type(joint.sign) is not int or joint.sign not in (-1, 1)):
                    raise ValueError('direction/sign must be -1 or 1')
                if joint.zero is not None:
                    finite(joint.zero, 'zero')
                for field in ('min_position', 'max_position', 'max_velocity', 'max_position_step', 'max_tracking_error', 'max_current'):
                    value = getattr(joint.limits, field)
                    if value is not None:
                        finite(value, field, positive=field not in ('min_position', 'max_position'))
                lo, hi = joint.limits.min_position, joint.limits.max_position
                if lo is not None and hi is not None and lo >= hi:
                    raise ValueError('invalid joint limits')
            if group.max_tracking_error is not None:
                finite(group.max_tracking_error, 'group tracking error', positive=True)
            bounds = group.sdk_position_limits
            if bounds is not None:
                if group.count is None or len(bounds) != group.count:
                    raise ValueError('SDK bounds must cover the complete group')
                for pair in bounds:
                    if len(pair) != 2 or finite(pair[0], 'lower bound') >= finite(pair[1], 'upper bound'):
                        raise ValueError('invalid SDK bounds')
        if not execute:
            return
        if (not self.verified or not self.verification_note.strip() or not self.robot_type
                or not self.authority_verified or domain is None or not self.allowed_fsm):
            raise ValueError('unverified profile: identity, domain, allowed FSM and exclusive authority confirmation required')
        if self.simulated and not simulation:
            raise ValueError('simulated profile forbidden for real execute')
        if self.stop_policy != 'stop_publishing':
            raise ValueError('verified stop_policy=stop_publishing required; no lease or physical stop claim')
        if self.control_hz is None or self.arrival_tolerance_rad is None:
            raise ValueError('unverified control_hz/arrival tolerance')
        if self.control_period_s >= self.feedback_timeout_s:
            raise ValueError('control period must be shorter than feedback timeout')
        selected = self.groups if groups is None else groups
        if not selected:
            raise ValueError('no selected groups')
        for group in selected:
            if (not group.verified or group.name is None or group.count is None
                    or group.sdk_position_limits is None or group.max_tracking_error is None or not group.joints
                    or 'joint_position' not in group.capabilities):
                raise ValueError(f'{group.side}: unverified group/name/DOF/full-vector bounds/capability')
            if group.part == 'hand' and 'hand_joints' not in group.capabilities:
                raise ValueError('unsupported hand capability')
            for joint in group.joints:
                joint.require_transform()
                if not joint.verified or any(getattr(joint.limits, n) is None for n in (
                        'min_position', 'max_position', 'max_velocity', 'max_position_step', 'max_tracking_error')):
                    raise ValueError(f'{joint.name}: unverified joint calibration/safety limits')
                if joint.limits.max_current is not None or joint.limits.require_motor_error:
                    raise ValueError('current/per-motor error feedback unavailable')
                for value in (joint.limits.min_position, joint.limits.max_position):
                    lo, hi = group.sdk_position_limits[joint.index]
                    if not lo <= joint.to_sdk(value) <= hi:
                        raise ValueError('semantic limits exceed SDK slot bounds')

    def control_config(self, sides, parts=('arm',)):
        joints = {}
        for group in self.selected_groups(sides, parts):
            for joint in group.joints:
                name = f'{group.side}.{joint.name}' if len(sides) > 1 else joint.name
                if name in joints:
                    raise ValueError('ambiguous joint names')
                joints[name] = replace(joint.limits, name=name)
        # This default is only for constructing incomplete read-only profiles.
        period = self.control_period_s if self.control_hz is not None else 0.02
        return ControlConfig(SafetyConfig(2, period, period), joints)


def load_aurora_profile(path: str | Path):
    raw = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(raw, dict):
        raise ValueError('profile must be a YAML mapping')
    groups = []
    for g in raw.pop('groups', []):
        joints = []
        for j in g.pop('joints', []):
            raw_limits = j.pop('limits', {})
            if raw_limits.pop('name', j['name']) != j['name']:
                raise ValueError('joint limit name mismatch')
            limits = JointSafetyParameters(name=j['name'], **raw_limits)
            joints.append(AuroraJoint(limits=limits, **j))
        g['capabilities'] = tuple(g.get('capabilities', ('joint_position',)))
        if g.get('sdk_position_limits') is not None:
            g['sdk_position_limits'] = tuple(tuple(pair) for pair in g['sdk_position_limits'])
        groups.append(AuroraGroup(joints=tuple(joints), **g))
    raw['allowed_fsm'] = tuple(raw.get('allowed_fsm', ()))
    result = AuroraRobotProfile(groups=tuple(groups), **raw)
    result.validate()
    return result

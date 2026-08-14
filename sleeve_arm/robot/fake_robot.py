from __future__ import annotations

import math
from collections.abc import Mapping

from sleeve_arm.config import RobotConfig
from sleeve_arm.domain.joint import JOINT_NAMES, JointState
from sleeve_arm.robot.base import RobotArm, RobotError


class FakeRobotArm(RobotArm):
    """In-memory semantic-joint backend for tests and hardware-free flows."""

    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.connected = False
        self.enabled = False
        self.closed = False
        self.disable_count = 0
        self.events: list[str] = []
        self._positions = dict.fromkeys(JOINT_NAMES, 0.0)
        self._errors = dict.fromkeys(JOINT_NAMES, 0)

    def connect(self) -> None:
        if self.connected:
            raise RobotError("fake robot is already connected")
        self.connected = True
        self.closed = False
        self.events.append("connect")

    def prepare_feedback(self) -> None:
        self._require_connected()

    def enable(self) -> None:
        self._require_connected()
        if any(self._errors.values()):
            raise RobotError("cannot enable fake robot with a joint error")
        self.enabled = True
        self.events.append("enable")

    def disable(self) -> None:
        if self.connected:
            self.disable_count += 1
            self.events.append("disable")
        self.enabled = False

    def read_joint_state(self, joint_name: str) -> JointState:
        self._require_connected()
        self._require_joint(joint_name)
        return JointState(
            name=joint_name,
            position=self._positions[joint_name],
            velocity=0.0,
            current=0.0,
            torque=0.0,
            state=2 if self.enabled else 0,
            bus=0,
            error=self._errors[joint_name],
        )

    def set_joint_position(self, joint_name: str, position: float) -> None:
        self.set_joint_positions({joint_name: position})

    def set_joint_positions(self, targets: Mapping[str, float]) -> None:
        self._require_connected()
        if not self.enabled:
            raise RobotError("fake robot is not enabled")
        if not targets:
            raise RobotError("at least one joint target is required")
        checked: dict[str, float] = {}
        for name, position in targets.items():
            self._require_joint(name)
            value = float(position)
            if not math.isfinite(value):
                raise RobotError(f"non-finite target for {name}")
            checked[name] = value
        self._positions.update(checked)

    def close(self) -> None:
        self.events.append("close")
        self.enabled = False
        self.connected = False
        self.closed = True

    def set_joint_error(self, joint_name: str, error: int) -> None:
        self._require_joint(joint_name)
        self._errors[joint_name] = int(error)

    def _require_connected(self) -> None:
        if not self.connected:
            raise RobotError("fake robot is not connected")

    @staticmethod
    def _require_joint(joint_name: str) -> None:
        if joint_name not in JOINT_NAMES:
            raise RobotError(f"unknown joint: {joint_name}")

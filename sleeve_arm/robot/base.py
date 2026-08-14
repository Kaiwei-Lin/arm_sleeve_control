from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Mapping

from sleeve_arm.domain.joint import JointState


class RobotError(RuntimeError):
    """A robot backend operation failed."""


class RobotArm(ABC):
    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def prepare_feedback(self) -> None: ...

    @abstractmethod
    def enable(self) -> None: ...

    @abstractmethod
    def disable(self) -> None: ...

    @abstractmethod
    def read_joint_state(self, joint_name: str) -> JointState: ...

    @abstractmethod
    def set_joint_position(self, joint_name: str, position: float) -> None: ...

    @abstractmethod
    def set_joint_positions(self, targets: Mapping[str, float]) -> None: ...

    @abstractmethod
    def close(self) -> None: ...

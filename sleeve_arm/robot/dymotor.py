from __future__ import annotations

import ctypes
import math
import os
from dataclasses import dataclass
from collections.abc import Mapping
from pathlib import Path

from sleeve_arm.config import PROJECT_ROOT, JointConfig, RobotConfig
from sleeve_arm.domain.joint import JOINT_NAMES, JointState
from sleeve_arm.robot.base import RobotArm, RobotError


_JOINT_INDEX = {name: index for index, name in enumerate(JOINT_NAMES)}


def semantic_to_sdk_position(joint: JointConfig, semantic_position: float) -> float:
    """Apply the exact semantic-to-vendor position transform without I/O."""
    zero = 0.0 if joint.zero_position is None else joint.zero_position
    return zero + joint.direction * float(semantic_position)


class _ArmOpenConfig(ctypes.Structure):
    _fields_ = [
        ("device_id", ctypes.c_uint16),
        ("local_ip", ctypes.c_char_p),
        ("local_port", ctypes.c_int),
        ("remote_ip", ctypes.c_char_p),
        ("remote_port", ctypes.c_int),
        ("fast_mode", ctypes.c_int),
        ("motor_ids", ctypes.c_uint16 * 3),
        ("can_ids", ctypes.c_uint16 * 3),
    ]


@dataclass(frozen=True)
class BridgeJointState:
    wrapper_status: int
    position: float
    velocity: float
    current: float | None
    torque: float
    state: int
    bus: int
    motor_error_code: int


def _configure_library(lib: ctypes.CDLL) -> None:
    lib.arm_open.argtypes = [ctypes.POINTER(_ArmOpenConfig)]
    lib.arm_open.restype = ctypes.c_int
    lib.arm_enable.argtypes = []
    lib.arm_enable.restype = ctypes.c_int
    lib.arm_disable.argtypes = []
    lib.arm_disable.restype = ctypes.c_int
    float_pointer = ctypes.POINTER(ctypes.c_float)
    uint32_pointer = ctypes.POINTER(ctypes.c_uint32)
    lib.arm_get_joint_state.argtypes = [
        ctypes.c_int,
        float_pointer,
        float_pointer,
        float_pointer,
        float_pointer,
        uint32_pointer,
        uint32_pointer,
        uint32_pointer,
    ]
    lib.arm_get_joint_state.restype = ctypes.c_int
    lib.arm_set_diagnostics.argtypes = [ctypes.c_int]
    lib.arm_set_diagnostics.restype = None
    lib.arm_set_joint_position.argtypes = [ctypes.c_int, ctypes.c_float]
    lib.arm_set_joint_position.restype = ctypes.c_int
    lib.arm_set_joint_positions.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_uint32]
    lib.arm_set_joint_positions.restype = ctypes.c_int
    lib.arm_set_three_joint_positions.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float]
    lib.arm_set_three_joint_positions.restype = ctypes.c_int
    lib.arm_close.argtypes = []
    lib.arm_close.restype = None
    lib.arm_last_error.argtypes = []
    lib.arm_last_error.restype = ctypes.c_char_p


class DyMotorArm(RobotArm):
    """ctypes backend exposing semantic joints, not vendor SDK objects."""

    def __init__(
        self,
        config: RobotConfig,
        library_path: str | Path | None = None,
        diagnostics: bool = False,
    ) -> None:
        self.config = config
        self._library_path = Path(library_path).expanduser() if library_path else None
        self._lib: ctypes.CDLL | None = None
        self._loaded_library_path: Path | None = None
        self._diagnostics = diagnostics
        self.connected = False
        self.enabled = False

    def connect(self) -> None:
        if self.connected:
            raise RobotError("DyMotor arm is already connected")
        self._lib = self._load_library()
        network = self.config.network
        config = _ArmOpenConfig(
            device_id=network.device_id,
            local_ip=network.local_ip.encode("ascii"),
            local_port=network.local_port,
            remote_ip=network.remote_ip.encode("ascii"),
            remote_port=network.remote_port,
            fast_mode=network.fast_mode,
            motor_ids=(ctypes.c_uint16 * 3)(
                *(self.config.joints[name].motor_id for name in JOINT_NAMES)
            ),
            can_ids=(ctypes.c_uint16 * 3)(
                *(self.config.joints[name].can_id for name in JOINT_NAMES)
            ),
        )
        try:
            self._check(self._lib.arm_open(ctypes.byref(config)), "arm_open")
            self.connected = True
            if self._diagnostics:
                self.set_bridge_diagnostics(True)
        except BaseException:
            self._lib.arm_close()
            self.connected = False
            raise

    def enable(self) -> None:
        lib = self._require_connected()
        self._check(lib.arm_enable(), "arm_enable")
        self.enabled = True

    def disable(self) -> None:
        if self._lib is None or not self.connected or not self.enabled:
            self.enabled = False
            return
        try:
            self._check(self._lib.arm_disable(), "arm_disable")
        finally:
            self.enabled = False

    def read_joint_state(self, joint_name: str) -> JointState:
        raw = self.read_bridge_joint_state(joint_name)
        self._check(raw.wrapper_status, "arm_get_joint_state")
        joint = self.config.joints[joint_name]
        zero = 0.0 if joint.zero_position is None else joint.zero_position
        return JointState(
            name=joint_name,
            position=joint.direction * (raw.position - zero),
            velocity=joint.direction * raw.velocity,
            current=raw.current,
            torque=joint.direction * raw.torque,
            state=raw.state,
            bus=raw.bus,
            error=raw.motor_error_code,
        )

    def read_bridge_joint_state(self, joint_name: str) -> BridgeJointState:
        lib = self._require_connected()
        index = self._joint_index(joint_name)
        position = ctypes.c_float()
        velocity = ctypes.c_float()
        current = ctypes.c_float()
        torque = ctypes.c_float()
        state = ctypes.c_uint32()
        bus = ctypes.c_uint32()
        motor_error = ctypes.c_uint32()
        wrapper_status = int(lib.arm_get_joint_state(
            index,
            ctypes.byref(position),
            ctypes.byref(velocity),
            ctypes.byref(current),
            ctypes.byref(torque),
            ctypes.byref(state),
            ctypes.byref(bus),
            ctypes.byref(motor_error),
        ))
        current_value = float(current.value)
        return BridgeJointState(
            wrapper_status=wrapper_status,
            position=float(position.value),
            velocity=float(velocity.value),
            current=current_value if math.isfinite(current_value) else None,
            torque=float(torque.value),
            state=int(state.value),
            bus=int(bus.value),
            motor_error_code=int(motor_error.value),
        )

    def set_bridge_diagnostics(self, enabled: bool) -> None:
        lib = self._require_connected()
        lib.arm_set_diagnostics(int(enabled))

    @property
    def loaded_library_path(self) -> Path | None:
        return self._loaded_library_path

    def set_joint_position(self, joint_name: str, position: float) -> None:
        self.set_joint_positions({joint_name: position})

    def set_joint_positions(self, targets: Mapping[str, float]) -> None:
        lib = self._require_connected()
        if not self.enabled:
            raise RobotError("DyMotor arm is not enabled")
        if not targets:
            raise RobotError("at least one joint target is required")

        positions = (ctypes.c_float * 3)(0.0, 0.0, 0.0)
        mask = 0
        for name, semantic_position in targets.items():
            index = self._joint_index(name)
            joint = self.config.joints[name]
            positions[index] = semantic_to_sdk_position(joint, semantic_position)
            mask |= 1 << index
        self._check(lib.arm_set_joint_positions(positions, mask), "arm_set_joint_positions")

    def close(self) -> None:
        if self._lib is not None:
            self._lib.arm_close()
        self.connected = False
        self.enabled = False

    def _load_library(self) -> ctypes.CDLL:
        path = self._resolve_library_path()
        try:
            lib = ctypes.CDLL(str(path))
        except OSError as exc:
            raise RobotError(f"failed to load DyMotor bridge {path}: {exc}") from exc

        _configure_library(lib)
        self._loaded_library_path = path
        return lib

    def _resolve_library_path(self) -> Path:
        if self._library_path is not None:
            path = self._library_path.resolve()
            if not path.is_file():
                raise RobotError(f"DyMotor bridge does not exist: {path}")
            return path

        environment_path = os.environ.get("DYMOTOR_BRIDGE_LIBRARY")
        candidates = []
        if environment_path:
            candidates.append(Path(environment_path).expanduser())
        candidates.extend(
            (
                PROJECT_ROOT / "build" / "native" / "dymotor_bridge" / "libdymotor_bridge.so",
                PROJECT_ROOT / "build" / "libdymotor_bridge.so",
            )
        )
        for candidate in candidates:
            if candidate.is_file():
                return candidate.resolve()
        attempted = ", ".join(str(path) for path in candidates)
        raise RobotError(f"DyMotor bridge was not found; tried: {attempted}")

    def _require_connected(self) -> ctypes.CDLL:
        if not self.connected or self._lib is None:
            raise RobotError("DyMotor arm is not connected")
        return self._lib

    @staticmethod
    def _joint_index(joint_name: str) -> int:
        try:
            return _JOINT_INDEX[joint_name]
        except KeyError as exc:
            raise RobotError(f"unknown joint: {joint_name}") from exc

    def _check(self, result: int, operation: str) -> None:
        if result == 0:
            return
        detail = "unknown bridge error"
        if self._lib is not None:
            raw = self._lib.arm_last_error()
            if raw:
                detail = raw.decode("utf-8", errors="replace")
        raise RobotError(f"{operation} failed ({result}): {detail}")

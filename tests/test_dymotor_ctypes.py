from __future__ import annotations

import ctypes

import pytest

from sleeve_arm.config import load_robot_config
from sleeve_arm.control.safety import FeedbackError, validate_feedback
from sleeve_arm.robot.dymotor import DyMotorArm, _ArmOpenConfig, _configure_library


def test_ctypes_bridge_abi_layout_on_64_bit_linux() -> None:
    assert ctypes.sizeof(ctypes.c_void_p) == 8
    assert ctypes.sizeof(_ArmOpenConfig) == 56
    assert _ArmOpenConfig.local_ip.offset == 8
    assert _ArmOpenConfig.remote_ip.offset == 24
    assert _ArmOpenConfig.motor_ids.offset == 40
    assert _ArmOpenConfig.can_ids.offset == 46


def test_explicit_pvct_pointer_types() -> None:
    class Function:
        argtypes = None
        restype = None

        def __call__(self, *args):
            return 0

    class Library:
        arm_open = Function()
        arm_enable = Function()
        arm_disable = Function()
        arm_get_joint_state = Function()
        arm_set_diagnostics = Function()
        arm_set_joint_position = Function()
        arm_set_joint_positions = Function()
        arm_set_three_joint_positions = Function()
        arm_close = Function()
        arm_last_error = Function()

    library = Library()
    _configure_library(library)  # type: ignore[arg-type]
    float_pointer = ctypes.POINTER(ctypes.c_float)
    uint32_pointer = ctypes.POINTER(ctypes.c_uint32)
    assert library.arm_get_joint_state.argtypes == [
        ctypes.c_int, float_pointer, float_pointer, float_pointer, float_pointer,
        uint32_pointer, uint32_pointer, uint32_pointer,
    ]
    assert library.arm_get_joint_state.restype is ctypes.c_int


def test_wrapper_status_and_motor_error_are_separate() -> None:
    class Library:
        @staticmethod
        def arm_get_joint_state(joint, position, velocity, current, torque, state, bus, error):
            ctypes.cast(position, ctypes.POINTER(ctypes.c_float))[0] = 1.25
            ctypes.cast(velocity, ctypes.POINTER(ctypes.c_float))[0] = 0.5
            ctypes.cast(current, ctypes.POINTER(ctypes.c_float))[0] = float("nan")
            ctypes.cast(torque, ctypes.POINTER(ctypes.c_float))[0] = 0.75
            ctypes.cast(state, ctypes.POINTER(ctypes.c_uint32))[0] = 2
            ctypes.cast(bus, ctypes.POINTER(ctypes.c_uint32))[0] = 480
            ctypes.cast(error, ctypes.POINTER(ctypes.c_uint32))[0] = 7
            return -4

    robot = DyMotorArm(load_robot_config())
    robot._lib = Library()  # type: ignore[assignment]
    robot.connected = True
    raw = robot.read_bridge_joint_state("shoulder_flexion")
    assert raw.wrapper_status == -4
    assert raw.motor_error_code == 7
    assert raw.current is None
    assert raw.bus == 480


def test_nonzero_motor_error_is_not_ignored() -> None:
    class Library:
        @staticmethod
        def arm_get_joint_state(joint, position, velocity, current, torque, state, bus, error):
            ctypes.cast(position, ctypes.POINTER(ctypes.c_float))[0] = 1.25
            ctypes.cast(velocity, ctypes.POINTER(ctypes.c_float))[0] = 0.0
            ctypes.cast(current, ctypes.POINTER(ctypes.c_float))[0] = float("nan")
            ctypes.cast(torque, ctypes.POINTER(ctypes.c_float))[0] = 0.0
            ctypes.cast(state, ctypes.POINTER(ctypes.c_uint32))[0] = 0
            ctypes.cast(bus, ctypes.POINTER(ctypes.c_uint32))[0] = 0
            ctypes.cast(error, ctypes.POINTER(ctypes.c_uint32))[0] = 21894
            return 0

    config = load_robot_config()
    robot = DyMotorArm(config)
    robot._lib = Library()  # type: ignore[assignment]
    robot.connected = True
    state = robot.read_joint_state("shoulder_flexion")
    assert state.error == 21894
    with pytest.raises(FeedbackError, match="motor error code 21894"):
        validate_feedback(state, config.joints["shoulder_flexion"])


def test_disable_does_not_call_bridge_before_enable() -> None:
    class Library:
        disable_calls = 0

        def arm_disable(self):
            self.disable_calls += 1
            return 0

    library = Library()
    robot = DyMotorArm(load_robot_config())
    robot._lib = library  # type: ignore[assignment]
    robot.connected = True
    robot.disable()
    assert library.disable_calls == 0

from __future__ import annotations

import ctypes

from sleeve_arm.robot.dymotor import _ArmJointState, _ArmOpenConfig


def test_ctypes_bridge_abi_layout_on_64_bit_linux() -> None:
    assert ctypes.sizeof(ctypes.c_void_p) == 8
    assert ctypes.sizeof(_ArmOpenConfig) == 56
    assert _ArmOpenConfig.local_ip.offset == 8
    assert _ArmOpenConfig.remote_ip.offset == 24
    assert _ArmOpenConfig.motor_ids.offset == 40
    assert _ArmOpenConfig.can_ids.offset == 46
    assert ctypes.sizeof(_ArmJointState) == 20
    assert _ArmJointState.error.offset == 16

from __future__ import annotations

import builtins
import inspect
import sys
from types import SimpleNamespace

from tools import test_imu_motion


class FakeDualImuArmEstimator:
    instances = []

    def __init__(self, **kwargs):
        self.constructor_kwargs = kwargs
        self.forward_calibrated = False
        self.__class__.instances.append(self)

    def calibrate(self, chest_rest_samples, arm_rest_samples):
        assert len(chest_rest_samples) == len(arm_rest_samples) >= 2
        return SimpleNamespace(sample_count=len(chest_rest_samples))

    def calibrate_forward(self, chest_forward_samples, arm_forward_samples):
        assert len(chest_forward_samples) == len(arm_forward_samples) >= 2
        self.forward_calibrated = True

    def update(self, chest_q, arm_q):
        assert tuple(chest_q) == (1.0, 0.0, 0.0, 0.0)
        assert tuple(arm_q) == (1.0, 0.0, 0.0, 0.0)
        return SimpleNamespace(direction="Rest", magnitude_deg=0.0, confidence=1.0)


def test_four_imu_tool_runs_without_sleeve_or_robot(monkeypatch, capsys) -> None:
    FakeDualImuArmEstimator.instances.clear()
    monkeypatch.setitem(
        sys.modules,
        "flexarm",
        SimpleNamespace(DualImuArmEstimator=FakeDualImuArmEstimator),
    )
    monkeypatch.setattr(builtins, "input", lambda _prompt: "")
    monkeypatch.setattr(sys, "argv", [
        "test_imu_motion.py",
        "--fake",
        "--calibration-seconds", "0.05",
        "--duration", "0.03",
        "--print-hz", "100",
    ])

    assert test_imu_motion.main() == 0

    output = capsys.readouterr().out
    assert "shoulder_direction=Rest" in output
    assert "shoulder_magnitude_deg=0.00" in output
    assert "rotation_signed_deg=+0.00" in output
    assert "双 IMU 肩部前抬方向标定完成" in output
    estimator = FakeDualImuArmEstimator.instances[-1]
    assert estimator.constructor_kwargs == {
        "down_axis": (-1, 0, 0),
        "forward_axis": (0, 0, 1),
        "lateral_axis": (0, 1, 0),
        "rest_threshold_deg": 5,
        "dominance_ratio": 1.1,
    }
    assert estimator.forward_calibrated
    source = inspect.getsource(test_imu_motion)
    assert "sleeve_arm.robot" not in source
    assert "FakeRobot" not in source
    assert "DyMotorArm" not in source

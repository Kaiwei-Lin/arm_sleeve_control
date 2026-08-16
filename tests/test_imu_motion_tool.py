from __future__ import annotations

import builtins
import inspect
import sys
from types import SimpleNamespace

from tools import test_imu_motion


class FakeDualImuArmEstimator:
    def calibrate(self, chest_rest_samples, arm_rest_samples):
        assert len(chest_rest_samples) == len(arm_rest_samples) >= 2
        return SimpleNamespace(sample_count=len(chest_rest_samples))

    def update(self, chest_q, arm_q):
        assert tuple(chest_q) == (1.0, 0.0, 0.0, 0.0)
        assert tuple(arm_q) == (1.0, 0.0, 0.0, 0.0)
        return SimpleNamespace(direction="Rest", magnitude_deg=0.0, confidence=1.0)


def test_four_imu_tool_runs_without_sleeve_or_robot(monkeypatch, capsys) -> None:
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
    source = inspect.getsource(test_imu_motion)
    assert "sleeve_arm.robot" not in source
    assert "FakeRobot" not in source
    assert "DyMotorArm" not in source

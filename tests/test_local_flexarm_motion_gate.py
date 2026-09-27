from __future__ import annotations

import numpy as np
import pytest

from sleeve_arm.estimation.flexarm.calibration import FlexCalibration
from sleeve_arm.estimation.flexarm.motion_gate import MotionGate


@pytest.fixture
def gate() -> MotionGate:
    calibration = FlexCalibration.fit(
        [[100, 200, 300], [101, 200, 299], [99, 201, 301], [100, 199, 300]],
        scales=[100, 100, 100],
    )
    return MotionGate.from_calibration(calibration)


def test_motion_gate_reports_rest_for_calibration_noise(gate: MotionGate) -> None:
    assert gate.classify(np.zeros((5, 3)), np.zeros(3)) is False


def test_motion_gate_detects_large_displacement(gate: MotionGate) -> None:
    window = np.zeros((5, 3))
    window[-1] = [0.8, 0.0, 0.0]
    assert gate.classify(window, np.zeros(3)) is True


def test_motion_gate_detects_window_range(gate: MotionGate) -> None:
    window = np.array([[0, 0, 0], [0.5, 0, 0], [0, 0, 0]], dtype=float)
    assert gate.classify(window, np.zeros(3)) is True


def test_motion_gate_detects_slope(gate: MotionGate) -> None:
    assert gate.classify(np.zeros((5, 3)), np.array([1.0, 0.0, 0.0])) is True


def test_motion_gate_rejects_nonfinite_values(gate: MotionGate) -> None:
    with pytest.raises(ValueError, match="finite"):
        gate.classify(np.array([[float("nan"), 0, 0]]), np.zeros(3))

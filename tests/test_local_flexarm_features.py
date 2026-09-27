from __future__ import annotations

import numpy as np
import pytest

from sleeve_arm.estimation.flexarm.features import CausalFeatureBuffer, FEATURE_NAMES, select_features
from sleeve_arm.estimation.flexarm.types import MotionPhase


def test_feature_buffer_extracts_exact_eight_features() -> None:
    buffer = CausalFeatureBuffer(3, phase_slope_deadband=0.01)
    for timestamp_s, row in enumerate(([0, 0, 0], [1, 2, 3], [2, 4, 6])):
        buffer.append(row, timestamp_s * 1_000_000_000)
    vector = buffer.extract()
    assert FEATURE_NAMES == (
        "f1_median", "f2_median", "f3_median",
        "f1_slope", "f2_slope", "f3_slope",
        "f2_minus_f3", "f1_plus_f3",
    )
    np.testing.assert_allclose(vector.values, [1, 2, 3, 1, 2, 3, -1, 4])
    assert vector.phase is MotionPhase.RAISING


def test_extracted_features_do_not_change_after_future_append() -> None:
    buffer = CausalFeatureBuffer(3)
    for timestamp_s, row in enumerate(([0, 0, 0], [1, 2, 3], [2, 4, 6])):
        buffer.append(row, timestamp_s * 1_000_000_000)
    at_t = buffer.extract().values.copy()
    buffer.append([100, 100, 100], 3_000_000_000)
    np.testing.assert_allclose(at_t, [1, 2, 3, 1, 2, 3, -1, 4])


def test_incomplete_window_does_not_crash_until_extract_requested() -> None:
    buffer = CausalFeatureBuffer(5)
    buffer.append([0, 0, 0], 1)
    assert not buffer.ready
    with pytest.raises(RuntimeError, match="window"):
        buffer.extract()


def test_phase_uses_causal_flex_trend() -> None:
    raising = CausalFeatureBuffer(3, phase_slope_deadband=0.01)
    lowering = CausalFeatureBuffer(3, phase_slope_deadband=0.01)
    holding = CausalFeatureBuffer(3, phase_slope_deadband=0.1)
    for i in range(3):
        raising.append([i, 0, 0], i * 1_000_000_000)
        lowering.append([3 - i, 0, 0], i * 1_000_000_000)
        holding.append([1 + i * 0.01, 0, 0], i * 1_000_000_000)
    assert raising.extract().phase is MotionPhase.RAISING
    assert lowering.extract().phase is MotionPhase.LOWERING
    assert holding.extract().phase is MotionPhase.HOLDING


def test_reset_removes_all_history() -> None:
    buffer = CausalFeatureBuffer(3)
    for i in range(3):
        buffer.append([i, i, i], i + 1)
    assert buffer.ready
    buffer.reset()
    assert not buffer.ready
    assert buffer.size == 0


@pytest.mark.parametrize(
    ("values", "timestamp"),
    [([1, 2, float("nan")], 1), ([1, 2], 1), ([1, 2, 3], -1)],
)
def test_append_rejects_invalid_values(values: list[float], timestamp: int) -> None:
    buffer = CausalFeatureBuffer(3)
    with pytest.raises(ValueError):
        buffer.append(values, timestamp)


def test_append_rejects_non_monotonic_timestamp() -> None:
    buffer = CausalFeatureBuffer(3)
    buffer.append([1, 2, 3], 10)
    with pytest.raises(ValueError, match="increasing"):
        buffer.append([2, 3, 4], 10)


def test_runtime_projects_selected_c_features() -> None:
    buffer = CausalFeatureBuffer(3)
    for i, row in enumerate(([0, 0, 0], [1, 2, 3], [2, 4, 6])):
        buffer.append(row, i * 1_000_000_000)
    vector = buffer.extract()
    projected = select_features(
        vector,
        ("f1_median", "f2_median", "f3_median", "f1_slope", "f2_slope", "f3_slope"),
    )
    np.testing.assert_allclose(projected, [1, 2, 3, 1, 2, 3])


def test_runtime_a_features_use_current_frame() -> None:
    buffer = CausalFeatureBuffer(3)
    for i, row in enumerate(([0, 0, 0], [1, 2, 3], [9, 8, 7])):
        buffer.append(row, i * 1_000_000_000)
    projected = select_features(buffer.extract(), ("f1_current", "f2_current", "f3_current"))
    np.testing.assert_allclose(projected, [9, 8, 7])

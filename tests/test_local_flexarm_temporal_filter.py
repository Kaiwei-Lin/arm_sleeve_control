from __future__ import annotations

import pytest

from sleeve_arm.estimation.flexarm.temporal_filter import ActionDebouncer, AngleFilter


def test_action_changes_only_after_three_matching_frames() -> None:
    state = ActionDebouncer(required_frames=3, confidence_threshold=0.55)
    assert state.update("Forward", 0.9) == "Unknown"
    assert state.update("Forward", 0.9) == "Unknown"
    assert state.update("Forward", 0.9) == "Forward"
    assert state.update("Lateral", 0.9) == "Forward"
    assert state.update("Lateral", 0.9) == "Forward"
    assert state.update("Lateral", 0.9) == "Lateral"


def test_low_confidence_has_bounded_grace_then_unknown() -> None:
    state = ActionDebouncer(
        required_frames=1,
        confidence_threshold=0.6,
        low_confidence_grace_frames=2,
    )
    assert state.update("Forward", 0.9) == "Forward"
    assert state.update("Forward", 0.2) == "Forward"
    assert state.update("Forward", 0.2) == "Forward"
    assert state.update("Forward", 0.2) == "Unknown"


def test_rest_switches_immediately_and_resets_candidate() -> None:
    state = ActionDebouncer(required_frames=3, confidence_threshold=0.5)
    state.update("Forward", 0.9)
    assert state.update("Rest", 1.0) == "Rest"
    assert state.update("Forward", 0.9) == "Rest"


def test_velocity_limit_precedes_ema() -> None:
    filt = AngleFilter(alpha=0.25, max_velocity_deg_s=40)
    assert filt.update(40, 1_000_000_000) == 40
    assert filt.update(80, 1_100_000_000) == pytest.approx(41.0)


def test_angle_filter_rejects_non_monotonic_timestamp() -> None:
    filt = AngleFilter(alpha=0.25, max_velocity_deg_s=40)
    filt.update(40, 10)
    with pytest.raises(ValueError, match="increasing"):
        filt.update(41, 10)


def test_angle_filter_resets_on_large_gap() -> None:
    filt = AngleFilter(alpha=0.25, max_velocity_deg_s=40, max_gap_s=1.0)
    filt.update(40, 1_000_000_000)
    assert filt.update(70, 3_000_000_000) == 70


def test_reset_and_rest_return_zero() -> None:
    filt = AngleFilter(alpha=0.25, max_velocity_deg_s=40)
    filt.update(40, 1_000_000_000)
    assert filt.rest() == 0.0
    assert filt.value == 0.0
    filt.reset()
    assert filt.value is None

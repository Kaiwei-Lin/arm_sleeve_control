from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from sleeve_arm.estimation.flexarm.config import FlexArmConfig
from sleeve_arm.estimation.flexarm.types import FlexArmPrediction, MotionPhase, VALID_ACTIONS


def test_config_accepts_supported_defaults() -> None:
    config = FlexArmConfig()
    config.validate()
    assert config.window_size == 5
    assert config.ema_alpha == pytest.approx(0.25)
    assert config.debounce_frames == 3


@pytest.mark.parametrize("window_size", [3, 5, 7, 9])
def test_config_accepts_supported_windows(window_size: int) -> None:
    FlexArmConfig(window_size=window_size).validate()


@pytest.mark.parametrize("window_size", [0, 1, 4, 11])
def test_config_rejects_unsupported_windows(window_size: int) -> None:
    with pytest.raises(ValueError, match="window_size"):
        FlexArmConfig(window_size=window_size).validate()


@pytest.mark.parametrize(
    ("kwargs", "message"),
    [
        ({"ema_alpha": 0.0}, "ema_alpha"),
        ({"ema_alpha": 1.1}, "ema_alpha"),
        ({"max_velocity_deg_s": 0.0}, "max_velocity_deg_s"),
        ({"debounce_frames": 0}, "debounce_frames"),
        ({"action_confidence_threshold": -0.1}, "action_confidence_threshold"),
        ({"action_confidence_threshold": 1.1}, "action_confidence_threshold"),
        ({"eps": 0.0}, "eps"),
    ],
)
def test_config_rejects_invalid_values(kwargs: dict[str, float], message: str) -> None:
    with pytest.raises(ValueError, match=message):
        FlexArmConfig(**kwargs).validate()


def test_prediction_is_immutable() -> None:
    result = FlexArmPrediction("Rest", 0.0, 1.0, 1.0, False)
    with pytest.raises(FrozenInstanceError):
        result.angle_deg = 2.0  # type: ignore[misc]


def test_public_enums_and_actions_are_stable() -> None:
    assert VALID_ACTIONS == ("Forward", "Backward", "Lateral", "Rest", "Unknown")
    assert {phase.value for phase in MotionPhase} == {"Raising", "Lowering", "Holding"}

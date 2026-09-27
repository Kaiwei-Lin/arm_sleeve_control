from typing import TYPE_CHECKING

from .config import FlexArmConfig
from .dual_imu import (
    ArmRaiseEstimate,
    DirectionCalibration,
    DualImuArmEstimator,
    DualImuCalibration,
    ShoulderPoseEstimate,
)
from .types import FlexArmPrediction, MotionPhase, VALID_ACTIONS

if TYPE_CHECKING:
    from .estimator import FlexArmEstimator


def __getattr__(name: str):
    if name == "FlexArmEstimator":
        from .estimator import FlexArmEstimator

        return FlexArmEstimator
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = [
    "FlexArmConfig",
    "FlexArmEstimator",
    "FlexArmPrediction",
    "DualImuArmEstimator",
    "DualImuCalibration",
    "DirectionCalibration",
    "ArmRaiseEstimate",
    "ShoulderPoseEstimate",
    "MotionPhase",
    "VALID_ACTIONS",
]

from __future__ import annotations

import importlib
import math
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

from sleeve_arm.config import FlexModelConfig
from sleeve_arm.domain import ArmAction, MotionIntent, SensorSample
from sleeve_arm.predictor.base import MotionPredictor
from sleeve_arm.predictor.calibration import calibrate_estimator


_ACTIVE_ACTIONS = {
    "Forward": ArmAction.FORWARD,
    "Lateral": ArmAction.LATERAL,
    "Backward": ArmAction.BACKWARD,
}
_NON_ACTIVE_ACTIONS = {"Rest", "Unknown"}


class FlexModelPredictor(MotionPredictor):
    """Adapt one stateful FlexArmEstimator to human shoulder semantics."""

    def __init__(self, config: FlexModelConfig, estimator: Any | None = None) -> None:
        self.config = config
        self._estimator = estimator if estimator is not None else self._load_model()
        self._last_active_flexion: float | None = None
        self._last_active_abduction: float | None = None
        self.last_flex: tuple[float, float, float] | None = None
        self.last_raw_action: ArmAction | None = None
        self.last_model_action: str | None = None
        self.last_confidence: float | None = None
        self.last_angle_confidence: float | None = None
        self.last_angle_deg: float | None = None
        self.last_moving: bool | None = None
        self.last_inference_ms: float | None = None

    def _load_model(self) -> Any:
        try:
            module = importlib.import_module("flexarm")
            estimator_class = getattr(module, "FlexArmEstimator")
            return estimator_class.from_pretrained(self.config.model_dir)
        except (ImportError, ModuleNotFoundError, AttributeError) as exc:
            raise RuntimeError(
                "FlexArmEstimator could not be imported; install the flexarm-estimator wheel"
            ) from exc
        except Exception as exc:
            raise RuntimeError(
                f"FlexArmEstimator could not load model artifacts from {self.config.model_dir}: {exc}"
            ) from exc

    def predict(self, sample: SensorSample) -> MotionIntent:
        indexes = tuple(channel - 1 for channel in self.config.sleeve_channels)
        if max(indexes) >= len(sample.sleeve.channels):
            raise ValueError(
                "CH3/CH4/CH5 are required; "
                f"SleeveFrame has {len(sample.sleeve.channels)} channels"
            )
        flex = tuple(float(sample.sleeve.channels[index]) for index in indexes)
        if not all(math.isfinite(value) for value in flex):
            raise ValueError("CH3/CH4/CH5 must be finite")

        started = time.perf_counter()
        result = self._estimator.update(
            flex1=flex[0],
            flex2=flex[1],
            flex3=flex[2],
            timestamp_ns=int(sample.timestamp * 1e9),
        )
        inference_ms = (time.perf_counter() - started) * 1000.0

        model_action = getattr(result, "action", None)
        if model_action not in _ACTIVE_ACTIONS and model_action not in _NON_ACTIVE_ACTIONS:
            raise ValueError(f"invalid model action: {model_action!r}")
        angle_deg = self._finite_number("angle_deg", getattr(result, "angle_deg", None))
        action_confidence = self._confidence(
            "action_confidence", getattr(result, "action_confidence", None)
        )
        angle_confidence = self._confidence(
            "angle_confidence", getattr(result, "angle_confidence", None)
        )
        moving = getattr(result, "moving", None)
        if type(moving) is not bool:
            raise ValueError("moving must be a boolean")

        action: ArmAction | None = None
        flexion = self._last_active_flexion
        abduction = self._last_active_abduction
        if model_action in _ACTIVE_ACTIONS:
            if not self.config.angle_min_deg <= angle_deg <= self.config.angle_max_deg:
                raise ValueError(
                    f"angle_deg must be in [{self.config.angle_min_deg}, {self.config.angle_max_deg}]"
                )
            action = _ACTIVE_ACTIONS[model_action]
            flexion = abduction = 0.0
            angle_rad = math.radians(angle_deg)
            if action is ArmAction.FORWARD:
                flexion = angle_rad
            elif action is ArmAction.BACKWARD:
                flexion = -angle_rad
            else:
                abduction = angle_rad
            self._last_active_flexion = flexion
            self._last_active_abduction = abduction

        self.last_flex = flex
        self.last_raw_action = action
        self.last_model_action = model_action
        self.last_confidence = action_confidence
        self.last_angle_confidence = angle_confidence
        self.last_angle_deg = angle_deg
        self.last_moving = moving
        self.last_inference_ms = inference_ms
        return MotionIntent(
            timestamp=sample.timestamp,
            shoulder_flexion_rad=flexion,
            shoulder_abduction_rad=abduction,
            action=action,
            confidence=action_confidence,
            angle_deg=angle_deg,
            inference_ms=inference_ms,
            model_action=model_action,
            angle_confidence=angle_confidence,
            moving=moving,
        )

    def reset(self) -> None:
        self._estimator.reset()
        self._last_active_flexion = None
        self._last_active_abduction = None

    def calibrate(self, rows: np.ndarray, output: Path) -> Any:
        calibration = calibrate_estimator(self._estimator, rows, output)
        self._last_active_flexion = None
        self._last_active_abduction = None
        return calibration

    def reuse_calibration(self, path: Path) -> None:
        try:
            calibration_module = importlib.import_module("flexarm.calibration")
            calibration = calibration_module.FlexCalibration.load(path)
            artifacts = replace(
                self._estimator.artifacts,
                default_calibration=calibration,
            )
            self._estimator = self._estimator.__class__(artifacts)
        except Exception as exc:
            raise RuntimeError(f"could not reuse FlexArm calibration from {path}: {exc}") from exc
        self._last_active_flexion = None
        self._last_active_abduction = None

    @staticmethod
    def _finite_number(name: str, value: Any) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be finite") from exc
        if not math.isfinite(result):
            raise ValueError(f"{name} must be finite")
        return result

    @classmethod
    def _confidence(cls, name: str, value: Any) -> float:
        result = cls._finite_number(name, value)
        if not 0.0 <= result <= 1.0:
            raise ValueError(f"{name} must be in [0, 1]")
        return result


class ArmMotionPredictor(MotionPredictor):
    def __init__(self, elbow: MotionPredictor, shoulder: FlexModelPredictor) -> None:
        self.elbow = elbow
        self.shoulder = shoulder

    def predict(self, sample: SensorSample) -> MotionIntent:
        elbow = self.elbow.predict(sample)
        shoulder = self.shoulder.predict(sample)
        return MotionIntent(
            timestamp=sample.timestamp,
            elbow_flexion=elbow.elbow_flexion,
            shoulder_flexion_rad=shoulder.shoulder_flexion_rad,
            shoulder_abduction_rad=shoulder.shoulder_abduction_rad,
            action=shoulder.action,
            confidence=shoulder.confidence,
            action_probabilities=shoulder.action_probabilities,
            angle_deg=shoulder.angle_deg,
            inference_ms=shoulder.inference_ms,
            model_action=shoulder.model_action,
            angle_confidence=shoulder.angle_confidence,
            moving=shoulder.moving,
        )

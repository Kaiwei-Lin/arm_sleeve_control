from __future__ import annotations

import importlib
import math
import operator
import time
from typing import Any

from sleeve_arm.config import FlexModelConfig
from sleeve_arm.domain import ArmAction, MotionIntent, SensorSample
from sleeve_arm.predictor.base import MotionPredictor


class FlexModelPredictor(MotionPredictor):
    """Isolate the pip-installed FlexPredictor and emit human shoulder semantics."""

    def __init__(self, config: FlexModelConfig, model: Any | None = None) -> None:
        self.config = config
        self._model = model if model is not None else self._load_model()
        self._accepted_action: ArmAction | None = None
        self._candidate_action: ArmAction | None = None
        self._candidate_count = 0
        self._last_accepted_angle_deg = 0.0
        self.last_flex: tuple[float, float, float] | None = None
        self.last_raw_action: ArmAction | None = None
        self.last_probabilities: tuple[float, ...] | None = None
        self.last_confidence: float | None = None
        self.last_angle_deg: float | None = None
        self.last_inference_ms: float | None = None

    def _load_model(self) -> Any:
        try:
            module = importlib.import_module(self.config.model_module)
        except (ImportError, ModuleNotFoundError) as exc:
            raise RuntimeError(
                "Flex model module could not be imported. "
                f"Expected module: {self.config.model_module}"
            ) from exc
        try:
            model_class = getattr(module, self.config.model_class)
        except AttributeError as exc:
            raise RuntimeError(
                f"Flex model class {self.config.model_class!r} was not found "
                f"in {self.config.model_module!r}"
            ) from exc
        return model_class()

    def predict(self, sample: SensorSample) -> MotionIntent:
        indexes = tuple(channel - 1 for channel in self.config.sleeve_channels)
        if max(indexes) >= len(sample.sleeve.channels):
            raise ValueError(
                f"CH2/CH3/CH4 are required; SleeveFrame has {len(sample.sleeve.channels)} channels"
            )
        flex = tuple(float(sample.sleeve.channels[index]) for index in indexes)
        if not all(math.isfinite(value) for value in flex):
            raise ValueError("CH2/CH3/CH4 must be finite")

        started = time.perf_counter()
        result = self._model.predict_raw(
            flex=list(flex),
            calibration_baseline=list(self.config.baseline),
            calibration_scale=list(self.config.scale),
            trial_rest=list(self.config.trial_rest),
        )
        inference_ms = (time.perf_counter() - started) * 1000.0
        action_value = getattr(result, "action", None)
        try:
            if isinstance(action_value, str):
                raw_action = {
                    "forward": ArmAction.FORWARD,
                    "lateral": ArmAction.LATERAL,
                    "backward": ArmAction.BACKWARD,
                }[action_value.strip().casefold()]
            else:
                raw_action = ArmAction(operator.index(action_value))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid model action: {action_value!r}") from exc
        try:
            probabilities = tuple(float(value) for value in result.action_probabilities)
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError("invalid model action_probabilities") from exc
        if len(probabilities) < 3 or not all(math.isfinite(value) and 0 <= value <= 1 for value in probabilities):
            raise ValueError("action_probabilities must contain at least three finite values in [0, 1]")
        try:
            angle_deg = float(result.angle_deg)
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError("invalid model angle_deg") from exc
        if not math.isfinite(angle_deg) or not self.config.angle_min_deg <= angle_deg <= self.config.angle_max_deg:
            raise ValueError(
                f"angle_deg must be finite and in [{self.config.angle_min_deg}, {self.config.angle_max_deg}]"
            )

        confidence = probabilities[int(raw_action)]
        accepted_action, accepted_angle = self._stabilize(raw_action, angle_deg)
        flexion = abduction = 0.0
        angle_rad = math.radians(accepted_angle)
        if accepted_action is ArmAction.FORWARD:
            flexion = angle_rad
        elif accepted_action is ArmAction.BACKWARD:
            flexion = -angle_rad
        else:
            abduction = angle_rad

        self.last_flex = flex
        self.last_raw_action = raw_action
        self.last_probabilities = probabilities
        self.last_confidence = confidence
        self.last_angle_deg = angle_deg
        self.last_inference_ms = inference_ms
        return MotionIntent(
            timestamp=sample.timestamp,
            shoulder_flexion_rad=flexion,
            shoulder_abduction_rad=abduction,
            action=accepted_action,
            confidence=confidence,
            action_probabilities=probabilities,
            angle_deg=accepted_angle,
            inference_ms=inference_ms,
        )

    def _stabilize(self, action: ArmAction, angle_deg: float) -> tuple[ArmAction, float]:
        if self._accepted_action is None:
            self._accepted_action = action
            self._last_accepted_angle_deg = angle_deg
        elif action is self._accepted_action:
            self._candidate_action = None
            self._candidate_count = 0
            self._last_accepted_angle_deg = angle_deg
        else:
            if action is self._candidate_action:
                self._candidate_count += 1
            else:
                self._candidate_action = action
                self._candidate_count = 1
            if self._candidate_count >= self.config.required_consecutive_frames:
                self._accepted_action = action
                self._last_accepted_angle_deg = angle_deg
                self._candidate_action = None
                self._candidate_count = 0
        return self._accepted_action, self._last_accepted_angle_deg


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
        )

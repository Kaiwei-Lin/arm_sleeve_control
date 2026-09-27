from __future__ import annotations

from collections import deque
import logging
import math
from pathlib import Path
from typing import Sequence

import numpy as np

from .calibration import FlexCalibration
from .features import CausalFeatureBuffer, select_features
from .model_io import ModelArtifacts, load_artifacts
from .motion_gate import MotionGate
from .normalization import FlexNormalizer
from .temporal_filter import ActionDebouncer, AngleFilter
from .types import FlexArmPrediction


LOGGER = logging.getLogger(__name__)
UNKNOWN = FlexArmPrediction("Unknown", 0.0, 0.0, 0.0, False)


class FlexArmEstimator:
    def __init__(self, artifacts: ModelArtifacts) -> None:
        artifacts.config.validate()
        self.artifacts = artifacts
        self.config = artifacts.config
        self._calibration = artifacts.default_calibration
        self._normalizer = FlexNormalizer(self._calibration, eps=self.config.eps)
        self._gate = MotionGate.from_calibration(self._calibration)
        self._buffer = CausalFeatureBuffer(
            self.config.window_size,
            phase_slope_deadband=self._calibration.rest_thresholds.phase_slope_deadband,
        )
        self._debouncer = ActionDebouncer(
            required_frames=self.config.debounce_frames,
            confidence_threshold=self.config.action_confidence_threshold,
            low_confidence_grace_frames=self.config.low_confidence_grace_frames,
        )
        self._angle_filter = AngleFilter(
            alpha=self.config.ema_alpha,
            max_velocity_deg_s=self.config.max_velocity_deg_s,
            max_gap_s=self.config.max_timestamp_gap_s,
        )
        self._recent_angles: deque[float] = deque(maxlen=self.config.window_size)

    @classmethod
    def from_pretrained(cls, directory: str | Path) -> "FlexArmEstimator":
        return cls(load_artifacts(directory))

    @property
    def calibration(self) -> FlexCalibration:
        return self._calibration

    @property
    def buffer_size(self) -> int:
        return self._buffer.size

    def update(
        self,
        flex1: float,
        flex2: float,
        flex3: float,
        timestamp_ns: int,
    ) -> FlexArmPrediction:
        return self.predict_raw(flex1, flex2, flex3, timestamp_ns=timestamp_ns)

    def predict_raw(
        self,
        flex1: float,
        flex2: float,
        flex3: float,
        *,
        timestamp_ns: int,
    ) -> FlexArmPrediction:
        try:
            normalized = self._normalizer.normalize([flex1, flex2, flex3])
        except (TypeError, ValueError):
            LOGGER.warning("Invalid raw Flex input; returning Unknown")
            self.reset()
            return UNKNOWN
        return self.predict_normalized(*normalized, timestamp_ns=timestamp_ns)

    def predict_normalized(
        self,
        flex1: float,
        flex2: float,
        flex3: float,
        *,
        timestamp_ns: int,
    ) -> FlexArmPrediction:
        try:
            values = np.asarray([flex1, flex2, flex3], dtype=float)
            if not np.isfinite(values).all():
                raise ValueError("non-finite normalized input")
            self._buffer.append(values, timestamp_ns)
            if not self._buffer.ready:
                return UNKNOWN
            vector = self._buffer.extract()
            moving = self._gate.classify(self._buffer.window, vector.slopes)
            if not moving:
                self._debouncer.update("Rest", 1.0)
                self._angle_filter.rest()
                self._recent_angles.clear()
                return FlexArmPrediction("Rest", 0.0, 1.0, 1.0, False)
            model_features = select_features(
                vector, self.artifacts.action_classifier.feature_names
            )
            candidate, classifier_confidence = self.artifacts.action_classifier.predict(model_features)
            heuristic_factor = self._physical_consistency(candidate, vector.medians)
            action_confidence = float(np.clip(classifier_confidence * heuristic_factor, 0.0, 1.0))
            stable_action = self._debouncer.update(candidate, action_confidence)
            if stable_action in ("Unknown", "Rest"):
                return FlexArmPrediction(stable_action, 0.0, action_confidence, 0.0, True)
            raw_angle, disagreement, coverage = self.artifacts.angle_estimator.predict(
                stable_action, model_features
            )
            filtered_angle = self._angle_filter.update(raw_angle, timestamp_ns)
            self._recent_angles.append(filtered_angle)
            angle_confidence = self._angle_confidence(disagreement, coverage)
            return FlexArmPrediction(
                stable_action,
                float(filtered_angle),
                action_confidence,
                angle_confidence,
                True,
            )
        except (TypeError, ValueError, RuntimeError) as exc:
            LOGGER.warning("FlexArm update rejected: %s", exc)
            self.reset()
            return UNKNOWN

    def calibrate(self, raw_samples: Sequence[Sequence[float]]) -> FlexCalibration:
        calibration = FlexCalibration.fit(raw_samples, scales=self._calibration.scale)
        self._calibration = calibration
        self._normalizer = FlexNormalizer(calibration, eps=self.config.eps)
        self._gate = MotionGate.from_calibration(calibration)
        self._buffer = CausalFeatureBuffer(
            self.config.window_size,
            phase_slope_deadband=calibration.rest_thresholds.phase_slope_deadband,
        )
        self.reset()
        return calibration

    def reset(self) -> None:
        self._buffer.reset()
        self._debouncer.reset()
        self._angle_filter.reset()
        self._recent_angles.clear()

    @staticmethod
    def _physical_consistency(action: str, medians: np.ndarray) -> float:
        f1, f2, f3 = medians
        if action == "Forward" and f2 < -0.1:
            return 0.7
        if action == "Backward" and f3 < -0.1:
            return 0.7
        if action == "Lateral" and not (f1 < 0.0 and f2 < 0.1 and f3 > 0.0):
            return 0.7
        return 1.0

    def _angle_confidence(self, disagreement: float, coverage: float) -> float:
        spread_score = math.exp(-max(0.0, disagreement) / 10.0)
        if len(self._recent_angles) < 2:
            stability = 1.0
        else:
            stability = math.exp(-float(np.std(self._recent_angles)) / 5.0)
        return float(np.clip(0.4 * spread_score + 0.4 * coverage + 0.2 * stability, 0.0, 1.0))

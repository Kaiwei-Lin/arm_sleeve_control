from __future__ import annotations

from collections.abc import Mapping, Sequence

import numpy as np

from .types import ACTIVE_ACTIONS


class AngleEstimator:
    def __init__(
        self,
        models: Mapping[str, object],
        feature_names: Sequence[str],
        angle_ranges: Mapping[str, tuple[float, float]],
        feature_bounds: Mapping[str, tuple[np.ndarray, np.ndarray]],
    ) -> None:
        self.models = dict(models)
        self.feature_names = tuple(str(name) for name in feature_names)
        self.angle_ranges = {
            str(action): (float(bounds[0]), float(bounds[1]))
            for action, bounds in angle_ranges.items()
        }
        self.feature_bounds = {
            str(action): (
                np.asarray(bounds[0], dtype=float),
                np.asarray(bounds[1], dtype=float),
            )
            for action, bounds in feature_bounds.items()
        }

    def predict(
        self,
        action: str,
        features: Sequence[float] | np.ndarray,
    ) -> tuple[float, float, float]:
        if action not in ACTIVE_ACTIONS:
            raise ValueError("angle prediction requires an active action")
        if action not in self.models or action not in self.angle_ranges:
            raise ValueError(f"no angle model or range for {action}")
        values = np.asarray(features, dtype=float)
        if values.shape != (len(self.feature_names),) or not np.isfinite(values).all():
            raise ValueError(f"expected {len(self.feature_names)} finite angle features")
        model = self.models[action]
        raw = float(np.asarray(model.predict(values.reshape(1, -1))).reshape(-1)[0])
        if not np.isfinite(raw):
            raise ValueError("angle model returned a non-finite prediction")
        lower, upper = self.angle_ranges[action]
        angle = float(np.clip(raw, lower, upper))
        disagreement = self._tree_disagreement(model, values)
        coverage = self._coverage(action, values)
        return angle, disagreement, coverage

    @staticmethod
    def _tree_disagreement(model: object, features: np.ndarray) -> float:
        estimators = getattr(model, "estimators_", None)
        if estimators is None:
            return 0.0
        if len(estimators) > 32:
            indices = np.linspace(0, len(estimators) - 1, 32, dtype=int)
            estimators = [estimators[index] for index in indices]
        predictions = np.asarray(
            [float(tree.predict(features.reshape(1, -1))[0]) for tree in estimators],
            dtype=float,
        )
        return float(np.std(predictions, ddof=0))

    def _coverage(self, action: str, features: np.ndarray) -> float:
        if action not in self.feature_bounds:
            return 0.0
        lower, upper = self.feature_bounds[action]
        if lower.shape != features.shape or upper.shape != features.shape:
            return 0.0
        return float(np.mean((features >= lower) & (features <= upper)))

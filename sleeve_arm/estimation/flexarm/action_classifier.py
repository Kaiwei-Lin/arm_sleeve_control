from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from .types import ACTIVE_ACTIONS


class ActionClassifier:
    def __init__(self, model: object, feature_names: Sequence[str]) -> None:
        names = tuple(str(name) for name in feature_names)
        if not names or len(set(names)) != len(names):
            raise ValueError("feature_names must be non-empty and unique")
        if not hasattr(model, "predict_proba") or not hasattr(model, "classes_"):
            raise ValueError("action model must expose predict_proba and classes_")
        classes = tuple(str(value) for value in getattr(model, "classes_"))
        if set(classes) != set(ACTIVE_ACTIONS):
            raise ValueError(f"classifier classes must be {ACTIVE_ACTIONS}")
        self.model = model
        self.feature_names = names

    def predict(self, features: Sequence[float] | np.ndarray) -> tuple[str, float]:
        values = np.asarray(features, dtype=float)
        if values.shape != (len(self.feature_names),):
            raise ValueError(f"expected {len(self.feature_names)} finite features")
        if not np.isfinite(values).all():
            raise ValueError("classifier features must be finite")
        probabilities = np.asarray(self.model.predict_proba(values.reshape(1, -1))[0], dtype=float)
        index = int(np.argmax(probabilities))
        classes = np.asarray(getattr(self.model, "classes_"), dtype=str)
        return str(classes[index]), float(np.clip(probabilities[index], 0.0, 1.0))


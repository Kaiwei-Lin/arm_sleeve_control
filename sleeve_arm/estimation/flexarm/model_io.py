from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import joblib
import numpy as np
import sklearn

from .action_classifier import ActionClassifier
from .angle_estimator import AngleEstimator
from .calibration import FlexCalibration
from .config import FlexArmConfig
from .types import ACTIVE_ACTIONS


MODEL_FORMAT_VERSION = "flexarm-model-v1"


@dataclass(frozen=True)
class ModelArtifacts:
    config: FlexArmConfig
    default_calibration: FlexCalibration
    action_classifier: ActionClassifier
    angle_estimator: AngleEstimator
    action_feature_bounds: dict[str, tuple[np.ndarray, np.ndarray]]
    metadata: dict[str, Any]


def _write_json_atomic(path: Path, payload: object) -> None:
    partial = path.with_suffix(path.suffix + ".partial")
    partial.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    partial.replace(path)


def _dump_atomic(path: Path, value: object) -> None:
    partial = path.with_suffix(path.suffix + ".partial")
    joblib.dump(value, partial)
    partial.replace(path)


def save_artifacts(directory: str | Path, artifacts: ModelArtifacts) -> None:
    output = Path(directory)
    output.mkdir(parents=True, exist_ok=True)
    feature_names = artifacts.action_classifier.feature_names
    if artifacts.angle_estimator.feature_names != feature_names:
        raise ValueError("classifier and regressors must use the same feature schema")
    if set(artifacts.angle_estimator.models) != set(ACTIVE_ACTIONS):
        raise ValueError("exactly three action-conditioned regressors are required")
    _dump_atomic(output / "action_classifier.joblib", artifacts.action_classifier.model)
    for action in ACTIVE_ACTIONS:
        _dump_atomic(output / f"angle_{action.lower()}.joblib", artifacts.angle_estimator.models[action])
    _write_json_atomic(output / "calibration.json", artifacts.default_calibration.to_dict())
    metadata = {
        "model_format_version": MODEL_FORMAT_VERSION,
        "scikit_learn_version": sklearn.__version__,
        "feature_names": list(feature_names),
        "config": asdict(artifacts.config),
        "angle_ranges": {
            action: list(artifacts.angle_estimator.angle_ranges[action]) for action in ACTIVE_ACTIONS
        },
        "angle_feature_bounds": {
            action: {
                "lower": artifacts.angle_estimator.feature_bounds[action][0].tolist(),
                "upper": artifacts.angle_estimator.feature_bounds[action][1].tolist(),
            }
            for action in ACTIVE_ACTIONS
        },
        "action_feature_bounds": {
            action: {
                "lower": artifacts.action_feature_bounds[action][0].tolist(),
                "upper": artifacts.action_feature_bounds[action][1].tolist(),
            }
            for action in ACTIVE_ACTIONS
        },
        "metadata": artifacts.metadata,
    }
    _write_json_atomic(output / "metadata.json", metadata)


def load_artifacts(directory: str | Path) -> ModelArtifacts:
    source = Path(directory)
    try:
        payload = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
        calibration = FlexCalibration.from_dict(
            json.loads((source / "calibration.json").read_text(encoding="utf-8"))
        )
    except (OSError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise ValueError(f"cannot load FlexArm metadata from {source}") from exc
    if payload.get("model_format_version") != MODEL_FORMAT_VERSION:
        raise ValueError("unsupported model format version")
    artifact_sklearn_version = payload.get("scikit_learn_version")
    if artifact_sklearn_version is None:
        try:
            training_payload = json.loads(
                (source / "training_metadata.json").read_text(encoding="utf-8")
            )
            artifact_sklearn_version = training_payload["versions"]["scikit_learn"]
        except (OSError, json.JSONDecodeError, KeyError, TypeError):
            artifact_sklearn_version = None
    if (
        artifact_sklearn_version is not None
        and str(artifact_sklearn_version) != sklearn.__version__
    ):
        raise ValueError(
            "FlexArm model scikit-learn version mismatch: "
            f"artifacts require {artifact_sklearn_version}, runtime is {sklearn.__version__}. "
            "Use the matching scikit-learn version or retrain the model artifacts."
        )
    try:
        feature_names = tuple(str(name) for name in payload["feature_names"])
        config = FlexArmConfig(**payload["config"])
        config.validate()
        angle_ranges = {
            action: tuple(float(value) for value in payload["angle_ranges"][action])
            for action in ACTIVE_ACTIONS
        }
        angle_bounds = {
            action: (
                np.asarray(payload["angle_feature_bounds"][action]["lower"], dtype=float),
                np.asarray(payload["angle_feature_bounds"][action]["upper"], dtype=float),
            )
            for action in ACTIVE_ACTIONS
        }
        action_bounds = {
            action: (
                np.asarray(payload["action_feature_bounds"][action]["lower"], dtype=float),
                np.asarray(payload["action_feature_bounds"][action]["upper"], dtype=float),
            )
            for action in ACTIVE_ACTIONS
        }
        classifier_model = joblib.load(source / "action_classifier.joblib")
        regressors = {
            action: joblib.load(source / f"angle_{action.lower()}.joblib")
            for action in ACTIVE_ACTIONS
        }
    except (KeyError, TypeError, ValueError, OSError) as exc:
        raise ValueError("invalid or incomplete FlexArm model artifacts") from exc
    classifier = ActionClassifier(classifier_model, feature_names)
    angle_estimator = AngleEstimator(regressors, feature_names, angle_ranges, angle_bounds)
    width = len(feature_names)
    for bounds in (*action_bounds.values(), *angle_bounds.values()):
        if bounds[0].shape != (width,) or bounds[1].shape != (width,):
            raise ValueError("model feature bounds do not match the feature schema")
    return ModelArtifacts(
        config=config,
        default_calibration=calibration,
        action_classifier=classifier,
        angle_estimator=angle_estimator,
        action_feature_bounds=action_bounds,
        metadata=dict(payload.get("metadata", {})),
    )

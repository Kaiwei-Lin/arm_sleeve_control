"""Local estimator integration; deterministic model doubles, no training/hardware."""
from dataclasses import asdict
import json
from pathlib import Path
import subprocess
import sys
import textwrap

import numpy as np
import pytest

from sleeve_arm.config import FlexModelConfig
from sleeve_arm.domain import ArmAction, ImuFrame, SensorSample, SleeveFrame
from sleeve_arm.estimation.flexarm import DualImuArmEstimator, FlexArmConfig, FlexArmEstimator
from sleeve_arm.estimation.flexarm.action_classifier import ActionClassifier
from sleeve_arm.estimation.flexarm.angle_estimator import AngleEstimator
from sleeve_arm.estimation.flexarm.calibration import FlexCalibration
from sleeve_arm.estimation.flexarm.features import FEATURE_NAMES
from sleeve_arm.estimation.flexarm.model_io import ModelArtifacts, load_artifacts, save_artifacts
from sleeve_arm.predictor.dual_imu_shoulder import DualImuShoulderPredictor
from sleeve_arm.predictor.flex_model import FlexModelPredictor

ROOT = Path(__file__).resolve().parents[1]


class FixedClassifier:
    classes_ = np.asarray(["Forward", "Backward", "Lateral"])

    def predict_proba(self, rows):
        return np.tile([1., 0., 0.], (len(rows), 1))


class FixedRegressor:
    def __init__(self, angle):
        self.angle = angle

    def predict(self, rows):
        return np.full(len(rows), self.angle)


def make_artifacts():
    bounds = {action: (np.full(8, -2.), np.full(8, 2.)) for action in FixedClassifier.classes_}
    return ModelArtifacts(
        config=FlexArmConfig(window_size=3, debounce_frames=1),
        default_calibration=FlexCalibration.fit(
            [[100, 100, 100], [101, 99, 100], [99, 101, 100]], scales=[100, 100, 100],
        ),
        action_classifier=ActionClassifier(FixedClassifier(), FEATURE_NAMES),
        angle_estimator=AngleEstimator(
            {action: FixedRegressor(angle) for action, angle in
             [("Forward", 45.), ("Backward", 35.), ("Lateral", 55.)]},
            FEATURE_NAMES,
            {"Forward": (10, 90), "Backward": (10, 60), "Lateral": (10, 95)},
            bounds,
        ),
        action_feature_bounds=bounds,
        metadata={"test_fixture": "no training"},
    )


def test_dual_imu_existing_outputs_and_intents_match_pre_migration_wheel(monkeypatch):
    golden = json.loads((ROOT / "tests/fixtures/dual_imu_migration_golden.json").read_text())
    estimator = DualImuArmEstimator(**golden["config"])
    for method, key in ((estimator.calibrate, "rest"), (estimator.calibrate_forward, "forward")):
        result = method([r["chest"] for r in golden[key]], [r["arm"] for r in golden[key]])
        for name, expected in golden[f"{key}_result"].items():
            assert getattr(result, name) == pytest.approx(expected, abs=1e-12)
    predictor = DualImuShoulderPredictor(estimator)
    monkeypatch.setattr("time.perf_counter", lambda: 0.)
    for row in golden["rows"]:
        arm = ImuFrame(row["timestamp"], 0, 0, 0, 0, 0, 0, *row["arm"])
        chest = ImuFrame(row["timestamp"], 0, 0, 0, 0, 0, 0, *row["chest"])
        intent = predictor.predict_imu_pair(arm, chest)
        for name, expected in row["estimate"].items():
            actual = getattr(predictor.last_result, name)
            if isinstance(expected, str):
                assert actual == expected, row["name"]
            else:
                assert actual == pytest.approx(expected, abs=1e-12), row["name"]
        actual_intent = asdict(intent)
        actual_intent["action"] = intent.action.name if intent.action is not None else None
        assert actual_intent == pytest.approx(row["intent"], abs=1e-12), row["name"]


def test_model_load_predict_calibrate_reuse_without_external_package(tmp_path):
    artifacts = make_artifacts()
    save_artifacts(tmp_path, artifacts)
    config = FlexModelConfig(
        model_dir=tmp_path, sleeve_channels=(3, 4, 5),
        calibration_file=tmp_path / "live.json",
        calibration_seconds=1., angle_min_deg=0., angle_max_deg=180.,
    )
    predictor = FlexModelPredictor(config)
    assert predictor._estimator.__class__ is FlexArmEstimator
    reference = FlexArmEstimator(artifacts)
    for index in range(1, 5):
        sample = SensorSample(float(index), SleeveFrame(float(index), (0, 0, 40, 160, 100)))
        intent = predictor.predict(sample)
        expected = reference.update(40, 160, 100, timestamp_ns=index * 1_000_000_000)
        assert intent.model_action == expected.action
        assert intent.angle_deg == expected.angle_deg
        assert intent.confidence == expected.action_confidence
        assert intent.angle_confidence == expected.angle_confidence
    assert intent.action is ArmAction.FORWARD
    assert intent.shoulder_flexion_rad == pytest.approx(np.pi / 4)

    rows = np.asarray([[200, 300, 400], [202, 298, 402], [198, 302, 398]])
    calibration = predictor.calibrate(rows, config.calibration_file)
    assert calibration.baseline == (200., 300., 400.)
    assert predictor._estimator.buffer_size == 0
    predictor.reuse_calibration(config.calibration_file)
    assert predictor._estimator.calibration == calibration
    assert predictor._estimator.buffer_size == 0
    for index in range(3):
        result = predictor._estimator.update(200, 300, 400, timestamp_ns=index + 1)
    assert result.action == "Rest"
    assert result.moving is False
    assert predictor._estimator.update(float("nan"), 300, 400, timestamp_ns=4).action == "Unknown"
    assert predictor._estimator.buffer_size == 0


def test_model_loader_keeps_version_check_before_unpickling(tmp_path):
    save_artifacts(tmp_path, make_artifacts())
    path = tmp_path / "metadata.json"
    metadata = json.loads(path.read_text())
    metadata["scikit_learn_version"] = "0.0.invalid"
    path.write_text(json.dumps(metadata))
    (tmp_path / "action_classifier.joblib").write_bytes(b"invalid")
    with pytest.raises(ValueError, match="scikit-learn version mismatch"):
        load_artifacts(tmp_path)


def test_dual_imu_needs_no_external_estimator_or_model_libraries():
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent('''
            import importlib.abc
            import sys
            class BlockExternal(importlib.abc.MetaPathFinder):
                def find_spec(self, fullname, path=None, target=None):
                    if fullname.split('.')[0] in {'flexarm', 'sklearn', 'joblib'}:
                        raise AssertionError('unexpected dependency: ' + fullname)
            sys.meta_path.insert(0, BlockExternal())
            from sleeve_arm.estimation.flexarm import DualImuArmEstimator
            estimator = DualImuArmEstimator()
            estimator.calibrate((1, 0, 0, 0), (1, 0, 0, 0))
            assert estimator.estimate((1, 0, 0, 0), (1, 0, 0, 0)).direction == 'Rest'
        ''')], cwd=ROOT, capture_output=True, text=True, timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("robot", ["fake", "aurora-fake"])
def test_fake_cli_uses_local_dual_imu_with_external_package_blocked(robot):
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent('''
            import importlib.abc
            import runpy
            import sys
            class BlockExternal(importlib.abc.MetaPathFinder):
                def find_spec(self, fullname, path=None, target=None):
                    if fullname.split('.')[0] in {'flexarm', 'fourier_aurora_client'}:
                        raise AssertionError('unexpected dependency: ' + fullname)
            sys.meta_path.insert(0, BlockExternal())
            sys.argv = ['tools/run_model_control.py', '--robot', sys.argv[1],
                        '--side', 'right', '--source-side', 'right',
                        '--sleeve', 'fake', '--imus', 'fake', '--shoulder-predictor', 'dual_imu',
                        '--duration', '0.15', '--calibration-seconds', '0.08']
            runpy.run_path(sys.argv[0], run_name='__main__')
        '''), robot], cwd=ROOT, input="\n\n\n", capture_output=True, text=True, timeout=20,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "shoulder_predictor=dual_imu" in result.stdout

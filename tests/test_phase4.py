from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sleeve_arm.config import FlexModelConfig, load_phase3_config, load_phase4_config, load_robot_config
from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.domain import ArmAction, MotionIntent, SensorSample, SleeveFrame
from sleeve_arm.predictor import ArmMotionPredictor, FlexModelPredictor, RuleBasedPredictor
from sleeve_arm.predictor.calibration import calibrate_estimator, collect_calibration_samples
from sleeve_arm.robot import FakeRobotArm
from sleeve_arm.sources import FakeSleeveSource
from tools.calibrate_flex_model import LatestFlexReader
from tools.debug_model_mapping import manual_intent


def model_config(**changes) -> FlexModelConfig:
    values = {
        "model_module": "unused_in_mock",
        "model_class": "FlexPredictor",
        "sleeve_channels": (2, 3, 4),
        "baseline": (1.0, 2.0, 3.0),
        "scale": (4.0, 5.0, 6.0),
        "trial_rest": (7.0, 8.0, 9.0),
        "min_action_confidence": 0.6,
        "required_consecutive_frames": 3,
        "angle_min_deg": 0.0,
        "angle_max_deg": 180.0,
    }
    values.update(changes)
    return FlexModelConfig(**values)


def write_phase4_config(
    tmp_path: Path,
    *,
    sleeve_channels: str = "[3, 4, 5]",
    calibration_seconds: str = "3",
) -> Path:
    (tmp_path / "models").mkdir(exist_ok=True)
    config = tmp_path / "phase4.yaml"
    config.write_text(f"""
predictor:
  backend: flexarm_estimator
  model_dir: models
  sleeve_channels: {sleeve_channels}
  calibration_file: runtime/calibration.json
  calibration_seconds: {calibration_seconds}
  angle: {{min_deg: 0, max_deg: 90}}
phase4_validation:
  max_consecutive_prediction_errors: 3
""", encoding="utf-8")
    return config


def test_phase4_loads_flexarm_paths_relative_to_config(tmp_path: Path) -> None:
    model_dir = tmp_path / "models"
    config = write_phase4_config(tmp_path)
    loaded = load_phase4_config(config)
    assert loaded.flex_model is not None
    assert loaded.flex_model.model_dir == model_dir.resolve()
    assert loaded.flex_model.calibration_file == (tmp_path / "runtime/calibration.json").resolve()
    assert loaded.flex_model.sleeve_channels == (3, 4, 5)
    assert loaded.flex_model.calibration_seconds == 3.0


def test_phase4_rejects_wrong_flexarm_channels(tmp_path: Path) -> None:
    config = write_phase4_config(tmp_path, sleeve_channels="[2, 3, 4]")
    with pytest.raises(ValueError, match=r"exactly \[3, 4, 5\]"):
        load_phase4_config(config)


def test_phase4_rejects_non_positive_calibration_duration(tmp_path: Path) -> None:
    config = write_phase4_config(tmp_path, calibration_seconds="0")
    with pytest.raises(ValueError, match="calibration_seconds must be positive"):
        load_phase4_config(config)


def test_motion_intent_preserves_flexarm_diagnostics() -> None:
    intent = MotionIntent(
        timestamp=1.0,
        model_action="Rest",
        angle_confidence=0.75,
        moving=False,
    )
    assert (intent.model_action, intent.angle_confidence, intent.moving) == (
        "Rest",
        0.75,
        False,
    )


@pytest.mark.parametrize("value", (-0.1, 1.1, math.nan))
def test_motion_intent_rejects_invalid_angle_confidence(value: float) -> None:
    with pytest.raises(ValueError, match="angle_confidence"):
        MotionIntent(timestamp=1.0, angle_confidence=value)


def test_motion_intent_rejects_non_boolean_moving() -> None:
    with pytest.raises(ValueError, match="moving"):
        MotionIntent(timestamp=1.0, moving=1)


def test_motion_intent_rejects_unknown_model_action_label() -> None:
    with pytest.raises(ValueError, match="model_action"):
        MotionIntent(timestamp=1.0, model_action="Turning")


class SequenceSleeveSource:
    def __init__(self, frames) -> None:
        self.frames = iter(frames)

    def latest(self):
        return next(self.frames, None)


class SequenceClock:
    def __init__(self, values: list[float]) -> None:
        self.values = iter(values)

    def __call__(self) -> float:
        return next(self.values)


def test_collect_calibration_samples_uses_fresh_ch3_ch4_ch5() -> None:
    source = SequenceSleeveSource([
        SleeveFrame(1.0, (10, 20, 30, 40, 50)),
        SleeveFrame(2.0, (11, 21, 31, 41, 51)),
    ])
    rows = collect_calibration_samples(
        source,
        (3, 4, 5),
        1.0,
        monotonic=SequenceClock([0.0, 0.1, 0.2, 1.0]),
    )
    assert rows.tolist() == [[30.0, 40.0, 50.0], [31.0, 41.0, 51.0]]


def test_collect_calibration_samples_skips_duplicate_and_invalid_frames() -> None:
    source = SequenceSleeveSource([
        SleeveFrame(1.0, (10, 20, 30, 40, 50)),
        SleeveFrame(1.0, (11, 21, 31, 41, 51)),
        SimpleNamespace(timestamp=2.0, channels=(10, 20, math.nan, 40, 50)),
        SleeveFrame(3.0, (12, 22, 32, 42, 52)),
    ])
    rows = collect_calibration_samples(
        source,
        (3, 4, 5),
        1.0,
        monotonic=SequenceClock([0.0, 0.1, 0.2, 0.3, 0.4, 1.0]),
    )
    assert rows.tolist() == [[30.0, 40.0, 50.0], [32.0, 42.0, 52.0]]


def test_collect_calibration_samples_requires_two_valid_rows() -> None:
    source = SequenceSleeveSource([SleeveFrame(1.0, (10, 20, 30, 40, 50))])
    with pytest.raises(RuntimeError, match="only 1 valid samples"):
        collect_calibration_samples(
            source,
            (3, 4, 5),
            1.0,
            monotonic=SequenceClock([0.0, 0.1, 1.0]),
        )


def test_collect_calibration_samples_rejects_missing_channels() -> None:
    source = SequenceSleeveSource([
        SleeveFrame(1.0, (10, 20, 30, 40)),
        SleeveFrame(2.0, (11, 21, 31, 41)),
    ])
    with pytest.raises(RuntimeError, match="only 0 valid samples"):
        collect_calibration_samples(
            source,
            (3, 4, 5),
            1.0,
            monotonic=SequenceClock([0.0, 0.1, 0.2, 1.0]),
        )


def test_calibrate_estimator_saves_then_resets(tmp_path: Path) -> None:
    events: list[object] = []

    class Calibration:
        def save(self, path: Path) -> None:
            events.append(("save", path))

    class Estimator:
        def calibrate(self, rows: np.ndarray):
            events.append(("calibrate", rows.tolist()))
            return Calibration()

        def reset(self) -> None:
            events.append("reset")

    output = tmp_path / "calibration.json"
    calibration = calibrate_estimator(
        Estimator(),
        np.asarray([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        output,
    )
    assert isinstance(calibration, Calibration)
    assert events == [
        ("calibrate", [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]),
        ("save", output),
        "reset",
    ]


def test_latest_flex_reader_returns_ch2_ch3_ch4_in_order() -> None:
    source = FakeSleeveSource()
    source.start()
    try:
        values = LatestFlexReader(source, (2, 3, 4))()
    finally:
        source.close()
    assert values == pytest.approx([1.0, 2.0, 3.0])


@dataclass
class Result:
    action: object
    action_probabilities: object
    angle_deg: object


class MockModel:
    def __init__(self, results: list[Result]) -> None:
        self.results = iter(results)
        self.calls: list[dict[str, object]] = []

    def predict_raw(self, **kwargs):
        self.calls.append(kwargs)
        return next(self.results)


def sample(ch2: float = 20.0, ch3: float = 30.0, ch4: float = 40.0) -> SensorSample:
    return SensorSample(1.0, SleeveFrame(1.0, (10.0, ch2, ch3, ch4, 50.0)))


@pytest.mark.parametrize(
    ("action", "flexion", "abduction"),
    (
        (0, math.pi / 2, 0.0),
        (1, 0.0, math.pi / 2),
        (2, -math.pi / 2, 0.0),
    ),
)
def test_action_mapping_and_exact_model_inputs(action: int, flexion: float, abduction: float) -> None:
    model = MockModel([Result(action, (0.7, 0.2, 0.1), 90.0)])
    predictor = FlexModelPredictor(model_config(), model)
    intent = predictor.predict(sample())
    assert intent.action is ArmAction(action)
    assert intent.shoulder_flexion_rad == pytest.approx(flexion)
    assert intent.shoulder_abduction_rad == pytest.approx(abduction)
    assert model.calls == [{
        "flex": [20.0, 30.0, 40.0],
        "calibration_baseline": [1.0, 2.0, 3.0],
        "calibration_scale": [4.0, 5.0, 6.0],
        "trial_rest": [7.0, 8.0, 9.0],
    }]


@pytest.mark.parametrize(
    ("label", "action"),
    (
        ("Forward", ArmAction.FORWARD),
        ("Lateral", ArmAction.LATERAL),
        ("Backward", ArmAction.BACKWARD),
    ),
)
def test_model_string_action_labels(label: str, action: ArmAction) -> None:
    result = Result(label, (0.7, 0.2, 0.1), 10.0)
    assert FlexModelPredictor(model_config(), MockModel([result])).predict(sample()).action is action


def test_model_labeled_probability_mapping() -> None:
    result = Result("Backward", {"Forward": 0.1, "Lateral": 0.2, "Backward": 0.7}, 10.0)
    intent = FlexModelPredictor(model_config(), MockModel([result])).predict(sample())
    assert intent.action_probabilities == pytest.approx((0.1, 0.2, 0.7))
    assert intent.confidence == pytest.approx(0.7)


def test_mapper_uses_absolute_shoulder_angles_not_startup_offsets() -> None:
    startup = {"shoulder_flexion": 1.0, "shoulder_abduction": -0.5, "elbow_flexion": 0.25}
    mapper = ArmMapper(load_phase3_config().elbow, startup)
    targets = mapper.map(MotionIntent(
        timestamp=1.0,
        elbow_flexion=0.5,
        shoulder_flexion_rad=0.4,
        shoulder_abduction_rad=0.2,
    ))
    assert targets["shoulder_flexion"] == pytest.approx(0.4)
    assert targets["shoulder_abduction"] == pytest.approx(0.2)


def test_manual_backward_model_output_uses_absolute_joint_semantics() -> None:
    intent = manual_intent(ArmAction.BACKWARD, shoulder_angle_deg=30.0, elbow_angle_deg=90.0)
    assert intent.shoulder_flexion_rad == pytest.approx(math.radians(-30.0))
    assert intent.shoulder_abduction_rad == 0.0
    assert intent.elbow_flexion == pytest.approx(math.radians(90.0))


def test_low_confidence_is_recorded_but_not_filtered() -> None:
    model = MockModel([Result(1, (0.49, 0.02, 0.49), 10.0)])
    intent = FlexModelPredictor(model_config(min_action_confidence=0.9), model).predict(sample())
    assert intent.action is ArmAction.LATERAL
    assert intent.confidence == pytest.approx(0.02)
    assert intent.shoulder_abduction_rad == pytest.approx(math.radians(10))


def test_model_instance_is_reused_across_predictions() -> None:
    model = MockModel([
        Result(0, (0.8, 0.1, 0.1), 10.0),
        Result(0, (0.8, 0.1, 0.1), 11.0),
    ])
    predictor = FlexModelPredictor(model_config(), model)
    predictor.predict(sample())
    predictor.predict(sample())
    assert len(model.calls) == 2


def test_action_transition_requires_consecutive_frames() -> None:
    results = [
        Result(0, (0.8, 0.1, 0.1), 20.0),
        Result(1, (0.1, 0.8, 0.1), 30.0),
        Result(0, (0.8, 0.1, 0.1), 21.0),
        Result(1, (0.1, 0.8, 0.1), 30.0),
        Result(1, (0.1, 0.8, 0.1), 31.0),
        Result(1, (0.1, 0.8, 0.1), 32.0),
    ]
    predictor = FlexModelPredictor(model_config(), MockModel(results))
    actions = [predictor.predict(sample()).action for _ in results]
    assert actions == [ArmAction.FORWARD] * 5 + [ArmAction.LATERAL]


@pytest.mark.parametrize(
    "result",
    (
        Result(99, (0.8, 0.1, 0.1), 10.0),
        Result("unknown", (0.8, 0.1, 0.1), 10.0),
        Result(1.5, (0.8, 0.1, 0.1), 10.0),
        Result(0, (0.8, 0.1), 10.0),
        Result(0, (math.nan, 0.1, 0.1), 10.0),
        Result(0, (0.8, 0.1, 0.1), math.nan),
        Result(0, (0.8, 0.1, 0.1), math.inf),
        Result(0, (0.8, 0.1, 0.1), 181.0),
    ),
)
def test_invalid_model_output_is_rejected(result: Result) -> None:
    with pytest.raises(ValueError):
        FlexModelPredictor(model_config(), MockModel([result])).predict(sample())


def test_three_dof_mock_model_to_fake_robot_through_safety() -> None:
    phase3 = load_phase3_config().elbow
    elbow_config = replace(
        phase3, input_min=0.0, input_max=40.0,
        angle_min_deg=0.0, angle_max_deg=90.0,
        filter=replace(phase3.filter, type="none", alpha=None),
    )
    shoulder = FlexModelPredictor(model_config(required_consecutive_frames=1), MockModel([
        Result(1, (0.1, 0.8, 0.1), 30.0)
    ]))
    predictor = ArmMotionPredictor(RuleBasedPredictor(elbow_config), shoulder)
    robot_config = load_robot_config()
    robot = FakeRobotArm(robot_config)
    controller = SafeArmController(robot, robot_config)
    controller.connect()
    startup = {name: state.position for name, state in controller.read_joint_states().items()}
    mapper = ArmMapper(elbow_config, startup)

    controller.enable()
    intent = predictor.predict(sample(ch2=40.0))
    safe = controller.set_joint_positions(mapper.map(intent), dt=0.05)
    controller.shutdown()

    assert safe["shoulder_flexion"] == 0.0
    # The model angle is absolute; Phase 1's one-degree step cap approaches it safely.
    assert safe["shoulder_abduction"] == pytest.approx(math.radians(1))
    assert safe["elbow_flexion"] == pytest.approx(math.radians(1))
    assert robot.events == ["connect", "enable", "disable", "close"]

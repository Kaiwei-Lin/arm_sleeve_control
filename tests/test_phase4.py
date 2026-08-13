from __future__ import annotations

import math
import json
from dataclasses import dataclass, replace

import pytest

from sleeve_arm.config import FlexModelConfig, load_phase3_config, load_phase4_config, load_robot_config
from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.domain import ArmAction, SensorSample, SleeveFrame
from sleeve_arm.predictor import ArmMotionPredictor, FlexModelPredictor, RuleBasedPredictor
from sleeve_arm.robot import FakeRobotArm
from sleeve_arm.sources import FakeSleeveSource
from tools.calibrate_flex_model import LatestFlexReader


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


def test_default_real_model_config_rejects_missing_calibration() -> None:
    with pytest.raises(ValueError, match="max_deg|three non-null"):
        load_phase4_config()


def test_missing_model_module_has_clear_error() -> None:
    with pytest.raises(RuntimeError, match="Expected module: module_that_does_not_exist"):
        FlexModelPredictor(model_config(model_module="module_that_does_not_exist"))


def test_phase4_loads_generated_calibration_file(tmp_path) -> None:
    calibration = tmp_path / "flex_calibration.json"
    calibration.write_text(json.dumps({
        "calibration_baseline": [1, 2, 3],
        "calibration_scale": [4, 5, 6],
        "trial_rest": [7, 8, 9],
    }), encoding="utf-8")
    config = tmp_path / "phase4.yaml"
    config.write_text(f"""
predictor:
  backend: flex_model
  model_module: flex_model_0003
  model_class: FlexPredictor
  sleeve_channels: [2, 3, 4]
  calibration_file: {calibration.as_posix()}
  min_action_confidence: 0.6
  action_stability: {{required_consecutive_frames: 3}}
  angle: {{min_deg: 0, max_deg: 90}}
phase4_validation:
  limited_motion: true
  shoulder_flexion_max_delta_deg: 5
  shoulder_abduction_max_delta_deg: 5
  max_consecutive_prediction_errors: 3
""", encoding="utf-8")
    loaded = load_phase4_config(config)
    assert loaded.flex_model is not None
    assert loaded.flex_model.baseline == (1.0, 2.0, 3.0)
    assert loaded.flex_model.scale == (4.0, 5.0, 6.0)
    assert loaded.flex_model.trial_rest == (7.0, 8.0, 9.0)


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
    elbow_config = replace(phase3, input_min=0.0, input_max=40.0, filter=replace(phase3.filter, type="none", alpha=None))
    shoulder = FlexModelPredictor(model_config(required_consecutive_frames=1), MockModel([
        Result(1, (0.1, 0.8, 0.1), 30.0)
    ]))
    predictor = ArmMotionPredictor(RuleBasedPredictor(elbow_config), shoulder)
    robot_config = load_robot_config()
    robot = FakeRobotArm(robot_config)
    controller = SafeArmController(robot, robot_config)
    controller.connect()
    startup = {name: state.position for name, state in controller.read_joint_states().items()}
    mapper = ArmMapper(elbow_config, startup, 5.0, 5.0)

    controller.enable()
    intent = predictor.predict(sample(ch2=40.0))
    safe = controller.set_joint_positions(mapper.map(intent), dt=0.05)
    controller.shutdown()

    assert safe["shoulder_flexion"] == 0.0
    assert safe["shoulder_abduction"] == pytest.approx(math.radians(1))
    assert safe["elbow_flexion"] == pytest.approx(math.radians(1))
    assert robot.events == ["connect", "enable", "disable", "close"]

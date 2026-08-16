from __future__ import annotations

import inspect
import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from sleeve_arm.config import (
    FlexModelConfig,
    load_phase3_config,
    load_phase4_config,
    load_robot_config,
)
from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.domain import ArmAction, ImuFrame, MotionIntent, SensorSample, SleeveFrame
from sleeve_arm.predictor import (
    ArmMotionPredictor,
    DualImuShoulderPredictor,
    FlexModelPredictor,
    RuleBasedPredictor,
    calibrate_dual_imu_estimator,
)
from sleeve_arm.predictor.calibration import calibrate_estimator, collect_calibration_samples
from sleeve_arm.robot import FakeRobotArm
from sleeve_arm.sync import SensorSynchronizer
from tools.debug_model_mapping import manual_intent
from tools import run_manual_model_control, run_model_control
from tools.run_model_control import _add_latest_pair, prepare_flexarm_predictor
from tools.test_flex_model import replay


def model_config(**changes) -> FlexModelConfig:
    values = {
        "model_dir": Path("unused-models"),
        "sleeve_channels": (3, 4, 5),
        "calibration_file": Path("unused-calibration.json"),
        "calibration_seconds": 3.0,
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


def test_default_phase4_uses_dual_imu_without_flex_model_artifacts() -> None:
    loaded = load_phase4_config()
    assert loaded.predictor_backend == "dual_imu"
    assert loaded.flex_model is None


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


class RecordingPredictor:
    def __init__(self) -> None:
        self.calibration_rows: np.ndarray | None = None
        self.calibration_output: Path | None = None
        self.reused_path: Path | None = None

    def calibrate(self, rows: np.ndarray, output: Path) -> SimpleNamespace:
        self.calibration_rows = rows
        self.calibration_output = output
        return SimpleNamespace(
            baseline=(30.5, 40.5, 50.5),
            scale=(1.0, 1.0, 1.0),
            sample_count=len(rows),
        )

    def reuse_calibration(self, path: Path) -> None:
        self.reused_path = path


def test_prepare_predictor_live_calibrates_by_default(tmp_path: Path) -> None:
    config = model_config(
        calibration_file=tmp_path / "configured.json",
        calibration_seconds=1.0,
    )
    predictor = RecordingPredictor()
    messages: list[str] = []
    prepared = prepare_flexarm_predictor(
        SequenceSleeveSource([
            SleeveFrame(1.0, (10, 20, 30, 40, 50)),
            SleeveFrame(2.0, (11, 21, 31, 41, 51)),
        ]),
        config,
        predictor=predictor,
        reuse_calibration=False,
        calibration_seconds=0.5,
        calibration_output=tmp_path / "override.json",
        input_fn=lambda _: "",
        print_fn=messages.append,
        monotonic=SequenceClock([0.0, 0.1, 0.2, 0.5]),
    )
    assert prepared is predictor
    assert predictor.calibration_rows is not None
    assert predictor.calibration_rows.tolist() == [
        [30.0, 40.0, 50.0],
        [31.0, 41.0, 51.0],
    ]
    assert predictor.calibration_output == tmp_path / "override.json"
    assert predictor.reused_path is None
    assert any("Calibration complete" in message for message in messages)


def test_prepare_predictor_reuses_only_when_explicit(tmp_path: Path) -> None:
    path = tmp_path / "saved.json"
    predictor = RecordingPredictor()
    prepared = prepare_flexarm_predictor(
        SequenceSleeveSource([]),
        model_config(calibration_file=path),
        predictor=predictor,
        reuse_calibration=True,
        input_fn=lambda _: pytest.fail("reuse must not prompt"),
        print_fn=lambda _: None,
    )
    assert prepared is predictor
    assert predictor.reused_path == path
    assert predictor.calibration_rows is None


@dataclass
class EstimatorResult:
    action: object
    angle_deg: object
    action_confidence: object
    angle_confidence: object
    moving: object


class FakeEstimator:
    def __init__(self, results: list[EstimatorResult]) -> None:
        self.results = iter(results)
        self.calls: list[dict[str, object]] = []
        self.reset_calls = 0

    def update(self, **kwargs):
        self.calls.append(kwargs)
        return next(self.results)

    def reset(self) -> None:
        self.reset_calls += 1


def sample(
    ch2: float = 20.0,
    ch3: float = 30.0,
    ch4: float = 40.0,
    ch5: float = 50.0,
    *,
    timestamp: float = 1.0,
) -> SensorSample:
    return SensorSample(timestamp, SleeveFrame(timestamp, (10.0, ch2, ch3, ch4, ch5, 60.0)))


@pytest.mark.parametrize(
    ("label", "action", "flexion", "abduction"),
    (
        ("Forward", ArmAction.FORWARD, math.pi / 2, 0.0),
        ("Lateral", ArmAction.LATERAL, 0.0, math.pi / 2),
        ("Backward", ArmAction.BACKWARD, -math.pi / 2, 0.0),
    ),
)
def test_action_mapping_and_exact_model_inputs(
    label: str,
    action: ArmAction,
    flexion: float,
    abduction: float,
) -> None:
    estimator = FakeEstimator([EstimatorResult(label, 90.0, 0.8, 0.7, True)])
    predictor = FlexModelPredictor(model_config(), estimator=estimator)
    intent = predictor.predict(sample(timestamp=1.25))
    assert intent.action is action
    assert intent.shoulder_flexion_rad == pytest.approx(flexion)
    assert intent.shoulder_abduction_rad == pytest.approx(abduction)
    assert estimator.calls == [{
        "flex1": 30.0,
        "flex2": 40.0,
        "flex3": 50.0,
        "timestamp_ns": 1_250_000_000,
    }]
    assert intent.model_action == label
    assert intent.confidence == pytest.approx(0.8)
    assert intent.angle_confidence == pytest.approx(0.7)
    assert intent.moving is True


def test_offline_replay_preserves_ch3_ch4_ch5_positions(tmp_path: Path) -> None:
    recording = tmp_path / "samples.csv"
    recording.write_text(
        "timestamp,sleeve_ch_3,sleeve_ch_4,sleeve_ch_5\n"
        "1.25,30,40,50\n",
        encoding="utf-8",
    )
    estimator = FakeEstimator([
        EstimatorResult("Unknown", 0.0, 0.0, 0.0, False)
    ])
    predictor = FlexModelPredictor(model_config(), estimator=estimator)

    assert replay(recording, predictor) == 0
    assert estimator.calls == [{
        "flex1": 30.0,
        "flex2": 40.0,
        "flex3": 50.0,
        "timestamp_ns": 1_250_000_000,
    }]


def test_mapper_uses_absolute_shoulder_angles_not_startup_offsets() -> None:
    startup = {
        "shoulder_flexion": 1.0,
        "shoulder_abduction": -0.5,
        "elbow_flexion": 0.25,
        "upper_arm_rotation": 0.75,
    }
    mapper = ArmMapper(load_phase3_config().elbow, startup)
    targets = mapper.map(MotionIntent(
        timestamp=1.0,
        elbow_flexion=0.5,
        shoulder_flexion_rad=0.4,
        shoulder_abduction_rad=0.2,
    ))
    assert targets["shoulder_flexion"] == pytest.approx(0.4)
    assert targets["shoulder_abduction"] == pytest.approx(0.2)
    assert targets["upper_arm_rotation"] == pytest.approx(0.75)


def test_manual_backward_model_output_uses_absolute_joint_semantics() -> None:
    intent = manual_intent(ArmAction.BACKWARD, shoulder_angle_deg=30.0, elbow_angle_deg=90.0)
    assert intent.shoulder_flexion_rad == pytest.approx(math.radians(-30.0))
    assert intent.shoulder_abduction_rad == 0.0
    assert intent.elbow_flexion == pytest.approx(math.radians(90.0))


def test_manual_runtime_accepts_absolute_upper_arm_rotation(monkeypatch, capsys) -> None:
    monkeypatch.setattr(sys, "argv", [
        "run_manual_model_control.py",
        "--action", "Forward",
        "--shoulder-angle-deg", "5",
        "--elbow-angle-deg", "30",
        "--upper-arm-rotation-deg", "10",
    ])

    assert run_manual_model_control.main() == 0
    output = capsys.readouterr().out
    assert "upper_arm_rotation=10.000 deg absolute" in output
    assert "upper_arm_rotation (motor 24, CAN 2)" in output
    assert "mapper target:      0.174533 rad" in output


def test_robot_connect_precedes_sensor_and_external_model_initialization() -> None:
    runtime = inspect.getsource(run_model_control.main)
    assert runtime.index("controller.connect()") < runtime.index("source.start()")
    assert runtime.index("controller.connect()") < runtime.index("prepare_dual_imu_estimator(")
    assert "prepare_flexarm_predictor(" not in runtime


def test_dual_imu_predictor_lives_in_reusable_predictor_package() -> None:
    assert DualImuShoulderPredictor.__module__ == "sleeve_arm.predictor.dual_imu_shoulder"


def imu_frame(timestamp: float, quaternion: tuple[float, float, float, float]) -> ImuFrame:
    return ImuFrame(
        timestamp=timestamp,
        accel_x=0.0,
        accel_y=0.0,
        accel_z=9.81,
        gyro_x=0.0,
        gyro_y=0.0,
        gyro_z=0.0,
        quat_w=quaternion[0],
        quat_x=quaternion[1],
        quat_y=quaternion[2],
        quat_z=quaternion[3],
    )


def test_four_imus_route_to_independent_shoulder_and_rotation_pairs() -> None:
    identity = (1.0, 0.0, 0.0, 0.0)
    frames = {
        "imu1": imu_frame(1.000, identity),
        "imu2": imu_frame(1.005, identity),
        "imu3": imu_frame(2.000, identity),
        "imu4": imu_frame(2.005, identity),
    }
    sources = {
        name: SimpleNamespace(latest=lambda frame=frame: frame)
        for name, frame in frames.items()
    }
    shoulder_sync = SensorSynchronizer(20.0, 500.0, True, True)
    rotation_sync = SensorSynchronizer(20.0, 500.0, True, True)

    _add_latest_pair(shoulder_sync, sources, ("imu1", "imu2"))
    _add_latest_pair(rotation_sync, sources, ("imu3", "imu4"))

    assert shoulder_sync.latest_imu_pair(20.0) == (frames["imu1"], frames["imu2"])
    assert rotation_sync.latest_imu_pair(20.0) == (frames["imu3"], frames["imu4"])


class RecordingDualImuEstimator:
    def __init__(self, results: list[SimpleNamespace] | None = None) -> None:
        self.results = iter(results or [])
        self.calibration_calls: list[tuple[list[tuple[float, ...]], list[tuple[float, ...]]]] = []
        self.update_calls: list[tuple[tuple[float, ...], tuple[float, ...]]] = []

    def calibrate(self, chest_rest_samples, arm_rest_samples):
        self.calibration_calls.append((list(chest_rest_samples), list(arm_rest_samples)))
        return SimpleNamespace(sample_count=len(chest_rest_samples))

    def update(self, chest_q, arm_q):
        self.update_calls.append((tuple(chest_q), tuple(arm_q)))
        return next(self.results)


def dual_imu_sample(
    timestamp: float,
    chest_q: tuple[float, float, float, float],
    arm_q: tuple[float, float, float, float],
) -> SensorSample:
    return SensorSample(
        timestamp,
        SleeveFrame(timestamp, (10.0, 20.0, 30.0, 40.0, 50.0)),
        imu1=imu_frame(timestamp, arm_q),
        imu2=imu_frame(timestamp, chest_q),
    )


def test_dual_imu_calibration_passes_chest_then_arm_wxyz_samples() -> None:
    chest = ((1.0, 0.0, 0.0, 0.0), (0.999, 0.001, 0.0, 0.0))
    arm = ((0.98, 0.2, 0.0, 0.0), (0.97, 0.24, 0.0, 0.0))
    pairs = [
        (imu_frame(float(index), arm_q), imu_frame(float(index), chest_q))
        for index, (chest_q, arm_q) in enumerate(zip(chest, arm), start=1)
    ]
    estimator = RecordingDualImuEstimator()

    calibration = calibrate_dual_imu_estimator(estimator, pairs)

    assert calibration.sample_count == 2
    assert estimator.calibration_calls == [(list(chest), list(arm))]


@pytest.mark.parametrize(
    ("direction", "magnitude", "action", "flexion", "abduction"),
    (
        ("Forward", 30.0, ArmAction.FORWARD, math.radians(30.0), 0.0),
        ("Backward", 20.0, ArmAction.BACKWARD, math.radians(-20.0), 0.0),
        ("Lateral", 45.0, ArmAction.LATERAL, 0.0, math.radians(45.0)),
        ("Rest", 4.0, None, 0.0, 0.0),
    ),
)
def test_dual_imu_update_uses_chest_arm_order_and_maps_direction_magnitude(
    direction: str,
    magnitude: float,
    action: ArmAction | None,
    flexion: float,
    abduction: float,
) -> None:
    chest_q = (1.0, 0.0, 0.0, 0.0)
    arm_q = (0.9238795, 0.3826834, 0.0, 0.0)
    estimator = RecordingDualImuEstimator([
        SimpleNamespace(direction=direction, magnitude_deg=magnitude, confidence=0.8)
    ])
    predictor = DualImuShoulderPredictor(
        estimator,
        max_age_s=None,
    )

    intent = predictor.predict(dual_imu_sample(1.0, chest_q, arm_q))

    assert estimator.update_calls == [(chest_q, arm_q)]
    assert intent.action is action
    assert intent.model_action == direction
    assert intent.angle_deg == pytest.approx(magnitude)
    assert intent.confidence == pytest.approx(0.8)
    assert intent.shoulder_flexion_rad == pytest.approx(flexion)
    assert intent.shoulder_abduction_rad == pytest.approx(abduction)


def test_dual_imu_transition_holds_last_unambiguous_shoulder_target() -> None:
    estimator = RecordingDualImuEstimator([
        SimpleNamespace(direction="Lateral", magnitude_deg=30.0, confidence=0.9),
        SimpleNamespace(direction="Transition", magnitude_deg=35.0, confidence=0.4),
    ])
    predictor = DualImuShoulderPredictor(
        estimator,
        max_age_s=None,
    )
    neutral = (1.0, 0.0, 0.0, 0.0)

    active = predictor.predict(dual_imu_sample(1.0, neutral, neutral))
    transition = predictor.predict(dual_imu_sample(2.0, neutral, neutral))

    assert transition.model_action == "Transition"
    assert transition.angle_deg == pytest.approx(35.0)
    assert transition.shoulder_flexion_rad == active.shoulder_flexion_rad
    assert transition.shoulder_abduction_rad == active.shoulder_abduction_rad


def test_model_instance_is_reused_across_predictions() -> None:
    estimator = FakeEstimator([
        EstimatorResult("Forward", 10.0, 0.8, 0.7, True),
        EstimatorResult("Forward", 11.0, 0.8, 0.7, True),
    ])
    predictor = FlexModelPredictor(model_config(), estimator=estimator)
    predictor.predict(sample())
    predictor.predict(sample(timestamp=1.1))
    assert len(estimator.calls) == 2


def test_rest_holds_last_active_shoulder_target() -> None:
    predictor = FlexModelPredictor(model_config(), estimator=FakeEstimator([
        EstimatorResult("Lateral", 30.0, 0.8, 0.7, True),
        EstimatorResult("Rest", 0.0, 1.0, 1.0, False),
    ]))
    active = predictor.predict(sample(timestamp=1.0))
    resting = predictor.predict(sample(timestamp=1.1))
    assert resting.action is None
    assert resting.model_action == "Rest"
    assert resting.shoulder_flexion_rad == active.shoulder_flexion_rad
    assert resting.shoulder_abduction_rad == active.shoulder_abduction_rad


@pytest.mark.parametrize("state", ("Rest", "Unknown"))
def test_non_active_before_first_action_leaves_shoulders_unset(state: str) -> None:
    intent = FlexModelPredictor(
        model_config(),
        estimator=FakeEstimator([EstimatorResult(state, 0.0, 0.0, 0.0, False)]),
    ).predict(sample())
    assert intent.action is None
    assert intent.shoulder_flexion_rad is None
    assert intent.shoulder_abduction_rad is None


def test_predictor_reset_clears_estimator_and_retained_targets() -> None:
    estimator = FakeEstimator([
        EstimatorResult("Forward", 30.0, 0.8, 0.7, True),
        EstimatorResult("Rest", 0.0, 1.0, 1.0, False),
    ])
    predictor = FlexModelPredictor(model_config(), estimator=estimator)
    predictor.predict(sample())
    predictor.reset()
    intent = predictor.predict(sample(timestamp=1.1))
    assert estimator.reset_calls == 1
    assert intent.shoulder_flexion_rad is None
    assert intent.shoulder_abduction_rad is None


def test_predictor_reuses_saved_calibration_with_loaded_artifacts(
    tmp_path: Path,
    monkeypatch,
) -> None:
    old_calibration = object()
    new_calibration = object()

    @dataclass(frozen=True)
    class Artifacts:
        default_calibration: object
        marker: str = "trained-models"

    class ReloadableEstimator:
        instances: list["ReloadableEstimator"] = []

        def __init__(self, artifacts: Artifacts) -> None:
            self.artifacts = artifacts
            self.instances.append(self)

        def reset(self) -> None:
            pass

    loaded_paths: list[Path] = []

    class FlexCalibration:
        @classmethod
        def load(cls, path: Path):
            loaded_paths.append(path)
            return new_calibration

    monkeypatch.setitem(
        sys.modules,
        "flexarm.calibration",
        SimpleNamespace(FlexCalibration=FlexCalibration),
    )
    estimator = ReloadableEstimator(Artifacts(old_calibration))
    predictor = FlexModelPredictor(model_config(), estimator=estimator)
    path = tmp_path / "saved.json"

    predictor.reuse_calibration(path)

    assert loaded_paths == [path]
    assert len(ReloadableEstimator.instances) == 2
    assert ReloadableEstimator.instances[-1].artifacts == Artifacts(new_calibration)


@pytest.mark.parametrize(
    "result",
    (
        EstimatorResult("Turning", 10.0, 0.8, 0.7, True),
        EstimatorResult("Forward", math.nan, 0.8, 0.7, True),
        EstimatorResult("Forward", math.inf, 0.8, 0.7, True),
        EstimatorResult("Forward", 181.0, 0.8, 0.7, True),
        EstimatorResult("Forward", 10.0, math.nan, 0.7, True),
        EstimatorResult("Forward", 10.0, -0.1, 0.7, True),
        EstimatorResult("Forward", 10.0, 0.8, 1.1, True),
        EstimatorResult("Forward", 10.0, 0.8, 0.7, 1),
    ),
)
def test_invalid_model_output_is_rejected(result: EstimatorResult) -> None:
    with pytest.raises(ValueError):
        FlexModelPredictor(model_config(), estimator=FakeEstimator([result])).predict(sample())


def test_three_dof_mock_model_to_fake_robot_through_safety() -> None:
    phase3 = load_phase3_config().elbow
    elbow_config = replace(
        phase3, sleeve_channel=2, input_min=0.0, input_max=40.0,
        angle_min_deg=0.0, angle_max_deg=90.0,
        filter=replace(phase3.filter, type="none", alpha=None),
    )
    shoulder = FlexModelPredictor(model_config(), estimator=FakeEstimator([
        EstimatorResult("Lateral", 30.0, 0.8, 0.7, True)
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
    assert safe["shoulder_abduction"] == pytest.approx(math.radians(30))
    assert safe["elbow_flexion"] == pytest.approx(math.radians(90))
    assert robot.events == ["connect", "enable", "disable", "close"]

"""Golden data captured from the pre-refactor working-tree prediction block."""
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import sys
import subprocess
from types import SimpleNamespace

import pytest

from sleeve_arm.config import load_phase3_config, load_phase4_config, load_sensor_config
from sleeve_arm.control.mapper import ArmMapper, AuroraIntentMapper
from sleeve_arm.domain import ImuFrame, SensorSample, SleeveFrame
from sleeve_arm.estimation import UpperArmRotationEstimator, quaternion_from_axis_angle
from sleeve_arm.predictor import DualImuShoulderPredictor, FlexModelPredictor, RuleBasedPredictor
from sleeve_arm.robot.aurora import AuroraRobotArm
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.runtime.configuration import ModelControlConfigs
from sleeve_arm.runtime import intent_pipeline as module
from sleeve_arm.runtime.intent_pipeline import IntentPipeline, RotationPredictionError
from sleeve_arm.runtime.sensors import SensorRuntime, _require_fresh_pair

GOLDEN = json.loads((Path(__file__).parent / "fixtures/model_control_golden.json").read_text())


def frame(timestamp, degrees=0):
    q = quaternion_from_axis_angle((1, 0, 0), math.radians(degrees))
    return ImuFrame(timestamp, 0, 0, 0, 0, 0, 0, *q)


def configs(backend="dual_imu"):
    phase3 = load_phase3_config()
    elbow = replace(phase3.elbow, input_min=0., input_max=2., angle_min_deg=0., angle_max_deg=90.)
    return ModelControlConfigs(phase3, load_phase4_config(), load_sensor_config(),
                               elbow=elbow, shoulder_backend=backend)


class ScriptedEstimator:
    def __init__(self, results):
        self.results = iter(results)
        self.calls = []

    def update(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return SimpleNamespace(**next(self.results))


@pytest.mark.parametrize("backend", ["dual_imu", "flexarm_estimator"])
def test_motion_intent_and_semantic_targets_match_legacy_golden(monkeypatch, backend):
    config = configs(backend)
    case = GOLDEN["cases"][backend]
    estimator = ScriptedEstimator(case["estimator_results"])
    pipeline = IntentPipeline(config)
    pipeline.sensors = SimpleNamespace(imu_error=None, rotation_pair=None)
    pipeline.shoulder = (DualImuShoulderPredictor(estimator, .2) if backend == "dual_imu"
                         else FlexModelPredictor(config.phase4.flex_model, estimator=estimator))
    pipeline.elbow = RuleBasedPredictor(config.elbow)
    pipeline.rotation = UpperArmRotationEstimator(.35, 20., "x")
    pipeline.rotation.calibrate([(frame(99), frame(99)), (frame(99.01), frame(99.01))])
    pipeline.last_rotation_rad = 0.
    mapper = ArmMapper(config.elbow, GOLDEN["startup"])
    robot = AuroraRobotArm(fake_session(profile=gr3_fake_profile()), sides=("right",))
    aurora_mapper = AuroraIntentMapper(robot, source_side="right", target_side="right")
    monkeypatch.setattr("time.perf_counter", lambda: 0.)
    for row, expected in zip(GOLDEN["inputs"], case["expected"]):
        now = row["timestamp"]
        monkeypatch.setattr("time.monotonic", lambda: now)
        pipeline.sensors.rotation_pair = (frame(now, row["upper_deg"]), frame(now, row["reference_deg"]))
        sample = SensorSample(now, SleeveFrame(now, (0, row["flex"], .3, .4, .5)), frame(now), frame(now))
        intent = pipeline.predict(sample)
        actual = asdict(intent)
        actual["action"] = None if intent.action is None else intent.action.name
        for name, value in expected["intent"].items():
            if isinstance(value, float):
                assert actual[name] == pytest.approx(value, abs=1e-12)
            else:
                assert actual[name] == value
        assert mapper.map(intent) == pytest.approx(expected["dymotor_targets"])
        assert aurora_mapper.map(intent) == pytest.approx(expected["aurora_targets"])
    if backend == "flexarm_estimator":
        assert all(set(kwargs) == {"flex1", "flex2", "flex3", "timestamp_ns"} for _, kwargs in estimator.calls)


def test_prepare_preserves_rest_forward_rotation_order_and_estimator_arguments(monkeypatch):
    config = configs()
    events = []
    pairs = [(frame(100, 10), frame(100, 5)), (frame(100.01, 12), frame(100.01, 6))]

    class Estimator:
        def __init__(self, **kwargs):
            events.append(("estimator", kwargs))

        def calibrate(self, chest, arm):
            events.append(("rest", chest, arm))
            return SimpleNamespace(sample_count=2)

        def calibrate_forward(self, chest, arm):
            events.append(("forward", chest, arm))

    def collect(sources, sensor_config, names, pair_config, timeout, **kwargs):
        events.append(("collect", names, kwargs["calibration_seconds"]))
        return pairs

    monkeypatch.setitem(sys.modules, "sleeve_arm.estimation.flexarm", SimpleNamespace(DualImuArmEstimator=Estimator))
    monkeypatch.setattr(module, "_collect_imu_pairs", collect)
    sensors = SensorRuntime(config.sensors)
    monkeypatch.setattr(sensors, "reset_synchronization", lambda: events.append(("sync_reset",)))
    pipeline = IntentPipeline(config, calibration_seconds=.5, print_fn=lambda _: None)
    pipeline.prepare(sensors)
    assert [e[0] for e in events] == ["estimator", "collect", "rest", "collect", "forward", "collect", "sync_reset"]
    assert events[0][1] == dict(down_axis=(-1, 0, 0), forward_axis=(0, 0, 1), lateral_axis=(0, 1, 0),
                                rest_threshold_deg=5, dominance_ratio=1.1)
    assert events[1][1:] == (sensors.shoulder_names, .5)
    assert events[5][1:] == (sensors.rotation_names, .5)
    assert events[2][1][0] == pytest.approx((pairs[0][1].quat_w, pairs[0][1].quat_x, 0, 0))
    assert events[2][2][0] == pytest.approx((pairs[0][0].quat_w, pairs[0][0].quat_x, 0, 0))
    assert pipeline.rotation.world.axis == config.sensors.upper_arm_rotation.twist_axis
    assert pipeline.rotation.world.ema_alpha == config.sensors.upper_arm_rotation.ema_alpha
    assert pipeline.last_rotation_rad == 0.


@pytest.mark.parametrize("reuse", [False, True])
def test_pipeline_forwards_flex_calibration_options_and_skips_shoulder_imus(monkeypatch, tmp_path, reuse):
    config = configs("flexarm_estimator")
    config = replace(config, sensors=replace(config.sensors, upper_arm_rotation=replace(config.sensors.upper_arm_rotation, enabled=False)))
    sensors = SensorRuntime(config.sensors, shoulder_backend=config.shoulder_backend)
    sensors.source = object()
    calls = []
    monkeypatch.setattr(module, "prepare_flexarm_predictor", lambda *a, **k: calls.append((a, k)) or object())
    pipeline = IntentPipeline(config, reuse_calibration=reuse, calibration_seconds=.4,
                              calibration_output=tmp_path / "calibration.json", print_fn=lambda _: None)
    pipeline.prepare(sensors)
    assert sensors.required_imus == ()
    assert calls[0][0] == (sensors.source, config.phase4.flex_model)
    assert calls[0][1]["reuse_calibration"] is reuse
    assert calls[0][1]["calibration_seconds"] == .4
    assert calls[0][1]["calibration_output"] == tmp_path / "calibration.json"


def test_calibration_collection_uses_unique_fresh_pairs_after_prompt(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    monkeypatch.setattr("time.sleep", clock.sleep)
    config = load_sensor_config()
    source = SimpleNamespace(latest=lambda: frame(math.floor(clock.now * 100) / 100))
    prompt_times = []

    def prompt(_):
        clock.sleep(.1)
        prompt_times.append(clock.now)

    pairs = module._collect_imu_pairs(
        {"imu1": source, "imu2": source}, config, ("imu1", "imu2"), config.shoulder_imu, .2,
        calibration_seconds=.04, prompt="test", status="test", input_fn=prompt, print_fn=lambda _: None,
    )
    keys = [(a.host_timestamp_ns, b.host_timestamp_ns) for a, b in pairs]
    assert len(keys) >= 2 and len(keys) == len(set(keys))
    assert all(a.timestamp >= prompt_times[0] and b.timestamp >= prompt_times[0] for a, b in pairs)


def test_calibration_rejects_stale_or_invalid_quaternions_before_prompt(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    monkeypatch.setattr("time.sleep", clock.sleep)
    config = load_sensor_config()
    pair_config = replace(config.shoulder_imu, startup_timeout_s=.01)
    source = SimpleNamespace(latest=lambda: frame(90))
    with pytest.raises(RuntimeError, match="no fresh valid"):
        module._collect_imu_pairs({"imu1": source, "imu2": source}, config, ("imu1", "imu2"), pair_config, .2,
                                  prompt="test", status="test", input_fn=lambda _: pytest.fail("prompt"))


def test_sensor_runtime_preserves_independent_pair_thresholds_and_freshness(monkeypatch):
    config = load_sensor_config()
    config = replace(config, shoulder_imu=replace(config.shoulder_imu, max_sync_ms=5),
                     upper_arm_rotation=replace(config.upper_arm_rotation, max_sync_ms=20))
    runtime = SensorRuntime(config)
    frames = {"imu1": frame(100), "imu2": frame(100.01), "imu3": frame(100), "imu4": frame(100.01)}
    runtime.imu_sources = {name: SimpleNamespace(latest=lambda name=name: frames[name]) for name in frames}
    runtime.source = SimpleNamespace(latest=lambda: SleeveFrame(100.02, (1, 2, 3, 4, 5)))
    sample = runtime.latest()
    assert sample.imu1 is sample.imu2 is None
    assert runtime.rotation_pair == (frames["imu3"], frames["imu4"])
    monkeypatch.setattr("time.monotonic", lambda: 101.)
    with pytest.raises(ValueError, match="stale"):
        _require_fresh_pair(runtime.rotation_pair, .2, "rotation")
    with pytest.raises(ValueError, match="synchronized"):
        _require_fresh_pair(None, .2, "rotation")


def test_rotation_failure_exposes_held_value_without_recomputing_other_predictors(monkeypatch):
    pipeline = IntentPipeline(configs())
    case = GOLDEN["cases"]["dual_imu"]
    pipeline.sensors = SimpleNamespace(imu_error=None, rotation_pair=None)
    pipeline.shoulder = DualImuShoulderPredictor(ScriptedEstimator(case["estimator_results"]), None)
    pipeline.elbow = RuleBasedPredictor(pipeline.configs.elbow)
    pipeline.rotation = UpperArmRotationEstimator()
    pipeline.last_rotation_rad = .123
    sample = SensorSample(100, SleeveFrame(100, (0, 0, 0, 0, 0)), frame(100), frame(100))
    with pytest.raises(RotationPredictionError) as caught:
        pipeline.predict(sample)
    assert caught.value.intent.upper_arm_rotation_rad == .123
    assert caught.value.intent.elbow_flexion == 0.
    assert pipeline.shoulder.last_result.direction == "Rest"


def test_sensor_close_attempts_every_source_even_after_failure():
    runtime = SensorRuntime(load_sensor_config())
    closed = []

    def close(name):
        closed.append(name)
        raise RuntimeError(name)

    runtime.source = SimpleNamespace(close=lambda: close("sleeve"))
    runtime.imu_sources = {name: SimpleNamespace(close=lambda name=name: close(name)) for name in ("imu1", "imu2")}
    with pytest.raises(RuntimeError, match="sleeve; imu2; imu1"):
        runtime.close()
    assert closed == ["sleeve", "imu2", "imu1"]


@pytest.mark.parametrize("backend", ["fake", "aurora-fake"])
def test_full_fake_cli_calibrates_real_dual_imu_estimator_without_hardware(backend):
    root = Path(__file__).resolve().parents[1]
    args = [sys.executable, "tools/run_model_control.py", "--robot", backend,
            "--sleeve", "fake", "--imus", "fake", "--duration", ".08", "--calibration-seconds", ".08"]
    if backend == "aurora-fake":
        args += ["--side", "right", "--source-side", "right"]
    result = subprocess.run(args, cwd=root, input="\n\n\n", capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "肩部前抬方向标定完成" in result.stdout
    assert "大臂旋转零位标定完成" in result.stdout

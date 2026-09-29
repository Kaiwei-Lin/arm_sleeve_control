"""Deterministic lifecycle/failure tests with no hardware or wall-clock sleeps."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from sleeve_arm.config import load_phase3_config, load_phase4_config
from sleeve_arm.domain import MotionIntent, SensorSample, SleeveFrame
from sleeve_arm.robot.aurora_fake import FakeClock
from sleeve_arm.runtime.intent_pipeline import RotationPredictionError
from sleeve_arm.runtime.model_control import ModelControlApp, RuntimeState


class FakeSensorRuntime:
    def __init__(self, clock, events):
        self.clock, self.events = clock, events
        self.reads = 0
        self.stats = SimpleNamespace(estimated_fps=100.)

    def start(self):
        self.events.append("sensor_start")

    def latest(self):
        self.reads += 1
        now = self.clock.monotonic()
        return SensorSample(now, SleeveFrame(now, (1., 2., 3., 4., 5.)))

    def close(self):
        self.events.append("sensor_close")


class FakeIntentPipeline:
    def __init__(self, events):
        self.events = events
        self.samples = []

    def prepare(self, sensors):
        self.events.append("intent_prepare")

    def predict(self, sample):
        self.samples.append(sample.timestamp)
        return MotionIntent(timestamp=sample.timestamp, shoulder_flexion_rad=.1)

    def diagnostics(self):
        return {}


class FakeRobotRuntime:
    fail_fast = False
    offline_preview = False

    def __init__(self, events):
        self.events = events
        self.commands = []
        self.startup = {"shoulder_flexion": 0.}

    def connect(self):
        self.events.append("robot_connect")

    def prepare(self):
        self.events.append("robot_prepare")
        return True

    def apply(self, intent, dt):
        assert isinstance(intent, MotionIntent)
        self.commands.append((intent, dt))
        return {"shoulder_flexion": intent.shoulder_flexion_rad}

    def check_health(self):
        self.events.append("health")

    def feedback(self):
        return {"shoulder_flexion": 0.}

    def shutdown(self):
        self.events.append("robot_shutdown")


def make_app(**kwargs):
    clock = FakeClock()
    events = []
    telemetry = SimpleNamespace(reports=[], report=lambda **kw: telemetry.reports.append(kw))
    app = ModelControlApp(
        sensors=FakeSensorRuntime(clock, events), intents=FakeIntentPipeline(events),
        robot=FakeRobotRuntime(events), phase3=load_phase3_config(), phase4=load_phase4_config(),
        clock=clock, telemetry=telemetry, duration=kwargs.pop("duration", .055), **kwargs,
    )
    return app, events


def test_new_samples_only_and_duration_stop_preserve_startup_order():
    app, events = make_app()
    assert app.run() == 0
    assert events[:4] == ["robot_connect", "sensor_start", "intent_prepare", "robot_prepare"]
    assert events[-2:] == ["robot_shutdown", "sensor_close"]
    assert len(app.intents.samples) == len(set(app.intents.samples))
    assert app.predictions == len(app.robot.commands) > 0
    assert [i.timestamp for i, _ in app.robot.commands] == app.intents.samples[1:]
    assert all(dt == 1 / app.phase3.control_hz for _, dt in app.robot.commands)
    assert app.clock.now >= 100.055 and app.clock.now < 100.07
    assert app.state is RuntimeState.STOPPING and app.fault is None


@pytest.mark.parametrize("between_samples", ["cached", "missing"])
def test_aurora_streams_fresh_target_at_control_rate_without_repeating_inference(monkeypatch, between_samples):
    app, _ = make_app(duration=.105)
    app.robot.stream_between_samples = True
    app.robot.fail_fast = True
    original = app.sensors.latest
    cached = None

    def latest():
        nonlocal cached
        if cached is None or app.clock.monotonic() - cached.timestamp >= .03 - 1e-9:
            cached = original()
            return cached
        return cached if between_samples == "cached" else None

    monkeypatch.setattr(app.sensors, "latest", latest)
    assert app.run() == 0
    assert app.predictions == 3
    assert len(app.intents.samples) == len(set(app.intents.samples)) == 4
    assert len(app.robot.commands) == 8
    assert all(dt == .01 for _, dt in app.robot.commands)


def test_aurora_stops_repeating_target_when_sensor_becomes_stale(monkeypatch):
    app, _ = make_app(duration=1.)
    app.robot.stream_between_samples = True
    app.robot.fail_fast = True
    original = app.sensors.latest
    first = original()
    final_sample = None
    submitted_at = []
    apply = app.robot.apply

    def latest():
        nonlocal final_sample
        if app.clock.monotonic() == first.timestamp:
            return first
        if final_sample is None:
            final_sample = original()
        return final_sample

    def record(intent, dt):
        submitted_at.append(app.clock.monotonic())
        return apply(intent, dt)

    monkeypatch.setattr(app.sensors, "latest", latest)
    monkeypatch.setattr(app.robot, "apply", record)
    assert app.run() == 1
    assert "sensor watchdog" in app.fault
    assert app.predictions == 1 and len(submitted_at) > 1
    assert all(now - final_sample.timestamp <= app.phase3.sensor_timeout_ms / 1000.
               for now in submitted_at)


@pytest.mark.parametrize("missing", [False, True])
@pytest.mark.parametrize("fail_fast", [False, True])
def test_stale_hold_then_hard_fault_even_when_source_returns_none(monkeypatch, missing, fail_fast):
    app, events = make_app(duration=2.)
    app.robot.fail_fast = fail_fast
    original = app.sensors.latest
    first = original()
    calls = 0

    def latest():
        nonlocal calls
        calls += 1
        return first if calls == 1 or not missing else None

    monkeypatch.setattr(app.sensors, "latest", latest)
    assert app.run() == 1
    assert not app.robot.commands
    assert "timeout" in app.fault or "watchdog" in app.fault
    assert app.stale > 0 if not fail_fast else app.stale == 0
    assert "health" in events and events[-2:] == ["robot_shutdown", "sensor_close"]


@pytest.mark.parametrize("rotation", [False, True])
@pytest.mark.parametrize("fail_fast", [False, True])
def test_prediction_error_policy_and_separate_counters(monkeypatch, rotation, fail_fast):
    app, _ = make_app(duration=.2)
    app.robot.fail_fast = fail_fast
    original = app.intents.predict
    calls = 0

    def predict(sample):
        nonlocal calls
        calls += 1
        intent = original(sample)
        if calls > 1:
            if rotation:
                raise RotationPredictionError(ValueError("rotation fault"), replace(intent, upper_arm_rotation_rad=.2))
            raise ValueError("shoulder fault")
        return intent

    monkeypatch.setattr(app.intents, "predict", predict)
    assert app.run() == 1
    count = 1 if fail_fast else app.phase4.max_consecutive_prediction_errors
    assert app.rotation_invalid == (count if rotation else 0)
    assert app.invalid == (0 if rotation else count)
    assert len(app.robot.commands) == (count - 1 if rotation else 0)
    assert all(intent.upper_arm_rotation_rad == .2 for intent, _ in app.robot.commands)


def test_prediction_error_recovers_and_resets_consecutive_count(monkeypatch):
    app, _ = make_app(duration=.075)
    original = app.intents.predict
    calls = 0

    def predict(sample):
        nonlocal calls
        calls += 1
        if calls in (2, 4):
            raise ValueError("one bad sample")
        return original(sample)

    monkeypatch.setattr(app.intents, "predict", predict)
    assert app.run() == 0
    assert app.invalid == 2 and app.consecutive_errors == 0
    assert app.predictions > 0


@pytest.mark.parametrize("component,method", [
    ("robot", "connect"), ("sensors", "start"), ("intents", "prepare"),
    ("robot", "prepare"), ("robot", "check_health"), ("robot", "apply"),
])
@pytest.mark.parametrize("error", [RuntimeError, KeyboardInterrupt])
def test_startup_and_robot_failures_or_ctrl_c_always_cleanup(monkeypatch, component, method, error):
    app, events = make_app()

    def fail(*args):
        raise error("injected failure")

    monkeypatch.setattr(getattr(app, component), method, fail)
    assert app.run() == (0 if error is KeyboardInterrupt else 1)
    assert events[-2:] == ["robot_shutdown", "sensor_close"]
    assert not app.robot.commands


def test_ctrl_c_in_prediction_is_not_counted_as_prediction_error(monkeypatch):
    app, events = make_app()
    monkeypatch.setattr(app.intents, "predict", lambda _: (_ for _ in ()).throw(KeyboardInterrupt()))
    assert app.run() == 0
    assert app.invalid == 0 and events[-2:] == ["robot_shutdown", "sensor_close"]


def test_cleanup_failures_are_independent(monkeypatch, capsys):
    app, events = make_app()

    def close(name):
        events.append(name)
        raise RuntimeError(name)

    monkeypatch.setattr(app.robot, "shutdown", lambda: close("robot_shutdown"))
    monkeypatch.setattr(app.sensors, "close", lambda: close("sensor_close"))
    assert app.run() == 0
    assert events[-2:] == ["robot_shutdown", "sensor_close"]
    assert "source close" in capsys.readouterr().err


def test_no_valid_prediction_during_readiness_faults_without_prepare_or_apply(monkeypatch):
    app, events = make_app()
    monkeypatch.setattr(app.intents, "predict", lambda _: (_ for _ in ()).throw(ValueError("invalid")))
    assert app.run() == 1
    assert "readiness timeout" in app.fault
    assert "robot_prepare" not in events and not app.robot.commands


def test_rotation_readiness_failure_cannot_arm_with_partial_intent(monkeypatch):
    app, events = make_app()

    def predict(sample):
        raise RotationPredictionError(ValueError("rotation not ready"), MotionIntent(sample.timestamp))

    monkeypatch.setattr(app.intents, "predict", predict)
    assert app.run() == 1
    assert "robot_prepare" not in events and not app.robot.commands


def test_slow_prediction_cannot_send_after_hard_timeout(monkeypatch):
    app, _ = make_app()
    original = app.intents.predict

    def predict(sample):
        if app.intents.samples:
            app.clock.sleep(1.1)
        return original(sample)

    monkeypatch.setattr(app.intents, "predict", predict)
    assert app.run() == 1
    assert "completed after Sleeve hard timeout" in app.fault and not app.robot.commands


def test_cancel_preparation_stops_cleanly(monkeypatch):
    app, events = make_app()
    monkeypatch.setattr(app.robot, "prepare", lambda: False)
    assert app.run() == 0
    assert not app.robot.commands and events[-2:] == ["robot_shutdown", "sensor_close"]


def test_telemetry_observes_without_driving_predictions():
    app, _ = make_app(duration=1.05)
    assert app.run() == 0
    assert len(app.telemetry.reports) == 1
    report = app.telemetry.reports[0]
    assert report["stats"]["predictions"] > 0
    assert report["targets"] == {"shoulder_flexion": .1}


@pytest.mark.parametrize("backend", ["dymotor", "aurora"])
def test_print_only_entry_runs_live_predictions_without_robot(monkeypatch, capsys, backend):
    from sleeve_arm.robot import factory
    from tools import run_model_control

    clock = FakeClock()
    events = []
    sensors = FakeSensorRuntime(clock, events)
    intents = FakeIntentPipeline(events)
    monkeypatch.setattr(run_model_control, "create_sensor_runtime", lambda *_: sensors)
    monkeypatch.setattr(run_model_control, "create_intent_pipeline", lambda *_: intents)
    monkeypatch.setattr(run_model_control, "ModelControlApp", lambda **kw: ModelControlApp(clock=clock, **kw))

    def forbidden(*args, **kwargs):
        pytest.fail("print-only attempted to create robot hardware")

    monkeypatch.setattr(factory, "create_robot", forbidden)
    assert run_model_control.main([
        "--print-only", "--robot", backend, "--side", "right", "--source-side", "right",
        "--duration", "2.05", "--sleeve", "fake", "--imus", "fake",
    ]) == 0
    output = capsys.readouterr().out
    assert output.count("[PRINT ONLY]") == 2
    assert "肩前屈/后伸=+5.7°" in output and "上臂旋转=未提供" in output
    assert "positions_rad=" not in output and "tracking_rad=" not in output
    assert len(intents.samples) > 100 and events == ["sensor_start", "intent_prepare", "sensor_close"]

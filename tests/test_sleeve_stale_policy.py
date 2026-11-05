"""Bounded Sleeve pauses must send no targets and preserve other fault policies."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from sleeve_arm.config import load_phase3_config
from test_model_control_app import make_app


def config_file(tmp_path, policy):
    path = Path(__file__).resolve().parents[1] / "configs/phase3.yaml"
    data = yaml.safe_load(path.read_text())
    if policy == "omitted":
        data["phase3"].pop("sleeve_stale_policy", None)
    else:
        data["phase3"]["sleeve_stale_policy"] = policy
    output = tmp_path / "phase3.yaml"
    output.write_text(yaml.safe_dump(data))
    return output


@pytest.mark.parametrize("policy,expected", [("omitted", "error"), ("error", "error"), ("pause", "pause")])
def test_sleeve_pause_is_explicit_and_defaults_to_previous_policy(tmp_path, policy, expected):
    assert load_phase3_config(config_file(tmp_path, policy)).sleeve_stale_policy == expected


@pytest.mark.parametrize("policy", [None, True, 1, {}, [], "hold_last", "ignore"])
def test_invalid_sleeve_policy_is_rejected(tmp_path, policy):
    with pytest.raises(ValueError, match="sleeve_stale_policy"):
        load_phase3_config(config_file(tmp_path, policy))


def paused_app(monkeypatch, *, missing=False, recover_after=.5, glove=False):
    app, events = make_app(duration=1.5)
    app.phase3 = replace(app.phase3, sleeve_stale_policy="pause")
    app.robot.fail_fast = app.robot.stream_between_samples = True
    if glove:
        app.glove = SimpleNamespace(
            validate_for_control=lambda: None, describe_startup=lambda _: None,
            start=lambda: None, wait_ready=lambda **_: None, close=lambda: None,
            targets=lambda: {"thumb_bend": .3}, report=lambda _: None,
        )
    latest = app.sensors.latest
    started = app.clock.monotonic()
    cached = None

    def read():
        nonlocal cached
        elapsed = app.clock.monotonic() - started
        if cached is None or elapsed < .02 or (recover_after is not None and elapsed >= recover_after):
            cached = latest()
            return cached
        return None if missing else cached

    monkeypatch.setattr(app.sensors, "latest", read)
    submitted = []
    apply = app.robot.apply

    def record(intent, dt, **kwargs):
        submitted.append((app.clock.monotonic() - started, intent.timestamp - started, dt))
        return apply(intent, dt)

    monkeypatch.setattr(app.robot, "apply", record)
    return app, events, submitted


@pytest.mark.parametrize("missing", [False, True])
@pytest.mark.parametrize("glove", [False, True])
@pytest.mark.parametrize("recover_after", [.28, .8])
def test_short_sleeve_gap_pauses_all_new_targets_then_resumes(monkeypatch, capsys, missing, glove, recover_after):
    app, events, sent = paused_app(monkeypatch, missing=missing, glove=glove, recover_after=recover_after)
    assert app.run() == 0 and app.fault is None
    assert app.stale > 0
    assert any(now < .2 for now, _, _ in sent)
    assert any(now >= recover_after for now, _, _ in sent)
    assert all(now - timestamp <= .2 for now, timestamp, _ in sent)
    assert all(dt == .01 for _, _, dt in sent)  # Resume with one normal limited step.
    assert events.count("health") == app.cycles
    output = capsys.readouterr().out
    assert output.count("pausing new targets") == 1
    assert output.count("Sleeve feedback recovered") == 1


@pytest.mark.parametrize("missing", [False, True])
def test_sleeve_pause_still_exits_after_one_second(monkeypatch, missing):
    app, events, sent = paused_app(monkeypatch, missing=missing, recover_after=None, glove=True)
    assert app.run() == 1
    assert "Sleeve hard timeout" in app.fault and "limit=1000 ms" in app.fault
    assert 101. < app.clock.monotonic() < 101.1
    assert all(now - timestamp <= .2 for now, timestamp, _ in sent)
    assert events[-2:] == ["robot_shutdown", "sensor_close"]


def test_sleeve_policy_does_not_change_imu_only_stale_behavior(monkeypatch):
    app, _, sent = paused_app(monkeypatch)
    app.sample_label = "IMU"
    assert app.run() == 1
    assert "sensor watchdog: stale/missing imu feedback" in app.fault
    assert app.clock.monotonic() < 100.3


@pytest.mark.parametrize("slow", ["prediction", "robot_health"])
def test_work_crossing_stale_threshold_cannot_send_old_target_or_catch_up_in_a_burst(monkeypatch, slow):
    app, _, sent = paused_app(monkeypatch, recover_after=.02, glove=True)
    component = app.intents if slow == "prediction" else app.robot
    name = "predict" if slow == "prediction" else "check_health"
    original = getattr(component, name)
    delayed = False

    def delay(*args):
        nonlocal delayed
        if app.clock.monotonic() > 100. and not delayed:
            app.clock.sleep(.25)
            delayed = True
        return original(*args)

    monkeypatch.setattr(component, name, delay)
    assert app.run() == 0 and app.stale > 0
    assert delayed and sent
    assert all(now - timestamp <= .2 for now, timestamp, _ in sent)
    assert all(b[0] - a[0] >= .01 - 1e-8 for a, b in zip(sent, sent[1:]))


@pytest.mark.parametrize("fault", ["robot", "imu", "source", "glove", "prediction"])
def test_sleeve_pause_does_not_suppress_other_faults(monkeypatch, fault):
    app, events, sent = paused_app(monkeypatch, glove=True)
    health = app.robot.check_health
    latest = app.sensors.latest
    predict = app.intents.predict

    def check():
        health()
        if app.clock.monotonic() >= 100.3:
            if fault == "robot":
                raise RuntimeError("robot injected failure")
            if fault == "imu":
                app.sensors.imu_error = OSError("imu injected failure")
            if fault == "glove":
                raise RuntimeError("glove injected failure")

    def read():
        if fault == "source" and app.clock.monotonic() >= 100.3:
            raise RuntimeError("source injected failure")
        return latest()

    def infer(sample):
        if fault == "prediction" and app.clock.monotonic() >= 100.5:
            raise ValueError("prediction injected failure")
        return predict(sample)

    if fault == "glove":
        monkeypatch.setattr(app.glove, "targets", check)
    else:
        monkeypatch.setattr(app.robot, "check_health", check)
    monkeypatch.setattr(app.sensors, "latest", read)
    monkeypatch.setattr(app.intents, "predict", infer)
    assert app.run() == 1
    assert f"{fault} injected failure" in app.fault
    assert all(now < (.5 if fault == "prediction" else .3) for now, _, _ in sent)
    assert events[-2:] == ["robot_shutdown", "sensor_close"]

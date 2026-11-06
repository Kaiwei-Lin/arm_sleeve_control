"""Exercise offline delay scenarios and their failure detector, without devices."""
from dataclasses import replace
import json
from pathlib import Path
import time

import pytest

from sleeve_arm.robot.aurora_session import AuroraSession
from sleeve_arm.robot import factory
from sleeve_arm.runtime import sensors
from tools import simulate_control_latency as simulation


@pytest.mark.parametrize("backend", ["bend5", "bend6"])
def test_all_scenarios_are_offline_and_write_inspectable_reports(tmp_path, monkeypatch, backend):
    def forbidden(*args, **kwargs):
        pytest.fail("offline simulation attempted a real hardware source/session")

    monkeypatch.setattr(AuroraSession, "real", forbidden)
    monkeypatch.setattr(sensors, "create_imu_source", forbidden)
    monkeypatch.setattr(sensors, "create_sleeve_source", forbidden)
    assert simulation.main(["--glove-backend", backend, "--output-dir", str(tmp_path)]) == 0
    reports = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))
    assert len(reports) == len(simulation.SCENARIOS)
    assert all(report["passed"] for report in reports)
    for report in reports:
        trace = (tmp_path / f"{report['scenario']}.csv").read_text(encoding="utf-8")
        assert "submitted_shoulder_flexion" in trace
        assert len(trace.splitlines()) == report["control_frames"] + 1
    timeout = next(report for report in reports if report["scenario"] == "imu_timeout")
    assert timeout["exit_code"] == 1 and "shoulder IMU pair is stale" in timeout["fault"]


def test_custom_no_sleeve_glove_jitter_and_recovery(tmp_path):
    assert simulation.main(["--scenario", "custom", "--no-sleeve", "--glove-ms", "300",
                            "--jitter-ms", "10", "--dropout", "glove:1:1",
                            "--output-dir", str(tmp_path)]) == 0
    report = json.loads((tmp_path / "summary.json").read_text(encoding="utf-8"))[0]
    assert report["glove_stale_ticks"] > 0 and report["sleeve_stale_ticks"] == 0
    assert report["checks"]["glove_recovered"]


def test_checker_detects_repeated_sleeve_filter_updates(monkeypatch):
    original = simulation.SimulatedIntents.predict
    count = iter(i * .0001 for i in range(1000))

    def bad_predict(self, sample):
        result = original(self, sample)
        return replace(result, elbow_flexion=result.elbow_flexion + next(count))

    # Deliberately corrupt the held elbow even when its input frame has not changed.
    monkeypatch.setattr(simulation.SimulatedIntents, "predict", bad_predict)
    report, _ = simulation.run_scenario("sleeve250", {
        "sleeve": simulation.Timing(250.), "imu": simulation.Timing(), "glove": simulation.Timing(),
    }, duration=.5)
    assert report["exit_code"] == 0
    assert not report["passed"] and not report["checks"]["sleeve_cached_target"]


def test_real_mode_preview_does_not_construct_hardware(monkeypatch, capsys):
    monkeypatch.setattr(simulation, "run_scenario", lambda *a, **kw: pytest.fail("preview started execution"))
    assert simulation.main(["--robot", "aurora", "--aurora-profile", "unused.yaml", "--scenario", "sleeve250"]) == 0
    assert "NO MOTION" in capsys.readouterr().out


def test_real_mode_keeps_wall_clock_and_confirmation_using_injected_fake_sdk(monkeypatch):
    session = simulation.fake_session(clock=time, profile=simulation.simulation_profile())
    original_factory = factory.create_robot
    prompts = []

    def confirmed(prompt):
        prompts.append(prompt)
        time.sleep(.06)  # Deliberately delay the operator without consuming the scenario.
        return "YES"

    def injected(backend, config, **kwargs):
        assert backend == "aurora"
        return original_factory(backend, config, session=session, **kwargs)

    monkeypatch.setattr(factory, "create_robot", injected)
    monkeypatch.setattr("builtins.input", confirmed)
    started = time.monotonic()
    report, rows = simulation.run_scenario("custom", {
        "imu": simulation.Timing(), "glove": simulation.Timing(300.),
    }, duration=.35, robot_backend="aurora", aurora_profile=Path("unused.yaml"), execute=True,
        max_gap_ms=200., print_fn=lambda *a, **kw: None)
    assert report["exit_code"] == 0, report
    assert time.monotonic() - started >= .35 + .06
    assert prompts and "YES" in prompts[0]
    assert rows and rows[0]["time_s"] < .06
    assert report["glove_stale_ticks"] > 0
    assert session.fake_client.closed
    assert "uniform_virtual_ticks" not in report["checks"]


@pytest.mark.parametrize("options", [
    ["--duration", "0"], ["--duration", "nan"], ["--imu-ms", "-1"],
    ["--jitter-ms", "inf"], ["--scenario", "custom", "--dropout", "glove:0:1"],
    ["--scenario", "custom", "--dropout", "invalid:1:1"], ["--no-sleeve"],
    ["--execute"], ["--robot", "aurora"],
    ["--robot", "aurora", "--aurora-profile", "unused.yaml"],
    ["--robot", "aurora", "--aurora-profile", "unused.yaml", "--scenario", "normal", "--duration", "61"],
])
def test_invalid_cli_settings_rejected(options):
    with pytest.raises(SystemExit):
        simulation.parse_args(options)

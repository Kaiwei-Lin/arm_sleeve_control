"""Offline monitor freshness, channel order and serial cleanup checks."""
from types import SimpleNamespace

import pytest

from sleeve_arm.domain import SleeveFrame
from tools import calibrate_sleeve_elbow as tool


def test_monitor_shows_all_channels_and_distinguishes_cached_frames():
    clock = SimpleNamespace(now=100.)
    clock.monotonic = lambda: clock.now
    clock.sleep = lambda seconds: setattr(clock, "now", clock.now + seconds)
    frame = SleeveFrame(100.2, (-5., 0., 100., 3., 4., 5., 6., 7., 8., 9., 1800.))
    frames = iter((None, frame, frame))
    lines = []
    tool.monitor(SimpleNamespace(latest=lambda: next(frames)), .55, 5., clock=clock,
                 print_fn=lambda text, **kw: lines.append(text))
    assert len(lines) == 3 and "WAITING" in lines[0]
    assert "NEW" in lines[1] and "age=0.000s" in lines[1]
    assert "HELD" in lines[2] and "age=0.200s" in lines[2]
    for i, value in enumerate(frame.channels, 1):
        assert f"CH{i}={value:.6g}" in lines[1]
    assert frame.timestamp == 100.2


@pytest.mark.parametrize("error,expected", [(None, 0), (RuntimeError, 1), (KeyboardInterrupt, 130)])
def test_monitor_closes_source_and_needs_no_elbow_config(monkeypatch, error, expected):
    events = []
    source = SimpleNamespace(start=lambda: events.append("start"), close=lambda: events.append("close"))
    monkeypatch.setattr(tool, "FakeSleeveSource", lambda: source)
    monkeypatch.setattr(tool, "load_phase3_config", lambda _: pytest.fail("loaded elbow config"))

    def monitor(*args):
        if error:
            raise error("stopped")

    monkeypatch.setattr(tool, "monitor", monitor)
    assert tool.main(["--monitor", "--fake"]) == expected
    assert events == ["start", "close"]


def test_calibration_still_samples_two_three_second_poses(monkeypatch, capsys):
    calls = []
    source = SimpleNamespace(start=lambda: None, close=lambda: calls.append("close"))
    monkeypatch.setattr(tool, "FakeSleeveSource", lambda: source)
    monkeypatch.setattr(tool, "load_phase3_config", lambda _: SimpleNamespace(elbow=SimpleNamespace(sleeve_channel=2)))

    def collect(reader, channel, duration):
        assert reader is source
        calls.append((channel, duration))
        return [10., 10.] if len(calls) == 1 else [90., 90.]

    monkeypatch.setattr(tool, "collect", collect)
    assert tool.main(["--fake"]) == 0
    assert calls == [(1, 3.), (1, 3.), "close"]
    output = capsys.readouterr().out
    assert "input_min: 10" in output and "input_max: 90" in output

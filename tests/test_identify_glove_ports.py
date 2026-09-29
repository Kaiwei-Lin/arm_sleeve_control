from __future__ import annotations

import io
import threading
import time
from collections import deque

import pytest

from tools import identify_glove_ports as identify


class FakeSerial:
    def __init__(self, chunks=(), error=None):
        self.chunks = deque(chunks)
        self.error = error
        self.closed = threading.Event()

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, count):
        if self.error:
            raise self.error
        if self.chunks:
            return self.chunks.popleft()
        self.closed.wait(0.002)
        return b""

    def write(self, data):
        raise AssertionError("port identification must never send commands")

    def close(self):
        self.closed.set()


def test_all_ports_parse_bend5_independently_and_show_none_when_unavailable():
    serials = {
        "csv": FakeSerial([b"10,20,", b"30,40,50,1,2,3,4,5,6;\r\n"]),
        "semicolon": FakeSerial([b"100;200;300;400;", b"500;6;5;4;3;2;1;"]),
        "invalid": FakeSerial([b"not-bend5\n"]),
        "unplugged": FakeSerial(error=OSError("disconnected")),
        "silent": FakeSerial(),
    }

    def factory(**kwargs):
        assert kwargs["baudrate"] == 115200
        assert 0 < kwargs["timeout"] <= 0.1
        if kwargs["port"] == "denied":
            raise PermissionError("permission\ndenied")
        return serials[kwargs["port"]]

    monitors = {port: identify.PortMonitor(port, 115200, factory) for port in (*serials, "denied")}
    try:
        for monitor in monitors.values():
            monitor.start()
        deadline = time.monotonic() + 2
        while True:
            lines = {port: monitor.format_line(time.monotonic(), 10) for port, monitor in monitors.items()}
            if ("bend5=" in lines["csv"] and "bend5=" in lines["semicolon"]
                    and "读取失败" in lines["invalid"] and "disconnected" in lines["unplugged"]):
                break
            assert time.monotonic() < deadline, "readers did not process fake input"
            time.sleep(0.005)
        assert "bend5=(10,20,30,40,50) | tail=(1,2,3,4,5,6)" in lines["csv"]
        assert "bend5=(100,200,300,400,500) | tail=(6,5,4,3,2,1)" in lines["semicolon"]
        for port in ("invalid", "unplugged", "silent", "denied"):
            assert lines[port].startswith(f"{port} | NONE")
        assert "permission denied" in lines["denied"]
        assert "NONE (数据过期" in monitors["csv"].format_line(time.monotonic() + 2, 1)
    finally:
        for monitor in monitors.values():
            monitor.close()
    assert all(serial.closed.is_set() for serial in serials.values())


@pytest.mark.parametrize("stop", ["duration", "interrupt", "interrupt_start", "close_failure"])
def test_scan_lists_matching_ports_and_closes_all_on_exit(tmp_path, monkeypatch, stop):
    prefix = str(tmp_path / "ttyUSB")
    for name in ("ttyUSB10", "ttyUSB2", "ttyUSB0", "ttyACM0"):
        (tmp_path / name).touch()
    (tmp_path / "ttyUSB-directory").mkdir()
    serials = {}

    def factory(**kwargs):
        if stop == "interrupt_start" and len(serials) == 1:
            raise KeyboardInterrupt
        serials[kwargs["port"]] = FakeSerial()
        return serials[kwargs["port"]]

    if stop == "close_failure":
        original_close = identify.PortMonitor.close

        def close(monitor):
            original_close(monitor)
            if monitor.port.endswith("0"):
                raise OSError("close failed")

        monkeypatch.setattr(identify.PortMonitor, "close", close)

    now = time.monotonic()

    def sleep(seconds):
        nonlocal now
        if stop == "interrupt":
            raise KeyboardInterrupt
        now += seconds

    args = identify.parse_args(["--prefix", prefix, "--duration", "0.4"])
    output = io.StringIO()
    result = identify.run(args, serial_factory=factory, output=output, clock=lambda: now, sleep=sleep)
    assert result == (1 if stop == "close_failure" else 0)
    assert serials and all(serial.closed.is_set() for serial in serials.values())
    if stop != "interrupt_start":
        ports = [prefix + suffix for suffix in ("0", "2", "10")]
        assert list(serials) == ports
        positions = [output.getvalue().index(f"{port} | NONE") for port in ports]
        assert positions == sorted(positions)
    if stop.startswith("interrupt"):
        assert "已停止" in output.getvalue()
    if stop == "duration":
        assert "0.4s" in output.getvalue()
    assert "\033" not in output.getvalue()


def test_defaults_and_no_matching_ports(tmp_path):
    args = identify.parse_args([])
    assert args.prefix == "/dev/ttyUSB"
    assert args.baudrate == 115200
    args.prefix = str(tmp_path / "missing")
    output = io.StringIO()

    def factory(**kwargs):
        pytest.fail("no serial should be opened")

    assert identify.run(args, serial_factory=factory, output=output) == 1
    assert "未找到" in output.getvalue()


@pytest.mark.parametrize("argv", [
    ["--print-hz", "0"], ["--print-hz", "nan"], ["--stale-seconds", "-1"],
    ["--duration", "inf"], ["--baudrate", "0"], ["--prefix", ""],
])
def test_invalid_cli_values_rejected(argv):
    with pytest.raises(SystemExit) as exc:
        identify.main(argv)
    assert exc.value.code == 2

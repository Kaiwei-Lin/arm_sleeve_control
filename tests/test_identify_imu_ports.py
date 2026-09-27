from __future__ import annotations

import io
import struct
import threading
import time
from collections import deque

import pytest

from tools import identify_imu_ports as identify
from tools.demo_upper_arm_twist_imu770 import Imu770FrameParser


def vector_tlv(tag: int, *values: int) -> bytes:
    payload = struct.pack(f"<{len(values)}i", *values)
    return bytes((tag, len(payload))) + payload


def frame(*tlvs: bytes) -> bytes:
    payload = b"".join(tlvs)
    body = struct.pack("<HB", 7, len(payload)) + payload
    ck1 = ck2 = 0
    for value in body:
        ck1 = (ck1 + value) & 255
        ck2 = (ck2 + ck1) & 255
    return b"\x59\x53" + body + bytes((ck1, ck2))


class FakeSerial:
    def __init__(self, chunks=(), error=None):
        self.chunks = deque(chunks)
        self.error = error
        self.is_open = True
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
        self.is_open = False
        self.closed.set()


def wait_until(predicate):
    deadline = time.monotonic() + 2.0
    while not predicate():
        assert time.monotonic() < deadline, "reader did not finish processing fake input"
        time.sleep(0.005)


def test_discovery_lists_all_matching_ports_in_numeric_order(tmp_path):
    for name in ("ttyCH9344USB10", "ttyCH9344USB2", "ttyCH9344USB0", "ttyUSB1"):
        (tmp_path / name).touch()
    (tmp_path / "ttyCH9344USB-directory").mkdir()
    prefix = str(tmp_path / "ttyCH9344USB")
    assert identify.discover_ports(prefix) == [prefix + suffix for suffix in ("0", "2", "10")]


def test_discovery_treats_prefix_as_literal_path(tmp_path):
    prefix = str(tmp_path / "tty[USB]")
    (tmp_path / "tty[USB]0").touch()
    assert identify.discover_ports(prefix) == [prefix + "0"]


@pytest.mark.parametrize("tag", [0x10, 0x20, 0x40])
def test_optional_quaternion_does_not_change_twist_parser_default(tag):
    raw = frame(vector_tlv(tag, 1_000_000, -2_000_000, 3_000_000))
    assert Imu770FrameParser().feed(raw) == []
    sample, = Imu770FrameParser(require_quaternion=False).feed(raw)
    assert sample.quaternion_wxyz is None
    assert (1.0, -2.0, 3.0) in (sample.accel_mps2, sample.gyro_dps, sample.euler_deg)


@pytest.mark.parametrize("raw", [
    frame(vector_tlv(0x41, 0, 0, 0, 0)),
    frame(vector_tlv(0x41, 1_000_000)),
    frame(vector_tlv(0x99, 1_000_000)),
    frame(bytes((0x51, 4)) + struct.pack("<I", 123)),
])
def test_identification_parser_still_rejects_invalid_or_non_motion_frames(raw):
    assert Imu770FrameParser(require_quaternion=False).feed(raw) == []


def test_all_readers_parse_independently_and_errors_or_stale_show_none():
    quaternion = frame(vector_tlv(0x41, 1_000_000, 0, 0, 0))
    measurements = frame(
        vector_tlv(0x10, 1_000_000, -2_000_000, 3_000_000),
        vector_tlv(0x20, 4_000_000, 5_000_000, -6_000_000),
        vector_tlv(0x40, 10_000_000, 20_000_000, 30_000_000),
    )
    corrupted = bytearray(quaternion)
    corrupted[-1] ^= 255
    serials = {
        "quat": FakeSerial([b"noise" + quaternion[:4], quaternion[4:]]),
        "vectors": FakeSerial([measurements]),
        "invalid": FakeSerial([bytes(corrupted)]),
        "unplugged": FakeSerial(error=OSError("disconnected")),
        "silent": FakeSerial(),
    }

    def factory(**kwargs):
        assert kwargs["baudrate"] == 460800
        assert (kwargs["bytesize"], kwargs["parity"], kwargs["stopbits"]) == (8, "N", 1)
        if kwargs["port"] == "denied":
            raise PermissionError("permission denied")
        return serials[kwargs["port"]]

    monitors = {port: identify.PortMonitor(port, 460800, factory) for port in (*serials, "denied")}
    try:
        for monitor in monitors.values():
            monitor.start()
        wait_until(lambda: monitors["quat"].reader.stats.valid_frames == 1)
        wait_until(lambda: monitors["vectors"].reader.stats.valid_frames == 1)
        wait_until(lambda: monitors["invalid"].reader.stats.checksum_errors == 1)
        wait_until(lambda: monitors["unplugged"].reader.error is not None)
        now = time.monotonic_ns()
        lines = {port: monitor.format_line(now, 10) for port, monitor in monitors.items()}
        assert "quat(wxyz)=(+1.0000,+0.0000,+0.0000,+0.0000)" in lines["quat"]
        assert "gyro(deg/s)=(+4.000,+5.000,-6.000)" in lines["vectors"]
        assert "accel(m/s2)=(+1.000,-2.000,+3.000)" in lines["vectors"]
        assert "euler(deg)=(+10.000,+20.000,+30.000)" in lines["vectors"]
        for port in ("invalid", "unplugged", "silent", "denied"):
            assert lines[port].startswith(f"{port} | NONE")
        assert "permission denied" in lines["denied"]
        assert "disconnected" in lines["unplugged"]
        assert "NONE (数据过期" in monitors["quat"].format_line(now + 2_000_000_000, 1)
    finally:
        for monitor in monitors.values():
            monitor.close()
    assert all(serial.closed.is_set() for serial in serials.values())


class FakeClock:
    def __init__(self):
        self.now = time.monotonic()

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


@pytest.mark.parametrize("stop", ["duration", "interrupt", "interrupt_start", "close_failure"])
def test_run_prints_every_port_and_closes_all_on_exit(tmp_path, monkeypatch, stop):
    prefix = str(tmp_path / "ttyCH9344USB")
    for index in range(3):
        (tmp_path / f"ttyCH9344USB{index}").touch()
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

    clock = FakeClock()

    def sleep(seconds):
        if stop == "interrupt":
            raise KeyboardInterrupt
        clock.sleep(seconds)

    args = identify.parse_args(["--prefix", prefix, "--duration", "0.4"])
    output = io.StringIO()
    result = identify.run(args, serial_factory=factory, output=output, clock=clock.time, sleep=sleep)
    assert result == (1 if stop == "close_failure" else 0)
    assert all(serial.closed.is_set() for serial in serials.values())
    if stop != "interrupt_start":
        for index in range(3):
            assert f"{prefix}{index} | NONE" in output.getvalue()
    if stop.startswith("interrupt"):
        assert "已停止" in output.getvalue()
    if stop == "duration":
        assert "0.4s" in output.getvalue()
    assert "\033" not in output.getvalue()  # redirected output never clears the terminal


def test_no_matching_ports_does_not_attempt_serial_open(tmp_path):
    def factory(**kwargs):
        pytest.fail("no serial should be opened")

    output = io.StringIO()
    args = identify.parse_args(["--prefix", str(tmp_path / "absent")])
    assert identify.run(args, serial_factory=factory, output=output) == 1
    assert "未找到" in output.getvalue()


@pytest.mark.parametrize("argv", [
    ["--print-hz", "0"], ["--print-hz", "nan"], ["--stale-seconds", "-1"],
    ["--duration", "inf"], ["--baudrate", "0"], ["--prefix", ""],
])
def test_invalid_cli_values_rejected_before_opening_ports(argv):
    with pytest.raises(SystemExit) as exc:
        identify.main(argv)
    assert exc.value.code == 2

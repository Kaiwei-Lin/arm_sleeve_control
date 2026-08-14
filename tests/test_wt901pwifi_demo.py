from __future__ import annotations

import json
import socket
import struct
import threading
import time

import pytest

import tools.read_wt901pwifi as demo
from tools.read_wt901pwifi import (
    FRAME_HEADER,
    FRAME_SIZE,
    WT901PFrame,
    WT901PStreamParser,
    build_argument_parser,
    decode_frame,
    format_frame,
    run_serial,
    run_tcp_client,
    run_tcp_server,
    run_udp,
)


def make_frame(
    *,
    accel: tuple[int, int, int] = (2048, -4096, 16384),
    gyro: tuple[int, int, int] = (164, -328, 0),
    mag: tuple[int, int, int] = (1024, -512, 256),
    angles: tuple[int, int, int] = (16384, -8192, 4096),
    temperature: int = 2534,
    battery: int = 387,
    rssi: int = -42,
    version: int = 0x1234,
) -> bytes:
    raw = bytearray(FRAME_HEADER)
    raw.extend(b"00001234")
    raw.extend(bytes((26, 8, 14, 12, 34, 56)))
    raw.extend(struct.pack("<H", 789))
    raw.extend(
        struct.pack(
            "<15hH",
            *accel,
            *gyro,
            *mag,
            *angles,
            temperature,
            battery,
            rssi,
            version,
        )
    )
    raw.extend(b"\r\n")
    assert len(raw) == FRAME_SIZE
    return bytes(raw)


@pytest.fixture
def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


@pytest.fixture
def free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def connect_with_retry(endpoint: tuple[str, int], payload: bytes) -> None:
    deadline = time.monotonic() + 2.0
    while True:
        try:
            with socket.create_connection(endpoint, timeout=0.2) as connection:
                connection.sendall(payload)
            return
        except ConnectionRefusedError:
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.01)


def one_shot_server(endpoint: tuple[str, int], payload: bytes) -> threading.Thread:
    ready = threading.Event()

    def serve() -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(endpoint)
            listener.listen(1)
            ready.set()
            connection, _ = listener.accept()
            with connection:
                connection.sendall(payload)

    thread = threading.Thread(target=serve)
    thread.start()
    assert ready.wait(timeout=2.0)
    return thread


class FakeSerial:
    def __init__(self, chunks: list[bytes]) -> None:
        self._chunks = list(chunks)
        self.closed = False

    def read(self, size: int) -> bytes:
        del size
        return self._chunks.pop(0) if self._chunks else b""

    def close(self) -> None:
        self.closed = True

    def __enter__(self) -> FakeSerial:
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()


def test_decode_frame_applies_vendor_units() -> None:
    frame = decode_frame(make_frame())

    assert frame.device_id == "00001234"
    assert frame.device_time.isoformat(timespec="milliseconds") == "2026-08-14T12:34:56.789"
    assert frame.accel_g == pytest.approx((1.0, -2.0, 8.0))
    assert frame.gyro_dps == pytest.approx((10.009765625, -20.01953125, 0.0))
    assert frame.mag_ut == pytest.approx((100.0, -50.0, 25.0))
    assert frame.euler_deg == pytest.approx((90.0, -45.0, 22.5))
    assert frame.temperature_c == pytest.approx(25.34)
    assert frame.battery_v == pytest.approx(3.87)
    assert frame.rssi_dbm == -42
    assert frame.version == 0x1234


def test_stream_parser_handles_split_and_concatenated_frames() -> None:
    parser = WT901PStreamParser()
    raw = make_frame()

    assert parser.feed(raw[:17]) == []
    frames = parser.feed(raw[17:] + raw)

    assert [item.device_id for item in frames] == ["00001234", "00001234"]


def test_stream_parser_recovers_from_noise_and_bad_terminator() -> None:
    parser = WT901PStreamParser()
    bad = bytearray(make_frame())
    bad[-1] = 0

    frames = parser.feed(b"noise" + bytes(bad) + make_frame())

    assert len(frames) == 1
    assert frames[0].device_id == "00001234"
    assert parser.invalid_frames >= 1


def test_cli_defaults_match_vendor_network_and_serial_defaults() -> None:
    parser = build_argument_parser()

    serial_args = parser.parse_args(["serial", "--port", "COM5"])
    udp_args = parser.parse_args(["udp"])
    tcp_args = parser.parse_args(["tcp-server"])

    assert serial_args.baudrate == 9600
    assert (udp_args.host, udp_args.port) == ("0.0.0.0", 1399)
    assert (tcp_args.host, tcp_args.port) == ("0.0.0.0", 1399)


def test_udp_help_exposes_bind_defaults(capsys) -> None:
    parser = build_argument_parser()

    with pytest.raises(SystemExit) as exit_info:
        parser.parse_args(["udp", "--help"])

    assert exit_info.value.code == 0
    help_text = capsys.readouterr().out
    assert "default: 0.0.0.0" in help_text
    assert "default: 1399" in help_text


def test_json_output_contains_all_measurement_groups() -> None:
    payload = json.loads(format_frame(decode_frame(make_frame()), json_output=True))

    assert payload["device_id"] == "00001234"
    assert payload["device_time"] == "2026-08-14T12:34:56.789"
    assert payload["accel_g"] == pytest.approx([1.0, -2.0, 8.0])
    assert payload["gyro_dps"][0] == pytest.approx(10.009765625)
    assert payload["mag_ut"] == pytest.approx([100.0, -50.0, 25.0])
    assert payload["euler_deg"] == pytest.approx([90.0, -45.0, 22.5])
    assert payload["temperature_c"] == pytest.approx(25.34)
    assert payload["battery_v"] == pytest.approx(3.87)
    assert payload["rssi_dbm"] == -42
    assert payload["version"] == 0x1234


def test_udp_runner_decodes_loopback_datagram(free_udp_port: int) -> None:
    received: list[WT901PFrame] = []
    thread = threading.Thread(
        target=run_udp,
        args=("127.0.0.1", free_udp_port, received.append),
        kwargs={"stop_after": 1},
    )
    thread.start()

    deadline = time.monotonic() + 2.0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        while thread.is_alive() and not received and time.monotonic() < deadline:
            sender.sendto(make_frame(), ("127.0.0.1", free_udp_port))
            time.sleep(0.01)

    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert [frame.device_id for frame in received] == ["00001234"]


def test_tcp_server_runner_decodes_loopback_stream(free_tcp_port: int) -> None:
    received: list[WT901PFrame] = []
    thread = threading.Thread(
        target=run_tcp_server,
        args=("127.0.0.1", free_tcp_port, received.append),
        kwargs={"stop_after": 1},
    )
    thread.start()

    connect_with_retry(("127.0.0.1", free_tcp_port), make_frame())

    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert [frame.device_id for frame in received] == ["00001234"]


def test_tcp_client_runner_decodes_test_server(free_tcp_port: int) -> None:
    server = one_shot_server(("127.0.0.1", free_tcp_port), make_frame())
    received: list[WT901PFrame] = []

    run_tcp_client("127.0.0.1", free_tcp_port, received.append, stop_after=1)

    server.join(timeout=2.0)
    assert not server.is_alive()
    assert [frame.device_id for frame in received] == ["00001234"]


def test_serial_runner_uses_9600_8n1_and_decodes_frame() -> None:
    calls: list[dict[str, object]] = []
    raw = make_frame()
    fake = FakeSerial([raw[:20], raw[20:]])

    def factory(**kwargs: object) -> FakeSerial:
        calls.append(kwargs)
        return fake

    received: list[WT901PFrame] = []
    run_serial(
        "COM5",
        9600,
        received.append,
        stop_after=1,
        serial_factory=factory,
        serial_constants=(8, "N", 1),
    )

    assert calls == [
        {
            "port": "COM5",
            "baudrate": 9600,
            "timeout": 0.2,
            "bytesize": 8,
            "parity": "N",
            "stopbits": 1,
        }
    ]
    assert [frame.device_id for frame in received] == ["00001234"]
    assert fake.closed


def test_main_dispatches_selected_transport(monkeypatch, capsys) -> None:
    monkeypatch.setattr(
        demo,
        "run_udp",
        lambda host, port, emit, stop_after=None: emit(decode_frame(make_frame())),
    )

    result = demo.main(
        ["--json", "udp", "--host", "127.0.0.1", "--port", "1399"]
    )

    assert result == 0
    assert json.loads(capsys.readouterr().out)["device_id"] == "00001234"

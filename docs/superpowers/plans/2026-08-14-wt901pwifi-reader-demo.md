# WT901PWIFI Real-Time Reader Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an independently runnable Python demo that decodes WT901PWIFI real-time frames received over Type-C serial, UDP, TCP server, or TCP client connections.

**Architecture:** Keep the deliverable in one executable module with explicit frame-model, streaming-parser, formatting, CLI, and transport-runner boundaries. Every transport yields byte chunks to the same parser; the program never configures WiFi or writes sensor registers.

**Tech Stack:** Python 3.10+, standard-library `argparse`, `dataclasses`, `datetime`, `json`, `socket`, `struct`; optional `pyserial>=3.5` for serial mode; `pytest>=7` for tests.

## Global Constraints

- Create a standalone demo; do not modify the existing IMU770 source, sensor configuration, synchronization, or robot-control code.
- WiFi association, credentials, sensor IP, destination IP, and device-side TCP/UDP settings are configured manually by the user.
- Default serial settings are 9600 baud, 8 data bits, no parity, and 1 stop bit.
- Default UDP and TCP server bind endpoint is `0.0.0.0:1399`.
- The program is receive-only and must not send register writes or configuration commands to the sensor.
- The 54-byte stream frame starts with `57 54 35 35`, ends with `0D 0A`, and has no vendor-defined checksum.
- Preserve unrelated staged `.idea` files and do not include them in feature commits.

---

### Task 1: Frame Model and Streaming Protocol Parser

**Files:**
- Create: `tools/read_wt901pwifi.py`
- Create: `tests/test_wt901pwifi_demo.py`

**Interfaces:**
- Consumes: Raw `bytes` chunks from later transport runners.
- Produces: `WT901PFrame`, `WT901PStreamParser.feed(data: bytes) -> list[WT901PFrame]`, and `decode_frame(raw: bytes) -> WT901PFrame`.

- [ ] **Step 1: Write failing decoding and stream-recovery tests**

Create a synthetic-frame helper that packs the documented bytes and tests signed scaling, metadata, chunking, concatenation, and malformed terminator recovery:

```python
from __future__ import annotations

import struct

import pytest

from tools.read_wt901pwifi import FRAME_HEADER, FRAME_SIZE, WT901PStreamParser, decode_frame


def make_frame(*, accel=(2048, -4096, 16384), gyro=(164, -328, 0),
               mag=(1024, -512, 256), angles=(16384, -8192, 4096),
               temperature=2534, battery=387, rssi=-42, version=0x1234) -> bytes:
    raw = bytearray(FRAME_HEADER)
    raw.extend(b"00001234")
    raw.extend(bytes((26, 8, 14, 12, 34, 56)))
    raw.extend(struct.pack("<H", 789))
    raw.extend(struct.pack("<3h3h3h3hhhH", *accel, *gyro, *mag, *angles,
                           temperature, battery, rssi, version))
    raw.extend(b"\r\n")
    assert len(raw) == FRAME_SIZE
    return bytes(raw)


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
    first = parser.feed(raw[17:] + raw)
    assert [item.device_id for item in first] == ["00001234", "00001234"]


def test_stream_parser_recovers_from_noise_and_bad_terminator() -> None:
    parser = WT901PStreamParser()
    bad = bytearray(make_frame())
    bad[-1] = 0
    frames = parser.feed(b"noise" + bytes(bad) + make_frame())
    assert len(frames) == 1
    assert frames[0].device_id == "00001234"
    assert parser.invalid_frames >= 1
```

- [ ] **Step 2: Run the focused tests and confirm RED**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -v`

Expected: collection fails with `ModuleNotFoundError: No module named 'tools.read_wt901pwifi'`.

- [ ] **Step 3: Implement the minimal frame model and parser**

Create `FRAME_HEADER = b"WT55"`, `FRAME_END = b"\r\n"`, and `FRAME_SIZE = 54`. Define an immutable dataclass with tuple-valued axes and implement decoding with exact byte offsets:

```python
@dataclass(frozen=True)
class WT901PFrame:
    device_id: str
    device_time: datetime
    accel_g: tuple[float, float, float]
    gyro_dps: tuple[float, float, float]
    mag_ut: tuple[float, float, float]
    euler_deg: tuple[float, float, float]
    temperature_c: float
    battery_v: float
    rssi_dbm: int
    version: int


def decode_frame(raw: bytes) -> WT901PFrame:
    if len(raw) != FRAME_SIZE or not raw.startswith(FRAME_HEADER) or not raw.endswith(FRAME_END):
        raise ValueError("invalid WT901PWIFI frame structure")
    device_id = raw[4:12].decode("ascii")
    if not device_id.isdigit():
        raise ValueError("WT901PWIFI device ID must contain eight ASCII digits")
    year, month, day, hour, minute, second = raw[12:18]
    milliseconds = struct.unpack_from("<H", raw, 18)[0]
    device_time = datetime(2000 + year, month, day, hour, minute, second,
                           milliseconds * 1000)
    values = struct.unpack_from("<15hH", raw, 20)
    accel = tuple(value / 32768.0 * 16.0 for value in values[0:3])
    gyro = tuple(value / 32768.0 * 2000.0 for value in values[3:6])
    mag = tuple(value * 100.0 / 1024.0 for value in values[6:9])
    angles = tuple(value / 32768.0 * 180.0 for value in values[9:12])
    return WT901PFrame(device_id, device_time, accel, gyro, mag, angles,
                       values[12] / 100.0, values[13] / 100.0,
                       values[14], values[15])


class WT901PStreamParser:
    def __init__(self) -> None:
        self._buffer = bytearray()
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[WT901PFrame]:
        self._buffer.extend(data)
        frames: list[WT901PFrame] = []
        while True:
            start = self._buffer.find(FRAME_HEADER)
            if start < 0:
                keep = min(len(self._buffer), len(FRAME_HEADER) - 1)
                del self._buffer[:-keep or None]
                return frames
            if start:
                del self._buffer[:start]
            if len(self._buffer) < FRAME_SIZE:
                return frames
            candidate = bytes(self._buffer[:FRAME_SIZE])
            try:
                frame = decode_frame(candidate)
            except (UnicodeDecodeError, ValueError):
                self.invalid_frames += 1
                del self._buffer[0]
                continue
            del self._buffer[:FRAME_SIZE]
            frames.append(frame)
```

- [ ] **Step 4: Run the focused tests and confirm GREEN**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -v`

Expected: all Task 1 tests pass.

- [ ] **Step 5: Commit the parser deliverable without unrelated staged files**

```powershell
git add -- tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py
git commit --only tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py -m "feat: decode WT901PWIFI stream frames"
```

### Task 2: Output Formatting and Command-Line Contract

**Files:**
- Modify: `tools/read_wt901pwifi.py`
- Modify: `tests/test_wt901pwifi_demo.py`

**Interfaces:**
- Consumes: `WT901PFrame` from Task 1 and `list[str] | None` CLI arguments.
- Produces: `frame_to_dict(frame) -> dict[str, object]`, `format_frame(frame, json_output: bool) -> str`, and `build_argument_parser() -> argparse.ArgumentParser`.

- [ ] **Step 1: Write failing CLI-default and serialization tests**

```python
import json

from tools.read_wt901pwifi import build_argument_parser, format_frame


def test_cli_defaults_match_vendor_network_and_serial_defaults() -> None:
    parser = build_argument_parser()
    serial_args = parser.parse_args(["serial", "--port", "COM5"])
    udp_args = parser.parse_args(["udp"])
    tcp_args = parser.parse_args(["tcp-server"])
    assert serial_args.baudrate == 9600
    assert (udp_args.host, udp_args.port) == ("0.0.0.0", 1399)
    assert (tcp_args.host, tcp_args.port) == ("0.0.0.0", 1399)


def test_json_output_contains_all_measurement_groups() -> None:
    payload = json.loads(format_frame(decode_frame(make_frame()), json_output=True))
    assert payload["device_id"] == "00001234"
    assert payload["accel_g"] == pytest.approx([1.0, -2.0, 8.0])
    assert payload["gyro_dps"][0] == pytest.approx(10.009765625)
    assert payload["mag_ut"] == pytest.approx([100.0, -50.0, 25.0])
    assert payload["euler_deg"] == pytest.approx([90.0, -45.0, 22.5])
    assert payload["temperature_c"] == pytest.approx(25.34)
    assert payload["battery_v"] == pytest.approx(3.87)
    assert payload["rssi_dbm"] == -42
    assert payload["version"] == 0x1234
```

- [ ] **Step 2: Run Task 2 tests and confirm RED**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -k "cli_defaults or json_output" -v`

Expected: import fails because `build_argument_parser` and `format_frame` do not exist.

- [ ] **Step 3: Implement parser construction and both output formats**

Use subparsers named `serial`, `udp`, `tcp-server`, and `tcp-client`. Add `--json` to the root parser so it precedes the subcommand, and add endpoint options to each relevant subparser. Serialize tuples as JSON arrays and the device time as ISO 8601:

```python
def frame_to_dict(frame: WT901PFrame) -> dict[str, object]:
    result = asdict(frame)
    result["device_time"] = frame.device_time.isoformat(timespec="milliseconds")
    return result


def format_frame(frame: WT901PFrame, json_output: bool) -> str:
    if json_output:
        return json.dumps(frame_to_dict(frame), ensure_ascii=False, separators=(",", ":"))
    ax, ay, az = frame.accel_g
    gx, gy, gz = frame.gyro_dps
    roll, pitch, yaw = frame.euler_deg
    return (f"id={frame.device_id} time={frame.device_time.isoformat(timespec='milliseconds')} "
            f"accel_g=({ax:.4f},{ay:.4f},{az:.4f}) "
            f"gyro_dps=({gx:.3f},{gy:.3f},{gz:.3f}) "
            f"rpy_deg=({roll:.3f},{pitch:.3f},{yaw:.3f}) "
            f"temp_c={frame.temperature_c:.2f} battery_v={frame.battery_v:.2f} "
            f"rssi_dbm={frame.rssi_dbm} version={frame.version}")
```

- [ ] **Step 4: Run Task 2 and parser tests**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -v`

Expected: all tests pass.

- [ ] **Step 5: Commit the CLI contract**

```powershell
git add -- tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py
git commit --only tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py -m "feat: add WT901PWIFI demo CLI output"
```

### Task 3: UDP and TCP Transport Runners

**Files:**
- Modify: `tools/read_wt901pwifi.py`
- Modify: `tests/test_wt901pwifi_demo.py`

**Interfaces:**
- Consumes: Endpoint arguments and a callable `emit(frame: WT901PFrame) -> None`.
- Produces: `run_udp(host, port, emit, stop_after=None)`, `run_tcp_server(host, port, emit, stop_after=None)`, and `run_tcp_client(host, port, emit, stop_after=None)`.

- [ ] **Step 1: Write failing loopback transport tests**

Use an ephemeral loopback endpoint, start each blocking runner in a thread, send one synthetic frame through a real socket, and assert that the callback receives it. Pass `stop_after=1` so each runner exits deterministically:

```python
def test_udp_runner_decodes_loopback_datagram(free_udp_port: int) -> None:
    received: list[WT901PFrame] = []
    thread = threading.Thread(target=run_udp,
        args=("127.0.0.1", free_udp_port, received.append),
        kwargs={"stop_after": 1})
    thread.start()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        sender.sendto(make_frame(), ("127.0.0.1", free_udp_port))
    thread.join(timeout=2)
    assert not thread.is_alive()
    assert [frame.device_id for frame in received] == ["00001234"]


def test_tcp_server_runner_decodes_loopback_stream(free_tcp_port: int) -> None:
    received: list[WT901PFrame] = []
    thread = threading.Thread(target=run_tcp_server,
        args=("127.0.0.1", free_tcp_port, received.append),
        kwargs={"stop_after": 1})
    thread.start()
    connect_with_retry(("127.0.0.1", free_tcp_port), make_frame())
    thread.join(timeout=2)
    assert not thread.is_alive()
    assert len(received) == 1


def test_tcp_client_runner_decodes_test_server(free_tcp_port: int) -> None:
    server = one_shot_server(("127.0.0.1", free_tcp_port), make_frame())
    received: list[WT901PFrame] = []
    run_tcp_client("127.0.0.1", free_tcp_port, received.append, stop_after=1)
    server.join(timeout=2)
    assert len(received) == 1
```

The test file must define `free_udp_port`, `free_tcp_port`, `connect_with_retry`, and `one_shot_server` locally using standard-library sockets. Port fixtures bind to port zero, capture the selected port, close the probe socket, and return the port.

- [ ] **Step 2: Run transport tests and confirm RED**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -k "udp_runner or tcp_server_runner or tcp_client_runner" -v`

Expected: import fails because the transport runner functions do not exist.

- [ ] **Step 3: Implement shared chunk processing and socket lifecycles**

Use a helper that feeds each chunk and calls `emit` for every frame. Set 0.2-second socket timeouts. UDP binds and calls `recvfrom(65535)`. TCP server sets `SO_REUSEADDR`, binds, listens, accepts repeatedly, and returns to `accept` after peer disconnect. TCP client uses `socket.create_connection((host, port), timeout=3.0)` and reads until the peer closes. `stop_after` is test-only deterministic termination and counts emitted frames.

```python
def _process_chunk(parser: WT901PStreamParser, chunk: bytes,
                   emit: Callable[[WT901PFrame], None], remaining: int | None) -> int | None:
    for frame in parser.feed(chunk):
        emit(frame)
        if remaining is not None:
            remaining -= 1
            if remaining <= 0:
                return 0
    return remaining
```

Catch `socket.timeout` only to continue polling. Allow `OSError` to reach `main`, where it becomes a concise endpoint-specific error. Use `with socket.socket(...)` and `with connection` so all descriptors close during success, failure, or Ctrl+C.

- [ ] **Step 4: Run all demo tests and confirm GREEN**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -v`

Expected: parser, CLI, UDP, and TCP tests all pass without warnings.

- [ ] **Step 5: Commit network transports**

```powershell
git add -- tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py
git commit --only tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py -m "feat: receive WT901PWIFI over UDP and TCP"
```

### Task 4: Serial Runner, Program Entry Point, and User Documentation

**Files:**
- Modify: `tools/read_wt901pwifi.py`
- Modify: `tests/test_wt901pwifi_demo.py`
- Modify: `README.md`

**Interfaces:**
- Consumes: Parsed CLI namespace from `build_argument_parser()`.
- Produces: `run_serial(port, baudrate, emit, stop_after=None)`, `main(argv: list[str] | None = None) -> int`, and documented user commands.

- [ ] **Step 1: Write failing serial-factory and main-dispatch tests**

Inject a serial factory so the test uses a fake context-managed serial stream and asserts 8N1 arguments and decoded output:

```python
def test_serial_runner_uses_9600_8n1_and_decodes_frame() -> None:
    calls: list[dict[str, object]] = []
    fake = FakeSerial([make_frame()[:20], make_frame()[20:]])

    def factory(**kwargs):
        calls.append(kwargs)
        return fake

    received: list[WT901PFrame] = []
    run_serial("COM5", 9600, received.append, stop_after=1,
               serial_factory=factory,
               serial_constants=(8, "N", 1))
    assert calls == [{"port": "COM5", "baudrate": 9600, "timeout": 0.2,
                      "bytesize": 8, "parity": "N", "stopbits": 1}]
    assert len(received) == 1
    assert fake.closed


def test_main_dispatches_selected_transport(monkeypatch, capsys) -> None:
    monkeypatch.setattr(demo, "run_udp",
        lambda host, port, emit, stop_after=None: emit(decode_frame(make_frame())))
    assert demo.main(["--json", "udp", "--host", "127.0.0.1", "--port", "1399"]) == 0
    assert json.loads(capsys.readouterr().out)["device_id"] == "00001234"
```

`FakeSerial.read(size)` returns queued chunks, `FakeSerial.close()` sets `closed = True`, and its context-manager exit calls `close()`.

- [ ] **Step 2: Run serial and dispatch tests and confirm RED**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -k "serial_runner or main_dispatches" -v`

Expected: import fails because `run_serial` and `main` do not exist.

- [ ] **Step 3: Implement lazy pyserial loading and main dispatch**

Import `serial` only inside `run_serial` when no factory was injected. Use `serial.EIGHTBITS`, `serial.PARITY_NONE`, and `serial.STOPBITS_ONE`; when import fails, raise `RuntimeError("serial mode requires pyserial: python -m pip install pyserial")`. Open the port with a 0.2-second timeout, read up to 4096 bytes, and feed non-empty chunks to the shared parser.

`main` prints a start line to stderr so JSON stdout remains machine-readable, dispatches the chosen runner, returns 0 on Ctrl+C, and returns 1 after printing `ERROR: ...` for `RuntimeError`, `OSError`, and serial-open errors. End the file with:

```python
if __name__ == "__main__":
    raise SystemExit(main())
```

- [ ] **Step 4: Document parameters, manual network setup, and commands**

Add a `WT901PWIFI standalone demo` README section containing:

```markdown
The demo does not connect the computer to WiFi and does not configure the sensor. Connect/configure WiFi manually, then pass the matching local bind or remote endpoint.

python tools/read_wt901pwifi.py serial --port COM5
python tools/read_wt901pwifi.py udp --host 0.0.0.0 --port 1399
python tools/read_wt901pwifi.py tcp-server --host 0.0.0.0 --port 1399
python tools/read_wt901pwifi.py tcp-client --host 192.168.4.1 --port 9250
python tools/read_wt901pwifi.py --json udp --host 0.0.0.0 --port 1399
```

Explain that inbound UDP/TCP may require allowing Python through Windows Firewall, the sensor defaults to AP+UDP, and `tcp-client` only applies when the sensor has been manually configured to accept a connection.

- [ ] **Step 5: Run focused and full regression tests**

Run: `python -m pytest tests/test_wt901pwifi_demo.py -v`

Expected: all demo tests pass without warnings.

Run: `python -m pytest -q`

Expected: the complete project suite passes.

Run: `python tools/read_wt901pwifi.py --help`

Expected: help lists `serial`, `udp`, `tcp-server`, and `tcp-client`.

Run: `python tools/read_wt901pwifi.py udp --help`

Expected: help shows defaults `0.0.0.0` and `1399` and does not claim to configure WiFi.

- [ ] **Step 6: Commit the completed standalone demo**

```powershell
git add -- tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py README.md docs/superpowers/plans/2026-08-14-wt901pwifi-reader-demo.md
git commit --only tools/read_wt901pwifi.py tests/test_wt901pwifi_demo.py README.md docs/superpowers/plans/2026-08-14-wt901pwifi-reader-demo.md -m "feat: add standalone WT901PWIFI reader demo"
```

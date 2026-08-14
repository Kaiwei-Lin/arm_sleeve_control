#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Any, Callable


FRAME_HEADER = b"WT55"
FRAME_END = b"\r\n"
FRAME_SIZE = 54


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
    device_time = datetime(
        2000 + year,
        month,
        day,
        hour,
        minute,
        second,
        milliseconds * 1000,
    )

    values = struct.unpack_from("<15hH", raw, 20)
    accel = tuple(value / 32768.0 * 16.0 for value in values[0:3])
    gyro = tuple(value / 32768.0 * 2000.0 for value in values[3:6])
    mag = tuple(value * 100.0 / 1024.0 for value in values[6:9])
    angles = tuple(value / 32768.0 * 180.0 for value in values[9:12])

    return WT901PFrame(
        device_id=device_id,
        device_time=device_time,
        accel_g=accel,
        gyro_dps=gyro,
        mag_ut=mag,
        euler_deg=angles,
        temperature_c=values[12] / 100.0,
        battery_v=values[13] / 100.0,
        rssi_dbm=values[14],
        version=values[15],
    )


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
                if len(self._buffer) > keep:
                    if keep:
                        del self._buffer[:-keep]
                    else:
                        self._buffer.clear()
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


def frame_to_dict(frame: WT901PFrame) -> dict[str, object]:
    result = asdict(frame)
    result["device_time"] = frame.device_time.isoformat(timespec="milliseconds")
    return result


def format_frame(frame: WT901PFrame, json_output: bool) -> str:
    if json_output:
        return json.dumps(frame_to_dict(frame), ensure_ascii=False, separators=(",", ":"))

    ax, ay, az = frame.accel_g
    gx, gy, gz = frame.gyro_dps
    mx, my, mz = frame.mag_ut
    roll, pitch, yaw = frame.euler_deg
    return (
        f"id={frame.device_id} "
        f"time={frame.device_time.isoformat(timespec='milliseconds')} "
        f"accel_g=({ax:.4f},{ay:.4f},{az:.4f}) "
        f"gyro_dps=({gx:.3f},{gy:.3f},{gz:.3f}) "
        f"mag_ut=({mx:.3f},{my:.3f},{mz:.3f}) "
        f"rpy_deg=({roll:.3f},{pitch:.3f},{yaw:.3f}) "
        f"temp_c={frame.temperature_c:.2f} "
        f"battery_v={frame.battery_v:.2f} "
        f"rssi_dbm={frame.rssi_dbm} "
        f"version={frame.version}"
    )


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read WT901PWIFI real-time data without configuring the sensor or WiFi."
    )
    parser.add_argument("--json", action="store_true", help="emit one JSON object per frame")
    subparsers = parser.add_subparsers(dest="transport", required=True)

    serial_parser = subparsers.add_parser(
        "serial",
        help="read the Type-C UART stream",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    serial_parser.add_argument("--port", required=True, help="serial port, for example COM5")
    serial_parser.add_argument("--baudrate", type=int, default=9600)

    for name, help_text in (
        ("udp", "listen for UDP datagrams from the sensor"),
        ("tcp-server", "listen for a TCP connection from the sensor"),
    ):
        network_parser = subparsers.add_parser(
            name,
            help=help_text,
            formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        )
        network_parser.add_argument("--host", default="0.0.0.0", help="local bind address")
        network_parser.add_argument("--port", type=int, default=1399, help="local bind port")

    tcp_client = subparsers.add_parser(
        "tcp-client",
        help="connect to a manually configured sensor TCP endpoint",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    tcp_client.add_argument("--host", required=True, help="sensor IP address")
    tcp_client.add_argument("--port", required=True, type=int, help="sensor TCP port")
    return parser


FrameEmitter = Callable[[WT901PFrame], None]


def _process_chunk(
    parser: WT901PStreamParser,
    chunk: bytes,
    emit: FrameEmitter,
    remaining: int | None,
) -> int | None:
    for frame in parser.feed(chunk):
        emit(frame)
        if remaining is not None:
            remaining -= 1
            if remaining <= 0:
                return 0
    return remaining


def run_udp(
    host: str,
    port: int,
    emit: FrameEmitter,
    stop_after: int | None = None,
) -> None:
    parser = WT901PStreamParser()
    remaining = stop_after
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind((host, port))
        receiver.settimeout(0.2)
        while remaining != 0:
            try:
                chunk, _ = receiver.recvfrom(65535)
            except socket.timeout:
                continue
            remaining = _process_chunk(parser, chunk, emit, remaining)


def run_tcp_server(
    host: str,
    port: int,
    emit: FrameEmitter,
    stop_after: int | None = None,
) -> None:
    parser = WT901PStreamParser()
    remaining = stop_after
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port))
        listener.listen(1)
        listener.settimeout(0.2)

        while remaining != 0:
            try:
                connection, _ = listener.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(0.2)
                while remaining != 0:
                    try:
                        chunk = connection.recv(4096)
                    except socket.timeout:
                        continue
                    except ConnectionResetError:
                        break
                    if not chunk:
                        break
                    remaining = _process_chunk(parser, chunk, emit, remaining)


def run_tcp_client(
    host: str,
    port: int,
    emit: FrameEmitter,
    stop_after: int | None = None,
) -> None:
    parser = WT901PStreamParser()
    remaining = stop_after
    with socket.create_connection((host, port), timeout=3.0) as connection:
        connection.settimeout(0.2)
        while remaining != 0:
            try:
                chunk = connection.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                return
            remaining = _process_chunk(parser, chunk, emit, remaining)


def run_serial(
    port: str,
    baudrate: int,
    emit: FrameEmitter,
    stop_after: int | None = None,
    serial_factory: Callable[..., object] | None = None,
    serial_constants: tuple[object, object, object] | None = None,
) -> None:
    if serial_factory is None:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise RuntimeError(
                "serial mode requires pyserial: python -m pip install pyserial"
            ) from exc
        serial_factory = serial.Serial
        serial_constants = (serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
    elif serial_constants is None:
        serial_constants = (8, "N", 1)

    bytesize, parity, stopbits = serial_constants
    try:
        serial_port: Any = serial_factory(
            port=port,
            baudrate=baudrate,
            timeout=0.2,
            bytesize=bytesize,
            parity=parity,
            stopbits=stopbits,
        )
    except Exception as exc:
        raise RuntimeError(f"failed to open serial port {port}: {exc}") from exc

    parser = WT901PStreamParser()
    remaining = stop_after
    with serial_port:
        while remaining != 0:
            chunk = serial_port.read(4096)
            if chunk:
                remaining = _process_chunk(parser, chunk, emit, remaining)


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    emit = lambda frame: print(format_frame(frame, args.json), flush=True)

    try:
        if args.transport == "serial":
            print(
                f"Reading WT901PWIFI serial data from {args.port} at {args.baudrate} baud; "
                "press Ctrl+C to stop.",
                file=sys.stderr,
            )
            run_serial(args.port, args.baudrate, emit)
        elif args.transport == "udp":
            print(
                f"Listening for WT901PWIFI UDP data on {args.host}:{args.port}; "
                "press Ctrl+C to stop.",
                file=sys.stderr,
            )
            run_udp(args.host, args.port, emit)
        elif args.transport == "tcp-server":
            print(
                f"Listening for a WT901PWIFI TCP connection on {args.host}:{args.port}; "
                "press Ctrl+C to stop.",
                file=sys.stderr,
            )
            run_tcp_server(args.host, args.port, emit)
        else:
            print(
                f"Connecting to WT901PWIFI TCP endpoint {args.host}:{args.port}; "
                "press Ctrl+C to stop.",
                file=sys.stderr,
            )
            run_tcp_client(args.host, args.port, emit)
    except KeyboardInterrupt:
        print("\nStopped.", file=sys.stderr)
        return 0
    except (OSError, RuntimeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

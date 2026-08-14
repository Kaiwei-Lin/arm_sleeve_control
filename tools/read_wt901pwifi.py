#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import struct
from dataclasses import asdict, dataclass
from datetime import datetime


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

    serial_parser = subparsers.add_parser("serial", help="read the Type-C UART stream")
    serial_parser.add_argument("--port", required=True, help="serial port, for example COM5")
    serial_parser.add_argument("--baudrate", type=int, default=9600)

    for name, help_text in (
        ("udp", "listen for UDP datagrams from the sensor"),
        ("tcp-server", "listen for a TCP connection from the sensor"),
    ):
        network_parser = subparsers.add_parser(name, help=help_text)
        network_parser.add_argument("--host", default="0.0.0.0", help="local bind address")
        network_parser.add_argument("--port", type=int, default=1399, help="local bind port")

    tcp_client = subparsers.add_parser(
        "tcp-client", help="connect to a manually configured sensor TCP endpoint"
    )
    tcp_client.add_argument("--host", required=True, help="sensor IP address")
    tcp_client.add_argument("--port", required=True, type=int, help="sensor TCP port")
    return parser

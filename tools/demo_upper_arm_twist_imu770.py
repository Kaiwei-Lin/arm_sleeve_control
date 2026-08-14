#!/usr/bin/env python3
"""Compare two upper-arm twist estimates from two read-only IMU770 streams."""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Imu770Sample:
    host_timestamp_ns: int
    tid: int
    device_timestamp_us: int | None
    dataready_timestamp_us: int | None
    accel_mps2: tuple[float, float, float] | None
    gyro_dps: tuple[float, float, float] | None
    euler_deg: tuple[float, float, float] | None
    quaternion_wxyz: tuple[float, float, float, float] | None


class Imu770FrameParser:
    HEADER = b"\x59\x53"
    KNOWN_LENGTHS = {0x10: 12, 0x20: 12, 0x40: 12, 0x41: 16, 0x51: 4, 0x52: 4}

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._last_tid: int | None = None
        self.frame_count = 0
        self.valid_frame_count = 0
        self.checksum_error_count = 0
        self.parser_error_count = 0
        self.tid_drop_count = 0

    @staticmethod
    def checksum(data: bytes) -> tuple[int, int]:
        ck1 = ck2 = 0
        for value in data:
            ck1 = (ck1 + value) & 0xFF
            ck2 = (ck2 + ck1) & 0xFF
        return ck1, ck2

    def feed(self, data: bytes, *, host_timestamp_ns: int | None = None) -> list[Imu770Sample]:
        self._buffer.extend(data)
        samples: list[Imu770Sample] = []
        while True:
            header_at = self._buffer.find(self.HEADER)
            if header_at < 0:
                keep_prefix = bool(self._buffer and self._buffer[-1] == self.HEADER[0])
                self._buffer.clear()
                if keep_prefix:
                    self._buffer.append(self.HEADER[0])
                return samples
            if header_at:
                del self._buffer[:header_at]
            if len(self._buffer) < 5:
                return samples

            frame_length = 7 + self._buffer[4]
            if len(self._buffer) < frame_length:
                return samples
            raw = bytes(self._buffer[:frame_length])
            self.frame_count += 1
            if (raw[-2], raw[-1]) != self.checksum(raw[2:-2]):
                self.checksum_error_count += 1
                del self._buffer[0]
                continue
            del self._buffer[:frame_length]

            tid = struct.unpack_from("<H", raw, 2)[0]
            try:
                sample = self._parse_message(
                    tid,
                    raw[5:-2],
                    time.monotonic_ns() if host_timestamp_ns is None else host_timestamp_ns,
                )
            except (ValueError, struct.error):
                self.parser_error_count += 1
                continue
            if sample is None:
                continue
            self._count_tid_gap(tid)
            self.valid_frame_count += 1
            samples.append(sample)

    def _count_tid_gap(self, tid: int) -> None:
        if self._last_tid is not None:
            self.tid_drop_count += (tid - self._last_tid - 1) % 60_000
        self._last_tid = tid

    def _parse_message(self, tid: int, message: bytes, host_timestamp_ns: int) -> Imu770Sample | None:
        if not 1 <= tid <= 60_000:
            raise ValueError("TID must be in [1, 60000]")
        fields: dict[int, tuple[float, ...] | int] = {}
        offset = 0
        while offset < len(message):
            if len(message) - offset < 2:
                raise ValueError("truncated TLV header")
            data_id, data_length = message[offset], message[offset + 1]
            offset += 2
            payload = message[offset : offset + data_length]
            if len(payload) != data_length:
                raise ValueError("truncated TLV payload")
            offset += data_length
            expected_length = self.KNOWN_LENGTHS.get(data_id)
            if expected_length is None:
                continue
            if data_length != expected_length:
                raise ValueError(f"invalid length for TLV 0x{data_id:02X}")
            if data_id in (0x10, 0x20, 0x40, 0x41):
                raw_values = struct.unpack(f"<{data_length // 4}i", payload)
                fields[data_id] = tuple(value * 1e-6 for value in raw_values)
            else:
                fields[data_id] = struct.unpack("<I", payload)[0]

        quaternion = fields.get(0x41)
        if not isinstance(quaternion, tuple):
            return None

        def vector(data_id: int) -> tuple[float, ...] | None:
            value = fields.get(data_id)
            return value if isinstance(value, tuple) else None

        def timestamp(data_id: int) -> int | None:
            value = fields.get(data_id)
            return value if isinstance(value, int) else None

        return Imu770Sample(
            host_timestamp_ns=host_timestamp_ns,
            tid=tid,
            device_timestamp_us=timestamp(0x51),
            dataready_timestamp_us=timestamp(0x52),
            accel_mps2=vector(0x10),  # type: ignore[arg-type]
            gyro_dps=vector(0x20),  # type: ignore[arg-type]
            euler_deg=vector(0x40),  # type: ignore[arg-type]
            quaternion_wxyz=quaternion,  # type: ignore[arg-type]
        )

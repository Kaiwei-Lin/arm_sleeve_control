#!/usr/bin/env python3
"""Compare two upper-arm twist estimates from two read-only IMU770 streams."""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from math import atan2, degrees
from pathlib import Path
from typing import Any, Callable, Sequence, TextIO

import numpy as np


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
        self.invalid_quaternion_count = 0
        self.tid_drop_count = 0
        self.tid_reset_count = 0

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
            try:
                normalized = normalize_quaternion(sample.quaternion_wxyz or ())
            except ValueError:
                self.invalid_quaternion_count += 1
                continue
            sample = replace(sample, quaternion_wxyz=tuple(float(value) for value in normalized))
            self._count_tid_gap(tid)
            self.valid_frame_count += 1
            samples.append(sample)

    def _count_tid_gap(self, tid: int) -> None:
        if self._last_tid is not None:
            advance = (tid - self._last_tid) % 60_000
            if 0 < advance <= 30_000:
                self.tid_drop_count += advance - 1
            elif advance > 30_000:
                self.tid_reset_count += 1
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


def normalize_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be nonzero")
    return values / norm


def quaternion_inverse(quaternion: Sequence[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    return np.array((q[0], -q[1], -q[2], -q[3]), dtype=float)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    w1, x1, y1, z1 = normalize_quaternion(left)
    w2, x2, y2, z2 = normalize_quaternion(right)
    return normalize_quaternion(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )
    )


def average_quaternions(quaternions: Sequence[Sequence[float]]) -> np.ndarray:
    if not quaternions:
        raise ValueError("at least one quaternion is required")
    reference = normalize_quaternion(quaternions[0])
    aligned: list[np.ndarray] = []
    for quaternion in quaternions:
        current = normalize_quaternion(quaternion)
        if float(np.dot(current, reference)) < 0.0:
            current = -current
        aligned.append(current)
    return normalize_quaternion(np.mean(aligned, axis=0))


def _x_twist_degrees(quaternion: Sequence[float]) -> float:
    q = normalize_quaternion(quaternion)
    twist_norm = float(np.hypot(q[0], q[1]))
    if twist_norm <= 1e-12:
        return 0.0
    w, x = q[0] / twist_norm, q[1] / twist_norm
    angle = degrees(2.0 * atan2(x, w))
    return (angle + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class TwistResult:
    raw_deg: float
    unwrapped_deg: float
    filtered_deg: float


class _FilteredTwist:
    def __init__(self, ema_alpha: float) -> None:
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        self.ema_alpha = float(ema_alpha)
        self._previous_raw: float | None = None
        self._unwrapped: float | None = None
        self._filtered: float | None = None

    def _reset_filter(self) -> None:
        self._previous_raw = None
        self._unwrapped = None
        self._filtered = None

    def _result(self, delta: Sequence[float]) -> TwistResult:
        raw = _x_twist_degrees(delta)
        if self._previous_raw is None:
            unwrapped = raw
        else:
            step = raw - self._previous_raw
            if step > 180.0:
                step -= 360.0
            elif step <= -180.0:
                step += 360.0
            unwrapped = float(self._unwrapped) + step
        filtered = unwrapped if self._filtered is None else (
            self.ema_alpha * unwrapped + (1.0 - self.ema_alpha) * self._filtered
        )
        self._previous_raw = raw
        self._unwrapped = unwrapped
        self._filtered = filtered
        return TwistResult(raw, unwrapped, filtered)


class TwistEstimator(_FilteredTwist):
    def __init__(self, ema_alpha: float) -> None:
        super().__init__(ema_alpha)
        self._zero: np.ndarray | None = None

    def calibrate(self, quaternions: Sequence[Sequence[float]]) -> None:
        self._zero = average_quaternions(quaternions)
        self._reset_filter()

    def update(self, quaternion: Sequence[float]) -> TwistResult:
        if self._zero is None:
            raise RuntimeError("estimator must be calibrated before update")
        delta = quaternion_multiply(quaternion_inverse(self._zero), quaternion)
        return self._result(delta)


class RelativeTwistEstimator(_FilteredTwist):
    def __init__(self, ema_alpha: float) -> None:
        super().__init__(ema_alpha)
        self._relative_zero: np.ndarray | None = None

    @staticmethod
    def _relative(upper: Sequence[float], forearm: Sequence[float]) -> np.ndarray:
        return quaternion_multiply(quaternion_inverse(forearm), upper)

    def calibrate(self, pairs: Sequence[tuple[Sequence[float], Sequence[float]]]) -> None:
        if not pairs:
            raise ValueError("at least one quaternion pair is required")
        relatives = [self._relative(upper, forearm) for upper, forearm in pairs]
        self._relative_zero = average_quaternions(relatives)
        self._reset_filter()

    def update(self, upper: Sequence[float], forearm: Sequence[float]) -> TwistResult:
        if self._relative_zero is None:
            raise RuntimeError("estimator must be calibrated before update")
        relative = self._relative(upper, forearm)
        delta = quaternion_multiply(quaternion_inverse(self._relative_zero), relative)
        return self._result(delta)


class SampleSynchronizer:
    def __init__(self, max_gap_ns: int, max_queue: int = 256) -> None:
        if max_gap_ns <= 0:
            raise ValueError("max_gap_ns must be positive")
        if max_queue <= 0:
            raise ValueError("max_queue must be positive")
        self.max_gap_ns = int(max_gap_ns)
        self._upper: deque[Imu770Sample] = deque(maxlen=max_queue)
        self._forearm: deque[Imu770Sample] = deque(maxlen=max_queue)
        self._lock = threading.Lock()
        self._minimum_timestamp_ns: int | None = None
        self.rejected_samples = 0

    def add_upper(self, sample: Imu770Sample) -> None:
        with self._lock:
            self._append(self._upper, sample)

    def add_forearm(self, sample: Imu770Sample) -> None:
        with self._lock:
            self._append(self._forearm, sample)

    def _append(self, queue: deque[Imu770Sample], sample: Imu770Sample) -> None:
        if self._minimum_timestamp_ns is not None and sample.host_timestamp_ns < self._minimum_timestamp_ns:
            self.rejected_samples += 1
            return
        if len(queue) == queue.maxlen:
            self.rejected_samples += 1
        queue.append(sample)

    def clear(self) -> int:
        with self._lock:
            discarded = len(self._upper) + len(self._forearm)
            self._upper.clear()
            self._forearm.clear()
            return discarded

    def discard_before(self, timestamp_ns: int) -> int:
        if timestamp_ns < 0:
            raise ValueError("timestamp_ns must be nonnegative")
        with self._lock:
            self._minimum_timestamp_ns = int(timestamp_ns)
            discarded = len(self._upper) + len(self._forearm)
            self._upper.clear()
            self._forearm.clear()
            return discarded

    def pop_pair(self) -> tuple[Imu770Sample, Imu770Sample, int] | None:
        with self._lock:
            if not self._upper or not self._forearm:
                return None
            best_upper = best_forearm = 0
            best_gap = abs(self._upper[0].host_timestamp_ns - self._forearm[0].host_timestamp_ns)
            for upper_index, upper in enumerate(self._upper):
                for forearm_index, forearm in enumerate(self._forearm):
                    gap = abs(upper.host_timestamp_ns - forearm.host_timestamp_ns)
                    if gap < best_gap:
                        best_upper, best_forearm, best_gap = upper_index, forearm_index, gap
            if best_gap <= self.max_gap_ns:
                for _ in range(best_upper):
                    self._upper.popleft()
                    self.rejected_samples += 1
                for _ in range(best_forearm):
                    self._forearm.popleft()
                    self.rejected_samples += 1
                return self._upper.popleft(), self._forearm.popleft(), best_gap

            if self._upper[0].host_timestamp_ns <= self._forearm[0].host_timestamp_ns:
                self._upper.popleft()
            else:
                self._forearm.popleft()
            self.rejected_samples += 1
            return None


@dataclass(frozen=True)
class ReaderStats:
    frames: int
    valid_frames: int
    checksum_errors: int
    parser_errors: int
    invalid_quaternions: int
    tid_drops: int
    tid_resets: int
    fps: float


class Imu770SerialReader:
    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        on_sample: Callable[[Imu770Sample], None],
        serial_factory: Callable[..., object] | None = None,
    ) -> None:
        if not port:
            raise ValueError("serial port is required")
        if baudrate <= 0 or timeout_s <= 0:
            raise ValueError("baudrate and timeout_s must be positive")
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self._on_sample = on_sample
        self._serial_factory = serial_factory
        self._serial: Any = None
        self._parser = Imu770FrameParser()
        self._error: BaseException | None = None
        self._first_sample_ns: int | None = None
        self._last_sample_ns: int | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        factory = self._serial_factory
        if factory is None:
            try:
                import serial
            except ImportError as exc:
                raise RuntimeError("IMU770 serial input requires: pip install pyserial") from exc
            factory = serial.Serial
        try:
            self._serial = factory(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout_s,
                bytesize=8,
                parity="N",
                stopbits=1,
            )
        except Exception as exc:
            raise RuntimeError(f"failed to open IMU770 serial port {self.port}: {exc}") from exc
        self._parser = Imu770FrameParser()
        with self._lock:
            self._error = None
            self._first_sample_ns = None
            self._last_sample_ns = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name=f"imu770-{self.port}", daemon=False)
        self._thread.start()

    @property
    def error(self) -> BaseException | None:
        with self._lock:
            return self._error

    @property
    def stats(self) -> ReaderStats:
        with self._lock:
            first, last = self._first_sample_ns, self._last_sample_ns
        elapsed = 0.0 if first is None or last is None else (last - first) / 1e9
        valid = self._parser.valid_frame_count
        fps = (valid - 1) / elapsed if valid > 1 and elapsed > 0.0 else 0.0
        return ReaderStats(
            frames=self._parser.frame_count,
            valid_frames=valid,
            checksum_errors=self._parser.checksum_error_count,
            parser_errors=self._parser.parser_error_count,
            invalid_quaternions=self._parser.invalid_quaternion_count,
            tid_drops=self._parser.tid_drop_count,
            tid_resets=self._parser.tid_reset_count,
            fps=fps,
        )

    def close(self) -> None:
        self._stop.set()
        serial_port, self._serial = self._serial, None
        if serial_port is not None and getattr(serial_port, "is_open", True):
            serial_port.close()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.timeout_s * 3.0))

    def _run(self) -> None:
        serial_port = self._serial
        try:
            while not self._stop.is_set():
                waiting = int(getattr(serial_port, "in_waiting", 0) or 0)
                chunk = serial_port.read(max(1, min(waiting or 1, 4096)))
                if not chunk:
                    continue
                for sample in self._parser.feed(chunk):
                    with self._lock:
                        self._first_sample_ns = self._first_sample_ns or sample.host_timestamp_ns
                        self._last_sample_ns = sample.host_timestamp_ns
                    self._on_sample(sample)
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = exc


def _positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not np.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be a positive finite number")
    return parsed


def _ema_alpha(value: str) -> float:
    parsed = float(value)
    if not np.isfinite(parsed) or not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("value must be in (0, 1]")
    return parsed


class _DemoArgumentParser(argparse.ArgumentParser):
    def parse_args(self, args: Sequence[str] | None = None, namespace: argparse.Namespace | None = None) -> argparse.Namespace:
        parsed = super().parse_args(args, namespace)
        if parsed.upper_port.casefold() == parsed.forearm_port.casefold():
            self.error("--upper-port and --forearm-port must be different")
        return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = _DemoArgumentParser(
        description="Read two IMU770 devices and compare upper-arm +X twist estimators."
    )
    parser.add_argument("--upper-port", required=True, help="Upper-arm IMU770 serial port, e.g. COM5")
    parser.add_argument("--forearm-port", required=True, help="Forearm IMU770 serial port, e.g. COM6")
    parser.add_argument("--baudrate", type=_positive_int, default=460800)
    parser.add_argument("--timeout", type=_positive_float, default=0.1, help="Serial read timeout in seconds")
    parser.add_argument("--startup-timeout", type=_positive_float, default=10.0, help="Seconds to wait for both streams")
    parser.add_argument("--calibration-seconds", type=_positive_float, default=2.0)
    parser.add_argument("--print-hz", type=_positive_float, default=10.0)
    parser.add_argument("--max-sync-ms", type=_positive_float, default=20.0)
    parser.add_argument("--ema-alpha", type=_ema_alpha, default=0.35)
    parser.add_argument("--csv", type=Path, help="Optional parsed comparison CSV path")
    return parser


CSV_FIELDS = (
    "upper_host_timestamp_ns",
    "forearm_host_timestamp_ns",
    "upper_tid",
    "forearm_tid",
    "upper_device_timestamp_us",
    "forearm_device_timestamp_us",
    "upper_qw",
    "upper_qx",
    "upper_qy",
    "upper_qz",
    "forearm_qw",
    "forearm_qx",
    "forearm_qy",
    "forearm_qz",
    "world_raw_deg",
    "world_unwrapped_deg",
    "world_filtered_deg",
    "relative_raw_deg",
    "relative_unwrapped_deg",
    "relative_filtered_deg",
    "difference_filtered_deg",
    "sync_gap_ms",
)


class CsvRecorder:
    def __init__(self, path: Path | None) -> None:
        self.path = path
        self._file: TextIO | None = None
        self._writer: csv.DictWriter[str] | None = None
        self._last_flush = time.monotonic()

    def __enter__(self) -> "CsvRecorder":
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._file = self.path.open("w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_FIELDS)
            self._writer.writeheader()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None

    def write(
        self,
        upper: Imu770Sample,
        forearm: Imu770Sample,
        gap_ns: int,
        world: TwistResult,
        relative: TwistResult,
    ) -> None:
        if self._writer is None:
            return
        upper_q = upper.quaternion_wxyz or (None, None, None, None)
        forearm_q = forearm.quaternion_wxyz or (None, None, None, None)
        self._writer.writerow(
            {
                "upper_host_timestamp_ns": upper.host_timestamp_ns,
                "forearm_host_timestamp_ns": forearm.host_timestamp_ns,
                "upper_tid": upper.tid,
                "forearm_tid": forearm.tid,
                "upper_device_timestamp_us": upper.device_timestamp_us,
                "forearm_device_timestamp_us": forearm.device_timestamp_us,
                "upper_qw": upper_q[0],
                "upper_qx": upper_q[1],
                "upper_qy": upper_q[2],
                "upper_qz": upper_q[3],
                "forearm_qw": forearm_q[0],
                "forearm_qx": forearm_q[1],
                "forearm_qy": forearm_q[2],
                "forearm_qz": forearm_q[3],
                "world_raw_deg": world.raw_deg,
                "world_unwrapped_deg": world.unwrapped_deg,
                "world_filtered_deg": world.filtered_deg,
                "relative_raw_deg": relative.raw_deg,
                "relative_unwrapped_deg": relative.unwrapped_deg,
                "relative_filtered_deg": relative.filtered_deg,
                "difference_filtered_deg": world.filtered_deg - relative.filtered_deg,
                "sync_gap_ms": gap_ns / 1e6,
            }
        )
        now = time.monotonic()
        if self._file is not None and now - self._last_flush >= 1.0:
            self._file.flush()
            self._last_flush = now


def _raise_reader_error(upper_reader: Imu770SerialReader, forearm_reader: Imu770SerialReader) -> None:
    if upper_reader.error is not None:
        raise RuntimeError(f"upper IMU770 reader failed: {upper_reader.error}") from upper_reader.error
    if forearm_reader.error is not None:
        raise RuntimeError(f"forearm IMU770 reader failed: {forearm_reader.error}") from forearm_reader.error


def run(
    args: argparse.Namespace,
    *,
    serial_factory: Callable[..., object] | None = None,
    input_fn: Callable[[str], str] = input,
) -> int:
    synchronizer = SampleSynchronizer(max_gap_ns=int(args.max_sync_ms * 1e6))
    upper_reader = Imu770SerialReader(
        args.upper_port,
        args.baudrate,
        args.timeout,
        synchronizer.add_upper,
        serial_factory=serial_factory,
    )
    forearm_reader = Imu770SerialReader(
        args.forearm_port,
        args.baudrate,
        args.timeout,
        synchronizer.add_forearm,
        serial_factory=serial_factory,
    )
    world_estimator = TwistEstimator(args.ema_alpha)
    relative_estimator = RelativeTwistEstimator(args.ema_alpha)

    try:
        upper_reader.start()
        forearm_reader.start()
        print(
            f"IMU770 connected: upper={args.upper_port}, forearm={args.forearm_port}, "
            f"{args.baudrate} baud, read-only"
        )
        print("Quaternion: Sensor->World [w,x,y,z]; upper-arm axis: local +X")

        startup_deadline = time.monotonic() + args.startup_timeout
        while upper_reader.stats.valid_frames == 0 or forearm_reader.stats.valid_frames == 0:
            _raise_reader_error(upper_reader, forearm_reader)
            if time.monotonic() >= startup_deadline:
                missing = []
                if upper_reader.stats.valid_frames == 0:
                    missing.append(f"upper ({args.upper_port})")
                if forearm_reader.stats.valid_frames == 0:
                    missing.append(f"forearm ({args.forearm_port})")
                raise RuntimeError("no valid quaternion frames from " + " and ".join(missing))
            time.sleep(0.005)

        input_fn("Hold the aligned zero pose still, then press Enter to calibrate...")
        pre_calibration_discarded = synchronizer.discard_before(time.monotonic_ns())
        print(f"Calibrating for {args.calibration_seconds:.2f} s; keep both IMUs still...")
        calibration_pairs: list[tuple[np.ndarray, np.ndarray]] = []
        calibration_deadline = time.monotonic() + args.calibration_seconds
        while time.monotonic() < calibration_deadline:
            _raise_reader_error(upper_reader, forearm_reader)
            pair = synchronizer.pop_pair()
            if pair is None:
                time.sleep(0.001)
                continue
            upper, forearm, _ = pair
            if upper.quaternion_wxyz is None or forearm.quaternion_wxyz is None:
                continue
            try:
                upper_q = normalize_quaternion(upper.quaternion_wxyz)
                forearm_q = normalize_quaternion(forearm.quaternion_wxyz)
            except ValueError:
                continue
            calibration_pairs.append((upper_q, forearm_q))
        if len(calibration_pairs) < 2:
            raise RuntimeError(
                f"calibration needs at least 2 synchronized valid pairs; received {len(calibration_pairs)}"
            )
        world_estimator.calibrate([upper for upper, _ in calibration_pairs])
        relative_estimator.calibrate(calibration_pairs)
        print(
            f"Calibration complete: {len(calibration_pairs)} synchronized pairs "
            f"(discarded {pre_calibration_discarded} pre-prompt samples). Ctrl+C to stop."
        )

        print_interval = 1.0 / args.print_hz
        last_print = float("-inf")
        with CsvRecorder(args.csv) as recorder:
            while True:
                pair = synchronizer.pop_pair()
                if pair is None:
                    _raise_reader_error(upper_reader, forearm_reader)
                    time.sleep(0.001)
                    continue
                upper, forearm, gap_ns = pair
                if upper.quaternion_wxyz is None or forearm.quaternion_wxyz is None:
                    continue
                try:
                    world = world_estimator.update(upper.quaternion_wxyz)
                    relative = relative_estimator.update(upper.quaternion_wxyz, forearm.quaternion_wxyz)
                except ValueError:
                    continue
                recorder.write(upper, forearm, gap_ns, world, relative)

                now = time.monotonic()
                if now - last_print >= print_interval:
                    upper_stats, forearm_stats = upper_reader.stats, forearm_reader.stats
                    print(
                        f"world={world.filtered_deg:+8.2f} deg  "
                        f"relative={relative.filtered_deg:+8.2f} deg  "
                        f"difference={world.filtered_deg - relative.filtered_deg:+8.2f} deg  "
                        f"sync={gap_ns / 1e6:5.2f} ms  "
                        f"fps=({upper_stats.fps:5.1f},{forearm_stats.fps:5.1f})  "
                        f"sync_rejected={synchronizer.rejected_samples}  "
                        f"checksum=({upper_stats.checksum_errors},{forearm_stats.checksum_errors})  "
                        f"malformed=({upper_stats.parser_errors},{forearm_stats.parser_errors})  "
                        f"invalid_quaternion=({upper_stats.invalid_quaternions},"
                        f"{forearm_stats.invalid_quaternions})"
                    )
                    last_print = now
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        upper_reader.close()
        forearm_reader.close()
        upper_stats, forearm_stats = upper_reader.stats, forearm_reader.stats
        print(
            f"Summary: upper_valid={upper_stats.valid_frames}, forearm_valid={forearm_stats.valid_frames}, "
            f"sync_rejected={synchronizer.rejected_samples}, "
            f"checksum=({upper_stats.checksum_errors},{forearm_stats.checksum_errors}), "
            f"malformed=({upper_stats.parser_errors},{forearm_stats.parser_errors}), "
            f"tid_drops=({upper_stats.tid_drops},{forearm_stats.tid_drops}), "
            f"tid_resets=({upper_stats.tid_resets},{forearm_stats.tid_resets}), "
            f"invalid_quaternion=({upper_stats.invalid_quaternions},{forearm_stats.invalid_quaternions})"
        )


def main() -> int:
    return run(build_argument_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())

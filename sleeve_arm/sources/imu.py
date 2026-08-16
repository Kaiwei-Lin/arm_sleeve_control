from __future__ import annotations

import struct
import threading
import time
from collections import deque
from typing import Any, Callable

from sleeve_arm.domain.sensor import ImuFrame
from sleeve_arm.sources.base import ImuSource, SourceStats


class Imu770Parser:
    HEADER = b"\x59\x53"
    KNOWN_LENGTHS = {0x10: 12, 0x20: 12, 0x40: 12, 0x41: 16, 0x51: 4, 0x52: 4}

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.invalid_frames = 0

    @staticmethod
    def checksum(data: bytes) -> tuple[int, int]:
        ck1 = ck2 = 0
        for value in data:
            ck1 = (ck1 + value) & 0xFF
            ck2 = (ck2 + ck1) & 0xFF
        return ck1, ck2

    def feed(self, data: bytes, host_timestamp_ns: int | None = None) -> list[ImuFrame]:
        arrival_ns = time.monotonic_ns() if host_timestamp_ns is None else int(host_timestamp_ns)
        self._buffer.extend(data)
        frames: list[ImuFrame] = []
        while True:
            header_at = self._buffer.find(self.HEADER)
            if header_at < 0:
                trailing = bool(self._buffer and self._buffer[-1] == self.HEADER[0])
                self._buffer.clear()
                if trailing:
                    self._buffer.append(self.HEADER[0])
                return frames
            if header_at:
                del self._buffer[:header_at]
            if len(self._buffer) < 5:
                return frames
            length = 7 + self._buffer[4]
            if len(self._buffer) < length:
                return frames
            raw = bytes(self._buffer[:length])
            if (raw[-2], raw[-1]) != self.checksum(raw[2:-2]):
                self.invalid_frames += 1
                del self._buffer[0]
                continue
            del self._buffer[:length]
            try:
                frame = self._parse_message(
                    struct.unpack_from("<H", raw, 2)[0],
                    raw[5:-2],
                    arrival_ns,
                )
            except ValueError:
                self.invalid_frames += 1
                continue
            if frame is not None:
                frames.append(frame)
        return frames

    def _parse_message(
        self,
        tid: int,
        message: bytes,
        host_timestamp_ns: int,
    ) -> ImuFrame | None:
        if not 1 <= tid <= 60000:
            raise ValueError("invalid TID")
        fields: dict[int, tuple[float, ...] | int] = {}
        offset = 0
        while offset < len(message):
            if len(message) - offset < 2:
                raise ValueError("truncated TLV header")
            data_id, data_length = message[offset], message[offset + 1]
            payload = message[offset + 2 : offset + 2 + data_length]
            if len(payload) != data_length:
                raise ValueError("truncated TLV payload")
            offset += 2 + data_length
            expected = self.KNOWN_LENGTHS.get(data_id)
            if expected is None:
                continue
            if expected != data_length:
                raise ValueError("invalid TLV length")
            if data_id in (0x10, 0x20, 0x40, 0x41):
                fields[data_id] = tuple(value * 1e-6 for value in struct.unpack(f"<{data_length // 4}i", payload))
            else:
                fields[data_id] = struct.unpack("<I", payload)[0]
        accel, gyro = fields.get(0x10), fields.get(0x20)
        if not isinstance(accel, tuple) or not isinstance(gyro, tuple):
            return None
        quat = fields.get(0x41)
        quat_values = quat if isinstance(quat, tuple) else (None, None, None, None)
        device_timestamp = fields.get(0x51)
        return ImuFrame(
            host_timestamp_ns / 1_000_000_000.0, *accel, *gyro, *quat_values, tid,
            device_timestamp if isinstance(device_timestamp, int) else None,
            host_timestamp_ns,
        )


class Imu770SerialSource(ImuSource):
    def __init__(self, port: str, baudrate: int = 460800, timeout_s: float = 0.1, serial_factory: Callable[..., object] | None = None) -> None:
        if not port:
            raise ValueError("IMU770 serial port is required")
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self._serial_factory = serial_factory
        self._serial: Any = None
        self._parser = Imu770Parser()
        self._latest: ImuFrame | None = None
        self._pending: deque[ImuFrame] = deque()
        self._received = 0
        self._first_timestamp: float | None = None
        self._last_error: BaseException | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        factory = self._serial_factory
        serial_module = None
        if factory is None:
            try:
                import serial as serial_module  # type: ignore
            except ImportError as exc:
                raise RuntimeError("IMU770 serial mode requires: pip install pyserial") from exc
            factory = serial_module.Serial
        kwargs: dict[str, object] = {"port": self.port, "baudrate": self.baudrate, "timeout": self.timeout_s}
        if serial_module is not None:
            kwargs.update(bytesize=serial_module.EIGHTBITS, parity=serial_module.PARITY_NONE, stopbits=serial_module.STOPBITS_ONE)
        try:
            self._serial = factory(**kwargs)
        except Exception as exc:
            raise RuntimeError(f"failed to open IMU770 serial port {self.port}: {exc}") from exc
        self._parser = Imu770Parser()
        with self._lock:
            self._pending.clear()
        self._stop.clear()
        self._last_error = None
        self._thread = threading.Thread(target=self._run, name=f"imu770-reader-{self.port}", daemon=False)
        self._thread.start()

    def latest(self) -> ImuFrame | None:
        with self._lock:
            error, frame = self._last_error, self._latest
            self._pending.clear()
        if error is not None:
            raise RuntimeError(f"IMU770 acquisition stopped: {error}") from error
        return frame

    def drain(self) -> tuple[ImuFrame, ...]:
        with self._lock:
            error = self._last_error
            frames = tuple(self._pending)
            self._pending.clear()
        if error is not None and not frames:
            raise RuntimeError(f"IMU770 acquisition stopped: {error}") from error
        return frames

    @property
    def stats(self) -> SourceStats:
        with self._lock:
            last = None if self._latest is None else self._latest.timestamp
            elapsed = 0.0 if last is None or self._first_timestamp is None else last - self._first_timestamp
            fps = (self._received - 1) / elapsed if self._received > 1 and elapsed > 0 else 0.0
            return SourceStats(self._received, self._parser.invalid_frames, last, fps)

    def close(self) -> None:
        self._stop.set()
        serial_port, self._serial = self._serial, None
        if serial_port is not None and getattr(serial_port, "is_open", False):
            serial_port.close()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.timeout_s * 3))

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                waiting = int(getattr(self._serial, "in_waiting", 0) or 0)
                chunk = self._serial.read(max(1, min(waiting or 1, 4096)))
                if not chunk:
                    continue
                host_timestamp_ns = time.monotonic_ns()
                for frame in self._parser.feed(chunk, host_timestamp_ns):
                    with self._lock:
                        self._received += 1
                        self._first_timestamp = self._first_timestamp or frame.timestamp
                        self._latest = frame
                        self._pending.append(frame)
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._last_error = exc

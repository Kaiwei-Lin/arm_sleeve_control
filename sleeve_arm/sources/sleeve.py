from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable

from sleeve_arm.domain.sensor import SleeveFrame
from sleeve_arm.sources.base import SleeveSource, SourceStats


class SerialSleeveSource(SleeveSource):
    """11-field ASCII sleeve protocol terminated by a semicolon."""

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout_s: float = 1.0,
        delimiter: str = ";",
        expected_fields: int = 11,
        serial_factory: Callable[..., object] | None = None,
    ) -> None:
        if not port:
            raise ValueError("sleeve serial port is required")
        if not delimiter:
            raise ValueError("sleeve delimiter cannot be empty")
        if expected_fields <= 0:
            raise ValueError("expected_fields must be positive")
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.delimiter = delimiter
        self.expected_fields = int(expected_fields)
        self._serial_factory = serial_factory
        self._serial: Any = None
        self._buffer = bytearray()
        self._latest: SleeveFrame | None = None
        self._received = 0
        self._invalid = 0
        self._first_timestamp: float | None = None
        self._last_error: BaseException | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @staticmethod
    def parse_record(record: str, expected_fields: int = 11) -> tuple[float, ...]:
        fields = record.split(",")
        if len(fields) != expected_fields:
            raise ValueError(f"expected {expected_fields} fields, got {len(fields)}")
        values = tuple(float(field.strip()) for field in fields)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("all sleeve fields must be finite")
        return values

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        factory = self._serial_factory
        serial_module = None
        if factory is None:
            try:
                import serial as serial_module  # type: ignore
            except ImportError as exc:
                raise RuntimeError("serial sleeve mode requires: pip install pyserial") from exc
            factory = serial_module.Serial
        kwargs: dict[str, object] = {"port": self.port, "baudrate": self.baudrate, "timeout": self.timeout_s}
        if serial_module is not None:
            kwargs.update(bytesize=serial_module.EIGHTBITS, parity=serial_module.PARITY_NONE, stopbits=serial_module.STOPBITS_ONE)
        try:
            self._serial = factory(**kwargs)
        except Exception as exc:
            raise RuntimeError(f"failed to open sleeve serial port {self.port}: {exc}") from exc
        self._buffer.clear()
        self._stop.clear()
        self._last_error = None
        self._thread = threading.Thread(target=self._run, name="sleeve-reader", daemon=False)
        self._thread.start()

    def latest(self) -> SleeveFrame | None:
        with self._lock:
            error = self._last_error
            frame = self._latest
        if error is not None:
            raise RuntimeError(f"sleeve acquisition stopped: {error}") from error
        return frame

    @property
    def stats(self) -> SourceStats:
        with self._lock:
            last = None if self._latest is None else self._latest.timestamp
            elapsed = 0.0 if last is None or self._first_timestamp is None else last - self._first_timestamp
            fps = (self._received - 1) / elapsed if self._received > 1 and elapsed > 0 else 0.0
            return SourceStats(self._received, self._invalid, last, fps)

    def close(self) -> None:
        self._stop.set()
        serial_port, self._serial = self._serial, None
        if serial_port is not None and getattr(serial_port, "is_open", False):
            serial_port.close()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.timeout_s * 2))

    def _run(self) -> None:
        delimiter = self.delimiter.encode("ascii")
        try:
            while not self._stop.is_set():
                waiting = int(getattr(self._serial, "in_waiting", 0) or 0)
                chunk = self._serial.read(max(1, min(waiting or 1, 8192)))
                if not chunk:
                    continue
                self._buffer.extend(chunk)
                if len(self._buffer) > 8192:
                    del self._buffer[:-8192]
                    with self._lock:
                        self._invalid += 1
                while True:
                    index = self._buffer.find(delimiter)
                    if index < 0:
                        break
                    raw = bytes(self._buffer[:index])
                    del self._buffer[: index + len(delimiter)]
                    if not raw.strip():
                        continue
                    try:
                        channels = self.parse_record(raw.decode("ascii", errors="strict").strip(), self.expected_fields)
                    except (UnicodeDecodeError, ValueError):
                        with self._lock:
                            self._invalid += 1
                        continue
                    timestamp = time.monotonic()
                    with self._lock:
                        self._received += 1
                        self._first_timestamp = self._first_timestamp or timestamp
                        self._latest = SleeveFrame(timestamp, channels, self._received)
        except BaseException as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._last_error = exc

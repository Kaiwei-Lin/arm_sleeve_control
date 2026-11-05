"""Independent Bend6 ASCII acquisition: eleven raw fields, nine named channels.

There is no frame header. A configured packet terminator establishes boundaries;
the first terminated record after each start is discarded, even if it looks valid.
"""
from __future__ import annotations

import json
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml


BEND6_CHANNEL_NAMES = (
    "little_ip", "little_ring_between", "ring_ip", "middle_ip",
    "index_middle_between", "index_mcp", "index_ip", "thumb_mcp", "thumb_tip_joint",
)
BEND6_RAW_CHANNEL_NAMES = BEND6_CHANNEL_NAMES + ("reserved_10", "reserved_11")
BEND6_EXPECTED_FIELDS = 11
BEND6_BETWEEN_CHANNELS = ("little_ring_between", "index_middle_between")


def _mapping(value, label, allowed=None):
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a mapping")
    if allowed is not None and set(value) - set(allowed):
        raise ValueError(f"{label} has unknown keys: {sorted(set(value) - set(allowed))}")
    return dict(value)


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


def load_bend6_calibration(path):
    """Accept JSON/YAML: glove.bend6.calibration, bend6.calibration,
    calibration, or a bare {mode, channels} calibration object.
    """
    path = Path(path)
    if path.suffix.lower() not in (".json", ".yaml", ".yml"):
        raise ValueError("Bend6 calibration requires a .json, .yaml or .yml file")
    text = path.read_text(encoding="utf-8")
    data = json.loads(text) if path.suffix.lower() == ".json" else yaml.safe_load(text)
    data = _mapping(data, "Bend6 calibration file")
    if "glove" in data:
        data = _mapping(data["glove"], "glove")
        if "bend6" not in data:
            raise ValueError("glove calibration file requires glove.bend6")
    if "bend6" in data:
        data = _mapping(data["bend6"], "bend6")
    if "calibration" in data:
        data = _mapping(data["calibration"], "Bend6 calibration")
    if "mode" not in data:
        raise ValueError("Bend6 calibration file requires an explicit mode")
    return data


class Bend6Calibration:
    def __init__(self, config=None):
        config = _mapping({} if config is None else config, "Bend6 calibration", ("mode", "channels"))
        self.mode = config.get("mode", "raw")
        if self.mode not in ("raw", "normalized"):
            raise ValueError("Bend6 calibration mode must be raw or normalized")
        channels = _mapping(config.get("channels", {}), "Bend6 calibration channels", BEND6_CHANNEL_NAMES)
        self.channels = {}
        self.raw_only_channels = tuple(name for name in BEND6_CHANNEL_NAMES
                                       if channels.get(name) == {"mode": "raw"})
        if self.mode == "normalized" and set(channels) != set(BEND6_CHANNEL_NAMES):
            raise ValueError("Bend6 normalized calibration requires all nine channels")
        for name, item in channels.items():
            if name in self.raw_only_channels:
                if name not in BEND6_BETWEEN_CHANNELS:
                    raise ValueError(f"{name}: per-channel raw mode is only allowed for between channels")
                continue
            item = _mapping(item, f"Bend6 calibration {name}", ("raw_min", "raw_max", "invert", "points"))
            low = _number(item.get("raw_min"), f"{name}.raw_min")
            high = _number(item.get("raw_max"), f"{name}.raw_max")
            if low == high or not math.isfinite(high - low):
                raise ValueError(f"{name} requires distinct endpoints with a finite span")
            invert = item.get("invert", False)
            if type(invert) is not bool:
                raise ValueError(f"{name}.invert must be boolean")
            points = item.get("points")
            if points is not None:
                if not isinstance(points, (list, tuple)) or len(points) < 2:
                    raise ValueError(f"{name}.points requires at least two [raw, normalized] pairs")
                pairs = []
                for point in points:
                    if not isinstance(point, (list, tuple)) or len(point) != 2:
                        raise ValueError(f"{name}.points requires [raw, normalized] pairs")
                    pairs.append(tuple(_number(v, f"{name}.points") for v in point))
                if any(b[0] <= a[0] or not math.isfinite(b[0] - a[0])
                       for a, b in zip(pairs, pairs[1:])):
                    raise ValueError(f"{name}.points raw values must strictly increase")
                if (pairs[0][1] != 0 or pairs[-1][1] != 1
                        or any(not 0 <= p[1] <= 1 for p in pairs)
                        or any(b[1] < a[1] for a, b in zip(pairs, pairs[1:]))):
                    raise ValueError(f"{name}.points outputs must increase monotonically from 0 to 1")
                points = tuple(pairs)
            self.channels[name] = (low, high, invert, points)

    def apply(self, values):
        values = _nine_values(values)
        if self.mode == "raw":
            return values
        result = []
        for name, raw in zip(BEND6_CHANNEL_NAMES, values):
            if name in self.raw_only_channels:
                result.append(raw)
                continue
            low, high, invert, points = self.channels[name]
            if points is None:
                # Clamp the input first to avoid overflow in raw - low.
                raw = min(max(raw, min(low, high)), max(low, high))
                value = (raw - low) / (high - low)
            elif raw <= points[0][0]:
                value = 0.
            elif raw >= points[-1][0]:
                value = 1.
            else:
                for (x0, y0), (x1, y1) in zip(points, points[1:]):
                    if raw <= x1:
                        value = y0 + (raw - x0) / (x1 - x0) * (y1 - y0)
                        break
            value = min(1., max(0., value))
            result.append(1. - value if invert else value)
        return tuple(result)


def _nine_values(values):
    values = tuple(values)
    if len(values) != 9 or not all(math.isfinite(v) for v in values):
        raise ValueError("Bend6 processing requires nine finite values")
    return values


class Bend6Filter:
    def __init__(self, config=None):
        config = _mapping({} if config is None else config, "Bend6 filter",
                          ("enabled", "type", "alpha", "deadband", "max_delta_per_sample"))
        self.enabled = config.get("enabled", False)
        if type(self.enabled) is not bool or config.get("type", "ema") != "ema":
            raise ValueError("Bend6 filter requires boolean enabled and type=ema")
        self.alpha = _number(config.get("alpha", .35), "filter.alpha")
        self.deadband = _number(config.get("deadband", .01), "filter.deadband")
        delta = config.get("max_delta_per_sample")
        self.max_delta = None if delta is None else _number(delta, "filter.max_delta_per_sample")
        if not 0 < self.alpha <= 1 or self.deadband < 0 or (self.max_delta is not None and self.max_delta <= 0):
            raise ValueError("Bend6 filter requires 0 < alpha <= 1, deadband >= 0 and max_delta > 0")
        self.reset()

    def reset(self):
        self._state = None

    def apply(self, values):
        values = _nine_values(values)
        if not self.enabled or self._state is None:
            self._state = values
            return values
        result = []
        for previous, current in zip(self._state, values):
            delta = current - previous
            if abs(delta) <= self.deadband:
                value = previous
            else:
                if self.max_delta is not None:
                    delta = min(self.max_delta, max(-self.max_delta, delta))
                    value = previous + self.alpha * delta
                else:
                    value = (1. - self.alpha) * previous + self.alpha * current
            result.append(value)
        self._state = _nine_values(result)
        return self._state


class Bend6Parser:
    """Buffer fragments only within known packet boundaries, never across bad frames.

    Default ';' terminated packets accept comma/whitespace fields (including
    newlines). For semicolon-separated fields choose packet_delimiter='\n'.
    An unframed stream of individual semicolon values cannot establish alignment.
    """
    def __init__(self, packet_delimiter=";"):
        if packet_delimiter not in (";", "\n"):
            raise ValueError("Bend6 packet_delimiter must be ';' or newline")
        self.delimiter = ord(packet_delimiter)
        self.reset(clear_stats=True)

    def reset(self, *, clear_stats=False):
        if clear_stats:
            self.received_records = self.valid_records = 0
            self.invalid_records = self.dropped_records = 0
            self.last_error = None
            self.last_invalid_record = b""
        self._buffer = bytearray()
        self._discard = True
        self.generation = getattr(self, "generation", 0) + 1

    def diagnostics(self):
        return dict(records=self.received_records, valid=self.valid_records,
                    invalid=self.invalid_records, dropped=self.dropped_records,
                    pending_bytes=len(self._buffer), synchronizing=self._discard,
                    last_error=self.last_error, last_invalid_record=self.last_invalid_record)

    def _reject(self, reason, record):
        self.invalid_records += 1
        self.generation += 1  # Reset downstream filter before the next valid packet.
        self.last_error = reason
        self.last_invalid_record = bytes(record[:160])

    def feed(self, chunk):
        for byte in chunk:
            if byte == self.delimiter:
                self.received_records += 1
                record = bytes(self._buffer)
                self._buffer.clear()
                if self._discard:
                    self._discard = False
                    self.dropped_records += 1
                    continue
                try:
                    text = record.decode("ascii").strip()
                    # Empty punctuation fields are corrupt; whitespace is a separator.
                    if not text or re.search(r"(?:^|[,;])\s*(?:[,;]|$)", text):
                        raise ValueError("empty field")
                    fields = re.split(r"[,;\s]+", text)
                    if len(fields) != 11:
                        raise ValueError(f"field_count={len(fields)}, expected=11")
                    # Use ASCII decimal syntax; Python float also accepts underscores.
                    if any(not re.fullmatch(
                            r"[+-]?(?:[0-9]+(?:\.[0-9]*)?|\.[0-9]+)(?:[eE][+-]?[0-9]+)?", f)
                            for f in fields):
                        raise ValueError("invalid numeric token")
                    packet = tuple(float(f) for f in fields)
                    if not all(math.isfinite(v) for v in packet):
                        raise ValueError("non-finite values")
                except UnicodeDecodeError:
                    self._reject("non-ASCII bytes", record)
                    continue
                except ValueError as exc:
                    self._reject(str(exc), record)
                    continue
                self.valid_records += 1
                yield packet
            elif not self._discard:
                self._buffer.append(byte)
                if len(self._buffer) > 8192:
                    self._reject("record exceeded 8192 bytes without packet delimiter", self._buffer)
                    self.reset()  # Discard through the next terminator, not just this chunk.


@dataclass(frozen=True)
class Bend6Reading:
    timestamp: float
    raw: tuple[float, ...]
    channels: Mapping[str, float]
    status: str
    side: str
    sequence: int
    calibration_mode: str
    filter_enabled: bool
    synthetic: bool = False
    raw_only_channels: tuple[str, ...] = ()

    @property
    def calibrated(self):
        return self.calibration_mode == "normalized"

    @property
    def raw_channels(self):
        return dict(zip(BEND6_CHANNEL_NAMES, self.raw[:9]))


class Bend6GloveSource:
    expected_fields = BEND6_EXPECTED_FIELDS

    def __init__(self, port, baudrate=115200, timeout=.02, *, calibration_path=None,
                 side="right", filter=None, packet_delimiter=";", calibration=None,
                 clock=None, serial_factory=None, synthetic=False):
        if not isinstance(port, str) or not port.strip() or type(baudrate) is not int or baudrate <= 0:
            raise ValueError("Bend6 requires a port and positive integer baudrate")
        if _number(timeout, "Bend6 timeout") <= 0 or side not in ("left", "right"):
            raise ValueError("Bend6 requires positive timeout and side=left/right")
        if calibration_path is not None and calibration is not None:
            raise ValueError("specify only one Bend6 calibration source")
        self.calibration = Bend6Calibration(load_bend6_calibration(calibration_path)
                                          if calibration_path is not None else calibration)
        self.calibration_loaded = self.calibration.mode == "normalized"
        self.filter = Bend6Filter(filter)
        self.parser = Bend6Parser(packet_delimiter)
        self.port, self.baudrate, self.timeout, self.side = port, baudrate, timeout, side
        self.clock, self._serial_factory = clock or time, serial_factory
        self.synthetic = synthetic
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._reading = self._error = self._serial = self._thread = None
        self._sequence = 0
        self._filter_generation = self.parser.generation
        self._reset_diagnostics()

    def _reset_diagnostics(self):
        self._received_bytes = self._empty_reads = 0
        self._last_byte_at = self._last_valid_at = None
        self._last_chunk = b""
        self._parser_diagnostics = self.parser.diagnostics()

    def diagnostics(self):
        """Snapshot only; never read serial, reset synchronization, or refresh a frame."""
        now = self.clock.monotonic()
        with self._lock:
            return dict(
                self._parser_diagnostics,
                bytes_received=self._received_bytes, empty_reads=self._empty_reads,
                byte_age_s=None if self._last_byte_at is None else now - self._last_byte_at,
                valid_age_s=None if self._last_valid_at is None else now - self._last_valid_at,
                last_chunk=self._last_chunk,
                reader_alive=self._thread is not None and self._thread.is_alive(),
                reader_error=None if self._error is None else str(self._error),
            )

    def _store_packet(self, packet, timestamp=None):
        raw = tuple(packet)
        if len(raw) != 11 or not all(math.isfinite(v) for v in raw):
            raise ValueError("Bend6 packet requires eleven finite values")
        timestamp = self.clock.monotonic() if timestamp is None else timestamp
        if not math.isfinite(timestamp):
            raise ValueError("Bend6 timestamp must be finite")
        raw_only = self.calibration.raw_only_channels
        calibrated = self.calibration.apply(raw[:9])
        # Uncalibrated between values bypass the normalized filter and retain their units.
        values = self.filter.apply(tuple(0. if name in raw_only else value
                                         for name, value in zip(BEND6_CHANNEL_NAMES, calibrated)))
        values = tuple(raw[i] if name in raw_only else values[i]
                       for i, name in enumerate(BEND6_CHANNEL_NAMES))
        self._sequence += 1
        reading = Bend6Reading(timestamp, raw, MappingProxyType(dict(zip(BEND6_CHANNEL_NAMES, values))),
                               "ok", self.side, self._sequence,
                               "partial" if self.calibration.mode == "normalized" and raw_only else self.calibration.mode,
                               self.filter.enabled, self.synthetic, raw_only)
        with self._lock:
            self._reading = reading
            self._last_valid_at = timestamp

    def _feed_bytes(self, chunk):
        if not chunk:
            return
        received_at = self.clock.monotonic()
        with self._lock:
            self._received_bytes += len(chunk)
            self._last_byte_at = received_at
            self._last_chunk = bytes(chunk[-160:])
        try:
            for packet in self.parser.feed(chunk):
                if self._filter_generation != self.parser.generation:
                    self.filter.reset()
                    self._filter_generation = self.parser.generation
                self._store_packet(packet, received_at)
        finally:
            with self._lock:
                self._parser_diagnostics = self.parser.diagnostics()

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Bend6 source is already started")
        self.parser.reset(clear_stats=True)
        self.filter.reset()
        self._filter_generation = self.parser.generation
        self._stop.clear()
        with self._lock:
            self._reading = self._error = None
            self._sequence = 0
            self._reset_diagnostics()
        serial_factory = self._serial_factory
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        try:
            self._serial = serial_factory(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self._thread = threading.Thread(target=self._run, args=(self._serial,), name="bend6-reader", daemon=False)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _run(self, serial_port):
        try:
            while not self._stop.is_set():
                chunk = serial_port.read(max(1, min(int(serial_port.in_waiting), 8192)))
                if chunk:
                    self._feed_bytes(chunk)
                else:
                    with self._lock:
                        self._empty_reads += 1
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = exc
        finally:
            # Release a disconnected port even before the application polls latest().
            try:
                serial_port.close()
            except Exception as exc:
                with self._lock:
                    self._error = self._error or exc

    def latest(self):
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"Bend6 acquisition stopped: {self._error}") from self._error
            if self._thread is not None and not self._thread.is_alive() and not self._stop.is_set():
                raise RuntimeError("Bend6 acquisition stopped: reader thread exited")
            return self._reading

    def close(self):
        self._stop.set()
        serial_port, thread = self._serial, self._thread
        try:
            if serial_port is not None:
                serial_port.close()
        finally:
            if thread is not None and thread.ident is not None:
                thread.join(timeout=max(1., 2 * self.timeout))
                if thread.is_alive():
                    raise RuntimeError("Bend6 reader did not stop after serial close")
            self._serial = self._thread = None
            self.parser.reset()
            self.filter.reset()
            with self._lock:
                self._reading = None

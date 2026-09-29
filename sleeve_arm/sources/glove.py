"""Standalone Bend5 ASCII acquisition and calibration, with named finger readings.

The calibration math follows the original Bend5 implementation; acquisition and
readings belong to this project and do not import another application or GUI.
"""
from __future__ import annotations

import json
import math
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np


FINGER_ORDER = ("pinky", "ring", "middle", "index", "thumb")


@dataclass(frozen=True)
class GloveReading:
    timestamp: float
    raw: tuple[float, ...]
    curls: dict[str, float]
    status: str


def _finger_order(value):
    names = value.split(",") if isinstance(value, str) else value
    order = tuple(str(name).strip().lower() for name in names)
    if len(order) != 5 or set(order) != set(FINGER_ORDER):
        raise ValueError("finger order must contain pinky, ring, middle, index and thumb once each")
    return order


def _vector(value, default, *, minimum=None):
    if value is None:
        value = default
    if isinstance(value, str):
        value = re.split(r"[,;\s]+", value.strip())
    vector = np.asarray(value, dtype=np.float32).reshape(-1)
    if not vector.size or not np.isfinite(vector).all():
        raise ValueError("Bend5 calibration vectors must contain finite numbers")
    vector = np.pad(vector, (0, max(0, 5 - vector.size)), mode="edge")[:5]
    return vector if minimum is None else np.maximum(vector, np.float32(minimum))


class Bend5GloveSource:
    def __init__(self, port, baudrate=115200, timeout=0.02, *, expected_fields=11,
                 calibration_path=None, calibration_mode="auto", input_max_value=1000.0,
                 per_finger_max_delta=None, zero_on_start=True, zero_frames=30,
                 bend_direction="absolute", deadzone_value=8.0, filter_alpha=0.75,
                 adaptive_baseline=True, adaptive_baseline_alpha=0.004,
                 sensor_map=FINGER_ORDER, renderer_remap_from_current=None,
                 clock=None, serial_factory=None):
        if not port or baudrate <= 0 or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Bend5 requires a port, positive baudrate and finite positive timeout")
        if type(expected_fields) is not int or expected_fields < 5:
            raise ValueError("Bend5 expected_fields must be an integer >= 5")
        if type(zero_frames) is not int or zero_frames < 0:
            raise ValueError("Bend5 zero_frames must be a nonnegative integer")
        if not math.isfinite(filter_alpha) or not 0.05 <= filter_alpha <= 1:
            raise ValueError("Bend5 filter_alpha must be in [0.05, 1]")
        if not math.isfinite(adaptive_baseline_alpha) or not 0 <= adaptive_baseline_alpha <= .2:
            raise ValueError("Bend5 adaptive_baseline_alpha must be in [0, 0.2]")
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        self.expected_fields, self.zero_frames = expected_fields, zero_frames
        self.clock, self._serial_factory = clock or time, serial_factory
        self.sensor_map = _finger_order(sensor_map)
        self.calibration = {}
        if calibration_path is not None:
            self.calibration = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
            if not isinstance(self.calibration, dict) or not self.calibration:
                raise ValueError("Bend5 calibration file must contain a nonempty JSON object")
        self.calibration_loaded = bool(self.calibration)
        self.calibration_order = _finger_order(
            self.calibration.get("finger_order")
            or self.calibration.get("protocol", {}).get("first5_order", FINGER_ORDER)
        )
        self.open_raw = self._calibration_vector("open_raw", 0.)
        calibrated_max = self._calibration_vector("max_delta", 1000., minimum=1.)
        maximum = per_finger_max_delta
        if maximum is None:
            maximum = calibrated_max if "max_delta" in self.calibration else input_max_value
        self.max_delta = _vector(maximum, 1000., minimum=1e-6)
        self.deadzone = (self._calibration_vector("deadzone", 0., minimum=0.)
                         if "deadzone" in self.calibration else _vector(deadzone_value, 8., minimum=0.))
        self.direction = np.sign(self._calibration_vector("direction", 1.))
        self.direction[self.direction == 0] = 1.
        directions = (re.split(r"[,;\s]+", bend_direction.strip())
                      if isinstance(bend_direction, str) else list(bend_direction))
        if not directions or any(v not in ("increase", "decrease", "absolute") for v in directions):
            raise ValueError("Bend5 bend_direction must be increase, decrease or absolute")
        self.bend_direction = (directions + [directions[-1]] * 5)[:5]
        remap = renderer_remap_from_current
        if remap is None:
            remap = (self.calibration.get("renderer_remap_from_current")
                     or self.calibration.get("renderer_finger_remap_from_current")
                     or self.calibration.get("renderer_remap") or {})
        if isinstance(remap, (list, tuple)) and len(remap) == 5:
            remap = dict(zip(reversed(FINGER_ORDER), remap))
        if not isinstance(remap, dict) or any(k not in FINGER_ORDER or v not in FINGER_ORDER
                                              for k, v in remap.items()):
            raise ValueError("Bend5 renderer_remap_from_current must map named fingers")
        self.remap = {name: remap.get(name, name) for name in FINGER_ORDER}
        if calibration_mode not in ("auto", "matrix", "per_finger", "none"):
            raise ValueError("Bend5 calibration_mode must be auto, matrix, per_finger or none")
        if calibration_mode == "auto":
            calibration_mode = self.calibration.get("calibration_mode", "auto")
        if calibration_mode not in ("auto", "matrix", "per_finger", "none"):
            raise ValueError("invalid calibration_mode in Bend5 calibration file")
        self.matrix_pinv = None
        self.calibration_mode_active = "none"
        if self.calibration_loaded and calibration_mode != "none":
            self.calibration_mode_active = "per_finger"
            actions = self.calibration.get("single_action_delta_matrix")
            if calibration_mode in ("auto", "matrix") and isinstance(actions, dict):
                columns = []
                fist_delta = self._calibration_vector("open_to_fist_delta", 0.)
                for index, finger in enumerate(FINGER_ORDER):
                    column = self._reorder_calibration(_vector(actions.get(finger), 0.))
                    if np.linalg.norm(column) < 1e-6:
                        column = np.zeros(5, dtype=np.float32)
                        column[index] = (fist_delta[index] if abs(fist_delta[index]) > 1e-6
                                         else calibrated_max[index])
                    columns.append(column)
                matrix = np.stack(columns, axis=1)
                self.matrix_pinv = np.linalg.pinv(matrix.astype(np.float64)).astype(np.float32)
                self.calibration_mode_active = "matrix"
        self.zero_on_start = bool(zero_on_start) and not self.calibration_loaded
        self.adaptive_baseline = bool(adaptive_baseline) and not self.calibration_loaded
        self.adaptive_alpha, self.filter_alpha = adaptive_baseline_alpha, filter_alpha
        self._baseline = self.open_raw.copy() if self.calibration_mode_active != "none" else None
        self._baseline_buffer = []
        self._filtered = None
        self._tokens = []
        self._buffer = bytearray()
        self._reading = self._error = self._serial = self._thread = None
        self._lock = threading.Lock()
        self._stop = threading.Event()

    def _reorder_calibration(self, vector):
        return vector[[self.calibration_order.index(name) for name in FINGER_ORDER]]

    def _calibration_vector(self, name, default, *, minimum=None):
        return self._reorder_calibration(_vector(self.calibration.get(name), default, minimum=minimum))

    def _normalize(self, raw):
        if self.zero_on_start and self._baseline is None:
            self._baseline_buffer.append(raw.copy())
            if len(self._baseline_buffer) >= max(1, self.zero_frames):
                self._baseline = np.median(np.stack(self._baseline_buffer), axis=0).astype(np.float32)
                self._filtered = np.zeros(5, dtype=np.float32)
            return np.zeros(5, dtype=np.float32)
        if self.calibration_mode_active != "none":
            delta = raw - self.open_raw
            if self.matrix_pinv is not None:
                delta[np.abs(delta) <= self.deadzone] = 0.
                curls = np.clip(self.matrix_pinv @ delta, 0., 1.)
                curls[curls < .015] = 0.
            else:
                curls = np.clip(np.maximum(self.direction * delta - self.deadzone, 0.)
                                / np.maximum(self.max_delta, 1.), 0., 1.)
                curls[curls < .002] = 0.
        else:
            delta = raw.copy()
            if self._baseline is not None:
                difference = raw - self._baseline
                delta = np.asarray([v if d == "increase" else -v if d == "decrease" else abs(v)
                                    for v, d in zip(difference, self.bend_direction)], dtype=np.float32)
                if self.adaptive_baseline:
                    near_zero = np.abs(difference) <= np.maximum(self.deadzone * 1.5, 1.)
                    a = np.float32(self.adaptive_alpha)
                    self._baseline[near_zero] = (1. - a) * self._baseline[near_zero] + a * raw[near_zero]
                delta = np.maximum(delta - self.deadzone, 0.)
            curls = np.clip(raw if self._baseline is None and np.max(np.abs(raw)) <= 1.0001
                            else np.maximum(delta, 0.) / self.max_delta, 0., 1.)
        if self._filtered is None:
            self._filtered = curls.copy()
        else:
            a = np.float32(self.filter_alpha)
            self._filtered = (1. - a) * self._filtered + a * curls
            self._filtered[np.abs(self._filtered) < .002] = 0.
        return np.clip(self._filtered, 0., 1.)

    def _store_packet(self, packet, timestamp=None):
        received_at = self.clock.monotonic() if timestamp is None else timestamp
        packet = np.asarray(packet, dtype=np.float32).reshape(-1)
        if packet.size != self.expected_fields or not np.isfinite(packet).all():
            raise ValueError("Bend5 packet has wrong length or non-finite values")
        curls = dict(zip(self.sensor_map, self._normalize(packet[:5])))
        reading = GloveReading(
            received_at, tuple(float(v) for v in packet),
            {name: float(curls[self.remap[name]]) for name in FINGER_ORDER},
            "calibrating" if self.zero_on_start and self._baseline is None else "ok",
        )
        with self._lock:
            self._reading = reading

    def _parse_record(self, record, timestamp=None):
        if not record.strip():
            return
        try:
            values = [float(token) for token in re.split(r"[,;\s]+", record.strip()) if token]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Bend5 record contains non-finite values")
            if len(values) == self.expected_fields:
                self._tokens.clear()  # A complete CSV record establishes a new packet boundary.
            self._tokens.extend(values)
            while len(self._tokens) >= self.expected_fields:
                packet = self._tokens[:self.expected_fields]
                del self._tokens[:self.expected_fields]
                self._store_packet(packet, timestamp)
        except ValueError:
            self._tokens.clear()
            raise

    def _feed_bytes(self, chunk):
        received_at = self.clock.monotonic()
        self._buffer.extend(chunk)
        if len(self._buffer) > 8192:
            raise ValueError("Bend5 serial buffer exceeded 8192 bytes without a packet boundary")
        records = re.split(rb"[;\r\n]", self._buffer)
        self._buffer = bytearray(records.pop())
        for record in records:
            self._parse_record(record.decode("ascii"), received_at)

    def start(self):
        if self._thread is not None:
            raise RuntimeError("Bend5 source is already started")
        serial_factory = self._serial_factory
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self._stop.clear()
        self._error = None
        self._tokens.clear()
        self._buffer.clear()
        try:
            self._serial = serial_factory(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            self._thread = threading.Thread(target=self._run, name="bend5-reader", daemon=False)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def _run(self):
        try:
            while not self._stop.is_set():
                size = max(1, min(int(self._serial.in_waiting), 8192))
                chunk = self._serial.read(size)
                if chunk:
                    self._feed_bytes(chunk)
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = exc

    def latest(self):
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"Bend5 acquisition stopped: {self._error}") from self._error
            return self._reading

    def close(self):
        self._stop.set()
        serial_port, self._serial = self._serial, None
        thread, self._thread = self._thread, None
        try:
            if serial_port is not None:
                serial_port.close()
        finally:
            if thread is not None and thread.ident is not None:
                thread.join(timeout=max(1., 2 * self.timeout))

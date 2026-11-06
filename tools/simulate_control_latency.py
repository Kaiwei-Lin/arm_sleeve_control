#!/usr/bin/env python3
"""Synthetic sensor delay injection with offline or explicitly enabled real Aurora control."""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
import csv
from dataclasses import dataclass, replace
import json
import math
from pathlib import Path
import random
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import GloveConfig, load_phase3_config, load_phase4_config, load_sensor_config
from sleeve_arm.control.parameters import JointSafetyParameters
from sleeve_arm.domain import ImuFrame, SleeveFrame
from sleeve_arm.estimation import UpperArmRotationEstimator, quaternion_from_axis_angle
from sleeve_arm.estimation.flexarm import DualImuArmEstimator
from sleeve_arm.predictor import DualImuShoulderPredictor, RuleBasedPredictor
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_profile, fake_session
from sleeve_arm.robot.aurora_profile import AuroraGroup, AuroraJoint
from sleeve_arm.runtime.configuration import ModelControlConfigs, parse_args as control_args
from sleeve_arm.runtime.glove_control import GloveRuntime, HAND_JOINTS, HAND_LIMITS
from sleeve_arm.runtime.intent_pipeline import IntentPipeline
from sleeve_arm.runtime.model_control import ModelControlApp
from sleeve_arm.runtime.robot_runtime import RobotRuntime
from sleeve_arm.runtime.sensors import SensorRuntime
from sleeve_arm.sources.base import SourceStats


@dataclass(frozen=True)
class Timing:
    period_ms: float = 10.
    jitter_ms: float = 0.
    pauses: tuple[tuple[float, float], ...] = ()  # start/end seconds after source startup


class ScheduledSource:
    """Deliver new frames on an independent schedule; repeated reads retain their age."""

    def __init__(self, clock, timing, make_frame, seed):
        self.clock, self.timing, self.make_frame = clock, timing, make_frame
        self.seed = seed
        self.random = random.Random(seed)
        self.reset(clock.monotonic())

    def reset(self, origin):
        self.origin = origin
        self.random.seed(self.seed)
        self.next_at = 0.
        self.frame = None
        self.received = 0

    def latest(self):
        elapsed = self.clock.monotonic() - self.origin
        while self.next_at <= elapsed + 1e-10:
            at = self.next_at
            self.next_at += (self.timing.period_ms + self.random.uniform(0, self.timing.jitter_ms)) / 1000.
            if any(start <= at < end for start, end in self.timing.pauses):
                continue
            self.received += 1
            self.frame = self.make_frame(min(self.origin + at, self.clock.monotonic()), at, self.received)
        return self.frame

    @property
    def stats(self):
        stamp = None if self.frame is None else self.frame.timestamp
        elapsed = max(self.clock.monotonic() - self.origin, 1e-9)
        return SourceStats(self.received, 0, stamp, self.received / elapsed)

    def close(self):
        pass


def imu_frame(stamp, angle=0., axis=(0, 1, 0), sequence=None):
    return ImuFrame(stamp, 0, 0, 9.81, 0, 0, 0,
                    *quaternion_from_axis_angle(axis, angle), sequence_id=sequence)


class SimulatedSensors(SensorRuntime):
    def __init__(self, config, clock, timings, seed):
        super().__init__(config, sleeve="none" if "sleeve" not in timings else "fake")
        self.clock, self.timings, self.seed = clock, timings, seed

    def start(self):
        if self.sleeve_mode != "none":
            self.source = ScheduledSource(self.clock, self.timings["sleeve"],
                lambda stamp, t, n: SleeveFrame(stamp, (0., .5 + .3 * math.sin(t), 0., 0., 0.), n), self.seed)
        for name in self.required_imus:
            if name == self.shoulder_names[0]:
                make = lambda stamp, t, n: imu_frame(stamp, math.radians(20 + 10 * math.sin(t)), sequence=n)
            elif name == self.rotation_names[0]:
                make = lambda stamp, t, n: imu_frame(stamp, math.radians(10 * math.sin(t)), (1, 0, 0), n)
            else:
                make = lambda stamp, t, n: imu_frame(stamp, sequence=n)
            # Equal seeds keep each simulated IMU pair synchronized; delay still differs from Sleeve/glove.
            self.imu_sources[name] = ScheduledSource(self.clock, self.timings["imu"], make, self.seed + 1)


class SimulatedIntents(IntentPipeline):
    def prepare(self, sensors):
        self.sensors = sensors
        estimator = DualImuArmEstimator(down_axis=(-1, 0, 0), forward_axis=(0, 0, 1), lateral_axis=(0, 1, 0),
                                        rest_threshold_deg=5, dominance_ratio=1.1)
        rest = (1., 0., 0., 0.)
        forward = tuple(quaternion_from_axis_angle((0, 1, 0), math.pi / 4))
        estimator.calibrate([rest, rest], [rest, rest])
        estimator.calibrate_forward([rest, rest], [forward, forward])
        self.shoulder = DualImuShoulderPredictor(estimator, self.configs.phase3.sensor_timeout_ms / 1000.)
        self.elbow = None if sensors.sleeve_mode == "none" else RuleBasedPredictor(self.configs.elbow)
        self.rotation = UpperArmRotationEstimator(.35, 20., "x")
        self.rotation.calibrate([(imu_frame(99.), imu_frame(99.)), (imu_frame(99.01), imu_frame(99.01))])
        self.last_rotation_rad = 0.


class SimulatedGlove(GloveRuntime):
    def __init__(self, clock, timing, backend, seed, print_fn):
        super().__init__(GloveConfig(backend=backend), mode="fake", clock=clock,
                         hold_on_stale=True, print_fn=print_fn)
        self.timing, self.seed = timing, seed

    def start(self):
        super().start()  # Creates an in-memory calibrated Bend5/Bend6 source, no serial port.

        def make(stamp, t, sequence):
            packet = [.5 + .3 * math.sin(t + .1 * i) for i in range(9)] + [0., 0.]
            # Acquisition timestamps are delivery times, as with the real readers.
            if self.config.backend == "bend6":
                self.source._store_packet(packet, timestamp=stamp)
            else:
                with patch.object(self.source, "clock", SimpleNamespace(monotonic=lambda: stamp)):
                    self.source._store_packet(packet)
            return self.source.latest()

        self.schedule = ScheduledSource(self.clock, self.timing, make, self.seed + 2)

    def latest(self):
        return None if self.source is None else self.schedule.latest()


def simulation_profile():
    """Synthetic limits avoid dependence on machine-specific real robot calibration."""
    profile = fake_profile()
    hand = AuroraGroup(
        name="FAKE_right_hand", side="right", part="hand", count=6,
        joints=tuple(AuroraJoint(name, index, 1, 0., JointSafetyParameters(
            name, low, high, max_velocity=1., max_position_step=.02, max_tracking_error=.3),
            verified=True, kind="finger") for index, (name, (low, high)) in enumerate(zip(HAND_JOINTS, HAND_LIMITS))),
        sdk_position_limits=HAND_LIMITS, max_tracking_error=.3, capabilities=("joint_position", "hand_joints"), verified=True,
    )
    return replace(profile, groups=profile.groups + (hand,), allowed_fsm=(11,), control_hz=100., smooth_limits=True)


def run_scenario(name, timings, *, duration=4., timeout_ms=200., backend="bend6", seed=1,
                 robot_backend="aurora-fake", aurora_profile=None, execute=False, max_gap_ms=50., print_fn=print):
    real_robot = robot_backend == "aurora"
    if robot_backend not in ("aurora", "aurora-fake"):
        raise ValueError("unsupported robot backend")
    if real_robot and (not execute or aurora_profile is None):
        raise ValueError("real Aurora requires --execute and --aurora-profile")
    clock = time if real_robot else FakeClock()
    logs, rows = [], []

    def log(message, **kwargs):
        logs.append(str(message))
        if real_robot:
            print_fn(message, **kwargs)
    sensor_config = load_sensor_config(Path(__file__).resolve().parents[1] / "configs/sensors.yaml")
    elbow = replace(load_phase3_config().elbow, sleeve_channel=2, input_min=0., input_max=1.,
                    angle_min_deg=0., angle_max_deg=90.)
    phase3 = replace(load_phase3_config(), elbow=elbow, control_hz=100., sensor_timeout_ms=timeout_ms,
                     hard_timeout_ms=max(1000., timeout_ms * 2))
    configs = ModelControlConfigs(phase3, load_phase4_config(), sensor_config, elbow=elbow,
                                  shoulder_backend="dual_imu", sleeve_mode="fake" if "sleeve" in timings else "none",
                                  glove_mode="fake")
    sensors = SimulatedSensors(sensor_config, clock, timings, seed)
    intents = SimulatedIntents(configs, independent_updates=True, print_fn=log)
    glove = SimulatedGlove(clock, timings["glove"], backend, seed, log)
    session = None if real_robot else fake_session(clock=clock, profile=simulation_profile())
    options = ["--execute", "--duration", str(duration), "--aurora-profile", str(aurora_profile)] if real_robot else []
    args = control_args(["--robot", robot_backend, "--side", "right", "--source-side", "right", *options])
    robot = RobotRuntime(args, configs, clock=clock, print_fn=log)
    actual_factory, actual_apply, actual_prepare = factory.create_robot, robot.apply, robot.prepare

    def prepare():
        ready = actual_prepare()  # Real backend retains its existing interactive YES confirmation.
        if ready:
            origin = clock.monotonic()
            for source in (*sensors.imu_sources.values(), glove.schedule, *([sensors.source] if sensors.source else [])):
                source.reset(origin)
        return ready

    robot.prepare = prepare

    def offline_factory(backend, config, **kwargs):
        if backend != "aurora-fake":
            raise ValueError("simulation only supports aurora-fake")
        return actual_factory(backend, config, session=session, **kwargs)

    def record(intent, dt, *, hand_targets):
        before = None if real_robot else len(session.fake_client.commands)
        targets = actual_apply(intent, dt, hand_targets=hand_targets)
        if not real_robot and len(session.fake_client.commands) != before + 1:
            raise RuntimeError("control cycle did not publish exactly one fake SDK command")
        now = clock.monotonic()
        sleeve = sensors._last_sleeve
        row = dict(time_s=now - sensors.imu_sources[sensors.shoulder_names[0]].origin,
                   sleeve_sequence=None if sleeve is None else sleeve.sequence_id,
                   sleeve_age_ms=None if sleeve is None else (now - sleeve.timestamp) * 1000,
                   glove_timestamp=glove.reading.timestamp, glove_age_ms=(now - glove.reading.timestamp) * 1000,
                   shoulder_rad=intent.shoulder_flexion_rad, rotation_rad=intent.upper_arm_rotation_rad,
                   elbow_rad=intent.elbow_flexion, hand_rad=hand_targets["index_flexion"])
        row.update({f"submitted_{key}": targets.get(key) for key in (
            "shoulder_flexion", "upper_arm_rotation", "elbow_flexion", "index_flexion")})
        rows.append(row)
        return targets

    robot.apply = record
    app = ModelControlApp(sensors=sensors, intents=intents, robot=robot, glove=glove,
                          phase3=phase3, phase4=configs.phase4, duration=duration, clock=clock,
                          print_fn=log, telemetry=SimpleNamespace(report=lambda **_: None))
    with ExitStack() as stack:
        if not real_robot:
            stack.enter_context(patch("time.monotonic", clock.monotonic))
            stack.enter_context(patch("time.sleep", clock.sleep))
            stack.enter_context(patch.object(factory, "create_robot", offline_factory))
        code = app.run()
    intervals = [(b["time_s"] - a["time_s"]) * 1000 for a, b in zip(rows, rows[1:])]
    checks = {"commands_published": bool(rows)}
    if real_robot:
        checks["publish_gap_within_budget"] = bool(intervals) and max(intervals) <= max_gap_ms
    else:
        checks["cleanup"] = session.fake_client.closed
        checks["uniform_virtual_ticks"] = bool(intervals) and max(abs(v - 10.) for v in intervals) < 1e-6
    for source, key, target in (("sleeve", "sleeve_sequence", "elbow_rad"),
                                ("glove", "glove_timestamp", "hand_rad")):
        checks[f"{source}_cached_target"] = all(a[target] == b[target]
            for a, b in zip(rows, rows[1:]) if a[key] == b[key])
        stale_rows = [r for r in rows if r[f"{source}_age_ms"] is not None
                      and r[f"{source}_age_ms"] > (timeout_ms if source == "sleeve" else 250.) + 1e-6]
        if stale_rows:
            checks[f"imu_updates_during_{source}_stale"] = len({r["shoulder_rad"] for r in stale_rows}) > 1
    expected_fault = name == "imu_timeout"
    checks["expected_outcome"] = (code == 1 and "shoulder IMU pair is stale" in (app.fault or "")) if expected_fault else code == 0
    if name in ("sleeve_dropout", "combined"):
        checks["sleeve_timeout_exercised"] = any((r["sleeve_age_ms"] or 0) > timeout_ms for r in rows)
    if name in ("glove_dropout", "imu_only_glove_dropout", "combined"):
        checks["glove_timeout_exercised"] = any(r["glove_age_ms"] > 250. for r in rows)
    for source, timing in timings.items():
        if source != "imu" and timing.pauses:
            end = max(end for _, end in timing.pauses)
            checks[f"{source}_recovered"] = any(r["time_s"] - r[f"{source}_age_ms"] / 1000. >= end for r in rows)
    return dict(scenario=name, passed=all(checks.values()), exit_code=code, fault=app.fault,
                simulated_duration_s=duration, control_frames=len(rows), max_publish_interval_ms=max(intervals, default=0.),
                timings={key: dict(period_ms=value.period_ms, jitter_ms=value.jitter_ms, pauses=value.pauses)
                         for key, value in timings.items()}, seed=seed, glove_backend=backend,
                sensor_timeout_ms=timeout_ms, robot_backend=robot_backend,
                mean_publish_interval_ms=sum(intervals) / len(intervals) if intervals else 0.,
                p95_publish_interval_ms=sorted(intervals)[int(.95 * (len(intervals) - 1))] if intervals else 0.,
                sleeve_stale_ticks=sum((r["sleeve_age_ms"] or 0) > timeout_ms for r in rows),
                glove_stale_ticks=sum(r["glove_age_ms"] > 250. for r in rows), checks=checks, logs=logs), rows


SCENARIOS = ("normal", "sleeve250", "sleeve_dropout", "glove_dropout", "combined",
             "imu_only_glove_dropout", "imu_timeout")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Synthetic sensor delay tests; offline by default, optional real Aurora with --execute.")
    parser.add_argument("--scenario", choices=("all", "custom", *SCENARIOS), default="all")
    parser.add_argument("--duration", type=float, default=4., help="Simulated seconds, not wall time")
    parser.add_argument("--glove-backend", choices=("bend5", "bend6"), default="bend6")
    parser.add_argument("--timeout-ms", type=float, default=200., help="Simulated Sleeve/IMU timeout; does not change config files")
    parser.add_argument("--sleeve-ms", type=float, default=10.)
    parser.add_argument("--imu-ms", type=float, default=10.)
    parser.add_argument("--glove-ms", type=float, default=10.)
    parser.add_argument("--jitter-ms", type=float, default=0., help="custom: uniform additional interarrival delay")
    parser.add_argument("--no-sleeve", action="store_true", help="custom: disable Sleeve")
    parser.add_argument("--dropout", action="append", default=[], metavar="SOURCE:START:DURATION", help="custom: skip arrivals for sleeve/imu/glove")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--robot", choices=("aurora-fake", "aurora"), default="aurora-fake")
    parser.add_argument("--aurora-profile", type=Path, help="Real robot's verified configuration")
    parser.add_argument("--execute", action="store_true", help="Enable real robot setup; interactive YES is still required")
    parser.add_argument("--max-gap-ms", type=float, default=50., help="Real run fails its timing check above this inter-command gap")
    parser.add_argument("--output-dir", type=Path, help="Write summary.json and per-scenario frame CSV files")
    args = parser.parse_args(argv)
    for key in ("duration", "timeout_ms", "sleeve_ms", "imu_ms", "glove_ms", "max_gap_ms"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(f"--{key.replace('_', '-')} must be finite and positive")
    if not math.isfinite(args.jitter_ms) or args.jitter_ms < 0:
        parser.error("--jitter-ms must be finite and nonnegative")
    if args.robot == "aurora":
        if args.aurora_profile is None:
            parser.error("--robot aurora requires --aurora-profile")
        if args.scenario == "all":
            parser.error("real robot tests require one explicit --scenario")
        if args.duration > 60:
            parser.error("real robot tests require --duration <= 60 seconds")
    elif args.execute:
        parser.error("--execute is only for --robot aurora")
    if args.scenario != "custom" and (args.no_sleeve or args.dropout or args.jitter_ms
            or any(getattr(args, f"{key}_ms") != 10. for key in ("sleeve", "imu", "glove"))):
        parser.error("timing overrides, --dropout and --no-sleeve require --scenario custom")
    args.pauses = {key: [] for key in ("sleeve", "imu", "glove")}
    for value in args.dropout:
        try:
            source, start, duration = value.split(":")
            start, duration = float(start), float(duration)
            if source not in args.pauses or not all(map(math.isfinite, (start, duration))) or start <= 0 or duration <= 0:
                raise ValueError()
            args.pauses[source].append((start, start + duration))
        except ValueError:
            parser.error("--dropout requires sleeve/imu/glove:positive_start:positive_duration")
    return args


def main(argv=None):
    args = parse_args(argv)
    reports = []
    if args.robot == "aurora":
        print("REAL AURORA / SYNTHETIC INPUT: right shoulder ~10-30deg, elbow ~18-72deg (if enabled), "
              "upper-arm rotation +/-10deg, fingers ~20-80% curl; robot limits still apply.")
        print(f"Scenario={args.scenario}; duration={args.duration:g}s; profile={args.aurora_profile}")
        if not args.execute:
            print("NO MOTION: preview only. Add --execute to connect; the normal YES confirmation is still required.")
            return 0
        print("Real clock and real feedback; the delay schedule starts after YES and robot preparation.")
    else:
        print("OFFLINE SIMULATION: virtual clock, synthetic calibration, fake Aurora SDK; no serial/network hardware.")
    print(f"control=100Hz, Sleeve/IMU timeout={args.timeout_ms:g}ms, glove timeout=250ms")
    for name in SCENARIOS if args.scenario == "all" else (args.scenario,):
        timings = {key: Timing() for key in ("sleeve", "imu", "glove")}
        if name == "custom":
            timings = {key: Timing(getattr(args, f"{key}_ms"), args.jitter_ms, tuple(args.pauses[key])) for key in timings}
            if args.no_sleeve:
                del timings["sleeve"]
        elif name == "sleeve250":
            timings["sleeve"] = Timing(250.)
        else:
            for key in timings:
                if name == f"{key}_dropout" or (name == "combined" and key != "imu") or (name == "imu_timeout" and key == "imu"):
                    timings[key] = Timing(pauses=((1., 2.5),))
            if name == "imu_only_glove_dropout":
                del timings["sleeve"]
                timings["glove"] = Timing(pauses=((1., 2.5),))
        report, rows = run_scenario(name, timings, duration=args.duration, timeout_ms=args.timeout_ms,
                                    backend=args.glove_backend, seed=args.seed, robot_backend=args.robot,
                                    aurora_profile=args.aurora_profile, execute=args.execute, max_gap_ms=args.max_gap_ms)
        reports.append(report)
        print(f"{'PASS' if report['passed'] else 'FAIL'} {name}: frames={report['control_frames']}, "
              f"max_gap={report['max_publish_interval_ms']:.3f}ms, sleeve_stale={report['sleeve_stale_ticks']}, "
              f"glove_stale={report['glove_stale_ticks']}")
        for line, count in Counter(line for line in report["logs"] if line.startswith(("WARNING", "ERROR"))).items():
            print(f"  {line}" + (f" (x{count})" if count > 1 else ""))
        if not report["passed"]:
            print("  failed checks: " + ", ".join(key for key, ok in report["checks"].items() if not ok))
        if args.output_dir is not None:
            args.output_dir.mkdir(parents=True, exist_ok=True)
            with (args.output_dir / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
                if rows:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
    if args.output_dir is not None:
        (args.output_dir / "summary.json").write_text(json.dumps(reports, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Reports: {args.output_dir.resolve()}")
    return 0 if all(report["passed"] for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

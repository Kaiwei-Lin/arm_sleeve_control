"""CLI and configuration for model control; loading never opens hardware."""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, replace
from pathlib import Path

from sleeve_arm.config import (
    DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, DEFAULT_PHASE4_CONFIG_PATH,
    DEFAULT_SENSOR_CONFIG_PATH, load_phase3_config, load_phase4_config,
    load_robot_config, load_sensor_config,
    Phase3Config, Phase4Config, SensorConfig, RobotConfig, Phase3ElbowConfig,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Selectable dual-IMU or three-flex shoulder control; real motion requires --execute.")
    parser.add_argument("--sleeve", choices=("fake", "real"), default="real")
    parser.add_argument("--imus", choices=("fake", "real"), default="real")
    parser.add_argument("--robot", choices=("fake", "dymotor", "aurora", "aurora-fake"), default="dymotor")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--aurora-profile", type=Path)
    parser.add_argument("--arm-side", "--side", dest="side", choices=("left", "right"))
    parser.add_argument("--source-side", choices=("left", "right"))
    parser.add_argument("--confirm", choices=("EXECUTE_AURORA",), help="deprecated; does not replace interactive YES")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument(
        "--shoulder-predictor",
        choices=("dual_imu", "flexarm_estimator"),
        help="override predictor.backend for shoulder estimation",
    )
    parser.add_argument(
        "--reuse-calibration",
        action="store_true",
        help="flexarm_estimator only: reuse its configured calibration file",
    )
    parser.add_argument(
        "--calibration-seconds",
        type=float,
        help="override selected shoulder and upper-arm-rotation calibration durations",
    )
    parser.add_argument(
        "--calibration-output",
        type=Path,
        help="flexarm_estimator only: override its calibration output file",
    )
    parser.add_argument("--library", type=Path)
    parser.add_argument(
        "--bridge-diagnostics",
        action="store_true",
        help="print raw C-side PVCT values during DyMotor reads",
    )
    parser.add_argument(
        "--imu-debug",
        action="store_true",
        help="include available IMU quaternions in 1 Hz telemetry",
    )
    parser.add_argument("--prepare-aurora-fsm", action="store_true",
                        help="explicitly prepare PdStand after interactive confirmation")
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be positive")
    if args.calibration_seconds is not None and (not math.isfinite(args.calibration_seconds) or args.calibration_seconds <= 0):
        parser.error("--calibration-seconds must be positive")
    is_aurora = args.robot in ("aurora", "aurora-fake")
    if is_aurora:
        if args.side != "right" or args.source_side != "right":
            parser.error("current shoulder predictors require explicit --source-side right --side right; cross-side mapping is unverified")
        if args.robot == "aurora" and args.aurora_profile is None:
            parser.error("Aurora requires --aurora-profile")
        if args.execute and args.duration is None:
            parser.error("Aurora execute requires a bounded --duration")
    if args.robot in ("dymotor", "aurora") and args.execute:
        if args.sleeve != "real":
            parser.error("real robot execution requires --sleeve real")
        if args.imus != "real":
            parser.error("real robot execution requires --imus real")

    if args.prepare_aurora_fsm and (args.robot not in ("aurora", "aurora-fake") or not args.execute):
        parser.error("--prepare-aurora-fsm requires --robot aurora/aurora-fake --execute")
    return args


@dataclass(frozen=True)
class ModelControlConfigs:
    phase3: Phase3Config | None = None
    phase4: Phase4Config | None = None
    sensors: SensorConfig | None = None
    robot: RobotConfig | None = None
    elbow: Phase3ElbowConfig | None = None
    shoulder_backend: str | None = None
    offline_preview: bool = False


def load_configs(args):
    # Preserve offline Aurora preview: no sensor/model files or hardware required.
    if args.robot == "aurora" and not args.execute:
        return ModelControlConfigs(offline_preview=True)
    phase3 = load_phase3_config(args.phase3_config)
    phase4 = load_phase4_config(args.phase4_config)
    sensors = load_sensor_config(args.sensor_config)
    backend = args.shoulder_predictor or phase4.predictor_backend
    if backend not in ("dual_imu", "flexarm_estimator"):
        raise ValueError("run_model_control shoulder predictor must be dual_imu or flexarm_estimator")
    if backend == "flexarm_estimator":
        if phase4.flex_model is None:
            raise ValueError("phase4 config does not define predictor.flexarm_estimator")
        if not phase4.flex_model.model_dir.is_dir():
            raise ValueError(f"FlexArm model directory was not found: {phase4.flex_model.model_dir}")
    elif args.reuse_calibration or args.calibration_output is not None:
        raise ValueError("--reuse-calibration and --calibration-output require the flexarm_estimator shoulder predictor")
    elbow = phase3.elbow
    if args.sleeve == "fake" and elbow.input_min is None:
        elbow = replace(elbow, input_min=0.0, input_max=2.0, angle_min_deg=0.0, angle_max_deg=90.0)
    robot = None if args.robot in ("aurora", "aurora-fake") else load_robot_config(args.robot_config)
    return ModelControlConfigs(phase3, phase4, sensors, robot, elbow, backend)

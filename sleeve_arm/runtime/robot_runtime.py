"""MotionIntent boundary: backend creation, mapping, safe streaming and shutdown."""
from __future__ import annotations

from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.control.mapper import AuroraIntentMapper
from sleeve_arm.control.safety import clamp_position
from sleeve_arm.domain import MotionIntent
from sleeve_arm.robot import DyMotorArm
from sleeve_arm.robot import factory


class RobotRuntime:
    def __init__(self, args, configs, *, clock=None, input_fn=None, print_fn=print):
        self.args = args
        self.configs = configs
        self.clock = clock
        self.input_fn = input_fn
        self.print = print_fn
        self.backend = args.robot
        self.is_aurora = self.backend in ("aurora", "aurora-fake")
        self.stream_between_samples = self.is_aurora
        # Preserve the existing fail-fast streaming policy as data for the app.
        self.fail_fast = self.is_aurora
        self.offline_preview = configs.offline_preview
        self.motion_enabled = self.backend in ("fake", "aurora-fake") or args.execute
        self.robot = None
        self.controller = None
        self.mapper = None
        self.startup = {}

    def describe_preview(self):
        from sleeve_arm.control.aurora_motion import preview_joint
        from sleeve_arm.robot.aurora_profile import load_aurora_profile
        profile = load_aurora_profile(self.args.aurora_profile)
        self.print("Aurora NO MOTION: offline preview only. Use aurora-fake to test the sensor chain.")
        self.print(preview_joint(profile, side=self.args.side, joint="shoulder_flexion", angle_deg=0))

    def connect(self):
        config = self.configs.robot
        if self.backend == "dymotor" and self.args.execute:
            controlled = ["shoulder_flexion", "shoulder_abduction", "elbow_flexion"]
            if self.configs.sensors.upper_arm_rotation.enabled:
                controlled.append("upper_arm_rotation")
            for name in controlled:
                joint = config.joints[name]
                if joint.zero_position is None or joint.min_position is None or joint.max_position is None:
                    raise ValueError(f"{name}: absolute model control requires calibrated zero_position, min_position, and max_position")
        self.robot = factory.create_robot(
            self.backend, config, library_path=self.args.library, diagnostics=self.args.bridge_diagnostics,
            profile=self.args.aurora_profile, side=self.args.side,
            execute=self.motion_enabled, operator_confirmed=False,
        )
        self.controller = SafeArmController(self.robot, self.robot.config, clock=self.clock)
        self.controller.connect()
        self.controller.read_joint_states()
        if self.is_aurora:
            limits = {
                name: dict(command_rad_s=joint.max_velocity,
                           feedback_rad_s=joint.max_feedback_velocity,
                           step_rad=joint.max_position_step)
                for name, joint in self.robot.config.joints.items()
            }
            self.print(f"Aurora streaming limits: {limits}; control_hz={self.configs.phase3.control_hz}")
        if isinstance(self.robot, DyMotorArm):
            self.print(f"loaded_so: {self.robot.loaded_library_path}")

    def prepare(self):
        self.startup = self.feedback()
        self.mapper = (AuroraIntentMapper(self.robot, source_side=self.args.source_side, target_side=self.args.side)
                       if self.is_aurora else ArmMapper(self.configs.elbow, self.startup))
        if not self.motion_enabled:
            self.print("DRY RUN: vendor startup used Servo On; no post-startup model target is sent.")
            return True
        if self.is_aurora:
            if self.backend == "aurora":
                self.print(f"Aurora semantic startup positions: {self.startup}")
                action = "prepare PdStand FSM 2 and enable streaming" if self.args.prepare_aurora_fsm else "enable streaming in current PdStand"
                if (self.input_fn or input)(f"Type YES to {action}: ") != "YES":
                    self.print("Cancelled: NO MOTION")
                    return False
            self.robot.prepare_control_mode(
                prepare_fsm=self.args.prepare_aurora_fsm, operator_confirmed=True,
            )
            # Operator delay/FSM preparation may change the pose. Seed from fresh
            # complete feedback after confirmation, never the old printed pose.
            self.startup = self.feedback()
        self.controller.enable()
        self.controller.set_joint_positions(self.startup, dt=1.0 / self.configs.phase3.control_hz)
        return True

    def apply(self, intent: MotionIntent, dt: float):
        if not self.motion_enabled:
            return self.preview(intent, dt)
        return self.controller.set_joint_positions(self._map_targets(intent), dt=dt)

    def preview(self, intent: MotionIntent, dt: float):
        return self.controller.preview_positions(self._map_targets(intent), dt=dt)

    def _map_targets(self, intent: MotionIntent):
        targets = self.mapper.map(intent)
        if self.is_aurora:
            # Sensor estimates may exceed robot travel. Saturate semantic angles
            # before the controller's unchanged range, slew and tracking checks.
            targets = {name: clamp_position(self.robot.config.joints[name], value)
                       for name, value in targets.items()}
        return targets

    def check_health(self):
        if self.is_aurora:
            self.controller.read_joint_states()

    def feedback(self):
        return {name: state.position for name, state in self.controller.read_joint_states().items()}

    def shutdown(self):
        if self.controller is not None:
            self.controller.shutdown()
        elif self.robot is not None:
            self.robot.close()


class PrintOnlyRuntime:
    """Consume semantic intents without creating a robot, controller or session."""

    offline_preview = False
    fail_fast = False
    stream_between_samples = False
    motion_enabled = False

    def __init__(self, *, print_fn=print):
        self.print = print_fn
        self.startup = {}

    def connect(self):
        self.print("PRINT ONLY: 仅显示传感器估计的手臂动作，不连接或控制机器人。")

    def prepare(self):
        return True

    def apply(self, intent: MotionIntent, dt: float):
        return {name: value for name, value in (
            ("shoulder_flexion", intent.shoulder_flexion_rad),
            ("shoulder_abduction", intent.shoulder_abduction_rad),
            ("elbow_flexion", intent.elbow_flexion),
            ("upper_arm_rotation", intent.upper_arm_rotation_rad),
        ) if value is not None}

    def check_health(self):
        pass

    def feedback(self):
        # Estimates are never presented as measured robot feedback.
        return {}

    def shutdown(self):
        pass


def create_robot_runtime(args, configs):
    if args.print_only:
        return PrintOnlyRuntime()
    return RobotRuntime(args, configs)

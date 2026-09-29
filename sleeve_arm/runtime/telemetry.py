"""One-way status formatting; never sends commands or reads hardware."""

import math


class Telemetry:
    def __init__(self, *, imu_debug=False, print_only=False, print_fn=print):
        self.imu_debug = imu_debug
        self.print_only = print_only
        self.print = print_fn

    def report(self, *, sample, intent, targets, feedback, stats, sensor_stats, diagnostics):
        rotation = diagnostics.get("rotation_result")
        extra = ""
        if rotation is not None and not self.print_only:
            extra = (
                f" rotation={rotation.difference_deg:+.2f}deg"
                f" world={rotation.world_filtered_deg:+.2f}deg"
                f" relative={rotation.relative_filtered_deg:+.2f}deg"
                f" imu_sync={rotation.sync_gap_ms:.2f}ms"
                f" rotation_rad={diagnostics['rotation_rad']:+.4f}"
                f" rotation_target={targets.get('upper_arm_rotation')}"
                f" rotation_position={feedback.get('upper_arm_rotation')}"
                f" imu_sync_rejected={diagnostics['sync_rejected']}"
                f" rotation_invalid={stats['rotation_invalid']}"
            )
        if self.imu_debug:
            for field, names in (("shoulder_frames", ("shoulder_arm", "shoulder_chest")),
                                 ("rotation_frames", ("rotation_upper", "rotation_reference"))):
                frames = diagnostics.get(field)
                if frames is not None:
                    for name, frame in zip(names, frames):
                        extra += f" {name}_q={(frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)}"
        if self.print_only:
            self.report_action(intent=intent, stats=stats, extra=extra)
            return
        tracking = {name: targets[name] - feedback[name] for name in targets}
        elapsed = max(stats["elapsed"], 1e-9)
        source = "imu" if sample.sleeve is None else "sleeve"
        self.print(
            f"state={stats['state']} {source}_fps={sensor_stats.estimated_fps:.1f} "
            f"control_fps={stats['cycles'] / elapsed:.1f} "
            f"model_fps={stats['predictions'] / elapsed:.1f} age_ms={stats['age'] * 1000:.1f} "
            f"predictor={diagnostics['backend']} direction={intent.model_action} "
            f"magnitude_deg={intent.angle_deg:.2f} confidence={intent.confidence:.3f} "
            f"inference_ms={intent.inference_ms:.3f} "
            f"targets_rad={targets} positions_rad={feedback} tracking_rad={tracking} "
            f"invalid={stats['invalid']} stale={stats['stale']}{extra}"
        )

    def report_action(self, *, intent, stats, extra):
        actions = {
            "Forward": "前抬", "Backward": "后伸", "Lateral": "侧抬",
            "Rest": "静止", "Transition": "过渡", "Unknown": "未知",
        }
        action = actions.get(intent.model_action, "未知")
        angles = " ".join(
            f"{label}={'未提供' if value is None else f'{math.degrees(value):+.1f}°'}"
            for label, value in (
                ("肩前屈/后伸", intent.shoulder_flexion_rad),
                ("肩外展", intent.shoulder_abduction_rad),
                ("肘屈曲", intent.elbow_flexion),
                ("上臂旋转", intent.upper_arm_rotation_rad),
            )
        )
        confidence = "未提供" if intent.confidence is None else f"{intent.confidence:.3f}"
        self.print(
            f"[PRINT ONLY] state={stats['state']} 动作={action}({intent.model_action or 'Unknown'}) "
            f"估计角度: {angles} 置信度={confidence} "
            f"age_ms={stats['age'] * 1000:.1f} invalid={stats['invalid']} "
            f"stale={stats['stale']} rotation_invalid={stats['rotation_invalid']}{extra}",
            flush=True,
        )

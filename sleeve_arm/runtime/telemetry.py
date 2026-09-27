"""One-way status formatting; never sends commands or reads hardware."""


class Telemetry:
    def __init__(self, *, imu_debug=False, print_fn=print):
        self.imu_debug = imu_debug
        self.print = print_fn

    def report(self, *, sample, intent, targets, feedback, stats, sensor_stats, diagnostics):
        tracking = {name: targets[name] - feedback[name] for name in targets}
        rotation = diagnostics.get("rotation_result")
        extra = ""
        if rotation is not None:
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
        elapsed = max(stats["elapsed"], 1e-9)
        self.print(
            f"state={stats['state']} sleeve_fps={sensor_stats.estimated_fps:.1f} "
            f"control_fps={stats['cycles'] / elapsed:.1f} "
            f"model_fps={stats['predictions'] / elapsed:.1f} age_ms={stats['age'] * 1000:.1f} "
            f"predictor={diagnostics['backend']} direction={intent.model_action} "
            f"magnitude_deg={intent.angle_deg:.2f} confidence={intent.confidence:.3f} "
            f"inference_ms={intent.inference_ms:.3f} "
            f"targets_rad={targets} positions_rad={feedback} tracking_rad={tracking} "
            f"invalid={stats['invalid']} stale={stats['stale']}{extra}"
        )

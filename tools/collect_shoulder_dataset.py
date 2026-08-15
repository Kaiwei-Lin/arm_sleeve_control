#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import PROJECT_ROOT
from sleeve_arm.shoulder_collection import ShoulderDatasetCollector, fake_sources
from sleeve_arm.shoulder_dataset_config import MOTIONS, load_shoulder_dataset_config


DEFAULT_CONFIG = PROJECT_ROOT / "configs" / "shoulder_dataset.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "data" / "shoulder"


class NonBlockingKeyboard:
    def __init__(self) -> None:
        self._windows = os.name == "nt"
        self._settings = None

    def __enter__(self) -> NonBlockingKeyboard:
        if not sys.stdin.isatty():
            raise RuntimeError("interactive collection needs a terminal; use --duration for timed mode")
        if not self._windows:
            import termios
            import tty

            self._settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        if not self._windows and self._settings is not None:
            import termios

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._settings)

    def poll(self) -> list[str]:
        if self._windows:
            import msvcrt

            keys: list[str] = []
            while msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ("\x00", "\xe0") and msvcrt.kbhit():
                    msvcrt.getwch()
                    continue
                keys.append(key)
            return keys

        import select

        keys = []
        while select.select([sys.stdin], [], [], 0.0)[0]:
            keys.append(sys.stdin.read(1))
        return keys


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Collect three flex channels and dual-IMU shoulder-quaternion ground truth; "
            "this program never connects to the robot."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--motion", choices=MOTIONS, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--duration", type=float, default=None, help="timed automatic repetition")
    parser.add_argument("--repetition", type=int, default=1, help="initial repetition number")
    parser.add_argument("--fake", action="store_true", help="use 30/60/100 Hz synthetic sources")
    parser.add_argument("--no-countdown", action="store_true", help="skip only the 3-2-1 countdown")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.repetition <= 0:
        parser.error("--repetition must be positive")

    config = load_shoulder_dataset_config(args.config)
    kwargs = {}
    if args.fake:
        sleeve, torso, arm, activate = fake_sources(
            args.motion,
            int(config.sensors.sleeve.expected_fields or 0),
        )
        kwargs = {
            "sleeve_source": sleeve,
            "torso_source": torso,
            "arm_source": arm,
            "on_repetition_start": activate,
        }
    collector = ShoulderDatasetCollector(config, args.motion, args.output, **kwargs)
    if args.duration is not None:
        collector.run(
            duration_s=args.duration,
            initial_repetition=args.repetition,
            countdown=not args.no_countdown,
        )
    else:
        with NonBlockingKeyboard() as keyboard:
            collector.run(
                initial_repetition=args.repetition,
                key_reader=keyboard.poll,
                countdown=not args.no_countdown,
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

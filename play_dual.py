"""Play two keyframe sequences in parallel with a configurable phase offset.

Usage:
  python play_dual.py [--offset SECONDS] [--loop] [--left FILE] [--right FILE]

Both sequences start at the same wall-clock time. The right sequence's
internal phase is shifted by `offset` seconds — positive values make the
right leg *lead* the left (i.e. right is `offset` seconds ahead); negative
values make it lag.

Each side runs its own `advance_keyframe_sequence` independently, so the
two files can have different lengths or total durations.
"""

import json
import os
import sys
import time
from pylx16a.lx16a import *

from util import advance_keyframe_sequence

LX16A.initialize("/dev/ttyUSB0")

SERVO_CONFIG = {
    "left": {
        "front": {"id": 4, "limits": (203.76, 102.96)},
        "rear": {"id": 5, "limits": (236.64, 163.68)},
        "hip": {"id": 6, "limits": (63.36, 155.76)},
    },
    "right": {
        "front": {"id": 3, "limits": (89.04, 189.84)},
        "rear": {"id": 2, "limits": (66.00, 138.96)},
        "hip": {"id": 1, "limits": (222.0, 123.12)},
    },
}

SERVO_NAMES = {
    cfg["id"]: f"{side}_{part}"
    for side, parts in SERVO_CONFIG.items()
    for part, cfg in parts.items()
}

TIME_STEP = 0.02

servos = {}
try:
    for side in SERVO_CONFIG.values():
        for servo_config in side.values():
            servos[servo_config["id"]] = {
                "id": servo_config["id"],
                "limits": servo_config["limits"],
                "servo": LX16A(servo_config["id"]),
            }
except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    sys.exit(1)


def load_sequence(path):
    if not os.path.exists(path):
        print(f"file not found: {path}")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def read_current_pose():
    return {
        SERVO_NAMES[sid]: servo["servo"].get_physical_angle()
        for sid, servo in servos.items()
    }


def enable_all_torque():
    for servo in servos.values():
        servo["servo"].enable_torque()


def parse_args(argv):
    left_path = "keyframes_left.json"
    right_path = "keyframes_right.json"
    offset = 0.0
    loop = False
    it = iter(argv[1:])
    for arg in it:
        if arg == "--left":
            left_path = next(it)
        elif arg == "--right":
            right_path = next(it)
        elif arg == "--offset":
            offset = float(next(it))
        elif arg == "--loop":
            loop = True
        else:
            print(f"unknown arg: {arg}")
            sys.exit(1)
    return left_path, right_path, offset, loop


def play_dual(left_frames, right_frames, offset=0.0, loop=False):
    if not left_frames and not right_frames:
        print("both sequences are empty")
        return

    enable_all_torque()

    # Each side sees only its own servos, so the two calls don't fight.
    servo_map = {SERVO_NAMES[sid]: servo for sid, servo in servos.items()}

    left_total = sum(f["max_duration"] for f in left_frames) / 1000
    right_total = sum(f["max_duration"] for f in right_frames) / 1000

    initial = read_current_pose()
    left_initial = {k: v for k, v in initial.items() if k.startswith("left_")}
    right_initial = {k: v for k, v in initial.items() if k.startswith("right_")}
    left_last = left_frames[-1]["servo_angles"] if left_frames else {}
    right_last = right_frames[-1]["servo_angles"] if right_frames else {}

    print(
        f"left: {len(left_frames)} frame(s) ({left_total:.2f}s)  "
        f"right: {len(right_frames)} frame(s) ({right_total:.2f}s)  "
        f"offset: {offset:+.2f}s  {'looping' if loop else 'one-shot'}"
    )
    print("Ctrl+C to stop")

    t = 0.0
    left_step = 0
    right_step = 0
    try:
        while True:
            left_t = t
            right_t = t + offset

            if left_frames and (loop or left_t < left_total):
                left_prior = left_last if left_t >= left_total else left_initial
                _, left_step = advance_keyframe_sequence(
                    servo_map,
                    left_frames,
                    left_step,
                    left_t,
                    time_step=TIME_STEP,
                    speed_factor=1,
                    prior_pose=left_prior,
                )

            if right_frames and (loop or 0 <= right_t < right_total):
                # Before right_t reaches 0 we haven't started the right side yet;
                # after it exceeds right_total (one-shot) we stop advancing it.
                if right_t >= 0:
                    right_prior = (
                        right_last if right_t >= right_total else right_initial
                    )
                    _, right_step = advance_keyframe_sequence(
                        servo_map,
                        right_frames,
                        right_step,
                        right_t,
                        time_step=TIME_STEP,
                        speed_factor=1,
                        prior_pose=right_prior,
                    )

            t += TIME_STEP
            if not loop and t >= max(left_total, right_total - offset):
                print("done")
                return
            time.sleep(TIME_STEP)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        for servo in servos.values():
            servo["servo"].disable_torque()


def main():
    left_path, right_path, offset, loop = parse_args(sys.argv)
    left_frames = load_sequence(left_path)
    right_frames = load_sequence(right_path)
    play_dual(left_frames, right_frames, offset=offset, loop=loop)


if __name__ == "__main__":
    main()

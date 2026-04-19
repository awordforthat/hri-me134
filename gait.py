"""Walk a keyframe gait built from 4 recorded poses in poses.json.

Cycle: swing_left → plant_left → swing_right → plant_right → repeat.

Tune timing with the constants below, then run:
  python gait.py
"""

import json
import os
import time

from pylx16a.lx16a import LX16A, ServoTimeoutError

from util import advance_keyframe_sequence


# ─────────────────────────────────────────────
#  TUNING CONSTANTS
# ─────────────────────────────────────────────

# Duration of each transition (ms). "swing" is the flight phase leading into
# foot-plant; "plant" is the brief double-support transition before the other
# leg swings.
SWING_MS = 1000
PLANT_MS = 1000

# 0 = loop forever; N > 0 = run N full cycles then stop.
CYCLES = 0

# Optional: also dump the generated keyframe list to this file. Set to None to skip.
SAVE_KEYFRAMES_TO = None  # e.g. "keyframes_gait.json"

# ─────────────────────────────────────────────


POSES_FILE = "poses.json"
TIME_STEP = 0.02

SERVO_IDS = {
    "left_rear": 5,
    "left_front": 4,
    "left_hip": 6,
    "right_rear": 2,
    "right_front": 3,
    "right_hip": 1,
}

REQUIRED_POSES = ["swing_left", "plant_left", "swing_right", "plant_right"]


def build_keyframes(poses, swing_ms, plant_ms):
    missing = [n for n in REQUIRED_POSES if n not in poses]
    if missing:
        raise SystemExit(f"missing poses in {POSES_FILE}: {missing}")
    return [
        {
            "servo_angles": poses["swing_left"],
            "max_duration": swing_ms,
            "ease": "linear",
        },
        {
            "servo_angles": poses["plant_left"],
            "max_duration": plant_ms,
            "ease": "linear",
        },
        {
            "servo_angles": poses["swing_right"],
            "max_duration": swing_ms,
            "ease": "linear",
        },
        {
            "servo_angles": poses["plant_right"],
            "max_duration": plant_ms,
            "ease": "linear",
        },
    ]


def main():
    if not os.path.exists(POSES_FILE):
        raise SystemExit(f"{POSES_FILE} not found")
    with open(POSES_FILE) as f:
        poses = json.load(f)

    keyframes = build_keyframes(poses, SWING_MS, PLANT_MS)
    cycle_s = sum(kf["max_duration"] for kf in keyframes) / 1000

    if SAVE_KEYFRAMES_TO:
        with open(SAVE_KEYFRAMES_TO, "w") as f:
            json.dump(keyframes, f, indent=2)
        print(f"wrote keyframes to {SAVE_KEYFRAMES_TO}")

    LX16A.initialize("/dev/ttyUSB0")
    try:
        servo_objs = {name: LX16A(sid) for name, sid in SERVO_IDS.items()}
    except ServoTimeoutError as e:
        raise SystemExit(f"Servo {e.id_} not responding.")

    servo_map = {
        name: {"id": SERVO_IDS[name], "servo": s} for name, s in servo_objs.items()
    }

    # Initial pose = current servo angles, so the first swing_left interpolates
    # smoothly from wherever the robot is standing now.
    initial_pose = {name: servo_objs[name].get_physical_angle() for name in SERVO_IDS}

    print(
        f"gait cycle: {cycle_s:.2f}s  ({'looping' if CYCLES == 0 else f'{CYCLES} cycles'})"
    )
    print("Ctrl-C to stop")

    t = 0.0
    step = 0
    total_s = CYCLES * cycle_s if CYCLES > 0 else float("inf")
    try:
        for servo in servo_objs.values():
            servo.enable_torque()
        while t < total_s:
            _, step = advance_keyframe_sequence(
                servo_map,
                keyframes,
                step,
                t,
                time_step=TIME_STEP,
                speed_factor=1,
                prior_pose=initial_pose,
            )
            t += TIME_STEP
            time.sleep(TIME_STEP)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        for servo in servo_objs.values():
            servo.disable_torque()


if __name__ == "__main__":
    main()

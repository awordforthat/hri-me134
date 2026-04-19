"""Toggle between two poses with a keypress.

Press any key (except q) to move to the other pose. q or Ctrl-C to quit.
Edit the constants below to pick poses and timing, then run:
  python balance.py
"""

import json
import os
import sys
import termios
import time
import tty

from pylx16a.lx16a import LX16A, ServoTimeoutError

from five_bar_ik import make_left_leg_ik, make_right_leg_ik


# ─────────────────────────────────────────────
#  TUNING CONSTANTS
# ─────────────────────────────────────────────

# Each step: pose name, transition duration (ms), and optional IK forward
# offset to apply to one leg in that pose. "home" is loaded from
# servo_calibration.json; other names come from poses.json.
SEQUENCE = [
    {"pose": "home",        "ms": 1500},
    {"pose": "shift_left",  "ms": 1500},
    {"pose": "plant_right", "ms": 1500},
    {"pose": "swing_left",  "ms": 1500, "forward_leg": "right", "forward_cm": 4.0},
    {"pose": "plant_left",  "ms": 1500},
]

# ─────────────────────────────────────────────


POSES_FILE = "poses.json"
CAL_FILE = "servo_calibration.json"

SERVO_IDS = {
    "left_rear": 5,
    "left_front": 4,
    "left_hip": 6,
    "right_rear": 2,
    "right_front": 3,
    "right_hip": 1,
}


def resolve_pose(name, poses, cal):
    """POSE_A/POSE_B == 'home' reads homes from servo_calibration.json; any
    other name reads from poses.json."""
    if name == "home":
        return {servo: cal[servo]["home"] for servo in SERVO_IDS}
    if name not in poses:
        raise SystemExit(f"pose '{name}' not in {POSES_FILE}")
    return poses[name]


def offset_foot_forward(pose, leg, cm):
    """Return a copy of `pose` with `leg`'s foot shifted forward by `cm` in the
    leg frame (via forward kinematics + IK)."""
    if cm == 0:
        return pose
    ik = make_left_leg_ik() if leg == "left" else make_right_leg_ik()
    rear_name, front_name = f"{leg}_rear", f"{leg}_front"
    foot = ik.forward_foot(pose[rear_name], pose[front_name])
    if foot is None:
        print(f"warning: couldn't compute {leg} foot position; skipping offset")
        return pose
    x, z = foot
    x_new = x + cm * 10  # cm → mm
    sol = ik.solve(x_new, z)
    if sol is None:
        print(
            f"warning: {leg} foot target ({x_new:.1f}, {z:.1f}) unreachable; skipping offset"
        )
        return pose
    new_pose = dict(pose)
    new_pose[rear_name] = sol[0]
    new_pose[front_name] = sol[1]
    print(f"  offset {leg} foot: ({x:.1f}, {z:.1f}) → ({x_new:.1f}, {z:.1f})")
    return new_pose


def goto(servo_objs, pose, duration_ms):
    for name, angle in pose.items():
        servo_objs[name].move(int(max(0, min(240, angle))), duration_ms, wait=True)
    for servo in servo_objs.values():
        if servo._waiting_for_move:
            servo.move_start()
    time.sleep(duration_ms / 1000)


def main():
    if not os.path.exists(POSES_FILE):
        raise SystemExit(f"{POSES_FILE} not found")
    with open(POSES_FILE) as f:
        poses = json.load(f)
    with open(CAL_FILE) as f:
        cal = json.load(f)

    sequence = []
    for step in SEQUENCE:
        pose = resolve_pose(step["pose"], poses, cal)
        if "forward_leg" in step and step.get("forward_cm", 0):
            pose = offset_foot_forward(pose, step["forward_leg"], step["forward_cm"])
        sequence.append((step["pose"], pose, step["ms"]))

    LX16A.initialize("/dev/ttyUSB0")
    try:
        servo_objs = {name: LX16A(sid) for name, sid in SERVO_IDS.items()}
    except ServoTimeoutError as e:
        raise SystemExit(f"Servo {e.id_} not responding.")

    for servo in servo_objs.values():
        servo.enable_torque()

    cycle_names = " → ".join(name for name, _, _ in sequence)
    print(f"press any key to step through: {cycle_names} → ...  (q to quit)")
    name, pose, dur = sequence[0]
    print(f"moving to '{name}'...")
    goto(servo_objs, pose, dur)

    idx = 0
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == "q":
                print("\nquit")
                break
            idx = (idx + 1) % len(sequence)
            name, pose, dur = sequence[idx]
            print(f"moving to '{name}'...")
            goto(servo_objs, pose, dur)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        for servo in servo_objs.values():
            servo.disable_torque()


if __name__ == "__main__":
    main()

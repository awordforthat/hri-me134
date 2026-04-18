"""Sweep one foot between two (x, z) targets so you can watch the physical
motion.

Usage:
  python test_foot_sweep.py [left|right] [x|z]
  - x sweep: (-20,-110) ↔ (+20,-110) — should move forward/back
  - z sweep: (+8,-90)   ↔ (+8,-130)  — should move up/down

Default: left x.
"""

import json
import sys
import time

from pylx16a.lx16a import LX16A, ServoTimeoutError

from five_bar_ik import make_left_leg_ik, make_right_leg_ik


CAL_FILE = "servo_calibration.json"

SERVO_IDS = {
    "left_rear": 5,
    "left_front": 4,
    "left_hip": 6,
    "right_rear": 2,
    "right_front": 3,
    "right_hip": 1,
}

SWEEPS = {
    "x": ((-20.0, -110.0), (+20.0, -110.0)),
    "z": ((+8.0, -90.0), (+8.0, -130.0)),
}
SWEEP_MS = 1500
PAUSE_S = 1.0


def move_to_home(cal, servos, duration_ms=1500):
    print(f"Moving all servos to home ({duration_ms} ms)...")
    for name, servo in servos.items():
        target = int(max(0, min(240, cal[name]["home"])))
        servo.move(target, duration_ms, wait=True)
    for servo in servos.values():
        servo.move_start()
    time.sleep(duration_ms / 1000 + 0.5)


def move_foot(ik, rear_servo, front_servo, x, z, duration_ms):
    sol = ik.solve(x, z)
    if sol is None:
        print(f"  ({x:+.1f}, {z:+.1f}) UNREACHABLE")
        return
    rear_cmd, front_cmd = sol
    c = lambda v: int(max(0, min(240, v)))
    rear_servo.move(c(rear_cmd), duration_ms, wait=True)
    front_servo.move(c(front_cmd), duration_ms, wait=True)
    rear_servo.move_start()
    front_servo.move_start()
    print(f"  foot → ({x:+.1f}, {z:+.1f})  rear={rear_cmd:.1f}  front={front_cmd:.1f}")


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"
    axis = sys.argv[2] if len(sys.argv) > 2 else "x"
    if side not in ("left", "right") or axis not in SWEEPS:
        print(f"usage: {sys.argv[0]} [left|right] [x|z]")
        return
    pt_a, pt_b = SWEEPS[axis]

    LX16A.initialize("/dev/ttyUSB0")
    with open(CAL_FILE) as f:
        cal = json.load(f)

    try:
        servos = {name: LX16A(sid) for name, sid in SERVO_IDS.items()}
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} not responding. Exiting.")
        return

    rear = servos[f"{side}_rear"]
    front = servos[f"{side}_front"]
    ik = make_left_leg_ik() if side == "left" else make_right_leg_ik()

    try:
        move_to_home(cal, servos)
        expected = "forward/back" if axis == "x" else "up/down"
        print(f"\nSweeping {side} foot ({axis}) between {pt_a} and {pt_b}.")
        print(f"Expected physical motion: {expected}.\n")
        print("Ctrl-C to stop.")
        while True:
            move_foot(ik, rear, front, pt_a[0], pt_a[1], SWEEP_MS)
            time.sleep(SWEEP_MS / 1000 + PAUSE_S)
            move_foot(ik, rear, front, pt_b[0], pt_b[1], SWEEP_MS)
            time.sleep(SWEEP_MS / 1000 + PAUSE_S)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        for servo in servos.values():
            servo.disable_torque()


if __name__ == "__main__":
    main()

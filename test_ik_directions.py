"""
Test each servo's rotation direction one at a time.

For each servo (left_rear, left_front, right_rear, right_front):
  - Hold other servos at home.
  - Command THAT servo by a small delta (+10° in raw cmd units).
  - Ask the user to observe which way the upper link (proximal link from
     servo axle to elbow) rotated, and compare to what the IK expects.

Leg-frame convention:
  +x = forward (toward robot's front)
  +z = up
  Origin at REAR servo axle; FRONT servo axle is at (+26.5, 0).

A servo cmd increment corresponds to a link angle increment:
  cmd_delta = dir * theta_delta_degrees
So +10° cmd with dir=+1 means theta increases by 10° (CCW in leg frame).
CCW means: for the rear servo (at origin), the elbow moves from +x toward +z
(i.e. from pointing forward to pointing up).
For the front servo (at +x), the elbow also rotates CCW the same way.

If the observed rotation is CCW in leg frame → dir = +1 (keep).
If the observed rotation is CW in leg frame → dir = -1 (flip that servo).
"""

import json
import time

from pylx16a.lx16a import *

LX16A.initialize("/dev/ttyUSB0")

CAL_FILE = "servo_calibration.json"

# name -> (servo id, axle position label, what "CCW in leg frame" looks like)
SERVOS = [
    (
        "left_rear",
        5,
        "rear of LEFT leg (axle at leg-frame origin)",
        "upper link rotates from pointing-forward toward pointing-up",
    ),
    (
        "left_front",
        4,
        "front of LEFT leg (axle 26.5mm forward of rear)",
        "upper link rotates from pointing-forward toward pointing-up",
    ),
    (
        "right_rear",
        2,
        "rear of RIGHT leg",
        "upper link rotates from pointing-forward toward pointing-up",
    ),
    (
        "right_front",
        3,
        "front of RIGHT leg",
        "upper link rotates from pointing-forward toward pointing-up",
    ),
]

TEST_DELTA = 10  # degrees of cmd change to apply


def main():
    with open(CAL_FILE) as f:
        cal = json.load(f)

    servo_objs = {name: LX16A(sid) for name, sid, *_ in SERVOS}

    print("Moving all servos to home first...")
    for name, servo in servo_objs.items():
        servo.move(int(cal[name]["home"]), 800, wait=True)
    for servo in servo_objs.values():
        servo.move_start()
    time.sleep(1.2)

    results = {}
    for name, sid, where, ccw_desc in SERVOS:
        home = cal[name]["home"]
        target = home + TEST_DELTA
        if target > 240 or target < 0:
            target = home - TEST_DELTA
            delta_sign = -1
        else:
            delta_sign = +1

        print(f"\n=== {name} (id {sid}) — {where} ===")
        print(f"  CCW (dir=+1) means: {ccw_desc}")
        print(
            f"  Commanding {home:.1f} → {target:.1f} "
            f"({'+' if delta_sign>0 else '-'}{TEST_DELTA}° cmd change)"
        )
        input("  press Enter to move...")

        servo_objs[name].move(int(target), 800, wait=True)
        servo_objs[name].move_start()
        time.sleep(1.2)

        ans = (
            input(
                "  did the upper link rotate CCW (i.e. toward the +z/up direction)? [y/n]: "
            )
            .strip()
            .lower()
        )
        # If we had to flip the sign of the commanded delta (home too close to
        # the limit), invert the answer so it still reflects dir at +theta.
        if delta_sign < 0:
            ans = "n" if ans == "y" else "y"
        results[name] = 1 if ans == "y" else -1

        print("  returning to home...")
        servo_objs[name].move(int(home), 800, wait=True)
        servo_objs[name].move_start()
        time.sleep(1.2)

    print("\n=== Paste these into five_bar_ik.py ===")
    print(f"LEFT_LEG_REAR_DIR   = {results['left_rear']}")
    print(f"LEFT_LEG_FRONT_DIR  = {results['left_front']}")
    print(f"RIGHT_LEG_REAR_DIR  = {results['right_rear']}")
    print(f"RIGHT_LEG_FRONT_DIR = {results['right_front']}")

    for servo in servo_objs.values():
        servo.disable_torque()


if __name__ == "__main__":
    main()

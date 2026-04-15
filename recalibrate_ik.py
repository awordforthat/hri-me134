"""
Recalibrate IK servo offsets after remounting linkages.

Pose the robot on a flat surface with both feet planted and the body level.
Measure each foot's (x, z) in the leg frame:
  x = horizontal offset from rear servo axle to foot tip (forward positive, mm)
  z = vertical offset from servo axle plane down to foot (negative, mm)

Then run this script. It reads current home servo cmds from
servo_calibration.json and computes the four offsets you need to paste into
five_bar_ik.py.
"""

import json

from five_bar_ik import ServoCalibrator, make_left_leg_ik, make_right_leg_ik


CAL_FILE = "servo_calibration.json"


def _read_float(prompt):
    while True:
        raw = input(prompt)
        # Handle raw DEL chars if terminal isn't cooking input
        cleaned = []
        for ch in raw:
            if ch == "\x7f" or ch == "\b":
                if cleaned:
                    cleaned.pop()
            else:
                cleaned.append(ch)
        s = "".join(cleaned).strip()
        try:
            return float(s)
        except ValueError:
            print(f"    couldn't parse {s!r} — try again")


def prompt_xz(label):
    x = _read_float(f"  {label} foot x (mm, forward positive): ")
    z = _read_float(f"  {label} foot z (mm, negative = below servos): ")
    return x, z


def main():
    with open(CAL_FILE) as f:
        cal = json.load(f)

    left_rear_cmd = cal["left_rear"]["home"]
    left_front_cmd = cal["left_front"]["home"]
    right_rear_cmd = cal["right_rear"]["home"]
    right_front_cmd = cal["right_front"]["home"]

    print("Current home servo cmds:")
    print(f"  left:  rear={left_rear_cmd}  front={left_front_cmd}")
    print(f"  right: rear={right_rear_cmd}  front={right_front_cmd}")
    print()
    print("Pose robot on flat surface, body level, both feet planted.")
    print("Measure each foot's (x, z) in leg frame.")
    print()

    print("LEFT leg:")
    lx, lz = prompt_xz("left")
    print("RIGHT leg:")
    rx, rz = prompt_xz("right")
    print()

    left_offsets = ServoCalibrator(make_left_leg_ik()).calibrate_from_known_pose(
        lx, lz, left_rear_cmd, left_front_cmd
    )
    right_offsets = ServoCalibrator(make_right_leg_ik()).calibrate_from_known_pose(
        rx, rz, right_rear_cmd, right_front_cmd
    )

    if left_offsets and right_offsets:
        print()
        print("Paste these into five_bar_ik.py:")
        print(f"  LEFT_LEG_REAR_OFFSET   = {left_offsets[0]:.2f}")
        print(f"  LEFT_LEG_FRONT_OFFSET  = {left_offsets[1]:.2f}")
        print(f"  RIGHT_LEG_REAR_OFFSET  = {right_offsets[0]:.2f}")
        print(f"  RIGHT_LEG_FRONT_OFFSET = {right_offsets[1]:.2f}")


if __name__ == "__main__":
    main()

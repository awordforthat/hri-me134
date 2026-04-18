"""
Recalibrate IK servo offsets after remounting linkages.

Pose the robot on a flat surface with both feet planted and the body level.
Measure each foot's (x, z) in the leg frame:
  x = horizontal offset from rear servo axle to foot tip (forward positive, mm)
  z = vertical offset from servo axle plane down to foot (negative, mm)

Then run this script. It reads current home servo cmds from
servo_calibration.json, computes the four offsets, and writes them to
ik_calibration.json (loaded automatically by five_bar_ik.py — no manual edits
needed).
"""

import json
import time

from pylx16a.lx16a import LX16A, ServoTimeoutError

from five_bar_ik import ServoCalibrator, make_left_leg_ik, make_right_leg_ik


CAL_FILE = "servo_calibration.json"
IK_CAL_FILE = "ik_calibration.json"

SERVO_IDS = {
    "left_rear": 5,
    "left_front": 4,
    "left_hip": 6,
    "right_rear": 2,
    "right_front": 3,
    "right_hip": 1,
}


def move_to_home(cal, duration_ms=1500):
    LX16A.initialize("/dev/ttyUSB0")
    servos = {}
    try:
        for name, sid in SERVO_IDS.items():
            servos[name] = LX16A(sid)
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} not responding. Exiting.")
        raise SystemExit(1)

    print(f"Moving all servos to home pose ({duration_ms} ms)...")
    for name, servo in servos.items():
        target = int(max(0, min(240, cal[name]["home"])))
        servo.move(target, duration_ms, wait=True)
    for servo in servos.values():
        servo.move_start()
    time.sleep(duration_ms / 1000 + 0.5)
    print("At home pose. Servos are holding torque.\n")
    return servos


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

    servos = move_to_home(cal)

    try:
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
            with open(IK_CAL_FILE) as f:
                ik_cal = json.load(f)
            ik_cal["left"]["rear_offset"] = round(left_offsets[0], 2)
            ik_cal["left"]["front_offset"] = round(left_offsets[1], 2)
            ik_cal["left"]["home_foot"] = [lx, lz]
            ik_cal["right"]["rear_offset"] = round(right_offsets[0], 2)
            ik_cal["right"]["front_offset"] = round(right_offsets[1], 2)
            ik_cal["right"]["home_foot"] = [rx, rz]
            with open(IK_CAL_FILE, "w") as f:
                json.dump(ik_cal, f, indent=2)
            print()
            print(f"Wrote new offsets to {IK_CAL_FILE}:")
            print(f"  left:  rear={left_offsets[0]:.2f}  front={left_offsets[1]:.2f}")
            print(f"  right: rear={right_offsets[0]:.2f}  front={right_offsets[1]:.2f}")
            print()
            print("Done — five_bar_ik.py will pick these up automatically on next import.")
            print("No manual edits required.")
    finally:
        for servo in servos.values():
            servo.disable_torque()


if __name__ == "__main__":
    main()

"""
Servo calibration helper.

Pick a motor, then physically move it to min/max/home and press Enter at
each pose to capture the angle. Results are written to servo_calibration.json.
"""

import json
import os
import select
import sys
import termios
import time
import tty
from pylx16a.lx16a import *


LX16A.initialize("/dev/ttyUSB0")

SERVOS = {
    "left_front": 4,
    "left_rear": 5,
    "left_hip": 6,
    "right_front": 3,
    "right_rear": 2,
    "right_hip": 1,
}

OUTPUT_FILE = "servo_calibration.json"


def load_calibration():
    if not os.path.exists(OUTPUT_FILE):
        return {}
    with open(OUTPUT_FILE) as f:
        return json.load(f)


def save_calibration(data):
    with open(OUTPUT_FILE, "w") as f:
        json.dump(data, f, indent=2, sort_keys=True)


def capture_live_angle(servo, label):
    """
    Poll and print the current angle live. Return the angle when the user
    presses a key. Keeps prompting if the captured value is out of range.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            print(f"  Move to {label.upper()}, press any key to capture:")
            while True:
                angle = servo.get_physical_angle()
                sys.stdout.write(f"\r    current: {angle:7.2f}°   ")
                sys.stdout.flush()
                if select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.read(1)
                    break
                time.sleep(0.05)
            sys.stdout.write("\n")
            if 0 <= angle <= 240:
                return round(angle, 2)
            print(f"    REJECTED: {angle} is outside [0, 240]. Try again.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def calibrate_servo(name, servo_id):
    servo = LX16A(servo_id)
    servo.disable_torque()

    print(f"\nCalibrating {name} (id {servo_id})")
    print("Torque is OFF — pose the servo by hand.")

    results = {}
    for label in ("min", "max", "home"):
        angle = capture_live_angle(servo, label)
        results[label] = angle
        print(f"    {label} = {angle}")

    return results


def pick_servo():
    names = list(SERVOS.keys())
    print("\nAvailable servos:")
    for i, name in enumerate(names):
        print(f"  {i}: {name}")

    choice = input("Select servo (number or name), or 'q' to quit: ").strip()
    if choice == "q":
        return None
    if choice.isdigit() and 0 <= int(choice) < len(names):
        return names[int(choice)]
    if choice in SERVOS:
        return choice
    print("Invalid selection.")
    return pick_servo()


def wait_for_keypress():
    """Block until the user presses any key. Cbreak mode for single-char read."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def record_all_homes():
    """
    Disable torque on every servo, wait for the user to pose the whole robot
    at its home stance, then on a single keypress capture every servo's
    current angle and write it as the new "home" value in the JSON file.
    Existing min/max values are preserved.
    """
    servo_objs = {name: LX16A(servo_id) for name, servo_id in SERVOS.items()}
    for servo in servo_objs.values():
        servo.disable_torque()

    print("\nTorque is OFF on all servos.")
    print("Pose the robot at its home stance, then press any key to capture.")
    wait_for_keypress()

    captured = {}
    for name, servo in servo_objs.items():
        angle = round(servo.get_physical_angle(), 2)
        if not 0 <= angle <= 240:
            print(f"  WARNING: {name} read {angle}° (outside [0, 240]) — skipping")
            continue
        captured[name] = angle
        print(f"  {name}: {angle}")

    data = load_calibration()
    for name, angle in captured.items():
        data.setdefault(name, {})["home"] = angle
    save_calibration(data)
    print(f"\nSaved {len(captured)} home positions to {OUTPUT_FILE}")


def main():
    data = load_calibration()
    print(f"Loaded {len(data)} existing entries from {OUTPUT_FILE}")

    try:
        while True:
            name = pick_servo()
            if name is None:
                break
            data[name] = calibrate_servo(name, SERVOS[name])
            save_calibration(data)
            print(f"Saved {name} to {OUTPUT_FILE}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        for servo_id in SERVOS.values():
            try:
                LX16A(servo_id).disable_torque()
            except Exception:
                pass
        print("All torque disabled. Done.")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Servo calibration helper")
    parser.add_argument(
        "--homes",
        action="store_true",
        help="Capture every servo's current angle as the new home position on a single keypress.",
    )
    args = parser.parse_args()

    try:
        if args.homes:
            record_all_homes()
        else:
            main()
    finally:
        for servo_id in SERVOS.values():
            try:
                LX16A(servo_id).disable_torque()
            except Exception:
                pass

import json
import os
import sys
import termios
import time
import tty
from pylx16a.lx16a import *

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
SERVO_IDS_BY_NAME = {v: k for k, v in SERVO_NAMES.items()}

DEFAULT_MOVE_MS = 800

POSES_FILE = "poses.json"

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


def load_poses():
    if not os.path.exists(POSES_FILE):
        return {}
    with open(POSES_FILE) as f:
        return json.load(f)


def save_poses(poses):
    with open(POSES_FILE, "w") as f:
        json.dump(poses, f, indent=2, sort_keys=True)


def read_current_pose():
    return {
        SERVO_NAMES[sid]: servo["servo"].get_physical_angle()
        for sid, servo in servos.items()
    }


def goto_pose(pose, duration_ms=DEFAULT_MOVE_MS):
    for name, angle in pose.items():
        sid = SERVO_IDS_BY_NAME.get(name)
        if sid is None:
            print(f"  skipping unknown servo '{name}'")
            continue
        servo = servos[sid]["servo"]
        servo.enable_torque()
        servo.move(angle, duration_ms, wait=False)
    time.sleep(duration_ms / 1000)


def set_torque(enabled):
    for servo in servos.values():
        if enabled:
            servo["servo"].enable_torque()
        else:
            servo["servo"].disable_torque()


def prompt_line(prompt):
    """Temporarily restore cooked mode so we can read a full line of input."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        sys.stdout.write(prompt)
        sys.stdout.flush()
        return sys.stdin.readline().strip()
    finally:
        tty.setcbreak(fd)


def print_help():
    print()
    print("Keys:")
    print("  r  record current pose (you'll be prompted for a name)")
    print("  g  go to a saved pose (you'll be prompted for a name)")
    print("  t  toggle torque on/off")
    print("  p  print current servo angles")
    print("  l  list saved poses")
    print("  h  show this help")
    print("  q  quit")
    print()


def main():
    poses = load_poses()
    set_torque(True)
    torque_enabled = True
    print(f"Loaded {len(poses)} pose(s) from {POSES_FILE}")
    print_help()
    print(f"torque: {'ON' if torque_enabled else 'OFF'}")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == "q":
                print("\nexiting")
                break
            elif ch == "t":
                torque_enabled = not torque_enabled
                set_torque(torque_enabled)
                print(f"torque: {'ON' if torque_enabled else 'OFF'}")
            elif ch == "r":
                pose = read_current_pose()
                name = prompt_line("\npose name: ")
                if not name:
                    print("cancelled (empty name)")
                    continue
                if name in poses:
                    confirm = prompt_line(f"'{name}' exists. overwrite? [y/N]: ")
                    if confirm.lower() != "y":
                        print("cancelled")
                        continue
                poses[name] = pose
                save_poses(poses)
                print(f"saved '{name}':")
                for k, v in pose.items():
                    print(f"  {k}: {v:.2f}")
            elif ch == "g":
                name = prompt_line("\ngo to pose: ")
                if not name:
                    print("cancelled")
                    continue
                if name not in poses:
                    print(f"no pose named '{name}'")
                    continue
                if not torque_enabled:
                    set_torque(True)
                    torque_enabled = True
                    print("torque: ON (re-enabled for move)")
                    time.sleep(0.1)
                print(f"moving to '{name}'")
                goto_pose(poses[name])
            elif ch == "p":
                print()
                for k, v in read_current_pose().items():
                    print(f"  {k}: {v:.2f}")
            elif ch == "l":
                print()
                if not poses:
                    print("  (no poses saved)")
                else:
                    for name in sorted(poses):
                        print(f"  {name}")
            elif ch == "h":
                print_help()
    except KeyboardInterrupt:
        print("\nexiting")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        for servo in servos.values():
            servo["servo"].disable_torque()


if __name__ == "__main__":
    main()

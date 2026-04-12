"""Manually capture a keyframe sequence by posing the robot.

Usage: python record_keyframes.py [sequence_file.json] [--side left|right]

If --side is given, only that side's servos are read/written in keyframes
(the other side is ignored so you can record each leg independently).

Keys:
  a  add current pose as the next keyframe (auto-saves)
  u  undo last keyframe
  l  list keyframes in the current sequence
  p  print current servo angles
  t  toggle torque on/off
  c  clear the sequence (asks for confirmation)
  h  show this help
  q  quit

Output format matches walk.py / advance_keyframe_sequence:
  [{"servo_angles": {name: angle, ...}, "max_duration": 1000, "ease": "linear"}, ...]

`max_duration` and `ease` are written with defaults so later tooling can tune
per-frame timing and easing without re-recording.
"""

import json
import os
import select
import sys
import termios
import time
import tty
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
SERVO_IDS_BY_NAME = {v: k for k, v in SERVO_NAMES.items()}

DEFAULT_DURATION_MS = 1000
DEFAULT_EASE = "linear"
PLAYBACK_TIME_STEP = 0.02

RECORDED_SIDES = None  # set in main(); either {"left"}, {"right"}, or {"left","right"}

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
        return []
    with open(path) as f:
        return json.load(f)


def save_sequence(path, frames):
    with open(path, "w") as f:
        json.dump(frames, f, indent=2)


def servo_belongs_to_recorded_side(servo_name):
    if RECORDED_SIDES is None:
        return True
    return any(servo_name.startswith(f"{side}_") for side in RECORDED_SIDES)


def read_current_pose():
    return {
        SERVO_NAMES[sid]: round(servo["servo"].get_physical_angle(), 2)
        for sid, servo in servos.items()
        if servo_belongs_to_recorded_side(SERVO_NAMES[sid])
    }


def move_to_pose(pose, duration_ms=800):
    for name, angle in pose.items():
        sid = SERVO_IDS_BY_NAME.get(name)
        if sid is None:
            continue
        servo = servos[sid]["servo"]
        servo.enable_torque()
        servo.move(angle, duration_ms, wait=False)
    time.sleep(duration_ms / 1000)


def parse_index(raw, frames):
    try:
        idx = int(raw)
    except ValueError:
        print(f"not a number: {raw!r}")
        return None
    if idx < 0:
        idx += len(frames)
    if not 0 <= idx < len(frames):
        print(f"index out of range (0..{len(frames) - 1})")
        return None
    return idx


def play_sequence(frames, speed_factor=1.0, loop=False):
    if not frames:
        print("nothing to play")
        return
    set_torque(True)
    servo_map = {SERVO_NAMES[sid]: servo for sid, servo in servos.items()}
    total = sum(f["max_duration"] for f in frames) / 1000 * speed_factor
    initial_pose = read_current_pose()
    last_frame_pose = frames[-1]["servo_angles"]
    if loop:
        print(f"looping {len(frames)} frame(s) ({total:.2f}s per cycle) — press any key to stop")
    else:
        print(f"playing {len(frames)} frame(s) over {total:.2f}s")
    t = 0.0
    step_num = 0
    try:
        while True:
            prior = last_frame_pose if t >= total else initial_pose
            _complete, step_num = advance_keyframe_sequence(
                servo_map,
                frames,
                step_num,
                t,
                time_step=PLAYBACK_TIME_STEP,
                speed_factor=speed_factor,
                prior_pose=prior,
            )
            t += PLAYBACK_TIME_STEP
            if loop:
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.read(1)
                    print("stopped")
                    return
            elif t >= total - PLAYBACK_TIME_STEP / 2:
                print("done")
                return
            time.sleep(PLAYBACK_TIME_STEP)
    except KeyboardInterrupt:
        print("stopped")


def loop_sequence(frames, speed_factor=1.0):
    play_sequence(frames, speed_factor=speed_factor, loop=True)


def set_torque(enabled):
    for servo in servos.values():
        if enabled:
            servo["servo"].enable_torque()
        else:
            servo["servo"].disable_torque()


def prompt_line(prompt, cooked_settings):
    """Temporarily restore cooked mode so readline works normally."""
    fd = sys.stdin.fileno()
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, cooked_settings)
        sys.stdout.write(prompt)
        sys.stdout.flush()
        return sys.stdin.readline().strip()
    finally:
        tty.setcbreak(fd)


def print_help():
    print()
    print("Keys:")
    print("  a      add (or replace, if a frame is selected)")
    print("  space  play the sequence once")
    print("  r      loop the sequence (any key stops)")
    print("  j      jump to a frame (selects it for editing)")
    print("  n      clear selection (return to append mode)")
    print("  d      set duration of selected frame")
    print("  u      undo last keyframe")
    print("  l      list keyframes")
    print("  p      print current servo angles")
    print("  t      toggle torque on/off")
    print("  c      clear the sequence")
    print("  h      show this help")
    print("  q      quit")
    print()


def format_frame(i, frame):
    angles = ", ".join(
        f"{k}={v:.1f}" for k, v in sorted(frame["servo_angles"].items())
    )
    return f"  [{i}] {frame['max_duration']}ms {frame.get('ease', DEFAULT_EASE)}: {angles}"


def parse_args(argv):
    path = None
    side = None
    it = iter(argv[1:])
    for arg in it:
        if arg == "--side":
            side = next(it, None)
            if side not in ("left", "right"):
                print("--side must be 'left' or 'right'")
                sys.exit(1)
        else:
            path = arg
    return path, side


def main():
    global RECORDED_SIDES
    path, side = parse_args(sys.argv)
    if side:
        RECORDED_SIDES = {side}
        default_name = f"keyframes_{side}.json"
    else:
        RECORDED_SIDES = {"left", "right"}
        default_name = "keyframes.json"
    if path is None:
        path = default_name
    frames = load_sequence(path)
    selected = None
    set_torque(False)
    torque_enabled = False
    side_desc = "/".join(sorted(RECORDED_SIDES)) if RECORDED_SIDES else "all"
    print(f"sequence: {path} ({len(frames)} frame(s)) — recording {side_desc}")
    print("torque: OFF (backdrive servos to pose the robot)")
    print_help()

    def status():
        return f"[selected: {selected}]" if selected is not None else "[append mode]"

    fd = sys.stdin.fileno()
    cooked_settings = termios.tcgetattr(fd)
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
            elif ch == " ":
                play_sequence(frames)
                torque_enabled = True
            elif ch == "r":
                loop_sequence(frames)
                torque_enabled = True
            elif ch == "a":
                pose = read_current_pose()
                if selected is not None:
                    frames[selected]["servo_angles"] = pose
                    save_sequence(path, frames)
                    print(f"\nreplaced frame {selected}:")
                    print(format_frame(selected, frames[selected]))
                else:
                    frame = {
                        "servo_angles": pose,
                        "max_duration": DEFAULT_DURATION_MS,
                        "ease": DEFAULT_EASE,
                    }
                    frames.append(frame)
                    save_sequence(path, frames)
                    print(f"\nadded frame {len(frames) - 1}:")
                    print(format_frame(len(frames) - 1, frame))
            elif ch == "j":
                raw = prompt_line("\njump to frame: ", cooked_settings)
                if not raw:
                    print("cancelled")
                    continue
                idx = parse_index(raw, frames)
                if idx is None:
                    continue
                selected = idx
                print(f"moving to frame {idx}")
                move_to_pose(frames[idx]["servo_angles"])
                torque_enabled = True
                print(f"{status()} — adjust pose and press 'a' to replace")
            elif ch == "n":
                selected = None
                print(f"\n{status()}")
            elif ch == "d":
                if selected is None:
                    raw = prompt_line("\nframe index: ", cooked_settings)
                    if not raw:
                        print("cancelled")
                        continue
                    idx = parse_index(raw, frames)
                    if idx is None:
                        continue
                else:
                    idx = selected
                raw = prompt_line(
                    f"new duration for frame {idx} (ms, current {frames[idx]['max_duration']}): ",
                    cooked_settings,
                )
                try:
                    ms = int(raw)
                except ValueError:
                    print(f"not a number: {raw!r}")
                    continue
                if ms <= 0:
                    print("duration must be > 0")
                    continue
                frames[idx]["max_duration"] = ms
                save_sequence(path, frames)
                print(format_frame(idx, frames[idx]))
            elif ch == "u":
                if not frames:
                    print("\nnothing to undo")
                else:
                    removed = frames.pop()
                    save_sequence(path, frames)
                    print(f"\nremoved frame {len(frames)}: {removed['servo_angles']}")
                    if selected is not None and selected >= len(frames):
                        selected = None
            elif ch == "l":
                print()
                if not frames:
                    print("  (empty sequence)")
                else:
                    for i, frame in enumerate(frames):
                        print(format_frame(i, frame))
            elif ch == "p":
                print()
                for k, v in read_current_pose().items():
                    print(f"  {k}: {v:.2f}")
            elif ch == "c":
                confirm = prompt_line(
                    f"\nclear all {len(frames)} frame(s)? [y/N]: ", cooked_settings
                )
                if confirm.lower() == "y":
                    frames = []
                    selected = None
                    save_sequence(path, frames)
                    print("cleared")
                else:
                    print("cancelled")
            elif ch == "h":
                print_help()
    except KeyboardInterrupt:
        print("\nexiting")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, cooked_settings)
        for servo in servos.values():
            servo["servo"].disable_torque()


if __name__ == "__main__":
    main()

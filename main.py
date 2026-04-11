from math import sin, cos
from pylx16a.lx16a import *
import json
import math
import time

import threading
import sys
import termios
import tty
import time
import select

from util import advance_keyframe_sequence
from walk import keyframes as walk_positions_left
from walk import keyframes_right as walk_positions_right


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

# left_hip:  200.88
# right_hip:  89.28
# left_front:  78.96
# right_front:  149.76
# left_rear:  150.0
# right_rear:  80.4
SERVO_TEMPERATURE_LIMIT = 45  # degrees Celsius
CALIBRATION_FILE = "servo_calibration.json"

# id -> "<side>_<part>" (e.g. 5 -> "left_front") for calibration-file lookups
SERVO_NAMES = {
    cfg["id"]: f"{side}_{part}"
    for side, parts in SERVO_CONFIG.items()
    for part, cfg in parts.items()
}

servos = {}

try:
    for side in SERVO_CONFIG.values():
        for servo_config in side.values():

            servo_obj = {}
            servo_obj["id"] = servo_config["id"]
            servo_obj["limits"] = servo_config["limits"]
            servo_obj["servo"] = LX16A(servo_config["id"])
            servos[servo_config["id"]] = servo_obj


except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    exit()

left_front = servos[4]
left_rear = servos[5]
left_hip = servos[6]
right_front = servos[3]
right_rear = servos[2]
right_hip = servos[1]


def move_solo(servo_num):
    for servo in servos.values():
        if servo["id"] != servo_num:
            servo["servo"].disable_torque()


def disable_torque(servo_ids=[1, 2, 3, 4, 5, 6]):
    for servo in servos.values():
        if servo["id"] in servo_ids:
            servo["servo"].disable_torque()


def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))


def servo_sin_cos(t):
    """
    Returns (front_angle, rear_angle)

    front  = follows sin(t)
    rear   = follows cos(t)

    Output angles are always between 0° and 240°.
    """

    mid = 120  # midpoint of 0–240
    amp = 120  # amplitude

    front = mid + amp * math.sin(t)
    rear = mid + amp * math.cos(t)

    # clamp to ensure limits
    front = max(0, min(240, front))
    rear = max(0, min(240, rear))

    return front, rear


def servo_pingpong(t, x, y):
    """
    Returns an angle that oscillates sinusoidally between x and y.

    t : time (radians)
    x : minimum angle
    y : maximum angle
    """

    mid = (x + y) / 2
    amp = (y - x) / 2

    angle = mid + amp * math.sin(t)

    return angle


def load_calibration():
    with open(CALIBRATION_FILE) as f:
        return json.load(f)


def get_home(servo):
    """Return the home angle for a servo by looking it up in the calibration JSON."""
    return load_calibration()[SERVO_NAMES[servo["id"]]]["home"]


def home(ids=[1, 2, 3, 4, 5, 6], duration=500):
    calibration = load_calibration()
    for servo in servos.values():
        if servo["id"] in ids:
            target = calibration[SERVO_NAMES[servo["id"]]]["home"]
            servo["servo"].move(target, duration, wait=True)
    start_servos(ids)


def start_servos(ids=[1, 2, 3, 4, 5, 6]):
    for servo in servos.values():
        if servo["id"] in ids and servo["servo"]._waiting_for_move:
            servo["servo"].move_start()


print_positions_requested = False
print_request_lock = threading.Lock()


def print_servo_positions():
    right_hip_pos = right_hip["servo"].get_physical_angle()
    left_hip_pos = left_hip["servo"].get_physical_angle()
    right_front_pos = right_front["servo"].get_physical_angle()
    left_front_pos = left_front["servo"].get_physical_angle()
    right_rear_pos = right_rear["servo"].get_physical_angle()
    left_rear_pos = left_rear["servo"].get_physical_angle()
    print("left_hip: ", left_hip_pos)
    print("right_hip: ", right_hip_pos)
    print("left_front: ", left_front_pos)
    print("right_front: ", right_front_pos)
    print("left_rear: ", left_rear_pos)
    print("right_rear: ", right_rear_pos)

    print()


def request_print_servo_positions():
    global print_positions_requested
    with print_request_lock:
        print_positions_requested = True


def consume_print_request():
    global print_positions_requested
    with print_request_lock:
        was_requested = print_positions_requested
        print_positions_requested = False
    return was_requested


def on_space():
    request_print_servo_positions()


def goto_position(servo, target_angle, duration):
    servo["servo"].move(target_angle, int(duration * 1000), wait=True)
    start_servos()


def key_listener():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setcbreak(fd)
        while True:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1)
                if ch == " ":
                    on_space()
            time.sleep(0.01)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


threading.Thread(target=key_listener, daemon=True).start()


def check_temperature():
    for servo in servos.values():
        temp = servo["servo"].get_temp()
        if temp > SERVO_TEMPERATURE_LIMIT:
            raise Exception(f"Servo {servo['id']} is overheating")


TIME_STEP = 0.005
GAIT_SPEED = 6
t = 0
walk_step_num = 0

max_left_front = left_front["servo"].get_physical_angle()
max_left_rear = left_rear["servo"].get_physical_angle()


def shift_right(duration):
    left_hip["servo"].move(get_home(left_hip) - 25, int(duration * 1000), wait=True)
    right_hip["servo"].move(get_home(right_hip) - 25, int(duration * 1000), wait=True)
    start_servos()


def shift_left(duration):
    left_hip["servo"].move(get_home(left_hip) + 25, int(duration * 1000), wait=True)
    right_hip["servo"].move(get_home(right_hip) + 25, int(duration * 1000), wait=True)
    start_servos()


def left_shuffle_forwards(duration):
    left_front["servo"].move(get_home(left_front) - 30, int(duration * 1000), wait=True)
    left_rear["servo"].move(get_home(left_rear) - 30, int(duration * 1000), wait=True)
    start_servos()


def left_shuffle_backwards(duration):
    left_front["servo"].move(get_home(left_front) + 40, int(duration * 1000), wait=True)
    left_rear["servo"].move(get_home(left_rear) + 45, int(duration * 1000), wait=True)
    start_servos()


def right_shuffle_forwards(duration):
    right_front["servo"].move(get_home(right_front) + 30, int(duration * 1000), wait=True)
    right_rear["servo"].move(get_home(right_rear) + 30, int(duration * 1000), wait=True)
    start_servos()


def right_shuffle_backwards(duration):
    right_front["servo"].move(get_home(right_front) - 40, int(duration * 1000), wait=True)
    right_rear["servo"].move(get_home(right_rear) - 45, int(duration * 1000), wait=True)
    start_servos()


def left_home(duration):
    left_front["servo"].move(get_home(left_front), int(duration * 1000), wait=True)
    left_rear["servo"].move(get_home(left_rear), int(duration * 1000), wait=True)
    start_servos()


def right_home(duration):
    right_front["servo"].move(get_home(right_front), int(duration * 1000), wait=True)
    right_rear["servo"].move(get_home(right_rear), int(duration * 1000), wait=True)
    start_servos()


ARC_DURATION = 4.0  # seconds for a one-way sweep (min → max)
SWEEP_PERIOD = 2 * ARC_DURATION  # full min → max → min cycle


def sweep_phase_offset(servo, opposite_branch=False):
    """Phase offset so that the servo starts exactly at its calibrated home.

    Using the principal asin branch gives cos(offset) >= 0 (one velocity sign);
    the (pi - asin) branch gives cos(offset) <= 0 (opposite sign). Passing
    opposite_branch=True picks the second branch so rear servos can still move
    against the front servos at t=0 while also starting at home.
    """
    home_val = get_home(servo)
    x, y = servo["limits"]
    mid = (x + y) / 2
    amp = (y - x) / 2
    ratio = max(-1.0, min(1.0, (home_val - mid) / amp))
    base = math.asin(ratio)
    if opposite_branch:
        base = math.pi - base
    return base


lf_offset = sweep_phase_offset(left_front)
lr_offset = sweep_phase_offset(left_rear, opposite_branch=True)
rf_offset = sweep_phase_offset(right_front)
rr_offset = sweep_phase_offset(right_rear, opposite_branch=True)

try:
    print("moving to home position")
    home(duration=1500)
    time.sleep(2)

    print("sweeping front/rear servos between min and max")
    t = 0
    while True:
        phase = 2 * math.pi * t / SWEEP_PERIOD

        lf = servo_pingpong(phase + lf_offset, *left_front["limits"])
        lr = servo_pingpong(phase + lr_offset, *left_rear["limits"])
        rf = servo_pingpong(phase + rf_offset, *right_front["limits"])
        rr = servo_pingpong(phase + rr_offset, *right_rear["limits"])

        left_front["servo"].move(lf, int(TIME_STEP * 1000), wait=False)
        left_rear["servo"].move(lr, int(TIME_STEP * 1000), wait=False)
        right_front["servo"].move(rf, int(TIME_STEP * 1000), wait=False)
        right_rear["servo"].move(rr, int(TIME_STEP * 1000), wait=False)

        # while True:

        #     check_temperature()

        #     if consume_print_request():
        #         print_servo_positions()

        #     left_front_min = 50
        #     left_front_max = 120
        #     left_rear_min = 120
        #     left_rear_max = 210

        #     right_front_min = 133
        #     right_front_max = 174
        #     right_rear_min = 25
        #     right_rear_max = 105

        #     left_hip_min = 190
        #     left_hip_max = 210
        #     right_hip_min = 75
        #     right_hip_max = 100

        #     left_front_angle = servo_pingpong(t, left_front_min, left_front_max)
        #     left_rear_angle = servo_pingpong(t - math.pi / 4, left_rear_min, left_rear_max)
        #     left_hip_angle = servo_pingpong(t + math.pi / 2, left_hip_min, left_hip_max)
        #     right_front_angle = servo_pingpong(t, right_front_min, right_front_max)
        #     right_rear_angle = servo_pingpong(
        #         t - math.pi / 4, right_rear_min, right_rear_max
        #     )
        #     right_hip_angle = servo_pingpong(t + math.pi / 2, right_hip_min, right_hip_max)

        #     left_front["servo"].move(left_front_angle, int(TIME_STEP * 1000), wait=True)
        #     left_rear["servo"].move(left_rear_angle, int(TIME_STEP * 1000), wait=True)
        #     left_hip["servo"].move(left_hip_angle, int(TIME_STEP * 1000), wait=True)
        #     right_front["servo"].move(right_front_angle, int(TIME_STEP * 1000), wait=True)
        #     right_rear["servo"].move(right_rear_angle, int(TIME_STEP * 1000), wait=True)
        #     right_hip["servo"].move(right_hip_angle, int(TIME_STEP * 1000), wait=True)
        #     start_servos()

        #     t += TIME_STEP * GAIT_SPEED
        #     time.sleep(TIME_STEP)

        # while True:
        #     shift_right(0.75)
        #     right_home(0.5)
        #     time.sleep(0.75)
        #     left_shuffle_backwards(0.1)
        #     time.sleep(0.1)
        #     shift_left(0.75)
        #     left_home(0.5)
        #     time.sleep(0.75)
        #     right_shuffle_backwards(0.1)
        #     time.sleep(0.1)

        # complete, walk_step_num = advance_keyframe_sequence(
        #     {
        #         "left_front": left_front,
        #         "left_rear": left_rear,
        #         "left_hip": left_hip,
        #         "right_front": right_front,
        #         "right_rear": right_rear,
        #         "right_hip": right_hip,
        #     },
        #     walk_positions_left,
        #     walk_step_num,
        #     t,
        #     time_step=TIME_STEP,
        #     speed_factor=1,
        # )
        t += TIME_STEP

        time.sleep(TIME_STEP)


except KeyboardInterrupt:
    print("Exiting...")
    time.sleep(1)
finally:
    for servo in servos.values():
        servo["servo"].disable_torque()

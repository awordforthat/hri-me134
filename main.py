from math import sin, cos
from pylx16a.lx16a import *
import math
import time

import threading
import sys
import termios
import tty
import time
import select

from util import advance_keyframe_sequence
from walk import keyframes as walk_positions


LX16A.initialize("/dev/ttyUSB0")

SERVO_CONFIG = {
    "left": {
        "front": {"id": 3, "limits": (120, 220), "home": 150, "squat": 50},
        "rear": {"id": 2, "limits": (20, 160), "home": 80, "squat": 175},
        "hip": {"id": 1, "limits": (10, 60), "home": 200, "squat": 200},
    },
    "right": {
        "front": {"id": 6, "limits": (0, 120), "home": 90, "squat": 175},
        "rear": {"id": 5, "limits": (90, 200), "home": 150, "squat": 55},
        "hip": {"id": 4, "limits": (55, 120), "home": 80, "squat": 80},
    },
}
SERVO_TEMPERATURE_LIMIT = 45  # degrees Celsius
servos = {}

try:
    for side in SERVO_CONFIG.values():
        for servo_config in side.values():
            servo_obj = {}
            servo_obj["id"] = servo_config["id"]
            servo_obj["limits"] = servo_config["limits"]
            servo_obj["home"] = servo_config["home"]
            servo_obj["squat"] = servo_config["squat"]
            servo_obj["servo"] = LX16A(servo_config["id"])
            servos[servo_config["id"]] = servo_obj

except ServoTimeoutError as e:
    print(f"Servo {e.id_} is not responding. Exiting...")
    exit()

left_front = servos[2]
left_rear = servos[3]
left_hip = servos[1]
right_front = servos[5]
right_rear = servos[4]
right_hip = servos[6]


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


def home(ids=[1, 2, 3, 4, 5, 6], duration=500):
    for servo in servos.values():
        if servo["id"] in ids:
            servo["servo"].move(servo["home"], duration)


def squat(duration=3000):
    breakpoint()
    left_front["servo"].move(left_front["squat"], duration, wait=True)
    left_rear["servo"].move(left_rear["squat"], duration, wait=True)
    right_front["servo"].move(right_front["squat"], duration, wait=True)
    right_rear["servo"].move(right_rear["squat"], duration, wait=True)

    start_servos()


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
    print("left_front: ", left_front_pos)
    print("left_rear: ", left_rear_pos)

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


# This produces a movement that keeps the foot in a straight vertical line  up and down.
# front_angle = servo_pingpong(t, 0, 120)
# rear_angle = servo_pingpong(t + math.pi, 0, 100)  # rear lags front by pi radians

# This moves the foot in a circular pattern
# front_angle = servo_pingpong(t, 0, 120)

# rear_angle = servo_pingpong(t + math.pi / 2, 0, 100)


TIME_STEP = 0.05
t = 0
walk_step_num = 0

try:
    home()
    time.sleep(1)
    # disable_torque()
    while True:

        check_temperature()

        if consume_print_request():
            print_servo_positions()

        sequence_complete, walk_step_num = advance_keyframe_sequence(
            {
                "right_hip": right_hip,
                "left_hip": left_hip,
                "right_front": right_front,
                "left_front": left_front,
                "right_rear": right_rear,
                "left_rear": left_rear,
            },
            walk_positions,
            step_num=walk_step_num,
            t=t,
            time_step=TIME_STEP,
            speed_factor=0.25,
        )

        # right_hip_angle = servo_pingpong(t, right_hip["limits"][0], right_hip["limits"][1])
        # print(right_hip_angle)
        # right_hip["servo"].move(right_hip_angle, 100)
        # left_front_angle = servo_pingpong(t, left_front["limits"][0], left_front["limits"][1])
        # left_rear_angle = servo_pingpong(t - math.pi/4, left_rear["limits"][0], left_rear["limits"][1])
        # right_front_angle = servo_pingpong(t + math.pi, right_front["limits"][0], right_front["limits"][1])
        # right_rear_angle = servo_pingpong(t + math.pi - math.pi/4, right_rear["limits"][0], right_rear["limits"][1])
        # left_front["servo"].move(left_front_angle, 100, wait=True)
        # left_rear["servo"].move(left_rear_angle, 100, wait=True)
        # right_front["servo"].move(right_front_angle, 100, wait=True)
        # right_rear["servo"].move(right_rear_angle, 100, wait=True)
        # start_servos()
        t += TIME_STEP
        time.sleep(TIME_STEP)


except KeyboardInterrupt:
    print("Exiting...")
finally:
    for servo in servos.values():
        servo["servo"].disable_torque()

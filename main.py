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
from five_bar_ik import make_left_leg_ik, make_right_leg_ik


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
            target = max(0, min(240, target))
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


TIME_STEP = 0.02
GAIT_SPEED = 4
GAIT_PERIOD = 6.0 / GAIT_SPEED  # seconds per full cycle; 2s at speed=3

# Shared nominal stance (x, z) in leg frame. Both legs target the same.
LEFT_STANCE = (10.0, -75.0)
RIGHT_STANCE = (-40.0, -75.0)  # right IK is miscalibrated; shift back to match left physically
STRIDE = 25.0  # forward step length (peak-to-peak in x)
FOOT_LIFT = 22.0
HIP_LEAN = 18.0  # degrees
HIP_LEAN_LEAD = math.pi / 4  # lean leads swing by this phase
HIP_LEAN_BIAS = 6.0  # constant bias; +value shifts average posture left
STANCE_X_OFFSET = 0.0  # backward pitch now baked into home pose
STANCE_Z_OFFSET = 0.0

left_ik = make_left_leg_ik()
right_ik = make_right_leg_ik()

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
    right_front["servo"].move(
        get_home(right_front) + 30, int(duration * 1000), wait=True
    )
    right_rear["servo"].move(get_home(right_rear) + 30, int(duration * 1000), wait=True)
    start_servos()


def right_shuffle_backwards(duration):
    right_front["servo"].move(
        get_home(right_front) - 40, int(duration * 1000), wait=True
    )
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


ARC_DURATION = 0.8  # seconds for a one-way sweep (min → max)
SWEEP_PERIOD = 2 * ARC_DURATION  # full min → max → min cycle


def left_leg_sin(t):
    """Drive the left leg with a single sinusoid.

    Front follows sin(t) across its limits; rear is pi/4 behind.
    """
    front = servo_pingpong(t, *left_front["limits"])
    rear = servo_pingpong(t - math.pi / 4, *left_rear["limits"])
    return front, rear


def right_leg_sin(t):
    """Drive the right leg with a single sinusoid, pi (half cycle) behind the left."""
    front = servo_pingpong(t - math.pi, *right_front["limits"])
    rear = servo_pingpong(t - math.pi - math.pi / 4, *right_rear["limits"])
    return front, rear


HIP_AMPLITUDE = 20  # degrees the hip dips from home at leg midpoint
LEFT_HIP_HOME = get_home(left_hip)
RIGHT_HIP_HOME = get_home(right_hip)


def left_hip_sin(t):
    """One dip per leg cycle: minimum when left leg front passes midpoint going forward."""
    return LEFT_HIP_HOME - (HIP_AMPLITUDE / 2) * (1 + math.cos(t))


def right_hip_sin(t):
    """One dip per leg cycle, pi offset from left hip so they alternate."""
    return RIGHT_HIP_HOME - (HIP_AMPLITUDE / 2) * (1 + math.cos(t - math.pi))


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


def foot_xz(phase, stance):
    """Foot (x, z) for a given phase, using the leg's own nominal stance."""
    x0, z0 = stance
    # Stance (sin≤0): foot moves backward in leg frame → pushes body forward.
    # Swing (sin>0): foot moves forward in leg frame → resets for next step.
    x = x0 - (STRIDE / 2) * math.cos(phase)
    lift = FOOT_LIFT * max(0.0, math.sin(phase))
    return x, z0 + lift


def gait_tick(t):
    phase_l = (t / GAIT_PERIOD) * 2 * math.pi
    phase_r = phase_l + math.pi

    lx, lz = foot_xz(phase_l, LEFT_STANCE)
    rx, rz = foot_xz(phase_r, RIGHT_STANCE)

    left_sol = left_ik.solve(lx, lz)
    right_sol = right_ik.solve(rx, rz)
    if left_sol is None or right_sol is None:
        print(f"unreachable: L=({lx:.1f},{lz:.1f}) R=({rx:.1f},{rz:.1f})")
        return

    # solve() returns (A_cmd, B_cmd) = (rear, front)
    l_rear_cmd, l_front_cmd = left_sol
    r_rear_cmd, r_front_cmd = right_sol

    # Lean toward stance leg: sin(phase_l)>0 means left swing → weight right → hips shift right (minus)
    # +lean = subtracted from hips = shift right; subtract bias to bias left
    lean = HIP_LEAN * math.sin(phase_l - HIP_LEAN_LEAD) - HIP_LEAN_BIAS
    l_hip_cmd = LEFT_HIP_HOME - lean
    r_hip_cmd = RIGHT_HIP_HOME - lean

    dur_ms = int(TIME_STEP * 1000 * 3)  # servo move duration longer than tick
    def c(v):
        return int(max(0, min(240, v)))
    left_rear["servo"].move(c(l_rear_cmd), dur_ms, wait=True)
    left_front["servo"].move(c(l_front_cmd), dur_ms, wait=True)
    right_rear["servo"].move(c(r_rear_cmd), dur_ms, wait=True)
    right_front["servo"].move(c(r_front_cmd), dur_ms, wait=True)
    left_hip["servo"].move(c(l_hip_cmd), dur_ms, wait=True)
    right_hip["servo"].move(c(r_hip_cmd), dur_ms, wait=True)
    start_servos()


def print_home_foot_positions():
    cal = load_calibration()
    l_rear = cal[SERVO_NAMES[left_rear["id"]]]["home"]
    l_front = cal[SERVO_NAMES[left_front["id"]]]["home"]
    r_rear = cal[SERVO_NAMES[right_rear["id"]]]["home"]
    r_front = cal[SERVO_NAMES[right_front["id"]]]["home"]
    # solve() returns (A=rear, B=front); forward_foot expects (left_cmd, right_cmd) = (A, B)
    left_foot = left_ik.forward_foot(l_rear, l_front)
    right_foot = right_ik.forward_foot(r_rear, r_front)
    print(f"home pose → left foot: {left_foot}")
    print(f"home pose → right foot: {right_foot}")
    return left_foot, right_foot


try:
    print("moving to home position")
    home(duration=1500)
    time.sleep(2)
    print_home_foot_positions()
    print(f"gait stance (fixed) — left: {LEFT_STANCE}, right: {RIGHT_STANCE}")

    print("easing into gait start pose")
    # Ease to each leg's pure nominal stance (no stride, no lift) so belly stays level.
    lx, lz = LEFT_STANCE
    rx, rz = RIGHT_STANCE
    l_sol = left_ik.solve(lx, lz)
    r_sol = right_ik.solve(rx, rz)
    if l_sol and r_sol:
        ease_ms = 2000
        clamp_cmd = lambda v: int(max(0, min(240, v)))
        left_rear["servo"].move(clamp_cmd(l_sol[0]), ease_ms, wait=True)
        left_front["servo"].move(clamp_cmd(l_sol[1]), ease_ms, wait=True)
        right_rear["servo"].move(clamp_cmd(r_sol[0]), ease_ms, wait=True)
        right_front["servo"].move(clamp_cmd(r_sol[1]), ease_ms, wait=True)
        start_servos()
        time.sleep(ease_ms / 1000 + 0.5)

    print("running five-bar IK gait (stationary stepping)")
    while True:
        gait_tick(t)
        t += TIME_STEP
        time.sleep(TIME_STEP)


except KeyboardInterrupt:
    print("Exiting...")
    time.sleep(1)
finally:
    for servo in servos.values():
        servo["servo"].disable_torque()

from pylx16a.lx16a import *
import json
import math
import time

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


TIME_STEP = 0.02
GAIT_SPEED = 4
GAIT_PERIOD = 6.0 / GAIT_SPEED  # seconds per full cycle; 2s at speed=3

# Shared nominal stance (x, z) in leg frame. Both legs target the same.
LEFT_STANCE = (15, -95)  # half ellipse: 15/-105
RIGHT_STANCE = (10.0, -115.0)  # half ellipse: 10/-115
STRIDE = 35.0  # forward step length (peak-to-peak in x)
FOOT_LIFT = 25.0
HIP_LEAN = 15  # degrees
HIP_LEAN_LEAD = math.pi  # lean leads swing by this phase
HIP_NARROW = (
    5  # degrees each hip is biased inward from home; flip sign if stance widens
)

# Foot trajectory shape. Options:
#   "half_ellipse" — flat stance, half-sine swing lift (no dig-in)
#   "full_ellipse" — full-sine z so foot digs below z0 during stance push
#   "trapezoidal"  — lift straight up, translate at height, drop straight down
GAIT_PATTERN = "trapezoidal"

# For trapezoidal: fraction of swing spent lifting and dropping (each).
# e.g. 0.2 → lift 20%, flat 60%, drop 20%.
TRAP_EDGE_FRAC = 0.2

left_ik = make_left_leg_ik()
right_ik = make_right_leg_ik()

LEFT_HIP_HOME = get_home(left_hip)
RIGHT_HIP_HOME = get_home(right_hip)

t = 0


def foot_xz(phase, stance):
    """Foot (x, z) for a given phase, using the leg's own nominal stance."""
    x0, z0 = stance
    # Stance (cos > 0 half): foot moves backward in leg frame → pushes body forward.
    # Swing (cos < 0 half): foot moves forward in leg frame → resets for next step.
    x = x0 - (STRIDE / 2) * math.cos(phase)
    if GAIT_PATTERN == "half_ellipse":
        # Lift only during swing; foot stays at z0 during stance.
        dz = FOOT_LIFT * max(0.0, math.sin(phase))
    elif GAIT_PATTERN == "full_ellipse":
        # Full sine: foot rises during swing, digs below z0 during stance push.
        dz = FOOT_LIFT * math.sin(phase)
    elif GAIT_PATTERN == "trapezoidal":
        # Swing = first half of cycle (sin > 0). Split into lift/flat/drop.
        cycle = (phase / (2 * math.pi)) % 1.0
        if cycle >= 0.5:
            dz = 0.0  # stance
        else:
            s = cycle / 0.5  # 0..1 across swing
            edge = max(1e-6, TRAP_EDGE_FRAC)
            if s < edge:
                dz = FOOT_LIFT * (s / edge)
            elif s > 1 - edge:
                dz = FOOT_LIFT * ((1 - s) / edge)
            else:
                dz = FOOT_LIFT
    else:
        raise ValueError(f"unknown GAIT_PATTERN: {GAIT_PATTERN!r}")
    return x, z0 + dz


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

    if int(t / TIME_STEP) % 10 == 0:
        print(f"t={t:5.2f}  L=({lx:+5.1f},{lz:+6.1f})  R=({rx:+5.1f},{rz:+6.1f})")

    # Lean toward stance leg: sin(phase_l)>0 means left swing → weight right → hips shift right (minus)
    lean = HIP_LEAN * math.sin(phase_l - HIP_LEAN_LEAD)
    l_hip_cmd = LEFT_HIP_HOME - lean - HIP_NARROW
    r_hip_cmd = RIGHT_HIP_HOME - lean + HIP_NARROW

    dur_ms = int(TIME_STEP * 1000)  # servo move duration = tick period

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
        left_hip["servo"].move(
            clamp_cmd(LEFT_HIP_HOME - HIP_NARROW), ease_ms, wait=True
        )
        right_hip["servo"].move(
            clamp_cmd(RIGHT_HIP_HOME + HIP_NARROW), ease_ms, wait=True
        )
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

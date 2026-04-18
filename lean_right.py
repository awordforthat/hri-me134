"""Hold pose: lean fully right, lift left foot. Ctrl-C to exit."""

import time
from pylx16a.lx16a import *
from five_bar_ik import make_left_leg_ik, make_right_leg_ik
import json

LX16A.initialize("/dev/ttyUSB0")

with open("servo_calibration.json") as f:
    cal = json.load(f)

left_rear = LX16A(5)
left_front = LX16A(4)
left_hip = LX16A(6)
right_rear = LX16A(2)
right_front = LX16A(3)
right_hip = LX16A(1)

LEAN_DEG = 20.0
HIP_NARROW = 0.0
LEFT_FOOT_LIFTED = (-20.0, -70.0)
RIGHT_FOOT_PLANTED = (8.0, -115.0)

left_ik = make_left_leg_ik()
right_ik = make_right_leg_ik()

l_rear_cmd, l_front_cmd = left_ik.solve(*LEFT_FOOT_LIFTED)
r_rear_cmd, r_front_cmd = right_ik.solve(*RIGHT_FOOT_PLANTED)

l_hip_cmd = cal["left_hip"]["home"] + LEAN_DEG - HIP_NARROW
r_hip_cmd = cal["right_hip"]["home"] + LEAN_DEG + HIP_NARROW


def c(v):
    return int(max(0, min(240, v)))


try:
    dur = 1500
    left_rear.move(c(l_rear_cmd), dur, wait=True)
    left_front.move(c(l_front_cmd), dur, wait=True)
    right_rear.move(c(r_rear_cmd), dur, wait=True)
    right_front.move(c(r_front_cmd), dur, wait=True)
    left_hip.move(c(l_hip_cmd), dur, wait=True)
    right_hip.move(c(r_hip_cmd), dur, wait=True)
    for s in (left_rear, left_front, right_rear, right_front, left_hip, right_hip):
        s.move_start()
    print("holding pose; Ctrl-C to release")
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    for s in (left_rear, left_front, right_rear, right_front, left_hip, right_hip):
        s.disable_torque()

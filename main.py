from math import sin, cos
from pylx16a.lx16a import *
import time
import keyboard
import math

LX16A.initialize("/dev/ttyUSB0")



LIMITS = {
	"front": (20, 210),
	"rear": (150, 240)
}

try:
	rear = LX16A(1)
	front = LX16A(2)
except ServoTimeoutError as e:
	print(f"Servo {e.id_} is not responding. Exiting...")
	exit()


import math
from typing import List, Optional, Tuple, Dict

LIMITS = {
    "front": (20.0, 210.0),  # degrees
    "rear":  (150.0, 240.0), # degrees
}

def clamp(x: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, x))

def wrap_deg_0_360(deg: float) -> float:
    # Map any degree value into [0, 360)
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def in_range(deg: float, lo: float, hi: float) -> bool:
    return lo <= deg <= hi

def fivebar_ik_zero_up_limited(
    x: float,
    y: float,
    *,
    d: float = 27.0,
    L1: float = 50.0,
    L2: float = 80.0,
    R1: float = 50.0,
    R2: float = 80.0,
    limits: Dict[str, Tuple[float, float]] = LIMITS,
    prev_deg: Optional[Tuple[float, float]] = None,  # (front, rear) in degrees
) -> List[Tuple[float, float]]:
    """
    Returns a list of (front_deg, rear_deg) IK solutions that satisfy servo limits.
    Angle convention: 0° = straight up (+y), positive = CCW (right-hand rule in plane).
    x,y are in the base frame with A=(0,0), B=(d,0), units match link lengths (mm here).
    """
    # Distances from each base joint to target
    rL = math.hypot(x, y)
    rR = math.hypot(x - d, y)

    # Reachability of each 2-link chain
    def reachable(r: float, a: float, b: float) -> bool:
        return abs(a - b) <= r <= (a + b)

    if not (reachable(rL, L1, L2) and reachable(rR, R1, R2)):
        return []

    # Bearings measured from +y (zero-up): phi = atan2(dx, dy)
    phiL = math.atan2(x, y)         # radians
    phiR = math.atan2(x - d, y)     # radians

    # Law of cosines for shoulder offset angles
    cL = clamp((L1*L1 + rL*rL - L2*L2) / (2.0 * L1 * rL))
    cR = clamp((R1*R1 + rR*rR - R2*R2) / (2.0 * R1 * rR))
    alphaL = math.acos(cL)          # radians
    alphaR = math.acos(cR)          # radians

    # Four combinations (elbow up/down on each side)
    raw = [
        (phiL + alphaL, phiR + alphaR),
        (phiL + alphaL, phiR - alphaR),
        (phiL - alphaL, phiR + alphaR),
        (phiL - alphaL, phiR - alphaR),
    ]

    # Convert to degrees in [0,360) and filter by servo limits
    front_lo, front_hi = limits["front"]
    rear_lo,  rear_hi  = limits["rear"]

    sols: List[Tuple[float, float]] = []
    for tL_rad, tR_rad in raw:
        front_deg = wrap_deg_0_360(math.degrees(tL_rad))
        rear_deg  = wrap_deg_0_360(math.degrees(tR_rad))

        if in_range(front_deg, front_lo, front_hi) and in_range(rear_deg, rear_lo, rear_hi):
            sols.append((front_deg, rear_deg))

    # Prefer the solution closest to previous angles (if provided)
    if prev_deg and sols:
        pf, pr = prev_deg
        def ang_dist(a: float, b: float) -> float:
            # distance on a circle (0..360 wrap)
            diff = abs(a - b) % 360.0
            return min(diff, 360.0 - diff)

        sols.sort(key=lambda s: ang_dist(s[0], pf) + ang_dist(s[1], pr))

    return sols

for i in range (0, 100):
    for j in range (0, 100):
        res = fivebar_ik_zero_up_limited(i, j)
        if res:
            print(f"({i}, {j}) -> {res}")
            front.move(res[0][0])
            rear.move(res[0][1])
            time.sleep(0.1)

# res = fivebar_ik_zero_up_limited(92, 18)

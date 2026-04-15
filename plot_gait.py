"""Dump the foot trajectory one full cycle to see the ellipse shape."""
import math

STRIDE = 40.0
FOOT_LIFT = 35.0
LEFT_STANCE = (12.0, -95.0)
RIGHT_STANCE = (6.0, -95.0)


def foot_xz(phase, stance):
    x0, z0 = stance
    x = x0 - (STRIDE / 2) * math.cos(phase)
    lift = FOOT_LIFT * max(0.0, math.sin(phase))
    return x, z0 + lift


print(f"{'phase':>6} {'deg':>5} {'lx':>6} {'lz':>7} {'rx':>6} {'rz':>7}  swing?")
for i in range(24):
    phase = 2 * math.pi * i / 24
    lx, lz = foot_xz(phase, LEFT_STANCE)
    rx, rz = foot_xz(phase + math.pi, RIGHT_STANCE)
    swing = "L" if math.sin(phase) > 0 else "R"
    print(f"{phase:6.2f} {math.degrees(phase):5.0f} {lx:6.1f} {lz:7.1f} {rx:6.1f} {rz:7.1f}  {swing}")

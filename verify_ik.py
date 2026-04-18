"""
Verify IK direction/offset calibration.

1. FK check: home cmds from servo_calibration.json should produce foot ≈ (-25, -76).
2. Sweep check: foot x from -20 to +20 at z=-110 should give monotonic servo cmds
   (no sign flips → correct dir signs, no branch flips → correct elbow_up).
"""

import json

from five_bar_ik import make_left_leg_ik, make_right_leg_ik

CAL_FILE = "servo_calibration.json"
IK_CAL_FILE = "ik_calibration.json"

with open(IK_CAL_FILE) as _f:
    _ik_cal = json.load(_f)

# Foot (x, z) measured during the most recent recalibrate_ik.py run.
EXPECTED_FOOT_LEFT = tuple(_ik_cal["left"]["home_foot"])
EXPECTED_FOOT_RIGHT = tuple(_ik_cal["right"]["home_foot"])


def fk_check(name, ik, rear_cmd, front_cmd, expected):
    foot = ik.forward_foot(rear_cmd, front_cmd)
    print(f"\n[{name}] FK(home): rear={rear_cmd:.2f}° front={front_cmd:.2f}°")
    if foot is None:
        print("  FK returned None — home cmds don't close the linkage.")
        return
    dx = foot[0] - expected[0]
    dz = foot[1] - expected[1]
    err = (dx * dx + dz * dz) ** 0.5
    mark = "✓" if err < 2.0 else "✗"
    print(
        f"  foot = ({foot[0]:+.2f}, {foot[1]:+.2f})  expected {expected}  "
        f"err={err:.2f}mm  {mark}"
    )


def sweep_check(name, ik, z=-110.0):
    print(f"\n[{name}] sweep x=-20..+20 at z={z}:")
    xs = [-20, -13, -6, 0, 6, 13, 20]
    prev_r, prev_f = None, None
    rear_deltas, front_deltas = [], []
    for x in xs:
        r = ik.solve(x, z)
        if r is None:
            print(f"  x={x:+3d}  UNREACHABLE")
            continue
        rear, front = r
        tag = ""
        if prev_r is not None:
            dr = rear - prev_r
            df = front - prev_f
            rear_deltas.append(dr)
            front_deltas.append(df)
            tag = f"  Δrear={dr:+.2f}  Δfront={df:+.2f}"
        print(f"  x={x:+3d}  rear={rear:6.2f}°  front={front:6.2f}°{tag}")
        prev_r, prev_f = rear, front

    def monotonic(deltas):
        if not deltas:
            return False
        s = deltas[0]
        return all((d > 0) == (s > 0) for d in deltas)

    print(
        f"  rear monotonic: {monotonic(rear_deltas)}   "
        f"front monotonic: {monotonic(front_deltas)}"
    )


def main():
    with open(CAL_FILE) as f:
        cal = json.load(f)

    left_ik = make_left_leg_ik()
    right_ik = make_right_leg_ik()

    fk_check(
        "LEFT",
        left_ik,
        cal["left_rear"]["home"],
        cal["left_front"]["home"],
        EXPECTED_FOOT_LEFT,
    )
    fk_check(
        "RIGHT",
        right_ik,
        cal["right_rear"]["home"],
        cal["right_front"]["home"],
        EXPECTED_FOOT_RIGHT,
    )

    sweep_check("LEFT", left_ik)
    sweep_check("RIGHT", right_ik)


if __name__ == "__main__":
    main()

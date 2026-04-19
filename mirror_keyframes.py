"""Mirror a left-leg keyframe sequence into a right-leg sequence.

Computes each right-side servo angle as `2*axis - left_angle`, using mirror
axes derived from the servo limits in SERVO_CONFIG (see main.py).

Usage:
  python mirror_keyframes.py [--in keyframes_left.json] [--out keyframes_right.json]
"""

import argparse
import json


# Mirror axes derived from SERVO_CONFIG limits (see main.py lines 13-21).
# right_angle = 2 * axis - left_angle
MIRROR_AXES = {
    "front": 146.4,
    "rear": 151.32,
    "hip": 141.06,
}


def mirror_angle(left_name, angle):
    if not left_name.startswith("left_"):
        raise ValueError(f"expected left_* servo name, got {left_name!r}")
    part = left_name[len("left_"):]
    if part not in MIRROR_AXES:
        raise ValueError(f"unknown servo part {part!r}")
    return round(2 * MIRROR_AXES[part] - angle, 2)


def mirror_pose(left_pose):
    out = {}
    for name, angle in left_pose.items():
        part = name.split("_", 1)[1]
        out[f"right_{part}"] = mirror_angle(name, angle)
    return out


def mirror_frames(frames):
    return [
        {
            "servo_angles": mirror_pose(f["servo_angles"]),
            "max_duration": f["max_duration"],
            "ease": f.get("ease", "linear"),
        }
        for f in frames
    ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="infile", default="keyframes_left.json")
    ap.add_argument("--out", dest="outfile", default="keyframes_right.json")
    args = ap.parse_args()

    with open(args.infile) as f:
        frames = json.load(f)

    mirrored = mirror_frames(frames)

    with open(args.outfile, "w") as f:
        json.dump(mirrored, f, indent=2)

    print(f"wrote {len(mirrored)} frame(s) to {args.outfile}")


if __name__ == "__main__":
    main()

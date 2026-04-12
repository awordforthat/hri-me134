"""
5-Bar Parallel Mechanism IK Solver
===================================
Leg frame convention (sagittal plane):
  - Origin at LEFT servo axle
  - X: positive = forward
  - Z: positive = upward
  - Both servo axles are at the TOP; foot coupler hangs below (z < 0 typically)

         A(0,0)----d----B(d,0)
          \              /
       l1  \            /  l2
            \          /
         elbow_L    elbow_R
              \      /
            l3 \    / l4
                \  /
                 P (foot target: x, z)

Fill in the CALIBRATION CONSTANTS section below before use.
"""

import numpy as np


# ─────────────────────────────────────────────
#  CALIBRATION CONSTANTS  ← fill these in
# ─────────────────────────────────────────────

# Geometry (measure from CAD, in mm)
SERVO_SEPARATION = 26.5  # horizontal distance between the two servo axles
L1_LEFT = 70.4  # proximal link: left servo axle → left elbow joint
L2_RIGHT = 70.4  # proximal link: right servo axle → right elbow joint
L3_LEFT = 77.6  # distal link:   left elbow joint → foot coupler
L4_RIGHT = 75.0  # distal link:   right elbow joint → foot coupler

# Servo zero-point offsets (degrees)
# cmd = mechanical_angle + offset
# A = rear servo (origin), B = front servo (x = +26.5mm)
# Calibrated from: foot at (-25, -76) mm with leg straight down

# Left leg
LEFT_LEG_REAR_OFFSET = 273.76  # A servo (rear)
LEFT_LEG_FRONT_OFFSET = 220.29  # B servo (front)

# Right leg
RIGHT_LEG_REAR_OFFSET = 120.16  # A servo (rear)
RIGHT_LEG_FRONT_OFFSET = 216.45  # B servo (front)

# Default (left leg) — swap to right leg offsets when instantiating for right leg
LEFT_SERVO_OFFSET = LEFT_LEG_REAR_OFFSET
RIGHT_SERVO_OFFSET = LEFT_LEG_FRONT_OFFSET

# Elbow configuration — which of the two IK solutions matches your physical build.
# "True" means the elbow joint swings outward/upward; "False" means inward/downward.
# Look at the CAD: if the left elbow is to the left of the proximal link → True, etc.
LEFT_ELBOW_UP = True
RIGHT_ELBOW_UP = True

# ─────────────────────────────────────────────


class FiveBarIK:
    """
    Inverse kinematics for a 5-bar parallel leg mechanism.
    The mechanism decomposes into two independent 2-link IK problems
    that share the same foot target point.
    """

    def __init__(
        self,
        servo_sep=SERVO_SEPARATION,
        l1=L1_LEFT,
        l2=L2_RIGHT,
        l3=L3_LEFT,
        l4=L4_RIGHT,
        left_offset=LEFT_SERVO_OFFSET,
        right_offset=RIGHT_SERVO_OFFSET,
        left_elbow_up=LEFT_ELBOW_UP,
        right_elbow_up=RIGHT_ELBOW_UP,
    ):

        self.d = servo_sep
        self.l1, self.l2 = l1, l2
        self.l3, self.l4 = l3, l4
        self.left_offset = left_offset
        self.right_offset = right_offset
        self.left_elbow_up = left_elbow_up
        self.right_elbow_up = right_elbow_up

        # Fixed servo axle positions in leg frame
        self.A = np.array([0.0, 0.0])  # left servo
        self.B = np.array([self.d, 0.0])  # right servo

        # Precompute workspace limits for fast rejection
        self._max_reach_L = l1 + l3
        self._max_reach_R = l2 + l4
        self._min_reach_L = abs(l1 - l3)
        self._min_reach_R = abs(l2 - l4)

    # ------------------------------------------------------------------
    #  Core solver
    # ------------------------------------------------------------------

    def _two_link_ik(self, base, l_prox, l_dist, target, elbow_up):
        """
        Solve 2-link planar IK.
        Returns proximal link angle in radians (measured CCW from +X), or None.
        """
        v = target - base
        dist = np.linalg.norm(v)

        reach_max = l_prox + l_dist
        reach_min = abs(l_prox - l_dist)

        if dist > reach_max * 0.9999 or dist < reach_min * 1.0001:
            return None  # target out of reach

        alpha = np.arctan2(v[1], v[0])  # angle to target
        cos_beta = (dist**2 + l_prox**2 - l_dist**2) / (2 * dist * l_prox)
        cos_beta = np.clip(cos_beta, -1.0, 1.0)
        beta = np.arccos(cos_beta)

        return alpha + beta if elbow_up else alpha - beta

    def solve(self, x, z):
        """
        Compute servo commands for foot position (x, z) in leg frame (mm).
        x: forward/back  (positive = forward direction of travel)
        z: up/down       (positive = up; foot is typically at z < 0)

        Returns (left_cmd_deg, right_cmd_deg) clipped to [0, 240],
        or None if the target is unreachable.
        """
        target = np.array([x, z])

        theta_L = self._two_link_ik(
            self.A, self.l1, self.l3, target, self.left_elbow_up
        )
        theta_R = self._two_link_ik(
            self.B, self.l2, self.l4, target, self.right_elbow_up
        )

        if theta_L is None or theta_R is None:
            return None

        left_cmd = np.degrees(theta_L) + self.left_offset
        right_cmd = np.degrees(theta_R) + self.right_offset

        # Clamp to servo range
        left_cmd = float(np.clip(left_cmd, 0, 240))
        right_cmd = float(np.clip(right_cmd, 0, 240))

        return left_cmd, right_cmd

    # ------------------------------------------------------------------
    #  Forward kinematics (for verification / debugging)
    # ------------------------------------------------------------------

    def forward(self, left_cmd, right_cmd):
        """
        Given servo commands, return (elbow_L, elbow_R, foot) positions in mm.
        foot is the midpoint of the two distal-link endpoints — for a well-calibrated
        mechanism these should coincide. Any gap indicates calibration error.
        """
        theta_L = np.radians(left_cmd - self.left_offset)
        theta_R = np.radians(right_cmd - self.right_offset)

        elbow_L = self.A + self.l1 * np.array([np.cos(theta_L), np.sin(theta_L)])
        elbow_R = self.B + self.l2 * np.array([np.cos(theta_R), np.sin(theta_R)])

        # Each elbow "claims" a foot position; average them
        # For the left arm, foot is along (target - elbow_L) normalised by l3
        # We can't fully resolve without knowing the foot — so return elbows
        # and let the caller check closure error via verify_closure()
        return elbow_L, elbow_R

    def forward_foot(self, left_cmd, right_cmd):
        """Given servo commands, return the foot (x, z) position, or None.
        Intersects the two distal-link circles around the elbows and picks the
        solution matching the configured elbow_up flags."""
        theta_L = np.radians(left_cmd - self.left_offset)
        theta_R = np.radians(right_cmd - self.right_offset)

        eL = self.A + self.l1 * np.array([np.cos(theta_L), np.sin(theta_L)])
        eR = self.B + self.l2 * np.array([np.cos(theta_R), np.sin(theta_R)])

        v = eR - eL
        d = np.linalg.norm(v)
        if d > self.l3 + self.l4 or d < abs(self.l3 - self.l4) or d < 1e-6:
            return None
        a = (self.l3**2 - self.l4**2 + d**2) / (2 * d)
        h2 = self.l3**2 - a**2
        if h2 < 0:
            return None
        h = np.sqrt(h2)
        mid = eL + a * v / d
        perp = np.array([-v[1], v[0]]) / d
        p1 = mid + h * perp
        p2 = mid - h * perp
        # Pick the candidate that, round-tripped through solve(), reproduces the input cmds.
        best, best_err = None, float("inf")
        for p in (p1, p2):
            sol = self.solve(float(p[0]), float(p[1]))
            if sol is None:
                continue
            err = abs(sol[0] - left_cmd) + abs(sol[1] - right_cmd)
            if err < best_err:
                best_err, best = err, p
        return tuple(best) if best is not None else None

    def verify_closure(self, x, z, left_cmd, right_cmd, tol_mm=0.5):
        """
        Check that the forward kinematics of the solved servo angles
        actually place the elbows where they need to be to reach (x, z).
        Returns (left_error_mm, right_error_mm, ok).
        """
        target = np.array([x, z])
        theta_L = np.radians(left_cmd - self.left_offset)
        theta_R = np.radians(right_cmd - self.right_offset)

        elbow_L = self.A + self.l1 * np.array([np.cos(theta_L), np.sin(theta_L)])
        elbow_R = self.B + self.l2 * np.array([np.cos(theta_R), np.sin(theta_R)])

        err_L = abs(np.linalg.norm(target - elbow_L) - self.l3)
        err_R = abs(np.linalg.norm(target - elbow_R) - self.l4)

        return err_L, err_R, (err_L < tol_mm and err_R < tol_mm)

    # ------------------------------------------------------------------
    #  Workspace sampling (useful for plotting reachable area)
    # ------------------------------------------------------------------

    def sample_workspace(self, resolution=200):
        """
        Returns arrays (xs, zs) of reachable foot positions for plotting.
        """
        xs, zs = [], []
        # Sweep all left/right servo angle combos
        angles = np.linspace(np.radians(-150), np.radians(150), resolution)
        for tL in angles:
            for tR in angles:
                eL = self.A + self.l1 * np.array([np.cos(tL), np.sin(tL)])
                eR = self.B + self.l2 * np.array([np.cos(tR), np.sin(tR)])
                # Foot is intersection of circles around eL (r=l3) and eR (r=l4)
                v = eR - eL
                d = np.linalg.norm(v)
                if d > self.l3 + self.l4 or d < abs(self.l3 - self.l4) or d < 1e-6:
                    continue
                a = (self.l3**2 - self.l4**2 + d**2) / (2 * d)
                h2 = self.l3**2 - a**2
                if h2 < 0:
                    continue
                h = np.sqrt(h2)
                mid = eL + a * v / d
                perp = np.array([-v[1], v[0]]) / d
                for sign in [1, -1]:
                    p = mid + sign * h * perp
                    xs.append(p[0])
                    zs.append(p[1])
        return np.array(xs), np.array(zs)


# ─────────────────────────────────────────────
#  CALIBRATION HELPER
# ─────────────────────────────────────────────


class ServoCalibrator:
    """
    Finds LEFT_SERVO_OFFSET and RIGHT_SERVO_OFFSET by asking you to
    move each servo to a known physical position and reading the command value.

    Usage:
        cal = ServoCalibrator(ik)
        cal.calibrate_from_known_pose(
            x_known, z_known,           # foot position you measured physically (mm)
            left_cmd_used,              # servo command you sent to get there
            right_cmd_used
        )
    """

    def __init__(self, ik: FiveBarIK):
        self.ik = ik

    def calibrate_from_known_pose(self, x, z, left_cmd_actual, right_cmd_actual):
        """
        Given a pose where you KNOW the foot position (x, z) and you KNOW
        what servo commands produced it, back-calculate the offsets.

        Steps:
          1. Command the robot to a known pose (e.g. leg straight down).
          2. Measure the actual foot position (x, z) with a ruler.
          3. Read the servo command values you used.
          4. Call this function.
        """
        target = np.array([x, z])

        # Solve with zero offsets to get theoretical mechanical angles
        ik_zero = FiveBarIK(
            self.ik.d,
            self.ik.l1,
            self.ik.l2,
            self.ik.l3,
            self.ik.l4,
            left_offset=0.0,
            right_offset=0.0,
            left_elbow_up=self.ik.left_elbow_up,
            right_elbow_up=self.ik.right_elbow_up,
        )
        result = ik_zero.solve(x, z)
        if result is None:
            print(
                f"ERROR: ({x}, {z}) is outside the workspace. Try a different calibration pose."
            )
            return

        left_theoretical, right_theoretical = (
            result  # these are in degrees from +X axis
        )

        left_offset = left_cmd_actual - left_theoretical
        right_offset = right_cmd_actual - right_theoretical

        print(f"Calibration result:")
        print(f"  LEFT_SERVO_OFFSET  = {left_offset:.2f}")
        print(f"  RIGHT_SERVO_OFFSET = {right_offset:.2f}")
        print(f"  → Copy these into the CALIBRATION CONSTANTS at the top of this file.")

        self.ik.left_offset = left_offset
        self.ik.right_offset = right_offset
        return left_offset, right_offset


# ─────────────────────────────────────────────
#  FACTORY FUNCTIONS
# ─────────────────────────────────────────────


def make_left_leg_ik():
    return FiveBarIK(
        left_offset=LEFT_LEG_REAR_OFFSET, right_offset=LEFT_LEG_FRONT_OFFSET
    )


def make_right_leg_ik():
    return FiveBarIK(
        left_offset=RIGHT_LEG_REAR_OFFSET, right_offset=RIGHT_LEG_FRONT_OFFSET
    )


# ─────────────────────────────────────────────
#  QUICK TEST
# ─────────────────────────────────────────────

if __name__ == "__main__":
    ik = FiveBarIK()

    # Test a foot position below the servos, centered, 120mm down
    test_x, test_z = 0.0, -120.0
    result = ik.solve(test_x, test_z)

    if result:
        l_cmd, r_cmd = result
        print(f"Foot ({test_x}, {test_z}) mm  →  left={l_cmd:.1f}°  right={r_cmd:.1f}°")
        err_L, err_R, ok = ik.verify_closure(test_x, test_z, l_cmd, r_cmd)
        print(
            f"Closure error: L={err_L:.4f} mm  R={err_R:.4f} mm  {'✓' if ok else '✗'}"
        )
    else:
        print("Target unreachable — check link lengths or target position")

    # Sweep a stride to check for reachability holes
    print("\nStride sweep (x from -30 to +30, z = -110):")
    for x in np.linspace(-30, 30, 7):
        r = ik.solve(x, -110.0)
        status = f"L={r[0]:.1f}° R={r[1]:.1f}°" if r else "UNREACHABLE"
        print(f"  x={x:+.0f}  {status}")

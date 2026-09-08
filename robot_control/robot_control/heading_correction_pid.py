"""
heading_correction_pid.py

MEMBER 4 - Heading Correction PID

This module is self-contained on purpose so it can be reviewed,
tested, and version-controlled independently of the linear/yaw
action servers written by other team members. It is meant to be
imported into movement_server.py and used INSIDE the existing
move_x loop (it does not run its own loop or publish anything by
itself).

Exposes:
    normalize_angle(angle) -> float
    HeadingCorrectionPID   -> class
"""

import math


def normalize_angle(angle):
    """
    ANGLE NORMALIZATION

    Keeps an angle within the range [-pi, pi] so error calculations
    never blow up when crossing the +pi / -pi boundary.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class HeadingCorrectionPID:
    """
    Watches how far the robot's current heading has drifted away
    from the heading it started with, and outputs a small angular
    velocity correction meant to be added into the SAME /cmd_vel
    message that the linear PID (Member 2) is already publishing.

    This does NOT drive the robot forward and does NOT replace the
    Yaw PID (Member 3), which is used for standalone rotation goals.
    """

    def __init__(
        self,
        kp,
        ki,
        kd,
        deadzone=0.01,
        integral_limit=0.3,
        output_limit=0.3
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.deadzone = deadzone
        self.integral_limit = integral_limit
        self.output_limit = output_limit

        self.integral = 0.0
        self.previous_error = 0.0

    def reset(self):
        """Call this at the start of every new move_x goal."""
        self.integral = 0.0
        self.previous_error = 0.0

    def update_gains(self, kp, ki, kd):
        """
        Hook for Member 5's dynamic parameters callback, so gains
        can be updated at runtime without recreating this object.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, start_yaw, current_yaw, dt):

        # ------------------------------------------------------
        # 1. Heading error (normalized)
        # ------------------------------------------------------

        error = normalize_angle(start_yaw - current_yaw)

        # ------------------------------------------------------
        # 2. TARGET DEADZONE
        # ------------------------------------------------------

        if abs(error) < self.deadzone:
            self.previous_error = 0.0
            return 0.0

        if dt <= 0.0:
            dt = 0.001

        # ------------------------------------------------------
        # 3. ZERO-CROSSING RESET
        # ------------------------------------------------------

        if self.previous_error * error < 0.0:
            self.integral = 0.0

        candidate_integral = self.integral + error * dt

        # ------------------------------------------------------
        # 4. INTEGRAL ANTI-WINDUP (CLAMPING)
        # ------------------------------------------------------

        candidate_integral = max(
            -self.integral_limit,
            min(candidate_integral, self.integral_limit)
        )

        proportional = self.kp * error
        derivative = self.kd * (error - self.previous_error) / dt

        candidate_output = (
            proportional
            + self.ki * candidate_integral
            + derivative
        )

        # ------------------------------------------------------
        # 5. CONDITIONAL INTEGRATION
        # ------------------------------------------------------

        if abs(candidate_output) > self.output_limit and (
            (candidate_output > 0 and error > 0)
            or (candidate_output < 0 and error < 0)
        ):
            integral_to_use = self.integral
        else:
            integral_to_use = candidate_integral

        self.integral = integral_to_use

        # ------------------------------------------------------
        # 6. FINAL OUTPUT
        # ------------------------------------------------------

        output = (
            self.kp * error
            + self.ki * self.integral
            + derivative
        )

        # ------------------------------------------------------
        # 7. OUTPUT CLAMPING
        # ------------------------------------------------------

        output = max(-self.output_limit, min(output, self.output_limit))

        self.previous_error = error

        return output
"""DWB policy for a base carrying a forward-facing depth camera.

The policy constrains translation and scores heading. Stock DWB critics retain
path distance, collision checking, oscillation handling, and final goal rotation.
"""
import math


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


class ForwardCameraPolicy:
    def configure(self, options):
        self.camera_half_angle = float(options.get("camera_half_angle", math.pi / 6))
        self.goal_tolerance = float(options.get("goal_tolerance", 0.25))
        self.heading_weight = float(options.get("heading_weight", 80.0))
        self.translation_epsilon = float(options.get("translation_epsilon", 0.0001))
        values = (self.camera_half_angle, self.goal_tolerance,
                  self.heading_weight, self.translation_epsilon)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Policy options must be finite and positive")
        if self.camera_half_angle >= math.pi / 2:
            raise ValueError("camera_half_angle must be less than pi/2 radians")

    def prepare(self, yaw, path_heading, goal_distance):
        self.yaw = yaw
        self.path_heading = path_heading
        self.near_goal = goal_distance <= self.goal_tolerance

    def score(self, vx, vy, wz, final_heading):
        translating = math.hypot(vx, vy) > self.translation_epsilon
        if translating:
            # DWB velocities use the base frame. Reverse and sideways motion
            # leave the front camera's configured viewing cone.
            if abs(math.atan2(vy, vx)) > self.camera_half_angle:
                return None
        # RotateToGoal owns the final yaw once inside the XY tolerance.
        if self.near_goal:
            return 0.0
        if translating:
            # Turn before moving into a new path direction. Pure rotation,
            # including a zero command, stays available for other DWB critics.
            if abs(angle(self.path_heading - self.yaw)) > self.camera_half_angle:
                return None
        error = angle(self.path_heading - final_heading)
        return self.heading_weight * error * error

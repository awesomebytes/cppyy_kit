"""Python position servo called by Gazebo's ros2_control update loop.

The bridge supplies ordered position feedback and dt in seconds. This class owns
its target, proportional law, speed bounds and position bounds.
"""

import json
import math
import os
from pathlib import Path


class PositionServo:
    def configure(self, joint_names, options):
        self.joints = list(joint_names)
        self.gain = float(options.get("gain", 6.0))
        self.speed = float(options.get("speed", 0.15))
        self.lower = options["lower_limits"]
        self.upper = options["upper_limits"]
        self.evidence_path = options.get("evidence_path")
        self.callback_count = 0
        if not all(math.isfinite(v) and v > 0 for v in (self.gain, self.speed)):
            raise ValueError("gain and speed must be finite and positive")
        if len(self.lower) != len(self.joints) or len(self.upper) != len(self.joints):
            raise ValueError("position limits must match the joint count")
        for lower, upper in zip(self.lower, self.upper):
            if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
                raise ValueError("each lower limit must be finite and below its upper limit")

    def checked_positions(self, values):
        if len(values) != len(self.joints):
            raise ValueError("positions must match the joint count")
        for value, lower, upper in zip(values, self.lower, self.upper):
            if not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError("position must be finite and within its joint limits")
        return list(values)

    def activate(self, measured_positions):
        self.target = self.checked_positions(measured_positions)
        self.callback_count = 0

    def set_target(self, values):
        self.target = self.checked_positions(values)

    def update(self, measured_positions, dt):
        measured = self.checked_positions(measured_positions)
        if not math.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("dt must be finite and between 0 and 0.1 seconds")
        step_limit = self.speed * dt
        gain_fraction = min(1.0, self.gain * dt)
        commands = []
        for position, target in zip(measured, self.target):
            step = gain_fraction * (target - position)
            step = max(-step_limit, min(step_limit, step))
            commands.append(position + step)
        self.callback_count += 1
        return commands

    def deactivate(self):
        # Write evidence outside update(), after the arm controller stops updating.
        if self.evidence_path:
            Path(self.evidence_path).write_text(json.dumps({
                "pid": os.getpid(), "callback_count": self.callback_count,
                "module": __name__, "class": type(self).__name__,
            }, indent=2) + "\n")

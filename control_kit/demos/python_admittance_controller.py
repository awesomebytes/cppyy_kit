"""Cartesian virtual-force admittance for the embedded Python controller bridge.

set_target([fx, fy, fz]) applies a virtual force in newtons, expressed in the
configured base_link frame. It does not read a physical force sensor.
"""

import json
import math
import os
import time
from pathlib import Path

import numpy as np

from arm_kinematics import Chain


class CartesianAdmittance:
    def configure(self, joint_names, options):
        self.joints = list(joint_names)
        defaults = {"mass": 2.0, "damping": 30.0, "stiffness": 80.0,
                    "max_displacement": 0.03, "max_cartesian_speed": 0.06,
                    "joint_speed": 0.10, "tracking_gain": 4.0, "damping_lambda": 0.04,
                    "force_timeout": 0.3, "max_force": 20.0}
        for key, default in defaults.items():
            value = float(options.get(key, default))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be finite and positive")
            setattr(self, key, value)
        self.lower = np.array(options["lower_limits"], dtype=np.float64)
        self.upper = np.array(options["upper_limits"], dtype=np.float64)
        if (self.lower.shape != (len(self.joints),) or self.upper.shape != self.lower.shape
                or not np.isfinite(self.lower).all() or not np.isfinite(self.upper).all()
                or np.any(self.lower >= self.upper)):
            raise ValueError("Finite ordered joint limits must match the joint count")
        self.base_link, self.tip_link = options["base_link"], options["tip_link"]
        self.chain = Chain(options["urdf_path"], self.joints, self.base_link, self.tip_link)
        self.evidence_path = options.get("evidence_path")
        self.active = False

    def _positions(self, values):
        q = np.ascontiguousarray(values, dtype=np.float64)
        if (q.shape != self.lower.shape or not np.isfinite(q).all()
                or np.any(q < self.lower) or np.any(q > self.upper)):
            raise ValueError(
                "Feedback requires finite positions within the configured joint limits"
            )
        return q

    @staticmethod
    def _bound(vector, limit):
        norm = np.linalg.norm(vector)
        if norm > limit:
            vector *= limit / norm
        return vector

    def activate(self, measured_positions):
        q = self._positions(measured_positions)
        self.reference = self.chain.fk(q).copy()
        self.target = self.reference.copy()
        self.offset = np.zeros(3)
        self.velocity = np.zeros(3)
        self.force = np.zeros(3)
        self.force_age = self.force_timeout
        self.force_stamp = -math.inf
        self.callback_count = 0
        self.native_elapsed_ns = 0
        self.native_max_ns = 0
        self.last_measured = q.copy()
        self.peak_displacement = 0.0
        self.peak_command_step = 0.0
        self.active = True

    def set_target(self, values):
        force = np.array(values, dtype=np.float64)
        if force.shape != (3,) or not np.isfinite(force).all():
            raise ValueError("Virtual force requires three finite values in newtons")
        if not self.active:
            raise ValueError("Activate the admittance controller before applying force")
        self.force = self._bound(force, self.max_force)
        self.force_age = 0.0
        self.force_stamp = time.monotonic()

    def update(self, measured_positions, dt):
        if not self.active:
            raise ValueError("Admittance controller is inactive")
        q = self._positions(measured_positions)
        if not math.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("dt must be finite and between 0 and 0.1 seconds")
        # Integrate M*x'' + D*x' + K*x = F. Substeps bound integration error
        # when simulation updates arrive less frequently than the nominal loop.
        steps = max(1, math.ceil(dt / 0.002))
        h = dt / steps
        for _ in range(steps):
            force = (self.force if self.force_age < self.force_timeout
                     and time.monotonic() - self.force_stamp < self.force_timeout
                     else np.zeros(3))
            acceleration = (
                force - self.damping*self.velocity - self.stiffness*self.offset
            ) / self.mass
            self.velocity += h * acceleration
            self._bound(self.velocity, self.max_cartesian_speed)
            self.offset += h * self.velocity
            if np.linalg.norm(self.offset) > self.max_displacement:
                self._bound(self.offset, self.max_displacement)
                normal = self.offset / self.max_displacement
                self.velocity -= max(0.0, float(self.velocity @ normal)) * normal
            self.force_age += h
        self.target[:3, 3] = self.reference[:3, 3] + self.offset
        started = time.perf_counter_ns()
        command = self.chain.step(q, self.target, self.lower, self.upper, dt,
                                  self.tracking_gain, self.damping_lambda, self.joint_speed)
        elapsed = time.perf_counter_ns() - started
        self.native_elapsed_ns += elapsed
        self.native_max_ns = max(self.native_max_ns, elapsed)
        self.last_measured[:] = q
        if not np.isfinite(command).all():
            raise ValueError("Native IK returned a nonfinite command")
        self.callback_count += 1
        self.peak_displacement = max(
            self.peak_displacement, float(np.linalg.norm(self.offset))
        )
        self.peak_command_step = max(self.peak_command_step, float(np.max(np.abs(command-q))))
        return command.tolist()

    def deactivate(self):
        self.active = False
        self.force[:] = 0.0
        if self.evidence_path:
            pose = self.chain.fk(self.last_measured).copy()
            rotation = self.reference[:3, :3].T @ pose[:3, :3]
            angle = math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
            Path(self.evidence_path).write_text(json.dumps({
                "pid": os.getpid(), "module": __name__, "class": type(self).__name__,
                "callback_count": self.callback_count, "base_link": self.base_link,
                "tip_link": self.tip_link, "peak_displacement_m": self.peak_displacement,
                "peak_command_step_rad": self.peak_command_step,
                "native_kernel": "cppyy_kit.cppdef_cached:demo_arm_kinematics",
                "native_step_mean_us": (
                    self.native_elapsed_ns / max(1, self.callback_count) / 1000
                ),
                "native_step_max_us": self.native_max_ns / 1000,
                "final_measured_displacement_m": float(
                    np.linalg.norm(pose[:3, 3] - self.reference[:3, 3])
                ),
                "final_measured_orientation_error_rad": angle,
            }, indent=2) + "\n")

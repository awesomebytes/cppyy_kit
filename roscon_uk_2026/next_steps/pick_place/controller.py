"""Weighted whole-body task servo for the TIAGo PRO demonstration.

This is a local velocity controller, not a motion planner. Goal weights and joint
costs express preferences. They do not implement strict task priority. Obstacle
checks cover sampled arm segments with a configurable radius, not the full URDF.
"""

from dataclasses import dataclass

import numpy as np
from control_kit.task_control import TaskModel

if __package__:
    from .robot_config import (
        ARM_SEGMENT_FRAMES,
        BASE_JOINTS,
        BASE_SPEED_LIMITS,
        DEFAULT_JOINT_COSTS,
        ELBOW_FRAMES,
        GAZE_FRAME,
        active_joint_names,
    )
else:
    from robot_config import (
        ARM_SEGMENT_FRAMES,
        BASE_JOINTS,
        BASE_SPEED_LIMITS,
        DEFAULT_JOINT_COSTS,
        ELBOW_FRAMES,
        GAZE_FRAME,
        active_joint_names,
    )


def _rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-10:
        raise ValueError("Orientation must be a finite nonzero XYZW quaternion")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _log_rotation(r):
    angle = np.arccos(np.clip((np.trace(r) - 1) * 0.5, -1, 1))
    vee = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1e-6:
        return 0.5 * vee
    if np.pi - angle < 1e-4:
        values, vectors = np.linalg.eigh((r + r.T) * 0.5)
        axis = vectors[:, np.argmax(values)]
        return angle * axis
    return angle / (2 * np.sin(angle)) * vee


def _xyz(value):
    v = np.asarray(value, dtype=float)
    if v.shape != (3,) or not np.isfinite(v).all():
        raise ValueError("Expected three finite coordinates")
    return v


def obstacle_distance(point, obstacle):
    """Signed distance to an axis-aligned box, sphere, or capsule in metres."""
    kind = obstacle["type"]
    if kind == "sphere":
        return float(
            np.linalg.norm(point - _xyz(obstacle["center"])) - obstacle["radius"]
        )
    if kind == "capsule":
        a, b = _xyz(obstacle["a"]), _xyz(obstacle["b"])
        ab = b - a
        t = np.clip(np.dot(point - a, ab) / max(np.dot(ab, ab), 1e-15), 0, 1)
        return float(np.linalg.norm(point - a - t * ab) - obstacle["radius"])
    if kind == "box":
        d = np.abs(point - _xyz(obstacle["center"])) - _xyz(obstacle["size"]) * 0.5
        return float(np.linalg.norm(np.maximum(d, 0)) + min(np.max(d), 0))
    raise ValueError(f"Unknown obstacle type: {kind}")


@dataclass
class Command:
    joints: dict
    base_velocity: tuple
    diagnostics: dict


class WholeBodyController:
    """Position, orientation, elbow and gaze goals with bounded joint increments."""

    def __init__(self, urdf, allow_base=True):
        names = active_joint_names(allow_base)
        self.model = TaskModel(urdf, names)
        self.joint_names = names
        self.goals = {}
        self.gaze = None
        self.obstacles = []
        self.gain = 4.0
        self.damping = 0.003
        self.margin = 0.04
        self.arm_radius = 0.055
        self.cost = np.array([DEFAULT_JOINT_COSTS[name] for name in names])
        self.speed = np.array(
            [
                BASE_SPEED_LIMITS[name]
                if name in BASE_JOINTS
                else min(0.35, self.model.limits[name][2])
                for name in names
            ]
        )
        self.segment_links = [
            [self.model.links[frame] for frame in frames]
            for frames in ARM_SEGMENT_FRAMES.values()
        ]

    def set_base_bounds(self, axis, lower, upper):
        """Bound a virtual base coordinate in metres, or radians for yaw.

        Axis accepts x, y, yaw, or the corresponding base coordinate name.
        Infinite endpoints leave that direction unbounded.
        """
        name = axis if axis in BASE_JOINTS else f"base_{axis}"
        if name not in BASE_JOINTS:
            raise ValueError("Base axis must be x, y or yaw")
        if name not in self.joint_names:
            raise ValueError("Base bounds require allow_base=True")
        lower, upper = float(lower), float(upper)
        if np.isnan(lower) or np.isnan(upper) or lower > upper:
            raise ValueError("Base bounds must be ordered and cannot contain NaN")
        self.model.limits[name] = (lower, upper, BASE_SPEED_LIMITS[name])

    def as_ros2_controller(self, base_pose=(0.0, 0.0, 0.0)):
        """Run this task servo as a real in-process ros2_control controller.

        Virtual planar base coordinates have no position hardware interfaces. Use
        allow_base=False for this adapter. The simulator uses native trajectory
        controllers through robot.py because its controller manager is external.
        """
        if any(name in BASE_JOINTS for name in self.joint_names):
            raise ValueError("Use allow_base=False for a position-interface controller")
        import control_kit

        return control_kit.make_position_task_controller(
            self.joint_names,
            lambda state, dt: (
                self.step(state, base_pose, min(0.2, max(0.001, dt))).joints
            ),
        )

    def set_goal(self, link, position=None, orientation=None, weight=1.0):
        if link not in self.model.links:
            raise KeyError(link)
        if position is None and orientation is None:
            self.goals.pop(link, None)
            return
        if weight <= 0 or not np.isfinite(weight):
            raise ValueError("Goal weight must be positive")
        self.goals[link] = (
            None if position is None else _xyz(position),
            None if orientation is None else _rotation(orientation),
            float(weight),
        )

    def set_elbow_goal(self, side, position, weight=0.03):
        self.set_goal(ELBOW_FRAMES[side], position=position, weight=weight)

    def look_at(self, position):
        self.gaze = None if position is None else _xyz(position)

    def set_obstacles(self, obstacles):
        checked = []
        for obstacle in obstacles:
            obstacle = dict(obstacle)
            obstacle_distance(np.zeros(3), obstacle)
            if obstacle["type"] in ("sphere", "capsule") and (
                not np.isfinite(obstacle["radius"]) or obstacle["radius"] <= 0
            ):
                raise ValueError("Obstacle radius must be finite and positive")
            if obstacle["type"] == "box" and np.any(_xyz(obstacle["size"]) <= 0):
                raise ValueError("Box size must be positive")
            checked.append(obstacle)
        self.obstacles = checked

    def pose(self, link, positions, base_pose=(0.0, 0.0, 0.0)):
        return self.model.fk(self.model.configuration(positions, base_pose))[
            self.model.links[link]
        ].copy()

    def _clearances(self, transforms):
        values = []
        for chain in self.segment_links:
            for a, b in zip(chain[:-1], chain[1:], strict=True):
                start, end = transforms[a, :3, 3], transforms[b, :3, 3]
                # Inflate each sample by half its interval to cover the entire segment.
                radius = self.arm_radius + np.linalg.norm(end - start) / 8
                for t in np.linspace(0, 1, 5):
                    p = start * (1 - t) + end * t
                    values.extend(
                        obstacle_distance(p, o) - radius for o in self.obstacles
                    )
        return np.asarray(values)

    def _residual(self, transforms, collision_rows):
        values = []
        for link, (position, rotation, weight) in self.goals.items():
            t = transforms[self.model.links[link]]
            scale = np.sqrt(weight)
            if position is not None:
                values.extend(scale * (position - t[:3, 3]))
            if rotation is not None:
                values.extend(scale * 0.3 * _log_rotation(rotation @ t[:3, :3].T))
        if self.gaze is not None:
            t = transforms[self.model.links[GAZE_FRAME]]
            direction = self.gaze - t[:3, 3]
            direction = direction / max(np.linalg.norm(direction), 1e-12)
            values.extend(0.2 * (direction - t[:3, 2]))
        if len(collision_rows):
            distances = self._clearances(transforms)[collision_rows]
            values.extend(5.0 * np.maximum(0.0, self.margin - distances))
        return np.asarray(values)

    def _task_velocity(self, q, error, collision_rows, dt):
        """Linearize task residuals and solve within coordinate speed bounds."""
        velocity = np.zeros(len(self.joint_names))
        jacobian = np.empty((len(error), len(velocity)))
        for col, index in enumerate(self.model.active):
            shifted = q.copy()
            shifted[index] += 1e-5
            jacobian[:, col] = (
                error - self._residual(self.model.fk(shifted), collision_rows)
            ) / 1e-5
        lower = -self.speed.copy()
        upper = self.speed.copy()
        for col, (index, name) in enumerate(
            zip(self.model.active, self.joint_names, strict=True)
        ):
            if name in self.model.limits:
                lo, hi, _ = self.model.limits[name]
                lower[col] = max(lower[col], (lo - q[index]) / dt)
                upper[col] = min(upper[col], (hi - q[index]) / dt)
        # Fix bound violations and re-solve the remaining coordinates. This keeps
        # a saturated torso or arm from suppressing the available base motion.
        fixed = np.zeros(len(velocity))
        free = jacobian.copy()
        for _ in range(len(velocity) + 1):
            velocity = (
                self.model.solve(
                    free,
                    self.gain * error - jacobian @ fixed,
                    self.cost,
                    damping=self.damping,
                )
                + fixed
            )
            clipped = np.clip(velocity, lower, upper)
            violating = np.abs(clipped - velocity) > 1e-10
            if not np.any(violating):
                break
            fixed[violating] = clipped[violating]
            free[:, violating] = 0.0
        velocity = np.clip(velocity, lower, upper)
        return velocity

    def _integrate(self, q, velocity, dt):
        """Apply a velocity step and clamp coordinates to their position bounds."""
        candidate = q.copy()
        for index, v, name in zip(
            self.model.active, velocity, self.joint_names, strict=True
        ):
            candidate[index] += v * dt
            if name in self.model.limits:
                lo, hi, _ = self.model.limits[name]
                candidate[index] = np.clip(candidate[index], lo, hi)
        return candidate

    def _accept_step(self, q, candidate, clearances):
        # A discrete command cannot cross a sampled safety boundary. Inside a boundary,
        # accept only steps that preserve or improve every sampled clearance.
        accepted = False
        for _ in range(10):
            clearance_new = self._clearances(self.model.fk(candidate))
            if not len(clearances) or np.all(
                clearance_new >= np.minimum(clearances, self.margin) - 1e-8
            ):
                accepted = True
                break
            candidate = q + 0.5 * (candidate - q)
        if not accepted:
            candidate = q.copy()
        return candidate, accepted

    def step(self, positions, base_pose=(0.0, 0.0, 0.0), dt=0.05):
        """Return bounded targets from measured state.

        Reject coordinates outside their position bounds by more than 1e-6.
        Clipping such a state into bounds could exceed the configured speed.
        """
        if not np.isfinite(dt) or not 0 < dt <= 0.2:
            raise ValueError("dt must be in (0, 0.2] seconds")
        missing = [
            n for n in self.joint_names if n not in BASE_JOINTS and n not in positions
        ]
        if missing:
            raise ValueError(f"Missing measured joint positions: {missing}")
        q = self.model.configuration(positions, base_pose)
        for name, index in zip(self.joint_names, self.model.active, strict=True):
            if name in self.model.limits:
                lower, upper, _ = self.model.limits[name]
                if q[index] < lower - 1e-6 or q[index] > upper + 1e-6:
                    raise ValueError(
                        f"Measured coordinate {name}={q[index]} is outside "
                        f"position bounds [{lower}, {upper}]"
                    )
        transforms = self.model.fk(q)
        clearances = self._clearances(transforms)
        collision_rows = np.flatnonzero(clearances < self.margin + 0.04)
        error = self._residual(transforms, collision_rows)
        velocity = (
            self._task_velocity(q, error, collision_rows, dt)
            if len(error)
            else np.zeros(len(self.joint_names))
        )
        candidate = self._integrate(q, velocity, dt)
        candidate, accepted = self._accept_step(q, candidate, clearances)
        achieved = (candidate - q) / dt
        joints = {
            name: float(candidate[index])
            for name, index in zip(self.joint_names, self.model.active, strict=True)
            if name not in BASE_JOINTS
        }
        return Command(
            joints,
            tuple(float(v) for v in achieved[:3]),
            {
                "task_error": float(np.linalg.norm(error)),
                "minimum_clearance": float(clearances.min())
                if len(clearances)
                else None,
                "step_accepted": accepted,
                "scaled_speed": float(np.max(np.abs(achieved))),
            },
        )

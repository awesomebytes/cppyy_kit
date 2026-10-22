"""Placo velocity IK with full URDF collision geometry and the robot SRDF.

Joint costs express soft motion preferences, not strict task priority. Native QP
constraints enforce joint limits, velocity limits and linearized collision
clearance. A distance check also rejects discrete steps that reduce clearance
below the margin. This local servo does not plan paths around obstacles.

API sources: https://placo.readthedocs.io/en/latest/kinematics/getting_started.html
https://placo.readthedocs.io/en/latest/kinematics/avoid_self_collisions_constraint.html
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import placo

if __package__:
    from .controller import Command, _rotation, _xyz, obstacle_distance
    from .robot_config import (
        BASE_JOINTS,
        BASE_SPEED_LIMITS,
        DEFAULT_JOINT_COSTS,
        ELBOW_FRAMES,
        GAZE_FRAME,
        active_joint_names,
    )
else:
    from controller import Command, _rotation, _xyz, obstacle_distance
    from robot_config import (
        BASE_JOINTS,
        BASE_SPEED_LIMITS,
        DEFAULT_JOINT_COSTS,
        ELBOW_FRAMES,
        GAZE_FRAME,
        active_joint_names,
    )


MODEL_ROOT = Path(__file__).resolve().parent.parent / "tiago_pro_jazzy"


class PlacoWholeBodyController:
    """Control arm poses, elbows and gaze with Placo's native kinematics QP.

    Obstacles use world coordinates and support boxes, spheres and capsules.
    Robot self-collision pairs come from the supplied SRDF. With no SRDF argument,
    the generated TIAGo PRO SRDF is required. Visual geometry is not loaded.
    """

    def __init__(self, urdf, allow_base=True, srdf=None, package_dirs=None):
        self.joint_names = active_joint_names(allow_base)
        self.allow_base = allow_base
        self.goals = {}
        self.gaze = None
        self.obstacles = []
        self.gain = 4.0
        self.orientation_scale = 0.3  # Metres per radian in the soft task cost.
        self.margin = 0.002
        self.trigger = 0.04
        self.base_bounds = {name: (-np.inf, np.inf) for name in BASE_JOINTS}
        self.srdf = (
            Path(srdf) if srdf else MODEL_ROOT / "moveit/generated/tiago_pro.srdf"
        )
        if not self.srdf.is_file():
            raise FileNotFoundError(f"Self-collision SRDF is required: {self.srdf}")
        self.package_dirs = [
            Path(p) for p in (package_dirs or [MODEL_ROOT / "generated/prefix/share"])
        ]
        self._urdf = self._prepare_urdf(urdf)
        self._build_solver()

    def _prepare_urdf(self, urdf):
        root = ET.fromstring(urdf)
        self._links = {link.get("name") for link in root.findall("link")}
        self._mimics = []
        for link in root.findall("link"):
            for visual in link.findall("visual"):
                link.remove(visual)
        for mesh in root.findall(".//mesh"):
            uri = mesh.get("filename", "")
            if uri.startswith("package://"):
                candidates = [
                    directory / uri.removeprefix("package://")
                    for directory in self.package_dirs
                ]
                path = next((p.resolve() for p in candidates if p.is_file()), None)
                if path is None:
                    raise FileNotFoundError(f"Collision mesh was not found: {uri}")
                mesh.set("filename", str(path))
        children = set()
        for joint in root.findall("joint"):
            children.add(joint.find("child").get("link"))
            mimic = joint.find("mimic")
            if mimic is not None:
                self._mimics.append(
                    (
                        joint.get("name"),
                        mimic.get("joint"),
                        float(mimic.get("multiplier", 1)),
                        float(mimic.get("offset", 0)),
                    )
                )
                joint.remove(mimic)
            # Placo's joint bounds require one configuration coordinate per URDF
            # DoF. Passive continuous wheels are masked and retain their angle.
            if joint.get("type") == "continuous":
                joint.set("type", "revolute")
                limit = joint.find("limit")
                if limit is None:
                    limit = ET.SubElement(joint, "limit", effort="0", velocity="1")
                limit.set("lower", "-1e10")
                limit.set("upper", "1e10")
        roots = self._links - children
        if len(roots) != 1:
            raise ValueError("URDF must have one root link")
        parent = "placo_world"
        ET.SubElement(root, "link", name=parent)
        for name, axis in zip(BASE_JOINTS, ("1 0 0", "0 1 0", "0 0 1")):
            child = f"placo_{name}_link"
            ET.SubElement(root, "link", name=child)
            joint = ET.SubElement(
                root,
                "joint",
                name=name,
                type="revolute" if name == "base_yaw" else "prismatic",
            )
            ET.SubElement(joint, "parent", link=parent)
            ET.SubElement(joint, "child", link=child)
            ET.SubElement(joint, "axis", xyz=axis)
            ET.SubElement(
                joint,
                "limit",
                lower="-1e10",
                upper="1e10",
                effort="0",
                velocity=str(BASE_SPEED_LIMITS[name]),
            )
            parent = child
        joint = ET.SubElement(root, "joint", name="placo_robot_root", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=roots.pop())
        return ET.tostring(root, encoding="unicode")

    def _build_solver(self):
        # Ignore default pairs during loading. The SRDF is applied before enabling
        # native collision avoidance, without filtering pairs from a measured pose.
        self.robot = placo.RobotWrapper(
            str(MODEL_ROOT), placo.Flags.ignore_collisions, self._urdf
        )
        geometry = self.robot.collision_model
        geometry.addAllCollisionPairs()
        pin.removeCollisionPairs(self.robot.model, geometry, str(self.srdf))
        self._robot_geometry_count = geometry.ngeoms
        self._collision_pairs = [
            (pair.first, pair.second) for pair in geometry.collisionPairs
        ]
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        for name in self.robot.joint_names():
            if name not in self.joint_names:
                self.solver.mask_dof(name)
                # Measured passive joints cannot be corrected by this solver.
                # Preserve their actual FK and mask their increments; applying
                # position bounds here would make feedback overshoot infeasible.
                self.robot.set_joint_limits(name, -np.inf, np.inf)
            elif name not in BASE_JOINTS:
                index = self.robot.get_joint_v_offset(name)
                self.robot.set_velocity_limit(
                    name, min(0.35, self.robot.model.velocityLimit[index])
                )
        self.solver.enable_joint_limits(True)
        self.solver.enable_velocity_limits(True)
        regularization = self.solver.add_regularization_task(0.003)
        for name in self.joint_names:
            regularization.set_joint_weight(name, 0.003 * DEFAULT_JOINT_COSTS[name])
        self.collision_constraint = self.solver.add_avoid_self_collisions_constraint()
        self.collision_constraint.configure("robot_and_environment_clearance", "hard")
        self._tasks = {}
        self._gaze_task = None
        if self.allow_base:
            for axis, bounds in self.base_bounds.items():
                self.set_base_bounds(axis, *bounds)
        self._cache_geometry()

    def set_base_bounds(self, axis, lower, upper):
        name = axis if axis in BASE_JOINTS else f"base_{axis}"
        if name not in BASE_JOINTS:
            raise ValueError("Base axis must be x, y or yaw")
        if name not in self.joint_names:
            raise ValueError("Base bounds require allow_base=True")
        lower, upper = float(lower), float(upper)
        if np.isnan(lower) or np.isnan(upper) or lower > upper:
            raise ValueError("Base limits must be ordered and must not be NaN")
        self.base_bounds[name] = (lower, upper)
        self.robot.set_joint_limits(name, lower, upper)

    def set_goal(self, link, position=None, orientation=None, weight=1.0):
        if link not in self._links:
            raise KeyError(link)
        if position is not None or orientation is not None:
            if weight <= 0 or not np.isfinite(weight):
                raise ValueError("Goal weight must be positive")
            goal = (
                None if position is None else _xyz(position).copy(),
                None if orientation is None else _rotation(orientation),
                float(weight),
            )
        else:
            goal = None
        for task in self._tasks.pop(link, []):
            self.solver.remove_task(task)
        if goal is None:
            self.goals.pop(link, None)
            return
        self.goals[link] = goal
        tasks = []
        if goal[0] is not None:
            tasks.append(self.solver.add_position_task(link, goal[0]))
        if goal[1] is not None:
            tasks.append(self.solver.add_orientation_task(link, goal[1]))
        for task in tasks:
            scale = (
                self.orientation_scale**2
                if isinstance(task, placo.OrientationTask)
                else 1.0
            )
            task.configure(link, "soft", weight * scale)
        self._tasks[link] = tasks

    def set_elbow_goal(self, side, position, weight=0.03):
        self.set_goal(ELBOW_FRAMES[side], position=position, weight=weight)

    def look_at(self, position):
        self.gaze = None if position is None else _xyz(position).copy()
        if self._gaze_task is not None:
            self.solver.remove_task(self._gaze_task)
            self._gaze_task = None
        if self.gaze is not None:
            self._gaze_task = self.solver.add_axisalign_task(
                GAZE_FRAME, np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])
            )
            self._gaze_task.configure("camera_gaze", "soft", 0.04)

    def set_obstacles(self, obstacles):
        checked = []
        for source in obstacles:
            obstacle = dict(source)
            obstacle_distance(np.zeros(3), obstacle)
            if obstacle["type"] in ("sphere", "capsule"):
                radius = float(obstacle["radius"])
                if not np.isfinite(radius) or radius <= 0:
                    raise ValueError("Obstacle radius must be finite and positive")
            elif np.any(_xyz(obstacle["size"]) <= 0):
                raise ValueError("Box size must be positive")
            checked.append(obstacle)
        # World geometry is attached to the masked floating root, before the
        # planar joints. Robot meshes and primitive obstacles share native pairs.
        import coal

        geometry = self.robot.collision_model
        geometry.removeAllCollisionPairs()
        for name in getattr(self, "_obstacle_names", []):
            geometry.removeGeometryObject(name)
        self._collision_pairs = [
            pair
            for pair in self._collision_pairs
            if max(pair) < self._robot_geometry_count
        ]
        self._obstacle_names = []
        for index, obstacle in enumerate(checked):
            kind = obstacle["type"]
            rotation = np.eye(3)
            if kind == "sphere":
                shape, center = (
                    coal.Sphere(obstacle["radius"]),
                    _xyz(obstacle["center"]),
                )
            elif kind == "box":
                shape, center = (
                    coal.Box(*_xyz(obstacle["size"])),
                    _xyz(obstacle["center"]),
                )
            else:
                a, b = _xyz(obstacle["a"]), _xyz(obstacle["b"])
                length = np.linalg.norm(b - a)
                center = (a + b) * 0.5
                shape = coal.Capsule(obstacle["radius"], length)
                if length > 1e-12:
                    rotation = pin.Quaternion.FromTwoVectors(
                        np.array([0.0, 0.0, 1.0]), (b - a) / length
                    ).matrix()
            name = f"placo_obstacle_{index}"
            obj = pin.GeometryObject(name, 0, pin.SE3(rotation, center), shape)
            obstacle_index = geometry.addGeometryObject(obj)
            for robot_index in range(self._robot_geometry_count):
                self._collision_pairs.append((robot_index, obstacle_index))
            self._obstacle_names.append(name)
        self.obstacles = checked
        self._cache_geometry()

    def _set_state(self, positions, base_pose):
        base = _xyz(base_pose)
        values = dict(positions)
        values.update(zip(BASE_JOINTS, base, strict=True))
        # The parallel gripper URDF contains passive mimic joints. Measured driver
        # values fill omitted passive joints. Actual measured passive positions
        # take precedence because contact and actuator dynamics can violate mimic FK.
        for _ in range(len(self._mimics)):
            for name, driver, multiplier, offset in self._mimics:
                if name not in positions and driver in values:
                    values[name] = multiplier * values[driver] + offset
        for name in self.robot.joint_names():
            if name.startswith("gripper_") and name not in values:
                raise ValueError(
                    f"Full collision geometry requires measured gripper position: {name}"
                )
            value = float(values.get(name, 0.0))
            if not np.isfinite(value):
                raise ValueError(f"Joint position must be finite: {name}")
            self.robot.set_joint(name, value)
        self.robot.update_kinematics()

    def pose(self, link, positions, base_pose=(0.0, 0.0, 0.0)):
        if link not in self._links:
            raise KeyError(link)
        self._set_state(positions, base_pose)
        return self.robot.get_T_world_frame(link).copy()

    def _cache_geometry(self):
        geometry = self.robot.collision_model
        self._geometry_data = pin.GeometryData(geometry)
        self._pin_data = self.robot.model.createData()
        centers, extents = [], []
        for obj in geometry.geometryObjects:
            obj.geometry.computeLocalAABB()
            box = obj.geometry.aabb_local
            centers.append((box.min_ + box.max_) * 0.5)
            extents.append((box.max_ - box.min_) * 0.5)
        self._local_centers = np.asarray(centers)
        self._local_extents = np.asarray(extents)

    def _distances(self):
        """Use world AABBs to exclude pairs provably beyond the trigger.

        All configured pairs remain eligible on every call. The QP receives the
        nearby exact mesh pairs. Candidate checks repeat the broad phase, so a
        new nearby pair cannot be missed when checking a discrete command.
        """
        geometry = self.robot.collision_model
        pin.updateGeometryPlacements(
            self.robot.model,
            self._pin_data,
            geometry,
            self._geometry_data,
            self.robot.state.q,
        )
        placements = self._geometry_data.oMg
        centers = np.array(
            [
                pose.rotation @ center + pose.translation
                for pose, center in zip(placements, self._local_centers)
            ]
        )
        extents = np.array(
            [
                np.abs(pose.rotation) @ extent
                for pose, extent in zip(placements, self._local_extents)
            ]
        )
        pairs = np.asarray(self._collision_pairs, dtype=int).reshape(-1, 2)
        gaps = np.maximum(
            np.abs(centers[pairs[:, 0]] - centers[pairs[:, 1]])
            - extents[pairs[:, 0]]
            - extents[pairs[:, 1]],
            0.0,
        )
        lower_bounds = np.linalg.norm(gaps, axis=1)
        near = lower_bounds < max(self.trigger, self.margin + 0.001)
        geometry.removeAllCollisionPairs()
        for first, second in pairs[near]:
            geometry.addCollisionPair(pin.CollisionPair(int(first), int(second)))
        # Placo creates GeometryData from the current pair list internally.
        distances = self.robot.distances()
        result = {
            tuple(pair): float(distance.min_distance)
            for pair, distance in zip(pairs[near], distances)
        }
        far_bound = float(lower_bounds[~near].min()) if np.any(~near) else np.inf
        minimum = min([far_bound, *result.values()])
        return result, minimum

    def as_ros2_controller(self, base_pose=(0.0, 0.0, 0.0), passive_positions=None):
        """Adapt to control_kit with fixed, explicitly supplied gripper positions.

        Use allow_base=False. The position interface reports active joints only;
        passive_positions must describe the fixed grippers used by collision FK.
        Use the simulator adapter for grippers that change during execution.
        """
        if self.allow_base:
            raise ValueError("Use allow_base=False for a position-interface controller")
        import control_kit

        passive = dict(passive_positions or {})
        return control_kit.make_position_task_controller(
            self.joint_names,
            lambda state, dt: (
                self.step(
                    {**passive, **state}, base_pose, min(0.2, max(0.001, dt))
                ).joints
            ),
        )

    def step(self, positions, base_pose=(0.0, 0.0, 0.0), dt=0.05):
        if not np.isfinite(dt) or not 0 < dt <= 0.2:
            raise ValueError("dt must be in (0, 0.2] seconds")
        missing = [
            name
            for name in self.joint_names
            if name not in BASE_JOINTS and name not in positions
        ]
        if missing:
            raise ValueError(f"Missing measured joint positions: {missing}")
        self._set_state(positions, base_pose)
        initial = self.robot.state.q.copy()
        clearances, minimum_clearance = self._distances()
        alpha = min(1.0, self.gain * dt)
        error_squared = 0.0
        for link, (position, rotation, weight) in self.goals.items():
            current = self.robot.get_T_world_frame(link)
            tasks = iter(self._tasks[link])
            if position is not None:
                error = position - current[:3, 3]
                next(tasks).target_world = current[:3, 3] + alpha * error
                error_squared += weight * float(error @ error)
            if rotation is not None:
                error = pin.log3(rotation @ current[:3, :3].T)
                next(tasks).R_world_frame = pin.exp3(alpha * error) @ current[:3, :3]
                error_squared += weight * float(error @ error)
        if self.gaze is not None:
            current = self.robot.get_T_world_frame(GAZE_FRAME)
            direction = self.gaze - current[:3, 3]
            length = np.linalg.norm(direction)
            if length < 1e-10:
                raise ValueError("Gaze target coincides with the camera origin")
            self._gaze_task.targetAxis_world = direction / length
        self.solver.dt = dt
        self.collision_constraint.self_collisions_margin = self.margin
        self.collision_constraint.self_collisions_trigger = max(
            self.trigger, self.margin + 0.001
        )
        failure = None
        try:
            self.solver.solve(True)
            candidate = self.robot.state.q.copy()
        except RuntimeError as exc:
            # An infeasible hard collision constraint has no safe servo command.
            candidate = initial.copy()
            failure = str(exc)
        accepted = False
        for _ in range(10):
            if not np.isfinite(candidate).all():
                break
            self.robot.state.q = candidate
            self.robot.update_kinematics()
            distances, _ = self._distances()
            if all(
                distance >= min(clearances.get(pair, self.margin), self.margin) - 1e-8
                for pair, distance in distances.items()
            ):
                accepted = failure is None
                break
            candidate = initial + (candidate - initial) * 0.5
        if not accepted:
            candidate = initial
            self.robot.state.q = initial
            self.robot.update_kinematics()
        velocity = pin.difference(self.robot.model, initial, candidate) / dt
        joints = {
            name: float(self.robot.get_joint(name))
            for name in self.joint_names
            if name not in BASE_JOINTS
        }
        base_velocity = tuple(
            float(velocity[self.robot.get_joint_v_offset(name)]) for name in BASE_JOINTS
        )
        return Command(
            joints,
            base_velocity,
            {
                "backend": "placo",
                "task_error": float(np.sqrt(error_squared)),
                "minimum_clearance": minimum_clearance
                if np.isfinite(minimum_clearance)
                else None,
                "step_accepted": accepted,
                "scaled_speed": float(np.max(np.abs(velocity))),
                "collision_pairs": len(self._collision_pairs),
                "near_collision_pairs": len(clearances),
                "solver_failure": failure,
            },
        )


WholeBodyController = PlacoWholeBodyController

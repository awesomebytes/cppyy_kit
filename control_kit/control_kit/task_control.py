"""URDF forward kinematics and weighted differential task control.

Coordinates use metres and radians. Targets and obstacles use the odometry frame.
This controller uses local gradients. It does not plan paths or prove collision safety.
"""

import hashlib
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import cppyy_kit
from cppyy_kit import cppyy

_NATIVE = None


def _native():
    global _NATIVE
    if _NATIVE is None:
        header = Path(__file__).with_name("task_kernel.hpp")
        prefix = Path(
            os.environ.get("CONDA_PREFIX", Path(os.sys.executable).parent.parent)
        )
        digest = hashlib.sha256(header.read_bytes()).hexdigest()
        cppyy_kit.cppdef_cached(
            f'// {digest}\n#include "{header}"',
            decls=(
                "namespace control_kit_tasks { "
                "void fk(int,const int*,const double*,const double*,"
                "const double*,double*); "
                "void solve(int,int,const double*,const double*,"
                "const double*,double,double*); }"
            ),
            include_paths=[str(prefix / "include/eigen3")],
            name="control_kit_tasks",
        )
        _NATIVE = cppyy.gbl.control_kit_tasks
    return _NATIVE


def _numbers(element, key, default):
    return (
        [float(x) for x in element.get(key, default).split()]
        if element is not None
        else [float(x) for x in default.split()]
    )


class TaskModel:
    """Read a URDF tree and evaluate every link with a cached native FK kernel."""

    def __init__(self, urdf, active_joints):
        robot = ET.fromstring(urdf)
        joints = robot.findall("joint")
        children = {j.find("child").get("link") for j in joints}
        roots = [
            link.get("name")
            for link in robot.findall("link")
            if link.get("name") not in children
        ]
        if len(roots) != 1:
            raise ValueError("URDF must contain one root link")
        self.links = {roots[0]: 0}
        self.names = ["base_x", "base_y", "base_yaw"]
        rows, origins, axes = [[-1, -1, 0]], [[0.0] * 6], [[0.0, 0.0, 1.0]]
        self.limits = {}
        self.mimics = []
        pending = list(joints)
        while pending:
            old = len(pending)
            for joint in pending[:]:
                parent = joint.find("parent").get("link")
                if parent not in self.links:
                    continue
                name, kind = joint.get("name"), joint.get("type")
                coordinate = -1
                if kind != "fixed":
                    if kind not in ("revolute", "continuous", "prismatic"):
                        raise ValueError(f"Unsupported joint type: {kind}")
                    coordinate = len(self.names)
                    self.names.append(name)
                    limit = joint.find("limit")
                    self.limits[name] = (
                        (-np.inf, np.inf, 1.0)
                        if limit is None
                        else (
                            float(limit.get("lower", "-inf")),
                            float(limit.get("upper", "inf")),
                            float(limit.get("velocity", "1")),
                        )
                    )
                    mimic = joint.find("mimic")
                    if mimic is not None:
                        self.mimics.append(
                            (
                                coordinate,
                                mimic.get("joint"),
                                float(mimic.get("multiplier", "1")),
                                float(mimic.get("offset", "0")),
                            )
                        )
                self.links[joint.find("child").get("link")] = len(rows)
                rows.append(
                    [
                        self.links[parent],
                        coordinate,
                        {"revolute": 1, "continuous": 1, "prismatic": 2}.get(kind, 0),
                    ]
                )
                origin = joint.find("origin")
                origins.append(
                    _numbers(origin, "xyz", "0 0 0") + _numbers(origin, "rpy", "0 0 0")
                )
                axes.append(_numbers(joint.find("axis"), "xyz", "1 0 0"))
                pending.remove(joint)
            if old == len(pending):
                raise ValueError("Disconnected URDF joint tree")
        self.tree = np.array(rows, dtype=np.int32)
        self.origins, self.axes = np.array(origins), np.array(axes)
        self.indices = {name: i for i, name in enumerate(self.names)}
        self.active_names = list(active_joints)
        self.active = [self.indices[n] for n in self.active_names]
        self.native = _native()

    def configuration(self, positions, base_pose=(0.0, 0.0, 0.0)):
        q = np.zeros(len(self.names))
        q[:3] = base_pose
        for name, value in positions.items():
            if name in self.indices:
                q[self.indices[name]] = value
        for index, leader, multiplier, offset in self.mimics:
            q[index] = q[self.indices[leader]] * multiplier + offset
        if not np.isfinite(q).all():
            raise ValueError("Nonfinite robot state")
        return q

    def fk(self, q):
        q = np.ascontiguousarray(q, dtype=np.float64)
        if q.shape != (len(self.names),) or not np.isfinite(q).all():
            raise ValueError("FK requires one finite value per coordinate")
        result = np.empty((len(self.tree), 4, 4))
        self.native.fk(len(self.tree), self.tree, self.origins, self.axes, q, result)
        return result

    def solve(self, jacobian, error, cost, damping=0.03):
        jacobian = np.ascontiguousarray(jacobian, dtype=np.float64)
        error = np.ascontiguousarray(error, dtype=np.float64)
        cost = np.ascontiguousarray(cost, dtype=np.float64)
        n = len(self.active)
        if error.ndim != 1 or jacobian.shape != (len(error), n) or cost.shape != (n,):
            raise ValueError(
                "Task Jacobian, residual and joint costs have inconsistent shapes"
            )
        if (
            not all(np.isfinite(a).all() for a in (jacobian, error, cost))
            or np.any(cost <= 0)
            or not np.isfinite(damping)
            or damping <= 0
        ):
            raise ValueError(
                "Task solve requires finite inputs and positive costs and damping"
            )
        result = np.empty(len(self.active))
        self.native.solve(
            len(error),
            len(result),
            np.ascontiguousarray(jacobian),
            np.ascontiguousarray(error),
            np.ascontiguousarray(cost),
            damping,
            result,
        )
        return result

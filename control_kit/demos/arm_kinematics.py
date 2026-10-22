"""Serial-chain kinematics compiled once through cppyy_kit.

Positions use metres. Rotations and revolute joint coordinates use radians.
The Jacobian and poses are expressed in the selected URDF base link frame.
"""

import hashlib
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import cppyy_kit
from cppyy_kit import cppyy

_NATIVE = None


def _native():
    global _NATIVE
    if _NATIVE is None:
        header = Path(__file__).with_name("arm_kinematics.hpp")
        prefixes = [
            Path(os.environ.get("CONDA_PREFIX", Path(os.sys.executable).parent.parent)),
                    Path("/usr")]
        includes = [str(p / "include/eigen3") for p in prefixes
                    if (p / "include/eigen3/Eigen/Core").exists()]
        if not includes:
            raise RuntimeError("Eigen headers are required to compile the admittance kernel")
        build = {
            "code": (
                f'// {hashlib.sha256(header.read_bytes()).hexdigest()}\n'
                f'#include "{header}"'
            ),
            "decls": ("namespace demo_arm_kinematics { "
                   "void kinematics(int,int,const int*,const int*,const double*,"
                   "const double*,const double*,double*,double*); "
                   "void step(int,int,const int*,const int*,const double*,"
                   "const double*,const double*,const double*,const double*,"
                   "const double*,double,double,double,double,double*); }"),
            "include_paths": includes, "name": "demo_arm_kinematics",
        }
        cppyy_kit.prebuild(**build)
        cppyy_kit.cppdef_cached(**build)
        _NATIVE = cppyy.gbl.demo_arm_kinematics
    return _NATIVE


def _vector(element, attribute, default):
    values = [float(x) for x in (element.get(attribute, default)
                               if element is not None else default).split()]
    if len(values) != 3 or not np.isfinite(values).all():
        raise ValueError("URDF vectors require three finite values")
    return values


class Chain:
    """Parse one URDF chain and warm its cached native FK and IK entry points."""

    def __init__(self, urdf_path, joint_names, base_link, tip_link):
        robot = ET.parse(urdf_path).getroot()
        parents = {j.find("child").get("link"): j for j in robot.findall("joint")}
        chain, link, visited = [], tip_link, set()
        while link != base_link:
            if link in visited or link not in parents:
                raise ValueError(f"No URDF chain from {base_link} to {tip_link}")
            visited.add(link)
            joint = parents[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        chain.reverse()
        active = {name: i for i, name in enumerate(joint_names)}
        if not active or len(active) != len(joint_names):
            raise ValueError("Joint names must be nonempty and unique")
        types, indices, origins, axes, found = [], [], [], [], set()
        for joint in chain:
            name, kind = joint.get("name"), joint.get("type")
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ValueError(f"Unsupported chain joint type: {kind}")
            moving = kind != "fixed"
            if moving and (name not in active or joint.find("mimic") is not None):
                raise ValueError(f"Chain joint must be independently controlled: {name}")
            if moving:
                found.add(name)
            types.append({"fixed": 0, "revolute": 1, "continuous": 1, "prismatic": 2}[kind])
            indices.append(active[name] if moving else -1)
            origin = joint.find("origin")
            origins.append(_vector(origin, "xyz", "0 0 0") + _vector(origin, "rpy", "0 0 0"))
            axis = np.array(_vector(joint.find("axis"), "xyz", "1 0 0"))
            norm = np.linalg.norm(axis)
            if moving and norm == 0:
                raise ValueError(f"Zero joint axis: {name}")
            axes.append(axis / norm if norm else axis)
        if found != set(active):
            raise ValueError("Every controlled joint must belong to the selected chain")
        self.count, self.joints = len(chain), len(active)
        self.types = np.array(types, dtype=np.int32)
        self.indices = np.array(indices, dtype=np.int32)
        self.origins = np.array(origins, dtype=np.float64)
        self.axes = np.array(axes, dtype=np.float64)
        self.native = _native()
        self.pose = np.empty((4, 4))
        self.jacobian = np.empty((6, self.joints))
        self.command = np.empty(self.joints)
        q = np.zeros(self.joints)
        pose = self.fk(q).copy()
        self.step(q, pose, np.full(self.joints, -1e6), np.full(self.joints, 1e6),
                  0.001, 4.0, 0.04, 0.25)

    def fk(self, q):
        self.native.kinematics(self.count, self.joints, self.types, self.indices,
                               self.origins, self.axes, q, self.pose, self.jacobian)
        return self.pose

    def step(self, q, target, lower, upper, dt, gain, damping, speed):
        self.native.step(self.count, self.joints, self.types, self.indices,
                         self.origins, self.axes, q, target, lower, upper,
                         dt, gain, damping, speed, self.command)
        return self.command

"""
Panda kinematics in pure NumPy -- the honest "no C++ solver" baseline row.

Parses the SAME ``panda.urdf`` the C++ solvers use (located via the ament index)
into the ``panda_link0 -> panda_link8`` chain, and implements forward kinematics,
the geometric Jacobian, and a damped-least-squares (DLS) / Jacobian-transpose IK
loop in NumPy. No MoveIt, no KDL, no cppyy -- this is the row that shows what plain
Python costs so the C++-via-cppyy rows have something to beat.

The FK here is validated against MoveIt's own FK to ~1e-9 in the correctness test
(``tests/test_ik_bench.py``), so the benchmark's target poses (generated with MoveIt
FK) are measured consistently for every solver.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np

GROUP_JOINTS = ["panda_joint%d" % i for i in range(1, 8)]  # 7 revolute
TIP = "panda_link8"
BASE = "panda_link0"


def _rpy_to_matrix(roll, pitch, yaw):
    """URDF fixed-axis rpy (X then Y then Z) -> 3x3 rotation."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _homog(rot, trans):
    t = np.eye(4)
    t[:3, :3] = rot
    t[:3, 3] = trans
    return t


def panda_urdf_path():
    """The panda.urdf path via the ament index (same file the C++ solvers parse)."""
    from ament_index_python.packages import get_package_share_directory
    desc = get_package_share_directory("moveit_resources_panda_description")
    return os.path.join(desc, "urdf", "panda.urdf")


class PandaChain:
    """The panda_arm chain parsed from URDF: per-joint fixed origin transform, axis
    and limits, plus the fixed tip transform (panda_joint8: link7 -> link8)."""

    def __init__(self, urdf_path=None):
        urdf_path = urdf_path or panda_urdf_path()
        root = ET.parse(urdf_path).getroot()
        joints = {j.get("name"): j for j in root.findall("joint")}
        self.origins = []      # 4x4 fixed transform of each movable joint
        self.axes = []         # local rotation axis (unit) of each movable joint
        self.lower = []
        self.upper = []
        for name in GROUP_JOINTS:
            j = joints[name]
            self.origins.append(self._origin(j))
            ax = j.find("axis")
            v = np.array([float(x) for x in ax.get("xyz").split()])
            self.axes.append(v / np.linalg.norm(v))
            lim = j.find("limit")
            self.lower.append(float(lim.get("lower")))
            self.upper.append(float(lim.get("upper")))
        # fixed panda_joint8 (link7 -> link8) is the tip transform
        self.tip_fixed = self._origin(joints["panda_joint8"])
        self.lower = np.array(self.lower)
        self.upper = np.array(self.upper)
        self.n = len(GROUP_JOINTS)

    @staticmethod
    def _origin(joint):
        o = joint.find("origin")
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if o is not None:
            if o.get("xyz"):
                xyz = [float(x) for x in o.get("xyz").split()]
            if o.get("rpy"):
                rpy = [float(x) for x in o.get("rpy").split()]
        return _homog(_rpy_to_matrix(*rpy), np.array(xyz))

    def fk_frames(self, q):
        """Return (T_tip, joint_world_axes, joint_world_origins) for a config q.
        Axes/origins are the world-frame revolute axis and origin of each joint --
        the ingredients of the geometric Jacobian."""
        t = np.eye(4)
        axes_w = []
        origins_w = []
        for i in range(self.n):
            t = t @ self.origins[i]
            # world axis + origin BEFORE applying this joint's rotation
            axes_w.append(t[:3, :3] @ self.axes[i])
            origins_w.append(t[:3, 3].copy())
            # apply the joint rotation about its local axis
            t = t @ _homog(_axis_angle(self.axes[i], q[i]), np.zeros(3))
        t = t @ self.tip_fixed
        return t, axes_w, origins_w

    def fk(self, q):
        """Forward kinematics: 4x4 world transform of panda_link8 for config q."""
        return self.fk_frames(q)[0]

    def jacobian(self, q):
        """6x7 geometric Jacobian of panda_link8 (linear rows, then angular rows)."""
        t_tip, axes_w, origins_w = self.fk_frames(q)
        p_ee = t_tip[:3, 3]
        jac = np.zeros((6, self.n))
        for i in range(self.n):
            z = axes_w[i]
            jac[:3, i] = np.cross(z, p_ee - origins_w[i])
            jac[3:, i] = z
        return jac

    def clamp(self, q):
        return np.minimum(np.maximum(q, self.lower), self.upper)

    def random_config(self, rng, margin=0.0):
        """A uniform-random config within limits (margin in [0,0.5) keeps away from
        the bounds; a NEGATIVE margin biases toward the limits for near-limit poses)."""
        lo = self.lower + margin * (self.upper - self.lower)
        hi = self.upper - margin * (self.upper - self.lower)
        return lo + rng.random(self.n) * (hi - lo)


def _axis_angle(axis, angle):
    """Rodrigues rotation matrix for a rotation of `angle` about unit `axis`."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    c1 = 1.0 - c
    return np.array([
        [c + x * x * c1, x * y * c1 - z * s, x * z * c1 + y * s],
        [y * x * c1 + z * s, c + y * y * c1, y * z * c1 - x * s],
        [z * x * c1 - y * s, z * y * c1 + x * s, c + z * z * c1],
    ])


# ---- pose helpers (matrix <-> (x,y,z,qx,qy,qz,qw)) --------------------------
def matrix_to_pose(t):
    trans = t[:3, 3]
    q = _matrix_to_quat(t[:3, :3])
    return (float(trans[0]), float(trans[1]), float(trans[2]),
            float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _matrix_to_quat(r):
    """3x3 rotation -> (qx, qy, qz, qw)."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw])
    return q / np.linalg.norm(q)


def pose_error(t_current, target_pose):
    """(position error [m], orientation error [rad]) between a 4x4 transform and a
    target (x,y,z,qx,qy,qz,qw)."""
    tx, ty, tz, qx, qy, qz, qw = target_pose
    pos_err = float(np.linalg.norm(t_current[:3, 3] - np.array([tx, ty, tz])))
    cur = _matrix_to_quat(t_current[:3, :3])
    dot = abs(float(np.dot(cur, np.array([qx, qy, qz, qw]))))
    dot = min(1.0, dot)
    ori_err = 2.0 * math.acos(dot)
    return pos_err, ori_err


def _pose_twist_error(t_current, target_pose):
    """6-vector [dpos; drot] error used by the DLS loop (rotation via the log map
    of the relative rotation)."""
    tx, ty, tz, qx, qy, qz, qw = target_pose
    dpos = np.array([tx, ty, tz]) - t_current[:3, 3]
    r_target = _quat_to_matrix(qx, qy, qz, qw)
    r_err = r_target @ t_current[:3, :3].T
    drot = _rotation_log(r_err)
    return np.concatenate([dpos, drot])


def _quat_to_matrix(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _rotation_log(r):
    """so(3) log map: rotation matrix -> rotation vector (axis * angle)."""
    cos_a = (np.trace(r) - 1.0) / 2.0
    cos_a = max(-1.0, min(1.0, cos_a))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(angle - math.pi) < 1e-6:
        # near pi: recover axis from the symmetric part
        w = np.array([
            math.sqrt(max(0.0, (r[0, 0] + 1) / 2)),
            math.sqrt(max(0.0, (r[1, 1] + 1) / 2)),
            math.sqrt(max(0.0, (r[2, 2] + 1) / 2)),
        ])
        return angle * w / (np.linalg.norm(w) + 1e-12)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return angle * axis / (2.0 * math.sin(angle))


def _dls_single(chain, target_pose, seed, pos_tol, ori_tol, max_iters, damping,
                deadline):
    """One DLS descent from `seed`: ``dq = J^T (J J^T + lambda^2 I)^-1 e`` with a
    capped step and joint clamping. Returns (success, q)."""
    import time
    q = chain.clamp(np.array(seed, dtype=float))
    lam2 = damping * damping
    best_q = q
    for _ in range(max_iters):
        t_cur = chain.fk(q)
        pos_err, ori_err = pose_error(t_cur, target_pose)
        if pos_err < pos_tol and ori_err < ori_tol:
            return True, q
        if deadline is not None and time.perf_counter() > deadline:
            break
        err = _pose_twist_error(t_cur, target_pose)
        jac = chain.jacobian(q)
        jjt = jac @ jac.T + lam2 * np.eye(6)
        dq = jac.T @ np.linalg.solve(jjt, err)
        norm = np.linalg.norm(dq)
        if norm > 0.5:
            dq = dq * (0.5 / norm)
        q = chain.clamp(q + dq)
        best_q = q
    return False, best_q


def dls_ik(chain, target_pose, seed, pos_tol=1e-3, ori_tol=1e-2,
           max_iters=200, damping=0.05, time_budget=None, rng=None):
    """Damped-least-squares Jacobian IK with random restarts. Returns (success, q, tries).

    Classic textbook local IK -- fast per step but prone to local minima and
    singularities. To be fair to the C++ solvers (KDL/TRAC-IK do random restarts
    within their timeout too), this descends from ``seed`` first and, on failure,
    restarts from fresh random configs until the position+orientation error is under
    tolerance or ``time_budget`` (seconds) elapses -- early-returning on success just
    like the plugins. The honest weakness the benchmark still exposes: even with
    restarts, doing this in Python is far slower per solve than the JIT'd C++ path."""
    import time
    start = time.perf_counter()
    deadline = (start + time_budget) if time_budget is not None else None
    if rng is None:
        rng = np.random.default_rng(0)
    tries = 0
    q = np.array(seed, dtype=float)
    while True:
        tries += 1
        ok, q = _dls_single(chain, target_pose, q, pos_tol, ori_tol, max_iters,
                            damping, deadline)
        if ok:
            return True, q, tries
        if deadline is None or time.perf_counter() > deadline:
            return False, q, tries
        q = chain.random_config(rng, margin=0.02)  # restart from a fresh seed

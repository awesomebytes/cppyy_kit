#!/usr/bin/env python
"""
retarget.py (Process B) -- the retargeting half of the capture rig.

    landmark stream -> whole-body targets -> a humanoid configuration per frame
    (pinocchio CLIK) -> Rerun (robot + human) + a "policy-kickstart" dataset.

Runs in the standalone ``wbc`` pixi env (pinocchio's conda stack pins libboost 1.86
vs the ROS stack's 1.90 -- they cannot share a process; docs/wbc/REPORT.md). It
consumes the landmark stream that ``perceive.py`` writes -- ``--replay FILE`` reads
a recorded stream (CI/headless), ``--follow FILE`` tails one a live perceive is
still writing.

**Where cppyy_kit wins here, honestly.** The natural "lower the CLIK to inline C++
calling pinocchio" move is BLOCKED in this env: instantiating ``pinocchio::Model``
from headers under Cling trips boost 1.90's variant template-arity wall (pinocchio's
25-type ``JointModel`` boost::variant -- the same wall docs/wbc/REPORT.md hit for
templated scalars, confirmed here for the default-double Model + URDF parser). So the
IK solve is a **pinocchio-bindings** job (the precompiled library carries the variant
-- the bindings are the right tool, matching the REPORT's "bindings are fine" cases).
The measured cppyy_kit win in this pipeline is the perception side's TF-message
marshaling (perceive.py --bench). Process B's own honest cppyy_kit contribution is
the per-frame retarget **glue kernel** -- coordinate transform + target mapping +
a sequential One-Euro landmark filter -- authored in one ``cppyy.cppdef`` C++ pass
over the whole stream (COMMON_PATTERNS s6/s26: the sequential per-frame filter is
exactly the Python-loop trap), measured against the identical Python loop
(``--bench``).
"""
import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from retarget_pipeline import landmark_stream as ls  # noqa: E402
# NOTE: retarget_pipeline.viz imports rerun at module load; it is imported lazily
# (only when visualizing) so --bench / --no-viz / the tests run without rerun.

DEFAULT_STREAM = os.path.join(REPO, "build", "pipeline", "demo.jsonl")


# --------------------------------------------------------------------------- #
# Robot configs: which URDF, which frames to retarget, per humanoid.
# --------------------------------------------------------------------------- #
class RobotConfig:
    def __init__(self, name, urdf_rel, l_grip, r_grip, head, l_sh, r_sh, hip):
        self.name = name
        self.urdf_rel = urdf_rel
        self.l_grip, self.r_grip, self.head = l_grip, r_grip, head
        self.l_sh, self.r_sh = l_sh, r_sh
        self.hip = hip                 # the robot hip/pelvis frame targets anchor to


ROBOTS = {
    "talos": RobotConfig(
        "talos", ("talos_data", "robots", "talos_reduced.urdf"),
        "gripper_left_base_link", "gripper_right_base_link", "head_2_link",
        "arm_left_1_link", "arm_right_1_link", "base_link"),
    "g1": RobotConfig(
        "g1", ("g1_description", "urdf", "g1_29dof_rev_1_0.urdf"),
        "left_wrist_yaw_link", "right_wrist_yaw_link", "head_link",
        "left_shoulder_pitch_link", "right_shoulder_pitch_link", "pelvis"),
}


def urdf_path(cfg):
    conda = os.environ["CONDA_PREFIX"]
    return os.path.join(conda, "share", "example-robot-data", "robots", *cfg.urdf_rel)


# --------------------------------------------------------------------------- #
# The cppyy_kit glue kernel (plain C++, no pinocchio -> not blocked by the variant
# wall): batch-retarget a landmark stream to smoothed EE targets in one C++ pass.
# The One-Euro filter is sequential across frames -- the per-element Python-loop
# trap (COMMON_PATTERNS s6/s26) -- so doing it in C++ is a genuine win.
# --------------------------------------------------------------------------- #
_GLUE = r"""
#include <cmath>
namespace retarget_glue {

static inline double euro_alpha(double cutoff, double dt) {
  double tau = 1.0 / (2.0 * M_PI * cutoff);
  return 1.0 / (1.0 + tau / dt);
}

// in_pw: (F, 99) MediaPipe pose_world (33 landmarks * xyz), row-major.
// out:   (F, 9)  robot-frame smoothed targets [L(3), R(3), Head(3)].
// HIP-RELATIVE mapping (the owner's chain): human point RELATIVE TO the human hip
// midpoint -> scaled by (robot torso / human torso, per frame) -> anchored at the
// robot hip frame (hip). Gripper targets are clamped to reach_frac*arm of the robot
// shoulder (ls/rs) for reachability, then One-Euro-smoothed across F frames.
void map_stream(uintptr_t in_pw, int F, uintptr_t out,
                double hipx, double hipy, double hipz,
                double lsx, double lsy, double lsz,
                double rsx, double rsy, double rsz,
                double robot_torso, double arm, double dt, double mincut, double beta,
                double reach_frac) {
  const double* pw = reinterpret_cast<const double*>(in_pw);
  double* o = reinterpret_cast<double*>(out);
  const int LSH = 11, RSH = 12, LWR = 15, RWR = 16, NOSE = 0, LHIP = 23, RHIP = 24;
  double prev[9], dprev[9];
  bool have_prev = false;
  double hip[3] = {hipx, hipy, hipz};
  double shA[6] = {lsx, lsy, lsz, rsx, rsy, rsz};   // robot shoulder clamp centers
  for (int f = 0; f < F; ++f) {
    const double* row = pw + (long)f * 99;
    // MediaPipe world (x left, y down, z toward cam) -> robot (x fwd, y left, z up).
    auto rob = [&](int idx, double* v) {
      const double* p = row + idx * 3;
      v[0] = -p[2]; v[1] = p[0]; v[2] = -p[1];
    };
    double lsh[3], rsh[3], lwr[3], rwr[3], nose[3], lhip[3], rhip[3], hipc[3], shc[3];
    rob(LSH, lsh); rob(RSH, rsh); rob(LWR, lwr); rob(RWR, rwr); rob(NOSE, nose);
    rob(LHIP, lhip); rob(RHIP, rhip);
    for (int k = 0; k < 3; ++k) { hipc[k] = 0.5 * (lhip[k] + rhip[k]); shc[k] = 0.5 * (lsh[k] + rsh[k]); }
    double th = 0;
    for (int k = 0; k < 3; ++k) th += (shc[k] - hipc[k]) * (shc[k] - hipc[k]);
    double human_torso = std::sqrt(th) + 1e-6;
    double s = robot_torso / human_torso;
    double tgt[9];
    for (int k = 0; k < 3; ++k) {
      tgt[k]     = hip[k] + s * (lwr[k] - hipc[k]);       // left gripper (hip-relative)
      tgt[3 + k] = hip[k] + s * (rwr[k] - hipc[k]);       // right gripper
      tgt[6 + k] = hip[k] + s * (nose[k] - hipc[k]);      // head
    }
    // Clamp gripper targets into the reachable sphere around each robot shoulder.
    for (int e = 0; e < 2; ++e) {
      double d2 = 0; int b = e * 3;
      for (int k = 0; k < 3; ++k) d2 += (tgt[b + k] - shA[b + k]) * (tgt[b + k] - shA[b + k]);
      double d = std::sqrt(d2), maxr = reach_frac * arm;
      if (d > maxr && d > 1e-9) {
        double sc = maxr / d;
        for (int k = 0; k < 3; ++k) tgt[b + k] = shA[b + k] + sc * (tgt[b + k] - shA[b + k]);
      }
    }
    // One-Euro filter across frames (sequential -> keep it in C++).
    if (!have_prev) {
      for (int k = 0; k < 9; ++k) { prev[k] = tgt[k]; dprev[k] = 0.0; o[f * 9 + k] = tgt[k]; }
      have_prev = true;
    } else {
      for (int k = 0; k < 9; ++k) {
        double dx = (tgt[k] - prev[k]) / dt;
        double ad = euro_alpha(1.0, dt);
        double dhat = ad * dx + (1 - ad) * dprev[k];
        double cutoff = mincut + beta * std::fabs(dhat);
        double a = euro_alpha(cutoff, dt);
        double xhat = a * tgt[k] + (1 - a) * prev[k];
        prev[k] = xhat; dprev[k] = dhat; o[f * 9 + k] = xhat;
      }
    }
  }
}

}  // namespace retarget_glue
"""

_GLUE_READY = False


def _bringup_glue():
    global _GLUE_READY
    import cppyy
    if not _GLUE_READY:
        cppyy.cppdef(_GLUE)
        _GLUE_READY = True
    return cppyy.gbl.retarget_glue


# Retarget target defaults, shared by the batch kernel path and the live stepper.
MINCUT, BETA, REACH_FRAC = 1.2, 0.03, 0.8


def _frame_target(r, hip_w, sh, robot_torso, arm, reach_frac):
    """One frame of the HIP-RELATIVE retarget map (the owner's chain): robot-frame
    landmarks ``r`` (33,3) -> EE targets (3,3) = [left gripper, right gripper, head].
    Each human point is taken RELATIVE TO the human hip midpoint, scaled by
    ``robot_torso / human_torso`` (per frame), and anchored at the robot hip frame
    ``hip_w``; the two gripper targets are then clamped to ``reach_frac*arm`` of the
    corresponding robot shoulder ``sh`` (=[L,R]) for reachability."""
    hipc = 0.5 * (r[ls.LEFT_HIP] + r[ls.RIGHT_HIP])
    shc = 0.5 * (r[ls.LEFT_SHOULDER] + r[ls.RIGHT_SHOULDER])
    s = robot_torso / (np.linalg.norm(shc - hipc) + 1e-6)
    tgt = np.stack([hip_w + s * (r[ls.LEFT_WRIST] - hipc),
                    hip_w + s * (r[ls.RIGHT_WRIST] - hipc),
                    hip_w + s * (r[ls.NOSE] - hipc)])
    for e in range(2):
        d = np.linalg.norm(tgt[e] - sh[e])
        if d > reach_frac * arm and d > 1e-9:
            tgt[e] = sh[e] + (reach_frac * arm / d) * (tgt[e] - sh[e])
    return tgt


class _EuroState:
    """A One-Euro low-pass filter over the (3,3) target, carrying state across frames
    (per-coordinate cutoff = mincut + beta*|velocity|). The sequential dependence is
    exactly why the *batch* form lives in the C++ kernel; this drives the *live* path
    one frame at a time with the identical formula, so replay and follow agree."""

    def __init__(self, dt, mincut=MINCUT, beta=BETA):
        self.dt, self.mincut, self.beta = dt, mincut, beta
        self.prev = None
        self.dprev = np.zeros((3, 3))

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / self.dt)

    def step(self, tgt):
        """Filter one raw (3,3) target, updating state; returns the smoothed (3,3)."""
        if self.prev is None:
            self.prev = tgt.copy()
            return tgt.copy()
        out = np.empty((3, 3))
        for k in range(3):
            dx = (tgt[k] - self.prev[k]) / self.dt
            ad = self._alpha(1.0)
            dhat = ad * dx + (1 - ad) * self.dprev[k]
            cutoff = self.mincut + self.beta * np.abs(dhat)
            a = self._alpha(cutoff)
            xhat = a * tgt[k] + (1 - a) * self.prev[k]
            self.prev[k], self.dprev[k] = xhat, dhat
            out[k] = xhat
        return out


def map_stream_python(pw_all, hip_w, sh, robot_torso, arm, dt, mincut, beta, reach_frac):
    """The naive baseline for --bench: the identical hip-relative retarget glue as a
    Python per-frame loop (the sequential One-Euro filter is the per-element trap).
    Same ``_frame_target`` + ``_EuroState`` steppers the live --follow/tf paths use."""
    hip_w = np.asarray(hip_w, dtype=np.float64)
    sh = np.asarray(sh, dtype=np.float64).reshape(2, 3)
    euro = _EuroState(dt, mincut, beta)
    out = np.zeros((pw_all.shape[0], 9), dtype=np.float64)
    for f in range(pw_all.shape[0]):
        r = ls.mediapipe_world_to_robot(pw_all[f].reshape(33, 3))
        out[f] = euro.step(_frame_target(r, hip_w, sh, robot_torso, arm, reach_frac)).reshape(9)
    return out


# --------------------------------------------------------------------------- #
# The retargeter: pinocchio-bindings CLIK (position tasks + posture, fixed base).
# --------------------------------------------------------------------------- #
class Retargeter:
    def __init__(self, cfg):
        import pinocchio as pin
        self._pin = pin
        self.cfg = cfg
        self.model = pin.buildModelFromUrdf(urdf_path(cfg), pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.q0 = pin.neutral(self.model)
        self.LF = self.model.getFrameId(cfg.l_grip)
        self.RF = self.model.getFrameId(cfg.r_grip)
        self.HD = self.model.getFrameId(cfg.head)
        self.LS = self.model.getFrameId(cfg.l_sh)
        self.RS = self.model.getFrameId(cfg.r_sh)
        pin.forwardKinematics(self.model, self.data, self.q0)
        pin.updateFramePlacements(self.model, self.data)
        self.anchor_L = self.data.oMf[self.LS].translation.copy()
        self.anchor_R = self.data.oMf[self.RS].translation.copy()
        self.anchor_H = self.data.oMf[self.HD].translation.copy()
        self.arm = float(np.linalg.norm(self.data.oMf[self.LF].translation - self.anchor_L))
        # Robot hip frame (targets are anchored here, hip-relative) + torso length
        # (shoulder-center to hip), the fixed scale reference vs the human torso.
        self.hip = self.data.oMf[self.model.getFrameId(cfg.hip)].translation.copy()
        self.robot_torso = float(np.linalg.norm(
            0.5 * (self.anchor_L + self.anchor_R) - self.hip))
        self._build_posture_weights()
        self._load_visual_geometry()

    # Per-actuated-DOF posture weight. The retarget only has arm (+head) position
    # tasks, so with a weak uniform posture the CLIK freely pitches the torso/waist to
    # reach -- Talos leaned ~52 deg. Fix: pin the non-arm chain (legs, torso/waist)
    # firmly to the reference pose and leave the arm + head chains free, so reaching
    # is done by the arms with an upright trunk.
    # Only the arm chain is freed to reach; the legs, torso/waist AND head/neck are
    # pinned to the reference (the head can't usefully track here anyway -- see the
    # REPORT: G1 has no neck joints and Talos's reduced-model head frame does not move
    # with its neck, so head position IK is a no-op; pinning keeps it stable).
    _ARM_TOKENS = ("arm", "shoulder", "elbow", "wrist", "hand", "gripper")
    _FREE_W, _PIN_W = 1e-3, 5.0

    def _build_posture_weights(self):
        w = np.full(self.model.nv - 6, self._PIN_W)
        for j in range(2, self.model.njoints):       # skip universe(0) + free-flyer(1)
            name = self.model.names[j].lower()
            free = any(tok in name for tok in self._ARM_TOKENS)
            v0 = self.model.joints[j].idx_v - 6
            nvj = self.model.joints[j].nv
            if 0 <= v0 and free:
                w[v0:v0 + nvj] = self._FREE_W
        self.posture_w = w

    def _load_visual_geometry(self):
        """Load the URDF's VISUAL geometry (the link meshes) so Rerun can show the
        real robot, not just the joint skeleton. Best-effort: if the meshes can't be
        loaded (missing files / no geometry backend), ``has_meshes`` stays False and
        the caller falls back to the skeleton."""
        pin = self._pin
        self.has_meshes = False
        self.geom_model = None
        self._mesh_geoms = []       # indices of geoms backed by a real mesh file
        try:
            import warnings
            share = os.path.join(os.environ["CONDA_PREFIX"], "share")
            with warnings.catch_warnings():          # pinocchio's package-dir notice
                warnings.simplefilter("ignore")
                gm = pin.buildGeomFromUrdf(self.model, urdf_path(self.cfg),
                                           pin.GeometryType.VISUAL, [share])
            objs = list(gm.geometryObjects)
            # Some links are inline primitives (BOX/CYLINDER, meshPath is the shape
            # name, not a file) -- skip those and render the file-backed meshes.
            self._mesh_geoms = [i for i, g in enumerate(objs)
                                if os.path.isfile(g.meshPath)]
            if self._mesh_geoms:
                self.geom_model = gm
                self.geom_data = gm.createData()
                self.has_meshes = True
                self.n_primitives = len(objs) - len(self._mesh_geoms)
        except Exception as exc:
            self._mesh_error = str(exc)

    def visual_meshes(self):
        """[(entity_name, absolute_mesh_path)] for one-time static Asset3D logging
        (file-backed visual geoms only; inline primitives are skipped)."""
        if not self.has_meshes:
            return []
        objs = self.geom_model.geometryObjects
        return [(objs[i].name, objs[i].meshPath) for i in self._mesh_geoms]

    def visual_placements(self, q):
        """[(entity_name, translation(3,), rotation(3x3))] -- world placement of each
        file-backed visual mesh at ``q`` (pinocchio FK + geometry update)."""
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateGeometryPlacements(self.model, self.data, self.geom_model,
                                     self.geom_data, q)
        objs = self.geom_model.geometryObjects
        out = []
        for i in self._mesh_geoms:
            M = self.geom_data.oMg[i]
            out.append((objs[i].name, np.asarray(M.translation, dtype=np.float32),
                        np.asarray(M.rotation, dtype=np.float32)))
        return out

    def solve(self, targets9, q_warm, iters=40, damp=1e-3, whead=0.0):
        """CLIK toward the 9-vector [L,R,Head] targets from warm start ``q_warm``.
        Returns ``(q, {frame: err_m})``. The free-flyer base is locked by solving
        over the *actuated* DOFs only (columns 6: of the Jacobian). Posture is
        regularized per-joint (``posture_w``): the legs + torso/waist are pinned to
        the reference so reaching is done by the arms with an upright trunk (a uniform
        weak posture let the trunk pitch ~52 deg). ``whead`` defaults off -- head
        position IK is a no-op on the shipped models (see the REPORT); left as a knob."""
        pin = self._pin
        q = q_warm.copy()
        na = self.model.nv - 6                       # actuated DOFs (skip base)
        Wp = self.posture_w                          # per-joint posture weight (na,)
        tL, tR, tH = targets9[:3], targets9[3:6], targets9[6:9]
        tasks = ((self.LF, tL, 1.0), (self.RF, tR, 1.0), (self.HD, tH, whead))
        for _ in range(iters):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            pin.computeJointJacobians(self.model, self.data, q)
            H = np.zeros((na, na))
            g = np.zeros(na)
            for fid, tgt, w in tasks:
                if w <= 0.0:
                    continue
                e = tgt - self.data.oMf[fid].translation
                J = pin.getFrameJacobian(self.model, self.data, fid,
                                         pin.LOCAL_WORLD_ALIGNED)[:3, 6:]
                H += w * (J.T @ J)
                g += w * (J.T @ e)
            H += np.diag(Wp) + damp * np.eye(na)
            g += Wp * pin.difference(self.model, q, self.q0)[6:]
            v = np.zeros(self.model.nv)
            v[6:] = np.linalg.solve(H, g)
            q = pin.integrate(self.model, q, v)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        errs = {self.cfg.l_grip: float(np.linalg.norm(tL - self.data.oMf[self.LF].translation)),
                self.cfg.r_grip: float(np.linalg.norm(tR - self.data.oMf[self.RF].translation))}
        return q, errs

    def joint_skeleton(self, q):
        """(points (njoints,3), bones list) of the kinematic tree at ``q`` -- for
        Rerun. Each joint connects to its parent joint placement."""
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pts = np.array([self.data.oMi[j].translation for j in range(self.model.njoints)],
                       dtype=np.float32)
        bones = [[pts[j], pts[self.model.parents[j]]] for j in range(1, self.model.njoints)]
        return pts, bones

    def frame_pos(self, q, fid):
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[fid].translation.copy()


# --------------------------------------------------------------------------- #
# Load the landmark stream into a (F,99) pose_world array (frames with a pose).
# --------------------------------------------------------------------------- #
def load_pose_world(stream_path):
    frames = [fr for fr in ls.StreamReader(stream_path).frames()
              if fr["pose_world"] is not None]
    if not frames:
        return np.zeros((0, 99)), []
    pw = np.array([fr["pose_world"].reshape(99) for fr in frames], dtype=np.float64)
    ts = [fr["t"] for fr in frames]
    return pw, ts


def compute_targets(rt, pw_all, dt, use_cpp=True, mincut=MINCUT, beta=BETA,
                    reach_frac=REACH_FRAC):
    """Retarget glue: (F,99) landmarks -> (F,9) smoothed HIP-RELATIVE targets. Uses
    the C++ kernel (cppyy_kit) by default, else the Python loop."""
    hip = np.asarray(rt.hip, dtype=np.float64)
    sh = np.stack([rt.anchor_L, rt.anchor_R]).astype(np.float64)
    if use_cpp:
        glue = _bringup_glue()
        out = np.zeros((pw_all.shape[0], 9), dtype=np.float64)
        c = np.ascontiguousarray(pw_all, dtype=np.float64)
        glue.map_stream(c.ctypes.data, int(c.shape[0]), out.ctypes.data,
                        hip[0], hip[1], hip[2], sh[0, 0], sh[0, 1], sh[0, 2],
                        sh[1, 0], sh[1, 1], sh[1, 2], rt.robot_torso, rt.arm,
                        dt, mincut, beta, reach_frac)
        return out
    return map_stream_python(pw_all, hip, sh, rt.robot_torso, rt.arm, dt, mincut,
                             beta, reach_frac)


# --------------------------------------------------------------------------- #
# Modes.
# --------------------------------------------------------------------------- #
def _no_stream_msg(path):
    return ("landmark stream not found: %s\n"
            "Record one first with the perception half, e.g.:\n"
            "  pixi run -e pipeline demo-perceive --record %s --duration 15\n"
            "then re-run this with --replay %s" % (path, path, path))


def run_retarget(args):
    stream = args.replay or DEFAULT_STREAM
    if not os.path.exists(stream):
        print("[retarget] " + _no_stream_msg(stream), flush=True)
        return 2
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    print("Robot %s: nq=%d nv=%d, arm reach %.3f m; anchors L=%s R=%s"
          % (cfg.name, rt.model.nq, rt.model.nv, rt.arm,
             np.round(rt.anchor_L, 3), np.round(rt.anchor_R, 3)), flush=True)

    pw_all, ts = load_pose_world(stream)
    if len(pw_all) == 0:
        print("No pose frames in %s." % stream)
        return
    dt = 1.0 / args.fps
    targets = compute_targets(rt, pw_all, dt, use_cpp=not args.no_cpp)
    print("Retarget glue: computed %d target frames (%s path)."
          % (len(targets), "Python" if args.no_cpp else "cppyy_kit C++ kernel"),
          flush=True)

    session = None
    rr = None
    viz_mode = "skeleton"
    if not args.no_viz:
        import rerun as rr
        from retarget_pipeline import viz
        session = viz.init_rerun("retarget", args.rrd,
                                 blueprint=viz.blueprint_retarget())
        viz_mode = _setup_robot_viz(rr, rt, args)

    F = len(targets)
    q = rt.q0.copy()
    Q = np.zeros((F, rt.model.nq))
    ee_err = np.zeros((F, 2))
    solve_ms = []
    for i in range(F):
        t0 = time.perf_counter()
        q, errs = rt.solve(targets[i], q)
        solve_ms.append((time.perf_counter() - t0) * 1e3)
        Q[i] = q
        ee_err[i] = list(errs.values())
        if rr is not None:
            _log_retarget(rr, rt, i, q, targets[i],
                          ls.mediapipe_world_to_robot(pw_all[i].reshape(33, 3)), viz_mode)
        if (i + 1) % 30 == 0:
            print("  frame %d/%d: solve %.2f ms, EE err L=%.3f R=%.3f m"
                  % (i + 1, F, np.median(solve_ms[-30:]), ee_err[i, 0], ee_err[i, 1]),
                  flush=True)

    _write_dataset(args, cfg, rt, Q, targets, ts, ee_err, source=stream)
    print("\nSUMMARY %s: %d frames | CLIK %.2f ms/frame (median) | EE err median "
          "L=%.3f R=%.3f m (mean %.3f)"
          % (cfg.name, F, float(np.median(solve_ms)), float(np.median(ee_err[:, 0])),
             float(np.median(ee_err[:, 1])), float(np.mean(ee_err))), flush=True)
    if session is not None:
        viz.announce(session)


def _log_retarget(rr, rt, i, q, targets9, human_robot, viz_mode="mesh"):
    """Log one retarget frame. ``human_robot`` is the (33,3) human skeleton already in
    the robot frame (callers convert from MediaPipe world; the TF path reads it
    pre-converted off /tf). ``viz_mode``: 'mesh' places the real URDF link meshes (the
    nice-looking robot); 'skeleton' draws the joint tree (the fallback)."""
    from retarget_pipeline import viz
    rr.set_time("frame", sequence=i)
    if viz_mode == "mesh" and rt.has_meshes:
        viz.log_robot_pose(rr, "robot/visual", rt.visual_placements(q))
    else:
        pts, bones = rt.joint_skeleton(q)
        rr.log("robot/joints", rr.Points3D(pts, radii=0.02, colors=[(120, 200, 255)]))
        rr.log("robot/bones", rr.LineStrips3D(bones, colors=[(120, 200, 255)]))
    tgt = targets9.reshape(3, 3)
    rr.log("robot/targets", rr.Points3D(tgt, radii=0.04, colors=[(255, 140, 60)]))
    if human_robot is not None:
        viz.log_skeleton_3d(rr, "human/pose", human_robot, ls.POSE_CONNECTIONS,
                            color=(160, 160, 160))


def _setup_robot_viz(rr, rt, args):
    """Resolve the robot viz mode and, for mesh mode, log the link meshes once
    (static). Returns the effective mode ('mesh' or 'skeleton')."""
    from retarget_pipeline import viz
    mode = getattr(args, "robot_viz", "mesh")
    if mode == "mesh":
        if rt.has_meshes:
            n = viz.log_robot_meshes_static(rr, "robot/visual", rt.visual_meshes())
            print("Rerun: %d %s link meshes loaded (real robot model)."
                  % (n, rt.cfg.name), flush=True)
        else:
            print("[retarget] no link meshes for %s; falling back to the joint "
                  "skeleton." % rt.cfg.name, flush=True)
            mode = "skeleton"
    return mode


def _write_dataset(args, cfg, rt, Q, targets, ts, ee_err, source):
    path = args.dataset or os.path.join(REPO, "build", "pipeline",
                                        "dataset_%s.npz" % cfg.name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, robot=cfg.name, q=Q.astype(np.float32),
             targets=targets.astype(np.float32),
             t=np.array(ts[:len(Q)], dtype=np.float64),
             ee_err=ee_err.astype(np.float32),
             joint_names=np.array(list(rt.model.names), dtype=object),
             source_stream=(os.path.abspath(source) if os.path.exists(source) else source))
    print("Policy-kickstart dataset -> %s (q %s, targets %s)."
          % (path, Q.shape, targets.shape), flush=True)


def run_follow(args):
    """Live teleop: tail a landmark stream a perceive process is still writing and
    retarget each frame as it arrives -- no offline record/replay step. Exits cleanly
    on stream idle-timeout / EOF / Ctrl-C, writing the dataset gathered so far. The
    Rerun robot updates per frame (that is the point). Per-frame targets use the same
    ``_frame_target`` + ``_EuroState`` steppers as replay, so the two paths agree."""
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    print("Robot %s: nq=%d nv=%d, arm reach %.3f m. Following %s (startup grace %.0fs, "
          "then idle-timeout %.1fs); start a perceive --record writing it ..."
          % (cfg.name, rt.model.nq, rt.model.nv, rt.arm, args.follow,
             args.startup_timeout, args.idle_timeout), flush=True)
    session = None
    rr = None
    viz_mode = "skeleton"
    if not args.no_viz:
        import rerun as rr
        from retarget_pipeline import viz
        session = viz.init_rerun("retarget", args.rrd,
                                 blueprint=viz.blueprint_retarget())
        viz_mode = _setup_robot_viz(rr, rt, args)

    dt = 1.0 / args.fps
    hip_w = np.asarray(rt.hip, dtype=np.float64)
    sh = np.stack([rt.anchor_L, rt.anchor_R]).astype(np.float64)
    euro = _EuroState(dt)
    q = rt.q0.copy()
    Q, targets_log, ts_log, ee_log, lag = [], [], [], [], []
    solve_ms = []

    # Heartbeat while waiting for the producer's first frame (a cold perceive takes
    # a few seconds to activate its env + load its model); throttled to ~5 s.
    hb = {"last": 0.0}

    def _heartbeat(elapsed):
        if elapsed - hb["last"] >= 5.0:
            hb["last"] = elapsed
            print("  waiting for the producer's first frame ... %.0fs (start a "
                  "perceive --record; startup grace %.0fs)"
                  % (elapsed, args.startup_timeout), flush=True)

    try:
        for fr in ls.follow(args.follow, idle_timeout=args.idle_timeout,
                            startup_timeout=args.startup_timeout, poll=0.005,
                            on_wait=_heartbeat):
            if fr["pose_world"] is None:
                continue
            r = ls.mediapipe_world_to_robot(fr["pose_world"])
            tgt = euro.step(_frame_target(r, hip_w, sh, rt.robot_torso, rt.arm,
                                          REACH_FRAC)).reshape(9)
            t0 = time.perf_counter()
            q, errs = rt.solve(tgt, q)
            solve_ms.append((time.perf_counter() - t0) * 1e3)
            # End-to-end lag: perceive stamps each frame with time.time() at write;
            # both processes share the wall clock, so (now - t) is the frame's age
            # when we finish solving it -- the true producer->consumer latency.
            lag.append(time.time() - fr["t"])
            Q.append(q.copy())
            targets_log.append(tgt)
            ts_log.append(fr["t"])
            ee_log.append(list(errs.values()))
            if rr is not None:
                _log_retarget(rr, rt, len(Q) - 1, q, tgt,
                              ls.mediapipe_world_to_robot(fr["pose_world"]), viz_mode)
            if len(Q) % 30 == 0:
                print("  live frame %d: lag %.1f ms (median %.1f), solve %.2f ms, "
                      "EE L=%.3f R=%.3f m"
                      % (len(Q), lag[-1] * 1e3, float(np.median(lag)) * 1e3,
                         float(np.median(solve_ms)), ee_log[-1][0], ee_log[-1][1]),
                      flush=True)
    except KeyboardInterrupt:
        print("\n[retarget] interrupted; writing what we have.", flush=True)
    if not Q:
        print("[retarget] no frames arrived on %s within the %.0fs startup grace -- "
              "is a perceive --record writing it? (raise --startup-timeout if the "
              "producer is slow to start; the coupling is the file)."
              % (args.follow, args.startup_timeout), flush=True)
        return
    Q = np.array(Q)
    ee = np.array(ee_log)
    lag = np.array(lag)
    _write_dataset(args, cfg, rt, Q, np.array(targets_log), ts_log, ee, source=args.follow)
    print("\nSUMMARY %s (LIVE follow): %d frames consumed as produced | end-to-end lag "
          "median %.1f ms (p90 %.1f, max %.1f) | CLIK %.2f ms/frame | EE err median "
          "L=%.3f R=%.3f m"
          % (cfg.name, len(Q), float(np.median(lag)) * 1e3,
             float(np.percentile(lag, 90)) * 1e3, float(np.max(lag)) * 1e3,
             float(np.median(solve_ms)), float(np.median(ee[:, 0])),
             float(np.median(ee[:, 1]))), flush=True)
    if session is not None:
        viz.announce(session)


TF_PARENT = "human_root"
_TF_CRITICAL = (ls.NOSE, ls.LEFT_SHOULDER, ls.RIGHT_SHOULDER, ls.LEFT_WRIST, ls.RIGHT_WRIST)


def run_tf(args):
    """LIVE ROS teleop (Process B, --source tf): consume the landmark frames
    perception broadcasts on ``/tf`` via rclcpp_kit's C++ TransformListener (ingest
    off the GIL on its own thread; Python only crosses on lookup), reconstruct the
    robot-frame skeleton, then the same target map + CLIK. Dataset written on exit.

    The frames arrive already in the robot frame (perception converts before it
    broadcasts), so no MediaPipe->robot conversion is needed here."""
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit import tf as rtf
    rc = bringup_rclcpp()
    if not rc.ok():
        rc.init()
    listener = rtf.TransformListener()          # C++ ingest thread; lookups cross on demand
    print("Robot %s: nq=%d nv=%d, arm reach %.3f m. Consuming /tf landmark frames via "
          "rclcpp_kit C++ TransformListener (startup grace %.0fs) ..."
          % (cfg.name, rt.model.nq, rt.model.nv, rt.arm, args.startup_timeout), flush=True)

    session = None
    rr = None
    viz_mode = "skeleton"
    if not args.no_viz:
        import rerun as rr
        from retarget_pipeline import viz
        if args.shared_viewer:
            session = viz.init_rerun_shared("retarget", "connect", viz.blueprint_shared(),
                                            url=args.viewer_url)
            print("Rerun: connected to the shared viewer (%s)."
                  % (args.viewer_url or viz.DEFAULT_VIEWER_URL), flush=True)
        else:
            session = viz.init_rerun("retarget", args.rrd, blueprint=viz.blueprint_retarget())
        viz_mode = _setup_robot_viz(rr, rt, args)

    pose_frames = ["pose/" + n for n in ls.POSE_LANDMARK_NAMES]
    nose_frame = pose_frames[ls.NOSE]
    dt = 1.0 / args.fps
    hip_w = np.asarray(rt.hip, dtype=np.float64)
    sh = np.stack([rt.anchor_L, rt.anchor_R]).astype(np.float64)
    euro = _EuroState(dt)
    q = rt.q0.copy()
    Q, targets_log, ts_log, ee_log, lag = [], [], [], [], []
    solve_ms = []

    if not listener.can_transform(TF_PARENT, nose_frame, timeout=args.startup_timeout):
        print("[retarget] no /tf landmark frames within the %.0fs startup grace -- is a "
              "perceive publishing /tf (same ROS_DOMAIN_ID)? Start perception first."
              % args.startup_timeout, flush=True)
        return
    print("Receiving /tf landmark frames; retargeting live.", flush=True)

    last_stamp = None
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while deadline is None or time.monotonic() < deadline:
            c0 = time.perf_counter()
            try:
                nose = listener.lookup_transform(TF_PARENT, nose_frame)
            except Exception:
                time.sleep(0.002)
                continue
            stamp = nose.header.stamp.sec + nose.header.stamp.nanosec * 1e-9
            now = time.time()
            if last_stamp is not None and stamp == last_stamp:
                if Q and now - stamp > args.idle_timeout:   # producer stopped
                    break
                time.sleep(0.002)
                continue
            last_stamp = stamp
            r = np.zeros((ls.N_POSE, 3))
            missing_critical = False
            for idx, fname in enumerate(pose_frames):
                try:
                    ts = listener.lookup_transform(TF_PARENT, fname)
                    r[idx] = [ts.transform.translation.x, ts.transform.translation.y,
                              ts.transform.translation.z]
                except Exception:
                    if idx in _TF_CRITICAL:
                        missing_critical = True
                        break
            if missing_critical:
                time.sleep(0.002)
                continue
            t0 = time.perf_counter()
            tgt = euro.step(_frame_target(r, hip_w, sh, rt.robot_torso, rt.arm,
                                          REACH_FRAC)).reshape(9)
            q, errs = rt.solve(tgt, q)
            solve_ms.append((time.perf_counter() - t0) * 1e3)
            lag.append(now - stamp)
            Q.append(q.copy())
            targets_log.append(tgt)
            ts_log.append(stamp)
            ee_log.append(list(errs.values()))
            if rr is not None:
                _log_retarget(rr, rt, len(Q) - 1, q, tgt, r, viz_mode)
            if len(Q) % 30 == 0:
                print("  live frame %d: lag %.1f ms (median %.1f), solve %.2f ms, "
                      "EE L=%.3f R=%.3f m"
                      % (len(Q), lag[-1] * 1e3, float(np.median(lag)) * 1e3,
                         float(np.median(solve_ms)), ee_log[-1][0], ee_log[-1][1]),
                      flush=True)
            time.sleep(max(0.0, dt - (time.perf_counter() - c0)))
    except KeyboardInterrupt:
        print("\n[retarget] interrupted; writing what we have.", flush=True)
    if not Q:
        print("[retarget] received no usable /tf pose frames.", flush=True)
        return
    Q = np.array(Q)
    ee = np.array(ee_log)
    lag = np.array(lag)
    _write_dataset(args, cfg, rt, Q, np.array(targets_log), ts_log, ee,
                   source="/tf (live ROS transport)")
    print("\nSUMMARY %s (LIVE /tf): %d frames retargeted | end-to-end lag median %.1f ms "
          "(p90 %.1f, max %.1f) | CLIK %.2f ms/frame | EE err median L=%.3f R=%.3f m"
          % (cfg.name, len(Q), float(np.median(lag)) * 1e3,
             float(np.percentile(lag, 90)) * 1e3, float(np.max(lag)) * 1e3,
             float(np.median(solve_ms)), float(np.median(ee[:, 0])),
             float(np.median(ee[:, 1]))), flush=True)
    if session is not None:
        viz.announce(session)


def run_bench(args):
    """A-vs-B: the retarget glue (coord transform + target map + One-Euro filter)
    over the whole stream, cppyy_kit C++ kernel vs the Python per-frame loop."""
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    if args.replay and os.path.exists(args.replay):
        pw_all, _ = load_pose_world(args.replay)
    else:
        if args.replay:
            print("[retarget] %s not found; benching on the synthetic scene "
                  "(record a stream to bench on real landmarks)." % args.replay, flush=True)
        pw_all = np.array([pw.reshape(99) for (_, pw, _, _, _, _, _)
                           in ls.synthetic_frames(args.bench_n)], dtype=np.float64)
    if len(pw_all) == 0:
        print("bench: no frames.")
        return
    dt = 1.0 / args.fps
    a = compute_targets(rt, pw_all, dt, use_cpp=True)
    b = compute_targets(rt, pw_all, dt, use_cpp=False)
    max_diff = float(np.max(np.abs(a - b)))

    def bench(use_cpp, reps=5):
        best = float("inf")
        for _ in range(reps):
            t0 = time.perf_counter()
            compute_targets(rt, pw_all, dt, use_cpp=use_cpp)
            best = min(best, (time.perf_counter() - t0) * 1e3)
        return best

    a_ms = bench(True)
    b_ms = bench(False)
    print("\n== retarget-glue A-vs-B (%d frames: coord xform + target map + "
          "One-Euro filter) ==" % len(pw_all))
    print("  A  cppyy_kit C++ kernel (one cppdef pass): %8.3f ms total" % a_ms)
    print("  B  Python per-frame loop:                  %8.3f ms total" % b_ms)
    print("  A speedup: %.1fx  |  max |A-B| = %.2e m (numeric agreement)\n"
          % (b_ms / a_ms if a_ms > 0 else float("nan"), max_diff))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=list(ROBOTS), default="talos")
    # Three input modes: --replay (offline recorded stream), --follow (live, tail a
    # stream a perceive is still writing), and --source tf (live, consume /tf directly
    # via rclcpp_kit). tf is the default when neither file mode is given. --replay and
    # --follow are mutually exclusive; both are file modes usable without ROS.
    ap.add_argument("--source", choices=["tf"], default="tf",
                    help="live input source when no --replay/--follow: 'tf' consumes "
                         "the landmark frames perception broadcasts on /tf")
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--replay", metavar="FILE", default=None,
                     help="retarget a recorded landmark stream (offline; no ROS)")
    src.add_argument("--follow", metavar="FILE", default=None,
                     help="LIVE teleop: tail a stream a perceive process is still "
                          "writing and retarget each frame as it arrives (no ROS)")
    ap.add_argument("--idle-timeout", type=float, default=2.0, dest="idle_timeout",
                    help="follow mode: exit this many seconds after the last new frame "
                         "(once frames have been flowing)")
    ap.add_argument("--startup-timeout", type=float, default=30.0, dest="startup_timeout",
                    help="follow mode: grace period for the producer's FIRST frame "
                         "(covers a cold perceive's env activation + model load)")
    ap.add_argument("--dataset", metavar="PATH", help="where to write the .npz dataset")
    ap.add_argument("--fps", type=float, default=30.0, help="stream fps (for dt)")
    ap.add_argument("--no-cpp", action="store_true",
                    help="use the Python glue loop instead of the cppyy_kit kernel")
    ap.add_argument("--no-viz", action="store_true", help="skip Rerun")
    ap.add_argument("--robot-viz", choices=["mesh", "skeleton"], default="mesh",
                    dest="robot_viz",
                    help="how to draw the robot in Rerun: 'mesh' = the real URDF link "
                         "meshes (default), 'skeleton' = the joint tree (fallback)")
    ap.add_argument("--shared-viewer", action="store_true", dest="shared_viewer",
                    help="tf mode: connect to perception's shared Rerun viewer "
                         "(one window: camera + skeleton + robot) instead of opening own")
    ap.add_argument("--viewer-url", default=None, dest="viewer_url",
                    help="gRPC URL of the shared viewer to connect to (default: %s)"
                         % "the local spawned viewer")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="tf mode: run seconds (0 = until the /tf stream goes idle "
                         "or Ctrl-C)")
    ap.add_argument("--bench", action="store_true",
                    help="retarget-glue A-vs-B micro-bench, then exit")
    ap.add_argument("--bench-n", type=int, default=200, dest="bench_n")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "pipeline",
                                                  "retarget.rrd"))
    args = ap.parse_args(argv)
    if args.bench:
        return run_bench(args)
    if args.follow:
        return run_follow(args)
    if args.replay:
        return run_retarget(args)
    return run_tf(args)


if __name__ == "__main__":
    sys.exit(main() or 0)

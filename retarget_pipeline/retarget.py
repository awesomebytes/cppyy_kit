#!/usr/bin/env python
"""
retarget.py (M6f, Process B) -- the retargeting half of the capture rig.

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


# --------------------------------------------------------------------------- #
# Robot configs: which URDF, which frames to retarget, per humanoid.
# --------------------------------------------------------------------------- #
class RobotConfig:
    def __init__(self, name, urdf_rel, l_grip, r_grip, head, l_sh, r_sh):
        self.name = name
        self.urdf_rel = urdf_rel
        self.l_grip, self.r_grip, self.head = l_grip, r_grip, head
        self.l_sh, self.r_sh = l_sh, r_sh


ROBOTS = {
    "talos": RobotConfig(
        "talos", ("talos_data", "robots", "talos_reduced.urdf"),
        "gripper_left_base_link", "gripper_right_base_link", "head_2_link",
        "arm_left_1_link", "arm_right_1_link"),
    "g1": RobotConfig(
        "g1", ("g1_description", "urdf", "g1_29dof_rev_1_0.urdf"),
        "left_wrist_yaw_link", "right_wrist_yaw_link", "head_link",
        "left_shoulder_pitch_link", "right_shoulder_pitch_link"),
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
namespace m6f_retarget {

static inline double euro_alpha(double cutoff, double dt) {
  double tau = 1.0 / (2.0 * M_PI * cutoff);
  return 1.0 / (1.0 + tau / dt);
}

// in_pw: (F, 99) MediaPipe pose_world (33 landmarks * xyz), row-major.
// out:   (F, 9)  robot-frame smoothed targets [L(3), R(3), Head(3)].
// Anchors (Talos shoulder L/R, head nominal) + arm length in robot frame; targets
// are scaled by (robot arm / human arm), clamped to reach_frac*arm of the shoulder,
// then One-Euro-smoothed across the F frames (mincut/beta) at timestep dt.
void map_stream(uintptr_t in_pw, int F, uintptr_t out,
                double lsx, double lsy, double lsz,
                double rsx, double rsy, double rsz,
                double hnx, double hny, double hnz,
                double arm, double dt, double mincut, double beta,
                double reach_frac) {
  const double* pw = reinterpret_cast<const double*>(in_pw);
  double* o = reinterpret_cast<double*>(out);
  const int LSH = 11, RSH = 12, LWR = 15, RWR = 16, NOSE = 0;
  double prev[9], dprev[9];
  bool have_prev = false;
  double anch[9] = {lsx, lsy, lsz, rsx, rsy, rsz, hnx, hny, hnz};
  for (int f = 0; f < F; ++f) {
    const double* row = pw + (long)f * 99;
    // MediaPipe world (x left, y down, z toward cam) -> robot (x fwd, y left, z up).
    auto rob = [&](int idx, double* v) {
      const double* p = row + idx * 3;
      v[0] = -p[2]; v[1] = p[0]; v[2] = -p[1];
    };
    double lsh[3], rsh[3], lwr[3], rwr[3], nose[3], shc[3];
    rob(LSH, lsh); rob(RSH, rsh); rob(LWR, lwr); rob(RWR, rwr); rob(NOSE, nose);
    for (int k = 0; k < 3; ++k) shc[k] = 0.5 * (lsh[k] + rsh[k]);
    double la = 0, ra = 0;
    for (int k = 0; k < 3; ++k) {
      la += (lwr[k] - lsh[k]) * (lwr[k] - lsh[k]);
      ra += (rwr[k] - rsh[k]) * (rwr[k] - rsh[k]);
    }
    double hum = 0.5 * (std::sqrt(la) + std::sqrt(ra)) + 1e-6;
    double s = arm / hum;
    double tgt[9];
    for (int k = 0; k < 3; ++k) {
      tgt[k]     = anch[k]     + s * (lwr[k] - lsh[k]);       // left gripper
      tgt[3 + k] = anch[3 + k] + s * (rwr[k] - rsh[k]);       // right gripper
      tgt[6 + k] = anch[6 + k] + s * (nose[k] - shc[k]);      // head
    }
    // Clamp gripper targets into the reachable sphere around each shoulder anchor.
    for (int e = 0; e < 2; ++e) {
      double d2 = 0; int b = e * 3;
      for (int k = 0; k < 3; ++k) d2 += (tgt[b + k] - anch[b + k]) * (tgt[b + k] - anch[b + k]);
      double d = std::sqrt(d2), maxr = reach_frac * arm;
      if (d > maxr && d > 1e-9) {
        double sc = maxr / d;
        for (int k = 0; k < 3; ++k) tgt[b + k] = anch[b + k] + sc * (tgt[b + k] - anch[b + k]);
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

}  // namespace m6f_retarget
"""

_GLUE_READY = False


def _bringup_glue():
    global _GLUE_READY
    import cppyy
    if not _GLUE_READY:
        cppyy.cppdef(_GLUE)
        _GLUE_READY = True
    return cppyy.gbl.m6f_retarget


def map_stream_python(pw_all, anchors, arm, dt, mincut, beta, reach_frac):
    """The naive baseline for --bench: the identical retarget glue as a Python
    per-frame loop (the sequential One-Euro filter is the per-element trap)."""
    F = pw_all.shape[0]
    out = np.zeros((F, 9), dtype=np.float64)
    anch = np.asarray(anchors, dtype=np.float64).reshape(3, 3)
    prev = None
    dprev = np.zeros((3, 3))

    def euro_alpha(cutoff):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    for f in range(F):
        r = ls.mediapipe_world_to_robot(pw_all[f].reshape(33, 3))
        lsh, rsh = r[ls.LEFT_SHOULDER], r[ls.RIGHT_SHOULDER]
        lwr, rwr = r[ls.LEFT_WRIST], r[ls.RIGHT_WRIST]
        nose, shc = r[ls.NOSE], 0.5 * (r[ls.LEFT_SHOULDER] + r[ls.RIGHT_SHOULDER])
        hum = 0.5 * (np.linalg.norm(lwr - lsh) + np.linalg.norm(rwr - rsh)) + 1e-6
        s = arm / hum
        tgt = np.stack([anch[0] + s * (lwr - lsh), anch[1] + s * (rwr - rsh),
                        anch[2] + s * (nose - shc)])
        for e in range(2):
            d = np.linalg.norm(tgt[e] - anch[e])
            if d > reach_frac * arm and d > 1e-9:
                tgt[e] = anch[e] + (reach_frac * arm / d) * (tgt[e] - anch[e])
        if prev is None:
            prev = tgt.copy()
            out[f] = tgt.reshape(9)
        else:
            for k in range(3):
                dx = (tgt[k] - prev[k]) / dt
                ad = euro_alpha(1.0)
                dhat = ad * dx + (1 - ad) * dprev[k]
                cutoff = mincut + beta * np.abs(dhat)
                a = euro_alpha(cutoff)
                xhat = a * tgt[k] + (1 - a) * prev[k]
                prev[k], dprev[k] = xhat, dhat
                out[f, k * 3:k * 3 + 3] = xhat
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

    def anchors_flat(self):
        return np.concatenate([self.anchor_L, self.anchor_R, self.anchor_H])

    def solve(self, targets9, q_warm, iters=40, damp=1e-3, wpost=1e-2, whead=0.0):
        """CLIK toward the 9-vector [L,R,Head] targets from warm start ``q_warm``.
        Returns ``(q, {frame: err_m})``. The free-flyer base is locked by solving
        over the *actuated* DOFs only (columns 6: of the Jacobian) -- solving the
        full system and zeroing the base afterwards would discard the solution's
        dominant base component and barely move the arms."""
        pin = self._pin
        q = q_warm.copy()
        na = self.model.nv - 6                       # actuated DOFs (skip base)
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
            H += (damp + wpost) * np.eye(na)
            g += wpost * pin.difference(self.model, q, self.q0)[6:]
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


def compute_targets(rt, pw_all, dt, use_cpp=True, mincut=1.2, beta=0.03, reach_frac=0.8):
    """Retarget glue: (F,99) landmarks -> (F,9) smoothed targets. Uses the C++
    kernel (cppyy_kit) by default, else the Python loop."""
    anch = rt.anchors_flat()
    if use_cpp:
        glue = _bringup_glue()
        out = np.zeros((pw_all.shape[0], 9), dtype=np.float64)
        c = np.ascontiguousarray(pw_all, dtype=np.float64)
        glue.map_stream(c.ctypes.data, int(c.shape[0]), out.ctypes.data,
                        anch[0], anch[1], anch[2], anch[3], anch[4], anch[5],
                        anch[6], anch[7], anch[8], rt.arm, dt, mincut, beta, reach_frac)
        return out
    return map_stream_python(pw_all, anch, rt.arm, dt, mincut, beta, reach_frac)


# --------------------------------------------------------------------------- #
# Modes.
# --------------------------------------------------------------------------- #
def run_retarget(args):
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    print("Robot %s: nq=%d nv=%d, arm reach %.3f m; anchors L=%s R=%s"
          % (cfg.name, rt.model.nq, rt.model.nv, rt.arm,
             np.round(rt.anchor_L, 3), np.round(rt.anchor_R, 3)), flush=True)

    pw_all, ts = load_pose_world(args.replay)
    if len(pw_all) == 0:
        print("No pose frames in %s." % args.replay)
        return
    dt = 1.0 / args.fps
    targets = compute_targets(rt, pw_all, dt, use_cpp=not args.no_cpp)
    print("Retarget glue: computed %d target frames (%s path)."
          % (len(targets), "Python" if args.no_cpp else "cppyy_kit C++ kernel"),
          flush=True)

    session = None
    rr = None
    if not args.no_viz:
        import rerun as rr
        from retarget_pipeline import viz
        session = viz.init_rerun("m6f_retarget", args.rrd,
                                 blueprint=viz.blueprint_retarget())

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
            _log_retarget(rr, rt, i, q, targets[i], pw_all[i])
        if (i + 1) % 30 == 0:
            print("  frame %d/%d: solve %.2f ms, EE err L=%.3f R=%.3f m"
                  % (i + 1, F, np.median(solve_ms[-30:]), ee_err[i, 0], ee_err[i, 1]),
                  flush=True)

    _write_dataset(args, cfg, rt, Q, targets, ts, ee_err)
    print("\nSUMMARY %s: %d frames | CLIK %.2f ms/frame (median) | EE err median "
          "L=%.3f R=%.3f m (mean %.3f)"
          % (cfg.name, F, float(np.median(solve_ms)), float(np.median(ee_err[:, 0])),
             float(np.median(ee_err[:, 1])), float(np.mean(ee_err))), flush=True)
    if session is not None:
        viz.announce(session)


def _log_retarget(rr, rt, i, q, targets9, pw):
    from retarget_pipeline import viz
    rr.set_time("frame", sequence=i)
    pts, bones = rt.joint_skeleton(q)
    rr.log("robot/joints", rr.Points3D(pts, radii=0.02, colors=[(120, 200, 255)]))
    rr.log("robot/bones", rr.LineStrips3D(bones, colors=[(120, 200, 255)]))
    tgt = targets9.reshape(3, 3)
    rr.log("robot/targets", rr.Points3D(tgt, radii=0.04, colors=[(255, 140, 60)]))
    human = ls.mediapipe_world_to_robot(pw.reshape(33, 3))
    viz.log_skeleton_3d(rr, "human/pose", human, ls.POSE_CONNECTIONS,
                        color=(160, 160, 160))


def _write_dataset(args, cfg, rt, Q, targets, ts, ee_err):
    path = args.dataset or os.path.join(REPO, "build", "pipeline",
                                        "dataset_%s.npz" % cfg.name)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, robot=cfg.name, q=Q.astype(np.float32),
             targets=targets.astype(np.float32),
             t=np.array(ts[:len(Q)], dtype=np.float64),
             ee_err=ee_err.astype(np.float32),
             joint_names=np.array(list(rt.model.names), dtype=object),
             source_stream=os.path.abspath(args.replay))
    print("Policy-kickstart dataset -> %s (q %s, targets %s)."
          % (path, Q.shape, targets.shape), flush=True)


def run_bench(args):
    """A-vs-B: the retarget glue (coord transform + target map + One-Euro filter)
    over the whole stream, cppyy_kit C++ kernel vs the Python per-frame loop."""
    cfg = ROBOTS[args.robot]
    rt = Retargeter(cfg)
    if args.replay:
        pw_all, _ = load_pose_world(args.replay)
    else:
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
    print("\n== M6f retarget-glue A-vs-B (%d frames: coord xform + target map + "
          "One-Euro filter) ==" % len(pw_all))
    print("  A  cppyy_kit C++ kernel (one cppdef pass): %8.3f ms total" % a_ms)
    print("  B  Python per-frame loop:                  %8.3f ms total" % b_ms)
    print("  A speedup: %.1fx  |  max |A-B| = %.2e m (numeric agreement)\n"
          % (b_ms / a_ms if a_ms > 0 else float("nan"), max_diff))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", choices=list(ROBOTS), default="talos")
    ap.add_argument("--replay", metavar="FILE",
                    default=os.path.join(REPO, "build", "pipeline", "demo.jsonl"),
                    help="landmark stream to retarget (recorded by perceive.py)")
    ap.add_argument("--dataset", metavar="PATH", help="where to write the .npz dataset")
    ap.add_argument("--fps", type=float, default=30.0, help="stream fps (for dt)")
    ap.add_argument("--no-cpp", action="store_true",
                    help="use the Python glue loop instead of the cppyy_kit kernel")
    ap.add_argument("--no-viz", action="store_true", help="skip Rerun")
    ap.add_argument("--bench", action="store_true",
                    help="retarget-glue A-vs-B micro-bench, then exit")
    ap.add_argument("--bench-n", type=int, default=200, dest="bench_n")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "pipeline",
                                                  "retarget.rrd"))
    args = ap.parse_args(argv)
    if args.bench:
        run_bench(args)
    else:
        run_retarget(args)


if __name__ == "__main__":
    main()

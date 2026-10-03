"""
test_retarget -- headless smoke for the retargeting process (Process B).

Runs in the ``wbc`` env (needs pinocchio). Auto-skips elsewhere, so the default
suite is unaffected. No ROS, no camera, no display, no network -- it retargets a
tiny synthetic landmark stream written on the fly.
"""
import math
import os
import sys
import threading
import time

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

pytest.importorskip("pinocchio", reason="retarget needs pinocchio (wbc env)")

from retarget_pipeline import landmark_stream as ls  # noqa: E402
from retarget_pipeline import retarget as R          # noqa: E402


def _write_stream(path, n=40):
    with ls.StreamWriter(path, source="synthetic", fps_target=30.0) as w:
        for (t, pw, pi, lh, rh, ww, hh) in ls.synthetic_frames(n):
            w.write(t=t, pose_world=pw, pose_image=pi, w=ww, h=hh)


def test_talos_retargeter_builds():
    rt = R.Retargeter(R.ROBOTS["talos"])
    assert rt.model.nq == 39 and rt.model.nv == 38
    assert rt.arm > 0.3                                   # arm reach in metres


def test_g1_config_loads():
    """Checks that the same retarget mapping works with G1 by changing the URDF."""
    rt = R.Retargeter(R.ROBOTS["g1"])
    assert rt.model.nq > 20                               # 29-DOF humanoid + base


def test_glue_kernel_matches_python(tmp_path):
    """Compares the C++ glue kernel with the Python loop. Their outputs must agree within
    float precision (COMMON_PATTERNS section 23)."""
    rt = R.Retargeter(R.ROBOTS["talos"])
    pw = np.array([p.reshape(99) for (_, p, _, _, _, _, _)
                   in ls.synthetic_frames(50)], dtype=np.float64)
    a = R.compute_targets(rt, pw, 1.0 / 30.0, use_cpp=True)
    b = R.compute_targets(rt, pw, 1.0 / 30.0, use_cpp=False)
    assert a.shape == (50, 9)
    assert np.max(np.abs(a - b)) < 1e-5


def test_retarget_synthetic_bounded(tmp_path):
    """Retargets a synthetic stream without a display. Checks the Talos trajectory error
    and loads the generated policy-kickstart dataset."""
    stream = str(tmp_path / "s.jsonl")
    ds = str(tmp_path / "ds.npz")
    _write_stream(stream, n=40)
    R.main(["--robot", "talos", "--replay", stream, "--dataset", ds,
            "--no-viz"])
    d = np.load(ds, allow_pickle=True)
    assert d["q"].shape[0] >= 30
    assert d["q"].shape[1] == 39
    assert d["targets"].shape[1] == 9
    assert float(np.median(d["ee_err"])) < 0.15           # reachable-workspace bound
    assert str(d["robot"]) == "talos"


def test_follow_mode_consumes_live_stream(tmp_path):
    """Checks that `--follow` reads frames while another thread writes the stream, retargets
    each frame, and writes the dataset after the stream is idle."""
    stream = str(tmp_path / "live.jsonl")
    ds = str(tmp_path / "live_ds.npz")

    def writer():
        with ls.StreamWriter(stream, source="synthetic", fps_target=30.0) as w:
            for (_t, pw, pi, lh, rh, ww, hh) in ls.synthetic_frames(25):
                w.write(t=time.time(), pose_world=pw, pose_image=pi, w=ww, h=hh)
                time.sleep(0.01)                          # produce faster than realtime

    th = threading.Thread(target=writer)
    th.start()
    R.main(["--robot", "talos", "--follow", stream, "--dataset", ds,
            "--no-viz", "--idle-timeout", "1.0"])
    th.join()
    d = np.load(ds, allow_pickle=True)
    assert d["q"].shape[0] >= 20                           # consumed most/all frames
    assert d["q"].shape[1] == 39
    assert d["targets"].shape[1] == 9
    assert os.path.abspath(stream) == str(d["source_stream"])


def test_follow_survives_cold_start(tmp_path):
    """Checks that the consumer waits for the first frame when producer startup exceeds
    `--idle-timeout`. The startup grace period covers environment activation and model
    loading when the consumer starts first."""
    stream = str(tmp_path / "cold.jsonl")
    ds = str(tmp_path / "cold_ds.npz")

    def writer():
        time.sleep(2.5)                            # file appears well after idle-timeout
        with ls.StreamWriter(stream, source="synthetic", fps_target=30.0) as w:
            for (_t, pw, pi, lh, rh, ww, hh) in ls.synthetic_frames(15):
                w.write(t=time.time(), pose_world=pw, pose_image=pi, w=ww, h=hh)
                time.sleep(0.01)

    th = threading.Thread(target=writer)
    th.start()
    # --idle-timeout 1.0 (< the 2.5 s cold delay) must NOT end the consumer before the
    # first frame; the startup grace (10 s) covers the late first frame.
    R.main(["--robot", "talos", "--follow", stream, "--dataset", ds, "--no-viz",
            "--idle-timeout", "1.0", "--startup-timeout", "10"])
    th.join()
    d = np.load(ds, allow_pickle=True)
    assert d["q"].shape[0] >= 12                    # consumed frames after the cold wait


def test_replay_and_follow_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        R.main(["--replay", "a.jsonl", "--follow", "b.jsonl"])


def _circle_motion(n, fps=30.0):
    """Both wrists trace 0.3 m circles below the shoulders in the MediaPipe world frame.
    Checks that the retarget target follows the motion."""
    base = ls._BASE_POSE.copy()
    out = []
    for i in range(n):
        a = 2 * math.pi * 0.3 * (i / fps)
        p = base.copy()
        p[ls.LEFT_WRIST] = base[ls.LEFT_SHOULDER] + np.array(
            [0.30 * math.cos(a), 0.35 + 0.30 * math.sin(a), 0.10 * math.sin(a)], np.float32)
        p[ls.RIGHT_WRIST] = base[ls.RIGHT_SHOULDER] + np.array(
            [-0.30 * math.cos(a), 0.35 + 0.30 * math.sin(a), 0.10 * math.cos(a)], np.float32)
        out.append(p)
    return np.array(out, np.float32)


def test_retarget_tracks_wrist_motion():
    """Checks motion fidelity: a wrist circle must produce an end-effector target with high
    per-axis correlation and non-zero amplitude. End-effector error alone does not catch
    a static or incorrect target map."""
    n = 200
    pw = _circle_motion(n)
    for robot in ("talos", "g1"):
        rt = R.Retargeter(R.ROBOTS[robot])
        tgt = R.compute_targets(rt, pw.reshape(n, 99).astype(np.float64), 1 / 30.0, use_cpp=True)
        # HIP-RELATIVE fidelity: the human wrist relative to the human hip must drive
        # the EE target relative to the robot hip (the owner's chain).
        for wr, cols in ((ls.LEFT_WRIST, slice(0, 3)), (ls.RIGHT_WRIST, slice(3, 6))):
            inp = np.array([(lambda r: r[wr] - 0.5 * (r[ls.LEFT_HIP] + r[ls.RIGHT_HIP]))(
                ls.mediapipe_world_to_robot(pw[f])) for f in range(n)])
            out = tgt[:, cols] - rt.hip
            assert out.std() > 0.02, "%s target barely moves" % robot
            for ax in range(3):
                if inp[:, ax].std() < 1e-3:
                    continue
                c = np.corrcoef(inp[:, ax], out[:, ax])[0, 1]
                assert c > 0.7, "%s axis %d hip-rel corr %.2f too low" % (robot, ax, c)


def _full_sweep(n, fps=30.0):
    """Each wrist moves through a wide diagonal arc relative to the shoulder. Checks the
    approximately 1:1 amplitude mapping."""
    base = ls._BASE_POSE.copy()
    out = []
    for i in range(n):
        s = math.sin(2 * math.pi * 0.25 * (i / fps))
        p = base.copy()
        p[ls.LEFT_WRIST] = base[ls.LEFT_SHOULDER] + np.array(
            [0.10, 0.10 + 0.45 * s, 0.10 + 0.40 * s], np.float32)
        p[ls.RIGHT_WRIST] = base[ls.RIGHT_SHOULDER] + np.array(
            [0.10, -0.10 - 0.45 * s, 0.10 + 0.40 * s], np.float32)
        out.append(p)
    return np.array(out, np.float32)


def test_full_sweep_amplitude_near_one_to_one():
    """Checks the approximately 1:1 mapping at `REACH_FRAC=0.95`. A wide arm sweep should
    move the grippers beyond the earlier 13-26 cm range. At scale 1.0, Talos moves about
    0.75 m and G1 about 0.50 m. Increasing `--motion-scale` must not reduce travel."""
    n = 240
    pw = _full_sweep(n).reshape(n, 99).astype(np.float64)
    bounds = {"talos": 0.55, "g1": 0.38}                  # solved-gripper 3D p2p floor
    for robot in ("talos", "g1"):
        rt = R.Retargeter(R.ROBOTS[robot])
        for scale in (1.0, 1.5):
            tgt = R.compute_targets(rt, pw, 1 / 30.0, use_cpp=True, motion_scale=scale)
            q = rt.q0.copy()
            ee = np.zeros((n, 3))
            import pinocchio as pin
            for i in range(n):
                r_i = ls.mediapipe_world_to_robot(pw[i].reshape(33, 3))
                q, _ = rt.solve(tgt[i], q, head_r=r_i)
                pin.forwardKinematics(rt.model, rt.data, q)
                pin.updateFramePlacements(rt.model, rt.data)
                ee[i] = rt.data.oMf[rt.LF].translation
            amp = float(np.linalg.norm(ee.max(0) - ee.min(0)))
            assert amp > bounds[robot], \
                "%s scale %.1f full-sweep amplitude %.3f m too small" % (robot, scale, amp)


def _head_r(yaw, pitch):
    """robot-frame landmarks whose ear-midpoint->nose direction encodes (yaw, pitch)."""
    fwd = np.array([math.cos(pitch) * math.cos(yaw),
                    math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
    r = np.zeros((ls.N_POSE, 3))
    r[ls.LEFT_EAR], r[ls.RIGHT_EAR] = [0, 0.1, 0], [0, -0.1, 0]
    r[ls.NOSE] = 0.5 * (r[ls.LEFT_EAR] + r[ls.RIGHT_EAR]) + fwd
    return r


def test_head_tracks_operator_yaw_pitch():
    """Checks Talos head tracking. Head yaw and pitch must follow the operator with the
    correct sign. Talos uses RY for pitch and RZ for yaw."""
    import pinocchio as pin
    rt = R.Retargeter(R.ROBOTS["talos"])
    assert rt.has_neck and rt.neck_yaw is not None and rt.neck_pitch is not None
    n = 120
    t = np.linspace(0, 4 * math.pi, n)
    yaws, pitches = 0.5 * np.sin(t), 0.3 * np.sin(0.5 * t)
    tgt = np.concatenate([rt.anchor_L, rt.anchor_R, rt.anchor_H])
    out_yaw, out_pitch = np.zeros(n), np.zeros(n)
    q = rt.q0.copy()
    for i in range(n):
        q, _ = rt.solve(tgt, q, head_r=_head_r(yaws[i], pitches[i]))
        pin.forwardKinematics(rt.model, rt.data, q)
        pin.updateFramePlacements(rt.model, rt.data)
        rpy = pin.rpy.matrixToRpy(rt.data.oMf[rt.HD].rotation)
        out_yaw[i], out_pitch[i] = rpy[2], -rpy[1]         # -pitch: +x-fwd points up
    assert np.corrcoef(yaws, out_yaw)[0, 1] > 0.9, "head yaw does not track"
    assert np.corrcoef(pitches, out_pitch)[0, 1] > 0.9, "head pitch does not track"
    # correct sign: peak look-left gives head-left, peak look-up gives head-up
    assert out_yaw[np.argmax(yaws)] > 0.1 and out_pitch[np.argmax(pitches)] > 0.05


def test_g1_head_is_rigid():
    """G1 has no neck joints and its head is mechanically rigid. `_detect_neck` must return
    `has_neck=False`, and `_apply_head` must do nothing."""
    rt = R.Retargeter(R.ROBOTS["g1"])
    assert not rt.has_neck
    q = rt.q0.copy()
    rt._apply_head(q, _head_r(0.5, 0.3))                   # must not raise / not move q
    assert np.array_equal(q, rt.q0)


def test_trunk_stays_upright():
    """Checks that the trunk stays near upright when both grippers reach forward. The posture
    weights should prevent the roughly 52 degree lean seen before."""
    torso = {"talos": "torso_2_link", "g1": "torso_link"}
    for robot in ("talos", "g1"):
        rt = R.Retargeter(R.ROBOTS[robot])
        tid = rt.model.getFrameId(torso[robot])
        tgt = np.concatenate([rt.anchor_L + [0.2, 0.05, 0.0],
                              rt.anchor_R + [0.2, -0.05, 0.0], rt.anchor_H])
        q, _ = rt.solve(tgt, rt.q0.copy(), iters=60)
        import pinocchio as pin
        pin.forwardKinematics(rt.model, rt.data, q)
        pin.updateFramePlacements(rt.model, rt.data)
        rot = rt.data.oMf[tid].rotation
        pitch = abs(math.degrees(math.atan2(-rot[2, 0], math.hypot(rot[2, 1], rot[2, 2]))))
        assert pitch < 15.0, "%s trunk pitched %.1f deg (lean regression)" % (robot, pitch)


def test_visual_meshes_load(tmp_path):
    """Checks that Talos and G1 URDF meshes load for Rerun and receive world placements
    from forward kinematics."""
    for robot, min_n in (("talos", 20), ("g1", 20)):
        rt = R.Retargeter(R.ROBOTS[robot])
        assert rt.has_meshes, "%s meshes should load" % robot
        meshes = rt.visual_meshes()
        assert len(meshes) >= min_n
        assert all(p.lower().endswith((".stl", ".obj", ".glb", ".gltf"))
                   and os.path.isfile(p) for _, p in meshes)
        placements = rt.visual_placements(rt.q0)
        assert len(placements) == len(meshes)
        name, t, rot = placements[0]
        assert t.shape == (3,) and rot.shape == (3, 3)


def test_source_dispatch(monkeypatch, tmp_path):
    """Checks that live TF is the default when no file mode is set, and that `--replay` and
    `--follow` select their respective modes. No live ROS graph is required."""
    calls = []
    monkeypatch.setattr(R, "run_tf", lambda a: calls.append("tf"))
    monkeypatch.setattr(R, "run_retarget", lambda a: calls.append("replay"))
    monkeypatch.setattr(R, "run_follow", lambda a: calls.append("follow"))
    R.main(["--robot", "talos"])
    assert calls == ["tf"]
    calls.clear()
    R.main(["--replay", str(tmp_path / "x.jsonl")])
    assert calls == ["replay"]
    calls.clear()
    R.main(["--follow", str(tmp_path / "y.jsonl")])
    assert calls == ["follow"]

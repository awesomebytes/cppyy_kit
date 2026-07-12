"""
test_retarget -- headless smoke for the retargeting process (Process B).

Runs in the ``wbc`` env (needs pinocchio). Auto-skips elsewhere, so the default
suite is unaffected. No ROS, no camera, no display, no network -- it retargets a
tiny synthetic landmark stream written on the fly.
"""
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
    """G1 stretch: the retarget mapping is model-generic, so G1 is a URDF swap."""
    rt = R.Retargeter(R.ROBOTS["g1"])
    assert rt.model.nq > 20                               # 29-DOF humanoid + base


def test_glue_kernel_matches_python(tmp_path):
    """The cppyy_kit C++ glue kernel and the Python loop agree to ~float epsilon --
    the tests-as-contract gate on the lowered kernel (COMMON_PATTERNS s23 discipline)."""
    rt = R.Retargeter(R.ROBOTS["talos"])
    pw = np.array([p.reshape(99) for (_, p, _, _, _, _, _)
                   in ls.synthetic_frames(50)], dtype=np.float64)
    a = R.compute_targets(rt, pw, 1.0 / 30.0, use_cpp=True)
    b = R.compute_targets(rt, pw, 1.0 / 30.0, use_cpp=False)
    assert a.shape == (50, 9)
    assert np.max(np.abs(a - b)) < 1e-5


def test_retarget_synthetic_bounded(tmp_path):
    """End-to-end: retarget a synthetic stream headless -> a bounded-error Talos
    trajectory + a policy-kickstart dataset that loads."""
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
    """Live teleop: --follow tails a stream a *concurrent* writer is still producing,
    retargets each frame as it arrives, and writes the dataset on stream-idle exit.
    A background thread plays the writer (perceive's role)."""
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
    """Cold start: the producer takes longer than --idle-timeout to write its FIRST
    frame (a fresh perceive's env activation + model load is several seconds). The
    consumer must wait through the startup grace, not give up -- the exact run-book
    flow ("start the consumer first"). Regression for the idle-vs-startup split."""
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


def test_visual_meshes_load(tmp_path):
    """Job 1: the real URDF link meshes load for Rerun (Asset3D-able STL paths) and
    every visual geom gets a world placement from FK -- for both Talos and G1."""
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
    """Mode routing: bare (no file mode) -> live tf; --replay -> replay; --follow ->
    follow. Guards that tf is the default source without needing a live ROS graph."""
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

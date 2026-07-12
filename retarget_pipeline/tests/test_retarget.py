"""
test_retarget -- headless smoke for the retargeting process (Process B).

Runs in the ``wbc`` env (needs pinocchio). Auto-skips elsewhere, so the default
suite is unaffected. No ROS, no camera, no display, no network -- it retargets a
tiny synthetic landmark stream written on the fly.
"""
import os
import sys

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

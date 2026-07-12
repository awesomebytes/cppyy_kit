#!/usr/bin/env python3
"""Smoke + parity tests for cv_kit/demos/webcam_demo.py (the M6b live webcam demo).

These need the pixi ``vision`` env (OpenCV C++ headers, cv2, rerun-sdk); the whole
module auto-skips when they are absent, so the default ``pixi run test`` is
unaffected. Run the real thing with ``pixi run -e vision test-vision``.

Everything here uses the **synthetic** source (a deterministic moving scene, no
camera) and runs **headless** (RCLCPPYY_RERUN_SPAWN=0), and is deadline-bounded, so
it is safe in CI / on a rehearsal laptop with no webcam. Covered:
  * the two pipelines (A = cppyy_kit C++ kernel, B = naive Python) compute the same
    NCC flow and the same motion on the same frames (correctness parity);
  * A is faster than B (the whole point) via the bench path;
  * the live loop runs synthetic + headless within a deadline and writes its .rrd;
  * the source fallback (bad webcam device under ``--source auto`` -> synthetic).
"""
import glob
import importlib.util
import os
import sys
import time

import numpy as np
import pytest

_CONDA = os.environ.get("CONDA_PREFIX", "")
_HAVE_CV = bool(glob.glob(os.path.join(_CONDA, "include", "opencv4")))
_HAVE_CV2 = importlib.util.find_spec("cv2") is not None
_HAVE_RERUN = importlib.util.find_spec("rerun") is not None
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))   # cv_kit/tests/ -> repo root

pytestmark = pytest.mark.skipif(
    not (_HAVE_CV and _HAVE_CV2 and _HAVE_RERUN),
    reason="vision env not present (OpenCV/cv2/rerun) -- use pixi run -e vision test-vision")

if _HAVE_CV and _HAVE_CV2 and _HAVE_RERUN:
    os.environ.setdefault("ROS_DOMAIN_ID", "62")
    sys.path.insert(0, os.path.join(_REPO, "cv_kit", "demos"))
    sys.path.insert(0, os.path.join(_REPO, "scripts", "datasets"))
    import webcam_demo as WD


@pytest.fixture(scope="module")
def frames():
    # 8 small frames with genuine inter-frame motion (panning crop of the canvas).
    return WD._synth_frames(320, 240, 8)


def test_pipelines_agree_on_flow_and_motion(frames):
    """A (C++ kernel) and B (naive Python) must compute the same tracker output on
    the same frames -- the honesty contract behind the A-vs-B comparison."""
    a = WD.VoTrackerCpp(120, 3, 5, 0.3)
    b = WD.VoTrackerPy(120, 3, 5, 0.3)
    compared = 0
    for i, f in enumerate(frames):
        ma, pa, fa = a.process(f)
        mb, pb, fb = b.process(f)
        if i < 2 or pa is None or pb is None:
            continue
        compared += 1
        # Same keypoints tracked (same detector under both), same coordinates.
        assert pa.shape == pb.shape
        assert np.allclose(pa, pb, atol=1e-3)
        # Same integer flow at the vast majority of keypoints (a handful may differ
        # by +-1px where two offsets have near-equal NCC and the float summation
        # order breaks the tie differently -- documented, not a bug).
        agree = np.mean(np.all(fa[:, :2] == fb[:, :2], axis=1))
        assert agree >= 0.9
        # The estimated motion is effectively identical.
        if ma is not None and mb is not None:
            assert abs(ma["dx"] - mb["dx"]) < 0.5
            assert abs(ma["dy"] - mb["dy"]) < 0.5
    assert compared >= 3


def test_bench_A_faster_than_B():
    """The bench path runs both pipelines and A must come out clearly ahead (the NCC
    stage is a custom kernel with no cv2 one-liner, so B loops in Python)."""
    rows = WD.run_bench([(320, 240)], nfeatures=120, track_points=120,
                        patch_r=3, search_s=5, min_score=0.3, n=15, warm=3)
    assert len(rows) == 1
    r = rows[0]
    assert r["tracked"] > 10
    assert r["a_ms"] > 0 and r["b_ms"] > 0
    # A is meaningfully faster; keep the assertion conservative (>2x) so a busy CI
    # box never flakes, though the measured gap is ~12-16x.
    assert r["speedup"] > 2.0


def test_live_synthetic_headless_bounded(tmp_path):
    """The live loop runs on the synthetic source, headless, within a deadline, and
    writes its .rrd -- the CI/rehearsal path (no camera, no window, no ROS)."""
    os.environ["RCLCPPYY_RERUN_SPAWN"] = "0"    # force headless
    rrd = str(tmp_path / "webcam_demo_test.rrd")
    t0 = time.monotonic()
    WD.main(["--source", "synthetic", "--duration", "2.0", "--no-ros",
             "--track-points", "40", "--rrd", rrd])
    elapsed = time.monotonic() - t0
    # duration 2 s + one-time bringup/warmup; must not hang.
    assert elapsed < 90.0
    assert os.path.isfile(rrd)
    assert os.path.getsize(rrd) > 0


def test_source_fallback_to_synthetic():
    """``--source auto`` with an unopenable webcam device must fall back to the
    synthetic scene rather than raise (stage robustness)."""
    src, note = WD.open_source("auto", device=999, width=320, height=240, synth_n=50)
    try:
        assert isinstance(src, WD.SyntheticSource)
        assert "synthetic" in note
        ok, frame = src.read()
        assert ok and frame is not None and frame.ndim == 2
    finally:
        src.close()


def test_webcam_source_open_raises_when_forced():
    """``--source webcam`` on a bad device raises (so the operator sees it), unlike
    the auto path which falls back."""
    with pytest.raises(Exception):
        WD.WebcamSource(device=999, width=320, height=240)

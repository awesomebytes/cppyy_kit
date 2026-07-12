"""
test_perceive -- headless smoke for the perception process (Process A).

Runs in the ``pipeline`` env (needs rerun for the viz import; mediapipe/webcam are
NOT needed -- the synthetic scene drives everything). Auto-skips elsewhere, so the
default suite is unaffected. No live camera, no network, no display.
"""
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

pytest.importorskip("rerun", reason="perception viz needs rerun (pipeline env)")

os.environ.setdefault("ROS_DOMAIN_ID", "62")
os.environ["RCLCPPYY_RERUN_SPAWN"] = "0"        # never pop a window under test

from retarget_pipeline import landmark_stream as ls  # noqa: E402
from retarget_pipeline import perceive               # noqa: E402


def test_synthetic_record_headless_roundtrips(tmp_path):
    """Synthetic live run with no camera/model/ROS/viz writes a stream that
    round-trips -- the record+replay-from-day-one guarantee, headless."""
    path = str(tmp_path / "rec.jsonl")
    perceive.main(["--source", "synthetic", "--duration", "0.4", "--fps", "30",
                   "--no-ros", "--no-viz", "--record", path])
    frames = ls.StreamReader(path).read_all()
    assert len(frames) >= 3
    assert frames[0]["pose_world"] is not None


def test_presence_gate():
    """Job 3: the presence gate. A high-visibility pose is present; no pose, or a
    below-threshold pose, is absent (so no phantom /tf); threshold 0 disables it."""
    pw = np.zeros((ls.N_POSE, 3), dtype=np.float32)
    hi = np.ones((ls.N_POSE, 4), dtype=np.float32)          # visibility col (3) = 1.0
    lo = np.ones((ls.N_POSE, 4), dtype=np.float32)
    lo[:, 3] = 0.1
    assert perceive._pose_present({"pose_world": pw, "pose_image": hi}, 0.5) is True
    assert perceive._pose_present({"pose_world": pw, "pose_image": lo}, 0.5) is False
    assert perceive._pose_present({"pose_world": pw, "pose_image": lo}, 0.0) is True
    assert perceive._pose_present({"pose_world": None, "pose_image": hi}, 0.5) is False


def test_landmarks_to_xyz_maps_pose_and_hands():
    pw, _ = ls.synthetic_pose(0.5)
    lm = {"pose_world": pw, "left_hand": None, "right_hand": None}
    xyz = perceive.landmarks_to_xyz(lm, with_hands=True)
    assert xyz.shape == (ls.N_POSE + 2 * ls.N_HAND, 3)
    # pose rows are the robot-frame conversion of the pose; hand rows stay zero
    assert not np.allclose(xyz[:ls.N_POSE], 0.0)
    assert np.allclose(xyz[ls.N_POSE:], 0.0)


def test_tf_build_bench_cpp_beats_python(tmp_path, capsys):
    """The cppyy_kit glue win: building the /tf message in C++ beats the per-field
    Python loop. Brings up rclcpp + the cppdef broadcaster (needs cppyy/ROS)."""
    pytest.importorskip("cppyy")
    try:
        import rclcpp_kit  # noqa: F401
    except Exception:
        pytest.skip("rclcpp_kit unavailable")

    class Args:
        replay = None
        bench_n = 30
    perceive.run_bench(Args())
    out = capsys.readouterr().out
    assert "A speedup:" in out
    # parse the speedup and require the C++ builder to win
    speedup = float(out.split("A speedup:")[1].split("x")[0])
    assert speedup > 1.0

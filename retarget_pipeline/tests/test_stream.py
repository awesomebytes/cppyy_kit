"""
test_stream -- the record/replay contract. Pure: stdlib + numpy only, so it
runs in any env (it is the seam both processes depend on). No ROS / mediapipe / cppyy.
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

from retarget_pipeline import landmark_stream as ls  # noqa: E402


def test_synthetic_pose_shapes_and_determinism():
    pw, pi = ls.synthetic_pose(1.234)
    assert pw.shape == (ls.N_POSE, 3)
    assert pi.shape == (ls.N_POSE, 4)
    pw2, _ = ls.synthetic_pose(1.234)
    assert np.allclose(pw, pw2)                       # deterministic
    pw3, _ = ls.synthetic_pose(2.0)
    assert not np.allclose(pw[ls.LEFT_WRIST], pw3[ls.LEFT_WRIST])  # arms move


def test_write_read_roundtrip(tmp_path):
    path = str(tmp_path / "s.jsonl")
    n = 20
    with ls.StreamWriter(path, source="synthetic", fps_target=30.0) as w:
        for (t, pw, pi, lh, rh, ww, hh) in ls.synthetic_frames(n):
            w.write(t=t, pose_world=pw, pose_image=pi, left_hand=lh,
                    right_hand=rh, w=ww, h=hh)
        assert w.n_written == n

    reader = ls.StreamReader(path)
    assert reader.meta["format"] == ls.FORMAT
    assert reader.meta["source"] == "synthetic"
    frames = reader.read_all()
    assert len(frames) == n
    # values survive the JSON round-trip (within the 5-decimal rounding)
    _, pw0, pi0, _, _, _, _ = next(iter(ls.synthetic_frames(1)))
    assert frames[0]["pose_world"].shape == (ls.N_POSE, 3)
    assert np.allclose(frames[0]["pose_world"], pw0, atol=1e-4)
    assert np.allclose(frames[0]["pose_image"], pi0, atol=1e-4)
    assert frames[0]["present"] is True


def test_missing_parts_are_none(tmp_path):
    path = str(tmp_path / "s.jsonl")
    with ls.StreamWriter(path, source="test") as w:
        w.write(t=0.0, pose_world=None)               # nothing detected
    fr = ls.StreamReader(path).read_all()[0]
    assert fr["pose_world"] is None
    assert fr["left_hand"] is None
    assert fr["present"] is False


def test_rejects_foreign_file(tmp_path):
    path = str(tmp_path / "bad.jsonl")
    with open(path, "w") as f:
        f.write('{"kind":"meta","format":"something.else"}\n')
    with pytest.raises(ValueError):
        ls.StreamReader(path)


def test_follow_tails_a_growing_file(tmp_path):
    path = str(tmp_path / "live.jsonl")
    n = 15

    def writer():
        with ls.StreamWriter(path, source="synthetic") as w:
            for (t, pw, pi, lh, rh, ww, hh) in ls.synthetic_frames(n):
                w.write(t=t, pose_world=pw, pose_image=pi, w=ww, h=hh)
                time.sleep(0.01)

    th = threading.Thread(target=writer)
    th.start()
    got = [fr["i"] for fr in ls.follow(path, idle_timeout=1.0, poll=0.005)]
    th.join()
    assert got == list(range(n))                      # every frame, in order


def test_world_to_robot_frame():
    # A point 1 m toward the camera + 1 m to subject-left + 1 m down (MediaPipe)
    # -> 1 m back + 1 m left + 1 m down in robot frame (x fwd, y left, z up).
    pts = np.array([[1.0, 1.0, 1.0]], dtype=np.float32)  # (mp_x, mp_y, mp_z)
    out = ls.mediapipe_world_to_robot(pts)[0]
    assert np.allclose(out, [-1.0, 1.0, -1.0])

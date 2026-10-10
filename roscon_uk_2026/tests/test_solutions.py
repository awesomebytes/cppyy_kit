"""Synthetic checks run without ROS middleware, camera hardware, or downloads."""
import hashlib
import importlib.util
import json
from pathlib import Path
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]


def load(path):
    spec = importlib.util.spec_from_file_location(path.stem,path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("hold",[0.,.02,.12])
def test_saved_pose_queries(hold):
    baseline = load(ROOT / "solutions/python_baseline.py")
    native = load(ROOT / "solutions/python_native.py")
    mcap = load(ROOT / "solutions/mcap_native.py")
    ros = load(ROOT / "solutions/ros_native.py")
    rng = np.random.default_rng(66)
    t = np.r_[0., np.cumsum(rng.uniform(.005,.04,100))]
    states = np.cumsum(rng.normal(0,.002,(101,12)),axis=0)
    thresholds = np.array([0.,.02,.08,.2])
    # Migrated ConstNDArray inputs must accept already contiguous read-only data.
    for value in (t, states, thresholds):
        value.flags.writeable = False
    counts, seconds = mcap.sweep(t,states,thresholds,hold)
    online_counts, online_seconds = ros.rolling_query(t,states,thresholds,hold)
    np.testing.assert_array_equal(online_counts,counts)
    np.testing.assert_allclose(online_seconds,seconds)
    for k, threshold in enumerate(thresholds):
        for hand, offset in enumerate((0,6)):
            xyz = states[:,offset:offset+3]
            expected = baseline.motion_mask(t,xyz,threshold,threshold/2,hold)
            actual = native.motion_mask(t,xyz,threshold,threshold/2,hold)
            np.testing.assert_array_equal(actual,expected)
            assert counts[k,hand] == np.count_nonzero(np.diff(np.r_[0,expected])==1)
            assert seconds[k,hand] == pytest.approx(np.sum(np.diff(t)*expected[1:]))


def test_saved_tracker_against_independent_search():
    tracker = load(ROOT / "solutions/webcam_native.py")
    rng = np.random.default_rng(7)
    for _ in range(8):
        a = rng.integers(0,256,(30,40),dtype=np.uint8)[:,::2]
        b = rng.integers(0,256,a.shape,dtype=np.uint8)
        x,y,r,s = 10,15,2,3
        template = a[y-r:y+r+1,x-r:x+r+1].astype(np.int64)
        candidates=[]
        for cy in range(y-s,y+s+1):
            for cx in range(x-s,x+s+1):
                patch = b[cy-r:cy+r+1,cx-r:cx+r+1].astype(np.int64)
                candidates.append((int(np.sum((patch-template)**2)),cy,cx))
        score,cy,cx = min(candidates)
        assert tracker.track(a,b,x,y,r,s) == (cx,cy)


def test_generated_source_provenance():
    provenance = json.loads((ROOT / "evaluation/provenance.json").read_text())
    for name, record in provenance["solutions"].items():
        # The 3 October measurements refer to these exact evaluated sources.
        # Current runnable solutions use the later checked NumPy annotations.
        archived = ROOT / f"solutions/evaluated_2026_10_03/{name}.py"
        digest = hashlib.sha256(archived.read_bytes()).hexdigest()
        assert digest == record["sha256"]
        run = json.loads((ROOT / f"evaluation/{record['run']}/report.json").read_text())
        assert digest == run["task_sha256"]


def test_presentation_contains_exact_prompts():
    text = (ROOT / "DEEP_DIVE_PRESENTATION.md").read_text()
    for prompt in (ROOT / "prompts").glob("*.txt"):
        assert prompt.read_text().strip() in text, prompt.name

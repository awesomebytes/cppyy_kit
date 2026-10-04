"""Native virtual dispatch, ownership, and independent continuous geometry."""
import gc
import glob
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import weakref

import pytest

_HAVE_OMPL = bool(glob.glob(os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                         "include", "ompl-*")))
pytestmark = pytest.mark.skipif(not _HAVE_OMPL, reason="OMPL not installed")
DEMO = Path(__file__).parents[1] / "demos" / "d03_validity_checker.py"


@pytest.fixture(scope="module")
def demo():
    spec = importlib.util.spec_from_file_location("ompl_validity_demo", DEMO)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _decisions(demo, si, points):
    state = si.allocState()
    try:
        concrete = demo.ompl_kit.as_state(state, demo.ob.RealVectorStateSpace.StateType)
        results = []
        for x, y in points:
            concrete[0], concrete[1] = x, y
            results.append(bool(si.isValid(state)))  # Native virtual dispatch.
        return results
    finally:
        si.freeState(state)


@pytest.mark.parametrize("bias", [0.0, 0.05, -0.05])
def test_python_native_decisions(demo, bias):
    center = 0.5 - bias
    points = [(0.1, 0.1), (center, 0.5), (center, 0.75), (0.9, 0.9),
              (0.0, 0.0), (1.0, 1.0), (-0.1, 0.5), (1.1, 0.5),
              (math.nan, 0.5), (0.5, math.inf), (0.5, -math.inf)]
    expected = [True, False, False, True, True, True, False, False,
                False, False, False]
    for kind in ("python", "native"):
        with demo.planning_setup(kind, bias) as (ss, checker):
            before = checker.calls
            assert _decisions(demo, ss.getSpaceInformation(), points) == expected
            assert checker.calls - before == len(points)


@pytest.mark.parametrize("bias", [math.nan, math.inf, -math.inf])
def test_invalid_bias(demo, bias):
    with pytest.raises(ValueError, match="bias must be finite"):
        with demo.planning_setup(bias=bias):
            pytest.fail("invalid bias was accepted")
    # Constructor overload resolution reports the native invalid_argument
    # inside a TypeError when no other constructor overload succeeds.
    with pytest.raises(TypeError, match="invalid_argument: bias must be finite"):
        with demo.planning_setup("native", bias):
            pytest.fail("invalid bias was accepted")


@pytest.mark.parametrize("operation,fail_at", [("check", 2), ("solve", 7)])
def test_callback_error_detaches_before_release(demo, operation, fail_at):
    with pytest.raises(ValueError, match="scripted validity failure"):
        with demo.planning_setup(fail_at=fail_at) as (ss, checker):
            ref = weakref.ref(checker)
            si = ss.getSpaceInformation()
            if operation == "check":
                _decisions(demo, si, [(0.1, 0.1), (0.9, 0.9)])
            else:
                ss.solve(1.0)
    assert checker.calls == fail_at
    # Retained SI now holds an all-valid checker, so no Python dispatch occurs.
    assert _decisions(demo, si, [(0.5, 0.5)]) == [True]
    assert checker.calls == fail_at
    del checker
    gc.collect()
    assert ref() is None


def test_checker_survives_gc_and_is_released_after_detach(demo):
    for _ in range(5):
        with demo.planning_setup() as (ss, checker):
            ref = weakref.ref(checker)
            before = checker.calls
            del checker
            gc.collect()
            si = ss.getSpaceInformation()
            assert _decisions(demo, si, [(0.1, 0.1), (0.5, 0.5)]) == [True, False]
            assert ref().calls == before + 2
        gc.collect()
        assert ref() is None
        assert _decisions(demo, si, [(0.5, 0.5)]) == [True]


def _minimum_segment_distance(points, center):
    """Independent projection oracle; checks all points on every line segment."""
    distances = []
    for a, b in zip(points, points[1:]):
        vector = tuple(end - start for start, end in zip(a, b))
        squared_length = sum(value * value for value in vector)
        offset = tuple(c - start for c, start in zip(center, a))
        fraction = (sum(v * o for v, o in zip(vector, offset)) / squared_length
                    if squared_length else 0.0)
        fraction = min(1.0, max(0.0, fraction))
        closest = tuple(start + fraction * v for start, v in zip(a, vector))
        distances.append(math.dist(closest, center))
    return min(distances)


def test_coarse_motion_accepts_segment_that_independent_oracle_rejects(demo):
    # Length 0.01 is below 0.01 * sqrt(2), so coarse OMPL checks endpoints only.
    points = [(0.495, 0.74999), (0.505, 0.74999)]
    assert all(math.dist(point, (0.5, 0.5)) > 0.25 for point in points)
    assert _minimum_segment_distance(points, (0.5, 0.5)) < 0.25
    for resolution, accepted in [(0.01, True), (0.001, False)]:
        with demo.planning_setup(bias=0.0, resolution=resolution) as (ss, _):
            si = ss.getSpaceInformation()
            a = demo.ob.ScopedState[demo.ob.RealVectorStateSpace](ss.getStateSpace())
            b = demo.ob.ScopedState[demo.ob.RealVectorStateSpace](ss.getStateSpace())
            a[0], a[1] = points[0]
            b[0], b[1] = points[1]
            assert bool(si.checkMotion(a.get(), b.get())) is accepted
    with pytest.raises(AssertionError, match="segment intersects keep-out circle"):
        demo.assert_safe_path(points, bias=0.0)


@pytest.mark.parametrize("kind", ["python", "native"])
def test_native_planner_solution_has_safe_complete_segments(kind):
    # Each child seeds before any sampler exists. No exact random path is asserted.
    process = subprocess.run([sys.executable, str(DEMO), "--kind", kind],
                             capture_output=True, text=True, timeout=30)
    assert process.returncode == 0, process.stdout + process.stderr
    result = json.loads(process.stdout.splitlines()[-1])
    assert result["exact"] and result["calls"] > 0
    points = result["points"]
    assert points[0] == pytest.approx((0.1, 0.1))
    assert points[-1] == pytest.approx((0.9, 0.9))
    assert all(math.isfinite(v) and 0 <= v <= 1 for point in points for v in point)
    assert _minimum_segment_distance(points, (0.45, 0.5)) > 0.25

"""Real planner, policy parity, error propagation, and detach tests in children."""
from argparse import Namespace
from pathlib import Path

import pytest

from check import check_path, check_results
from demo import run_child


@pytest.mark.parametrize("bias", [0.0, 0.05, -0.05])
def test_native_engine_extension(bias):
    args = Namespace(policy=str(Path(__file__).with_name("policy.py")),
                     repeats=100, repetitions=2, seed=41, bias=bias)
    results = [run_child(args, kind) for kind in ["python", "native", "lifecycle"]]
    check_results(results)


def test_missing_policy_is_detected():
    args = Namespace(policy=str(Path(__file__).with_name("policy_skeleton.py")),
                     repeats=1, repetitions=1, seed=41, bias=0.05)
    with pytest.raises(RuntimeError, match="implement the existing OMPL validity policy"):
        run_child(args, "python")


def test_coarse_motion_check_is_detected():
    args = Namespace(policy=str(Path(__file__).with_name("policy.py")),
                     repeats=1, repetitions=1, seed=41, bias=0.0, resolution=0.01)
    result = run_child(args, "native")
    assert result["exact"], "fixture no longer reproduces a discretely accepted path"
    with pytest.raises(AssertionError, match="segment intersects keep-out circle"):
        check_path(result)

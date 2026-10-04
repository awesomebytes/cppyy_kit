#!/usr/bin/env python3
"""Implement OMPL's existing validity virtual and check complete path segments.

Repository command: pixi run -e ompl python ompl_kit/demos/d03_validity_checker.py
The Python checker receives a borrowed State pointer for the current call only.
"""
import argparse
from contextlib import contextmanager
import json
import math

import cppyy
import ompl_kit

ob, og = ompl_kit.bringup_ompl()


class CircleChecker(ob.StateValidityChecker):
    """A Python override called by native SpaceInformation and planner code."""

    def __init__(self, si, bias=0.05, fail_at=None):
        super().__init__(si)
        self.bias = float(bias)
        if not math.isfinite(self.bias):
            raise ValueError("bias must be finite")
        self.fail_at = fail_at
        self.calls = 0

    def isValid(self, state):
        self.calls += 1
        if self.calls == self.fail_at:
            raise ValueError("scripted validity failure")
        # Copy coordinates now. Never retain or free this borrowed State pointer.
        x, y = float(state[0]), float(state[1])
        if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
            return False
        return (x + self.bias - 0.5) ** 2 + (y - 0.5) ** 2 > 0.25 ** 2


def native_checker():
    """Define the equivalent C++ checker once; this is demo code, not a kit API."""
    if not hasattr(cppyy.gbl, "ompl_callback_demo"):
        cppyy.cppdef(r"""
        #include <cmath>
        #include <stdexcept>
        namespace ompl_callback_demo {
        class CircleChecker : public ompl::base::StateValidityChecker {
        public:
            mutable unsigned long calls = 0;
            double bias;
            CircleChecker(const ompl::base::SpaceInformationPtr& si, double b)
                : ompl::base::StateValidityChecker(si), bias(b) {
                if (!std::isfinite(b))
                    throw std::invalid_argument("bias must be finite");
            }
            bool isValid(const ompl::base::State* state) const override {
                ++calls;
                const auto* s = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const double x = (*s)[0], y = (*s)[1];
                if (!std::isfinite(x) || !std::isfinite(y) ||
                    x < 0 || x > 1 || y < 0 || y > 1) return false;
                const double dx = x + bias - 0.5, dy = y - 0.5;
                return dx * dx + dy * dy > 0.25 * 0.25;
            }
        };
        }
        """)
    return cppyy.gbl.ompl_callback_demo.CircleChecker


@contextmanager
def planning_setup(kind="python", bias=0.05, resolution=0.001, fail_at=None):
    """Hold the checker proxy until native dispatch has been detached.

    The returned setup and checker are for synchronous use inside this context.
    Do not retain the checker, mutate it during dispatch, or reuse a failed solve.
    """
    if not math.isfinite(resolution) or not 0 < resolution <= 1:
        raise ValueError("resolution must be finite and in (0, 1]")
    if kind not in ("python", "native"):
        raise ValueError("kind must be python or native")
    if kind == "native" and fail_at is not None:
        raise ValueError("scripted failures require the Python checker")
    space = ob.RealVectorStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0.0)
    bounds.setHigh(1.0)
    space.setBounds(bounds)
    ss = og.SimpleSetup(ob.StateSpacePtr(space))
    si = ss.getSpaceInformation()
    si.setStateValidityCheckingResolution(resolution)
    checker = (CircleChecker(si, bias, fail_at) if kind == "python"
               else native_checker()(si, bias))
    # The local checker reference pins Python's dispatcher for the whole context.
    # The shared_ptr owns the C++ object; it does not keep Python's override alive.
    ss.setStateValidityChecker(ob.StateValidityCheckerPtr(checker))
    try:
        start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
        goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
        start[0], start[1] = 0.1, 0.1
        goal[0], goal[1] = 0.9, 0.9
        ss.setStartAndGoalStates(start, goal)
        planner = og.RRTConnect(si)
        planner.setRange(0.1)
        ss.setPlanner(ob.PlannerPtr(planner))
        ss.setup()
        yield ss, checker
    finally:
        # Clear planner state and detach before releasing the Python reference.
        ss.clear()
        ss.setStateValidityChecker(ob.StateValidityCheckerPtr(
            ob.AllValidStateValidityChecker(si)))


def assert_safe_path(points, bias=0.05):
    """Check continuous line segments independently of OMPL's sampled validator.

    This oracle applies only to this 2D square, circular obstacle, and straight
    interpolation. Other state spaces and obstacle models need their own oracle.
    """
    if len(points) < 2:
        raise AssertionError("path needs at least two points")
    center = (0.5 - bias, 0.5)
    for point in points:
        if not all(math.isfinite(v) and 0.0 <= v <= 1.0 for v in point):
            raise AssertionError("path leaves unit square")
    minimum = math.inf
    for a, b in zip(points, points[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_squared = dx * dx + dy * dy
        projection = ((center[0] - a[0]) * dx + (center[1] - a[1]) * dy)
        t = max(0.0, min(1.0, projection / length_squared)) if length_squared else 0.0
        distance = math.hypot(a[0] + t * dx - center[0],
                              a[1] + t * dy - center[1])
        minimum = min(minimum, distance)
        if distance <= 0.25:
            raise AssertionError("segment intersects keep-out circle")
    return minimum


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--kind", choices=("python", "native"), default="python")
    parser.add_argument("--bias", type=float, default=0.05)
    parser.add_argument("--resolution", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=41)
    args = parser.parse_args()
    ompl_kit.set_seed(args.seed)  # Call before constructing a sampling planner.
    with planning_setup(args.kind, args.bias, args.resolution) as (ss, checker):
        status = ss.solve(1.0)
        if status != ob.PlannerStatus.EXACT_SOLUTION:
            raise RuntimeError("planner did not return an exact solution")
        points = ompl_kit.path_to_list(ss.getSolutionPath(), dim=2)
        minimum = assert_safe_path(points, args.bias)
        print(json.dumps({"kind": args.kind, "exact": True, "calls": checker.calls,
                          "minimum_segment_distance": minimum, "points": points}))


if __name__ == "__main__":
    main()

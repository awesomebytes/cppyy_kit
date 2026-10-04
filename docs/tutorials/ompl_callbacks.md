# Implement a Python OMPL validity checker

Plan a path through a unit square while excluding a circular obstacle. Implement
OMPL's existing `StateValidityChecker::isValid` virtual in Python, let native
RRTConnect call it, and check every returned line segment against the circle.
The same example also supplies an equivalent C++ checker for correctness checks.

## Setup and run

These commands require this repository checkout and run from its root:

```bash
pixi install --locked -e ompl
pixi run --locked -e ompl python ompl_kit/demos/d03_validity_checker.py
pixi run --locked -e ompl python ompl_kit/demos/d03_validity_checker.py --kind native
pixi run --locked -e ompl pytest ompl_kit/tests/test_validity_checker.py -q
```

Each demo process seeds OMPL before constructing a sampling planner. It plans
from `(0.1, 0.1)` to `(0.9, 0.9)`. The circle has radius `0.25` and center
`(0.45, 0.5)` at the default bias `0.05`. Coordinates are dimensionless. Expected
output is a JSON object with `exact: true`, a positive `calls` count, waypoints,
and `minimum_segment_distance` greater than `0.25`. OMPL also writes planner
status messages. The example raises if planning fails or any segment intersects
the circle. See the complete [demo](https://github.com/awesomebytes/cppyy_kit/blob/main/ompl_kit/demos/d03_validity_checker.py).

These are repository demo commands. Application code uses the installed
`ompl_kit` package and OMPL dependencies in its own environment; it does not need
the demo module or experiment files.

## Override the existing native virtual

OMPL asks the application to define state validity. Its checker has the native
signature `bool isValid(const ompl::base::State*) const`. Derive from the real
class and define `isValid` in the Python class body. Chain the base constructor
with `super().__init__(si)`. cppyy generates the virtual dispatcher. Both
`SpaceInformation.isValid` and compiled planner code then invoke this override.
See [OMPL state validation](https://ompl.kavrakilab.org/stateValidation.html) and
[cppyy cross-inheritance](https://cppyy.readthedocs.io/en/latest/classes.html#cross-inheritance).

```python
import math
import ompl_kit

ob, og = ompl_kit.bringup_ompl()

class CircleChecker(ob.StateValidityChecker):
    def __init__(self, si, bias=0.05):
        super().__init__(si)
        self.bias = float(bias)
        if not math.isfinite(self.bias):
            raise ValueError("bias must be finite")

    def isValid(self, state):
        x, y = float(state[0]), float(state[1])
        if not (0 <= x <= 1 and 0 <= y <= 1):
            return False
        return (x + self.bias - 0.5)**2 + (y - 0.5)**2 > 0.25**2
```

Positive bias shifts the circle center left. The circle boundary, points outside
the square, and nonfinite coordinates are invalid. Constructors reject nonfinite
bias. The Python constructor raises `ValueError`; cppyy reports the native
constructor's `std::invalid_argument` inside an overload-resolution `TypeError`
in the tested environment. Native and Python implementations check the same
fixed points in the tests.
The native checker is a concrete subclass of the same OMPL interface, compiled
with `cppyy.cppdef`. The example measures no performance difference.

`state` is a borrowed pointer valid only during this callback. Copy coordinates
or another owned representation before returning. Do not store the pointer or
free its memory. Reading `state[0]` and `state[1]` works here because cppyy
downcasts the real-vector state using RTTI. Other state spaces require their own
state access; see `ompl_kit.as_state` for an explicit downcast.

## Hold the Python override through native dispatch

OMPL receives `ob.StateValidityCheckerPtr(checker)`. That shared pointer owns the
C++ object, but does not retain the Python proxy that implements its virtual.
An application that keeps its setup across calls can pin the proxy explicitly:

```python
import cppyy_kit

# ss is an already configured og.SimpleSetup.
checker = CircleChecker(ss.getSpaceInformation())
cppyy_kit.keep_alive(ss, checker)
ss.setStateValidityChecker(ob.StateValidityCheckerPtr(checker))
```

The complete demo instead holds a strong local reference inside the
`planning_setup` context manager. On exit, including an exception, it clears the
planner and replaces the checker with `AllValidStateValidityChecker` before
releasing that reference. Use its setup and checker only inside the context.
A retained `SpaceInformation` has the replacement checker after exit; the old
Python policy must never be used again. The tests force garbage collection while
the context is open, verify native dispatch still succeeds, then verify the
Python proxy is released after detach. They also confirm no further Python
dispatch occurs through a retained `SpaceInformation`.

A Python `ValueError` raised inside the override reaches the Python caller of
both `SpaceInformation.isValid` and `RRTConnect.solve` in the tested environment.
The tests assert its type, message, and exact callback count. Treat the failed
solve as finished and recreate the setup. The demo does not convert exceptions
to an invalid-state result. See [cppyy exceptions](https://cppyy.readthedocs.io/en/latest/exceptions.html).

All operations here are synchronous. Python callbacks hold the GIL. The example
does not create native workers or release the GIL on the enclosing call. Do not
mutate the checker or close its context during native dispatch. A concurrent
planner requires a checker that meets OMPL's thread safety contract.

## Check complete segments, including near tangencies

Valid waypoints do not establish that the lines joining them are valid. OMPL's
default motion validator checks intermediate states at a finite resolution.
The resolution is a fraction of the state space's maximum extent. Sampling can
miss a short crossing of an obstacle. OMPL documents this limitation in its
[motion validation reference](https://ompl.kavrakilab.org/stateValidation.html).

The demo uses resolution `0.001` and independently projects the circle center
onto every complete path segment. It checks that each closest distance is
strictly greater than `0.25`. The test suite computes these distances separately
from the demo's check. Because the square is convex, bounded endpoints also
ensure each straight segment stays within it. This analytic oracle applies to
this circular obstacle and real-vector interpolation; other geometry needs its
own continuous validation.

A preceding OMPL 1.7.0 experiment recorded an exact RRTConnect solution at seed
`41`, bias `0`, radius `0.25`, and resolution `0.01` whose closest segment distance
was `0.2499880425838933`. Vertex checks had accepted the path. Planner paths also
depend on the order of RNG construction, so that seed alone does not guarantee
the same crossing in this minimal demo.

The regression therefore reproduces the sampling failure with a fixed motion,
without relying on a particular random path. The segment from `(0.495, 0.74999)`
to `(0.505, 0.74999)` has valid endpoints outside the circle centered at
`(0.5, 0.5)`. Its closest distance is `0.24999`. At resolution `0.01`, the length
`0.01` is below the checking interval `0.01 * sqrt(2)`, so the native validator
accepts it. The analytic oracle rejects it, and resolution `0.001` samples the
crossing and rejects this particular motion. Run the deterministic reproduction:

```bash
# Requires this repository checkout.
pixi run --locked -e ompl pytest ompl_kit/tests/test_validity_checker.py -q -k coarse
```

Expected result is one passing test that verifies both the coarse acceptance and
the independent rejection. Smaller resolution improves this case; it does not
prove continuous safety for arbitrary geometry. Use an appropriate continuous
motion validator when the application requires that guarantee.

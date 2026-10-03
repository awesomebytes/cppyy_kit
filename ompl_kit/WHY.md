# ompl_kit: use OMPL from Python via cppyy

`ompl_kit` is for planning in a state space when you want to define validity in
Python and use OMPL's sampling planners, such as RRTConnect or RRT*. The C++
library owns the state spaces and planner; your Python supplies the start, goal,
and state-validity function the planner calls while searching.

The kit uses cppyy to read the installed OMPL headers and expose OMPL's C++
classes and templates to Python. Kit helpers handle Python validity callbacks,
object lifetimes, and path extraction. This avoids writing a separate Python
wrapper for each OMPL API you use.

For a first plan, run the included 2D example. From a source checkout, run this
from the `cppyy_kit` directory:

```bash
pixi run -e ompl demo-ompl-plan
```

It prints waypoints for a route around a circular obstacle. The commands here use
the source checkout's Pixi environment. For installed use, see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/)
and the published package `ros-jazzy-ompl-kit`.

For the API, see [SKILL.md](SKILL.md); for implementation evidence and detailed
limitations, see [REPORT.md](REPORT.md).

---

## A first plan: Python validity function and OMPL RRTConnect

The included example plans in a 2D unit square around a circular obstacle. The
Python function below is the state-validity checker called by OMPL's C++ planner.
The example uses OMPL's own state-space, setup, and planner classes.

### Python, `d01_first_plan.py` (ompl_kit, shipped in this repo)

```python
#!/usr/bin/env python
"""2D plan around a circular obstacle; validity checker in Python."""
import ompl_kit

ob, og = ompl_kit.bringup_ompl()

OBSTACLE = (0.5, 0.5, 0.25)

def is_state_valid(state):                       # the planner's inner-loop callback
    cx, cy, r = OBSTACLE                          # state auto-downcast -> state[i]
    return (state[0]-cx)**2 + (state[1]-cy)**2 > r**2

space = ob.RealVectorStateSpace(2)               # OMPL's own API, verbatim
bounds = ob.RealVectorBounds(2)
bounds.setLow(0.0); bounds.setHigh(1.0)
space.setBounds(bounds)

ss = og.SimpleSetup(ob.StateSpacePtr(space))
ss.setStateValidityChecker(ompl_kit.validity_checker(is_state_valid, owner=ss))

start = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
start[0], start[1] = 0.1, 0.1
goal = ob.ScopedState[ob.RealVectorStateSpace](ss.getStateSpace())
goal[0], goal[1] = 0.9, 0.9
ss.setStartAndGoalStates(start, goal)
ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))

if ss.solve(1.0):
    ss.simplifySolution()
    print(ompl_kit.path_to_list(ss.getSolutionPath(), dim=2))
```

The output is a list of waypoints; each is more than 0.25 units from the obstacle
centre, so the route passes around the circle.

---

## The same planning setup in C++

This follows the shape of OMPL's official
[geometric planning tutorial](https://ompl.kavrakilab.org/geometricPlanningSE3.html)
(`RigidBodyPlanning`). It uses the same `SimpleSetup`, validity checker,
start/goal, and RRTConnect planner as the Python example.

### C++, `plan.cpp` + `CMakeLists.txt` (official tutorial shape)

```cpp
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
  const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
  double x = (*s)[0], y = (*s)[1];
  return (x-0.5)*(x-0.5) + (y-0.5)*(y-0.5) > 0.25*0.25;   // outside the obstacle
}

int main() {
  auto space(std::make_shared<ob::RealVectorStateSpace>(2));
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0.0); bounds.setHigh(1.0);
  space->setBounds(bounds);

  og::SimpleSetup ss(space);
  ss.setStateValidityChecker(isStateValid);

  ob::ScopedState<> start(space), goal(space);
  start[0] = 0.1; start[1] = 0.1;
  goal[0]  = 0.9; goal[1]  = 0.9;
  ss.setStartAndGoalStates(start, goal);
  ss.setPlanner(std::make_shared<og::RRTConnect>(ss.getSpaceInformation()));

  if (ss.solve(1.0)) { ss.simplifySolution(); ss.getSolutionPath().print(std::cout); }
  return 0;
}
```

```cmake
cmake_minimum_required(VERSION 3.5)
project(plan)
find_package(ompl REQUIRED)
include_directories(${OMPL_INCLUDE_DIRS})
add_executable(plan plan.cpp)
target_link_libraries(plan ${OMPL_LIBRARIES})
```

To compile this C++ program, configure and link it against `libompl` with CMake,
then rebuild after edits. The Python example above uses the same OMPL classes
directly through cppyy; it does not require writing a wrapper for each class.

## Subclass an OMPL C++ class in Python

OMPL lets callers supply a validity checker or optimization objective by
subclassing a C++ base
(`ob::StateValidityChecker`, `ob::OptimizationObjective`) and override a virtual.
With cppyy you do that **in Python**:

```python
class CircleChecker(ob.StateValidityChecker):
    def __init__(self, si):
        super().__init__(si)                  # chain to the C++ base constructor
    def isValid(self, state):                 # override the C++ virtual, in Python
        return (state[0]-0.5)**2 + (state[1]-0.5)**2 > 0.25**2

ss.setStateValidityChecker(ob.StateValidityCheckerPtr(CircleChecker(si)))
```

The C++ planner calls the Python `isValid` override through its virtual interface.
This works because OMPL's relevant virtual methods are not `final`. For the
mechanics and measurements, see [REPORT.md](REPORT.md) §2–3.

---

## When to use a native C++ checker

If validity checking dominates a solve, you can define a native C++
`StateValidityChecker` with `cppyy.cppdef` while keeping the rest of the planning
setup in Python. See [REPORT.md](REPORT.md) §3 for measurements; run
`pixi run -e ompl bench-ompl` to reproduce the comparison.

---

## Two ways to use it

### Mode A: plan from Python with a Python checker
Prototype motion planning with the validity/cost logic in Python: put obstacles,
clearance, and heuristics in a plain function or subclass, iterate in seconds
against the real OMPL planners. This is `d01_first_plan.py`. Good for experimenting
with problem setups and planners where edit-run cycles matter more than raw solve
throughput.

### Mode B: plan from Python and publish to ROS 2
`ompl_kit/demos/d02_publish_path.py` plans a 2D path and publishes it as a
**C++ `nav_msgs/Path`** on a real topic via rclcppyy, directly consumable by RViz
or a navigation stack. The path message is a C++ message end to end; Python only
fills the waypoint poses. The planner and ROS 2 middleware run in the same process.

---

## Limits

ompl_kit is a v0 spike. Validity/cost checkers cross the boundary one state at a
time (no batching yet), only `RealVectorStateSpace` auto-downcasts to `state[i]`
(compound spaces need `as_state`), only RRTConnect/RRTstar headers are pre-included
(other planners are one `cppyy.include` away), only geometric planning is surfaced
(not `ompl::control`), and the global RNG can't be re-seeded mid-process (reproducible
runs use a fresh process). See [REPORT.md](REPORT.md) §5 for details.

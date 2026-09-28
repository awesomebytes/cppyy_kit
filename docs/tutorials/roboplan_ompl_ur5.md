# Comparing RoboPlan and OMPL on a UR5

This tutorial compares two ways to plan a joint-space path for the same UR5,
from the same start configuration to the same goal, against the same RoboPlan
collision scene. RoboPlan is a Pinocchio-based planning library whose `Scene`
owns the robot model and collision geometry. OMPL supplies general-purpose
sampling-based planners and lets the application provide the state-validity
checker. [RoboPlan architecture](https://roboplan.readthedocs.io/en/latest/design/architecture.html)
· [OMPL state validity](https://ompl.kavrakilab.org/core/stateValidation.html)

The aim is to check whether each planner can find a path that passes the common
validity contract, and to compare observed solve time under repeated trials.
This is a focused planner comparison, not a comparison of complete robot
software stacks.

## Setup and run

Use the comparison environment and the bundled RoboPlan UR5 model selected by
the demo. Both planners receive the same arm joints, start and goal, limits,
self-collision model, and edge-validation step size. This example adds no world
obstacle.

```bash
pixi run -e roboplan-ompl demo-ompl-roboplan
```

The command runs the comparison in
[`compare_roboplan_rrtconnect.py`](../../ompl_kit/demos/compare_roboplan_rrtconnect.py).
It prints the repeated-trial results and a solve-time ranking. Keep the machine
otherwise idle for repeatability. No timing or winner is asserted here in
advance; use the output from your run.

## Shared validity contract

RoboPlan's `Scene` exposes collision and joint-limit queries on joint vectors.
Its `SceneContext` provides the collision query with private scratch for a
consumer. The adapter must map OMPL's active arm coordinates into the same
full UR5 joint vector before asking the shared checker:

```python
from roboplan.core import SceneContext

context = SceneContext(scene)

def is_valid(q_arm):
    q_full = context.toFullJointPositions("arm", q_arm)
    return scene.isValidConfiguration(q_full) and not context.hasCollisions(q_full)
```

The OMPL kit accepts a Python validity function and pins its lifetime to the
setup object:

```python
import ompl_kit

ob, og = ompl_kit.bringup_ompl()
ss = make_ompl_setup_for_ur5(ob, start_arm, goal_arm)  # demo helper; arm bounds + metric
ss.setStateValidityChecker(ompl_kit.validity_checker(is_valid, owner=ss))
ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))
solved = bool(ss.solve(planning_time_limit_s))
```

The RoboPlan side uses its `Scene` and RRT API directly; its RRT planner calls
the scene's collision checks while exploring joint space. RoboPlan documents
the `Scene`/`JointConfiguration`/`RRT` flow in its
[sampling-based planning example](https://roboplan.readthedocs.io/en/latest/concepts/sampling_based_planning.html).

```python
from roboplan.core import JointConfiguration
from roboplan.rrt import RRT, RRTOptions

options = RRTOptions(group_name="arm", max_planning_time=planning_time_limit_s)
planner = RRT(scene, options)
start = JointConfiguration(); start.positions = start_arm
goal = JointConfiguration(); goal.positions = goal_arm
path = planner.plan(start, goal)
```

These excerpts show the public API shape; they are illustrative, not a copy of
the demo's full setup code. The runnable comparison script is the source of
truth for model loading, OMPL state-space setup, joint ordering, planner
options, seeding, and result validation.

## Reading the results

For each planner, the report should include the number of successful solves,
the number of returned paths that pass the common endpoint, limit, and
collision checks, and the median solve time over the configured trials. A path
that the planner returns but that fails the common validation is not counted
as valid. Rank solve time only among planners whose paths passed validation;
show unsuccessful or invalid results separately rather than treating them as
fast wins.

The timing covers the repeated planning trials only. It excludes model loading,
library bring-up, and an unmeasured warmup solve for each planner. OMPL's Python validity
callback also crosses Python/C++ once per state check, while RoboPlan's planner
uses its own C++ scene integration. Record validity-call counts alongside time
where available, and interpret the timing as an end-to-end result for these
configurations, not as a standalone planner-kernel measurement.

## Equivalence limits

Sharing the UR5, collision geometry, and state-validity function controls the
main correctness inputs, but the planners still differ in implementation and
configuration. Their sampling, distance/interpolation rules, termination,
shortcutting, and planner defaults need not match. OMPL's default discrete
motion validation samples edges at a configured resolution; a coarse
resolution can miss collisions between samples. RoboPlan's edge checks also
use a step size. The comparison must set and report comparable joint-space
edge-check spacing, then validate every returned segment with the shared
collision checker at that spacing. [OMPL motion validation](https://ompl.kavrakilab.org/core/stateValidation.html)

This experiment does not establish general performance superiority, physical
robot safety, or equivalence between all features in RoboPlan and OMPL. Its
conclusion is limited to the stated UR5 model, scene, start/goal pair,
validity resolution, options, software versions, and trial protocol.

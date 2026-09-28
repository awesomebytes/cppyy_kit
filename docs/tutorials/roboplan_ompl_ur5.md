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

The command runs `ompl_kit/demos/compare_roboplan_rrtconnect.py`.
It prints the repeated-trial results and a solve-time ranking. Keep the machine
otherwise idle for repeatability. Use your run's output for your machine.

### Local result (28 September 2026)

On an x86-64 Intel Core Ultra 9 285H with Python 3.12, RoboPlan 0.7.0,
and OMPL 1.7.0, ten fresh processes each ran five measured plans per library.
Each process also ran one unmeasured warmup plan per library. All 50 returned
paths from each library passed the shared endpoint, limit, and sampled-segment
checks. The table aggregates ten invocations of the command above; the script
reports the five-trial median for one invocation.

| Library | Valid paths | Median of process medians | Range of process medians |
|---|---:|---:|---:|
| RoboPlan | 50/50 | 0.48 ms | 0.47–0.49 ms |
| OMPL via `ompl_kit` | 50/50 | 1.33 ms | 1.13–1.41 ms |

RoboPlan ranked first by median solve time, about 2.8× faster in this setup.
This measures OMPL with a Python collision callback against RoboPlan's C++
scene integration. The scene has self-collision geometry but no world obstacle;
the result does not isolate planner-kernel speed or predict other problems.

## Shared validity contract

RoboPlan's `Scene` exposes collision and joint-limit queries on joint vectors.
Its `SceneContext` provides the collision query with private scratch. The
OMPL callback maps each arm state into the same full UR5 joint vector:

```python
import numpy as np
from roboplan.core import SceneContext

context = SceneContext(scene)
reference = full_start.copy()  # fixed values for joints outside the arm group

def is_valid(state):
    q_arm = np.asarray([state[i] for i in range(len(q_indices))])
    q_full = reference.copy()
    q_full[q_indices] = q_arm
    return scene.isValidConfiguration(q_full) and not context.hasCollisions(q_full)
```

The OMPL kit accepts a Python validity function and pins its lifetime to the
setup object:

```python
import ompl_kit

ob, og = ompl_kit.bringup_ompl()
space = ob.RealVectorStateSpace(len(q_indices))
# Configure arm bounds and start/goal states as in the runnable demo.
ss = og.SimpleSetup(ob.StateSpacePtr(space))
ss.setStateValidityChecker(ompl_kit.validity_checker(is_valid, owner=ss))
ss.setPlanner(ob.PlannerPtr(og.RRTConnect(ss.getSpaceInformation())))
ss.setup()  # complete setup before timing solve()
```

The RoboPlan side uses its `Scene` and RRT API directly; its RRT planner calls
the scene's collision checks while exploring joint space. RoboPlan documents
the `Scene`/`JointConfiguration`/`RRT` flow in its
[sampling-based planning example](https://roboplan.readthedocs.io/en/latest/concepts/sampling_based_planning.html).

```python
from roboplan.core import JointConfiguration
from roboplan.rrt import RRT, RRTOptions

options = RRTOptions(group_name="arm", rrt_connect=True, max_planning_time=1.0)
planner = RRT(scene, options)
start = JointConfiguration(joint_names, start_arm)
goal = JointConfiguration(joint_names, goal_arm)
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

The timing covers each `solve()` or `plan()` call. It excludes model loading,
library bring-up, planner construction, explicit OMPL setup, and one warmup
solve per planner. OMPL's Python validity callback crosses Python/C++ once per
state check; RoboPlan uses its C++ scene integration. Interpret the result as
API-call time for these Python configurations.

## Equivalence limits

Sharing the UR5, collision geometry, and state-validity function controls the
main correctness inputs, but the planners still differ in implementation and
configuration. Their sampling, distance/interpolation rules, termination,
shortcutting, and planner defaults need not match. OMPL's default discrete
motion validation samples edges at a configured resolution; a coarse
resolution can miss collisions between samples. RoboPlan's edge checks also
use a step size. The example sets 0.05-radian joint-space edge checks and
validates every returned segment with the shared collision checker at that
spacing. [OMPL motion validation](https://ompl.kavrakilab.org/core/stateValidation.html)

This experiment does not establish general performance superiority, physical
robot safety, or equivalence between all features in RoboPlan and OMPL. Its
conclusion is limited to the stated UR5 model, scene, start/goal pair,
validity resolution, options, software versions, and trial protocol.

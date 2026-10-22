# Detect an object and move the TIAGo PRO

These commands require this repository checkout. Run them from this README's
directory, `roscon_uk_2026/next_steps/pick_place`. They explicitly select the root
manifest because `next_steps` has a different Pixi workspace. The `pick-place`
environment is locked separately from the existing environments.

```bash
pixi install --manifest-path ../../../pixi.toml -e pick-place --locked
pixi run --manifest-path ../../../pixi.toml -e pick-place replay-pick-place
```

The recorded example finds three tabletop clusters and associates them with red,
blue and yellow image regions. It compares NumPy and PCL voxel centroids on the
same points, then reports processing time separately from initial compilation.
See the [annotated recorded image](evidence/perception_overlay.png).

## Use live sensors

Start the simulator in a second terminal. This source workspace must have been
set up as described in the [simulator guide](../tiago_pro_jazzy/README.md).

```bash
pixi run --manifest-path ../tiago_pro_jazzy/pixi.toml playground
```

From this README's directory, select a target without commanding motion:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-pick-place --observe
```

Expect a red object with a measured centre near `[-2.85, 2.23, 1.05]` metres in
`odom`, its bounds, and the matching image region. The default playground starts
facing this table. The application selects from measured cloud/image association.
A color-to-name catalog identifies simulator objects only for success validation
and optional attachment assistance.

## Pick and place

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place python roscon_uk_2026/next_steps/pick_place/demo.py \
  --report roscon_uk_2026/next_steps/pick_place/pick_result.json
```

The application orients the hand before the table, lifts it, opens the gripper,
approaches,
descends, closes, lifts, transfers to the placement location, releases and retreats.
Cartesian waypoints limit the change between local servo goals. A scene-specific
base Y limit of 1.60 m keeps its centre behind the table. Success requires
measured object motion and final placement within 60 mm. A gripper action response
alone does not establish a grasp.

`--allow-attachment` explicitly enables simulator pose-following if contact does
not lift the object. The result reports `mode: simulated_attachment` when used.
This fallback uses an owned Gazebo service helper process. It is not a physical
contact result. `--handover` is experimental: the current local controller stalls
during presentation, and a physical transfer has not been validated.
Consult [RESULTS.md](RESULTS.md) for the validated scope before rehearsing.

The validated fresh composed run used physical contact and took about 135 seconds
of waypoint motion, plus startup and gripper waits. Its measured lift was 93 mm
and placement error was 30 mm. The fresh distributed run also completed with
contact: 94 mm lift and 35 mm placement error. See [RESULTS.md](RESULTS.md).

The default target is red, the default arm is left, and the placement offset is
160 mm toward negative world X. `--color blue|yellow`, `--side right` and
`--no-base` change those choices. `--base-y-max` changes the table approach bound. These options do not imply that every target
and posture has been validated. The local controller can stop at an unreachable
pose or an obstacle. It is not a global motion planner.

Ctrl-C stops owned task resources and the base command. A failed stage holds the
current arm position when feedback is fresh. It does not automatically open a
gripper holding an object. Restart the perception application after resetting
the simulator clock.

## Change whole-body tasks from Python

Choose `--controller custom` (the default) or `--controller placo`. The latter
requires the separate Placo environment:

```bash
pixi install --manifest-path ../../../pixi.toml -e pick-place-placo --locked
pixi run --manifest-path ../../../pixi.toml -e pick-place-placo demo-pick-place --controller placo \
  --report roscon_uk_2026/next_steps/pick_place/placo_result.json
```

Both backends expose the task methods below. For Placo, replace the import with
`from placo_controller import PlacoWholeBodyController as WholeBodyController`.
The validated composed Placo run completed physical pick and place with 84.8 mm
object lift and 16.8 mm placement error, including retreat and without attachment
assistance. See [the measured results](RESULTS.md#placo-controller).
Robot joint names, task frames and default motion costs are collected in
[robot_config.py](robot_config.py). Set base workspace limits through
`controller.set_base_bounds('y', -float('inf'), 1.60)` when base motion is enabled.

Placo uses its existing Python bindings to the native
[kinematics solver](https://placo.readthedocs.io/en/latest/kinematics/getting_started.html).
The application still uses `cppyy_kit` for native perception composition and the
optional `control_kit` controller adapter. There is no additional Placo binding
layer. Both backends use weighted motion preferences, not a strict task hierarchy.

Placo loads all 61 URDF collision shapes and applies the generated SRDF's collision
exclusions. Native constraints cover robot self collision and supplied sphere, capsule
and box obstacles. A conservative bounding-box filter selects nearby pairs for
exact native mesh distance queries. It retains every configured pair as a
candidate on each step. The default clearance is 2 mm, with a 40 mm query trigger;
some gripper collision shapes are only 4.6 mm apart at the ready pose.
These are local, discrete command checks, not continuous trajectory validation.
The demo does not automatically insert a map or table collision object.

Supply the measured gripper driver positions as well as arm, head and torso
positions to Placo; its full collision model needs them. Missing gripper feedback
raises an error. Supply passive-joint feedback when available; mimic estimates
fill only omitted passive joints. Both controllers reject invalid input. A Placo
infeasible solve holds the measured state and reports `solver_failure`. The live application
stops on that failure. The Placo preparation moves the hand outward before
rotating it to avoid the torso.

With this directory on `sys.path`, use the measured robot state with the controller:

```python
from controller import WholeBodyController

controller = WholeBodyController(urdf_text)
controller.set_goal('gripper_left_grasping_link', position=(-2.85, 2.23, 1.20))
controller.set_goal('gripper_right_grasping_link', orientation=(0., 0., 0., 1.))
controller.set_elbow_goal('left', (-2.95, 1.8, 1.0), weight=.03)
controller.look_at((-2.85, 2.23, 1.05))
controller.set_obstacles([
    {'type': 'sphere', 'center': (-2.5, 2.0, 1.0), 'radius': .08},
    {'type': 'capsule', 'a': (-2.4, 1.9, .9), 'b': (-2.4, 1.9, 1.2), 'radius': .04},
    {'type': 'box', 'center': (-2.3, 2.0, .8), 'size': (.1, .2, .3)},
])
command = controller.step(measured_joint_positions, measured_base_xy_yaw, dt=.05)
robot.command(command, duration=.05)
```

Positions and obstacles use `odom` metres; orientations are XYZW quaternions.
Calling `set_goal` with neither position nor orientation removes that link's goal.
Boxes are axis-aligned. Elbow tasks use a lower weight than the hand tasks.
The default motion costs favor arms, then torso, then base. These are weighted
preferences, not strict task priorities.

For the custom backend, the native Eigen kernel computes forward kinematics and
solves weighted damped least squares. Python assembles tasks and applies
joint/speed bounds. The application
targets a 20 Hz task loop; the simulator controller manager runs at 100 Hz. Primitive
obstacle checks cover conservative samples of arm segments through the grasp
point. They do not cover self collision, the base, torso, or every URDF shape.
The default pick sequence uses table-clearance waypoints; the obstacle API is a
separate demonstrated controller capability.

The reusable `control_kit.task_control.TaskModel` supplies the native FK and solve.
`control_kit.make_position_task_controller` adapts a position task to the real
ros2_control `ControllerInterface`. `controller.as_ros2_controller()` exposes this
adapter for the custom backend when `allow_base=False`. The Placo adapter also
requires explicit fixed gripper values in `passive_positions`; use live ROS
feedback instead when grippers move. The live simulator path sends task outputs to
Gazebo's existing trajectory controllers and base velocity input. It does not
inject a Python controller into Gazebo's separate controller-manager process.

The live execution path is:

```text
Python task servo (20 Hz target)
  -> ROS joint trajectories and base velocity
  -> existing native ros2_control controllers (100 Hz)
  -> Gazebo joints and measured feedback
```

The separate in-process `ControllerInterface` adapter has been exercised with
mock hardware. Its update callback runs Python; it is not a hard real-time
controller. Switching the demo's solver does not change the simulator's native
controllers or their process.

## Interactive admittance controller

[Push the hand from a browser](ADMITTANCE.md). A Python mass-spring-damper law
converts virtual forces into Cartesian offsets. A cached C++ kernel computes
kinematics and joint commands inside Gazebo's controller manager. The guide
includes interactive controls, a repeatable force-pulse run and measured results.

## Python controller inside Gazebo

This repository example runs the control law in Python inside Gazebo's actual
ros2_control manager. The [Python class](../../../control_kit/demos/python_position_controller.py)
owns targets, proportional updates, speed limits and position limits. It claims
seven left-arm position interfaces and moves the wrist by 0.04 rad, then returns.
The WBC and pick-and-place application remain separate.

From this README's directory, prepare the reusable bridge:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-gazebo-python --prepare-only
```

Start the simulator using the overlay launch command in the next section.
After `TIAGO_PRO_READY`, run from this README's directory:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-gazebo-python
```

Expect `passed: true`, `stock_controller_restored: true`, and a `python_update`
record containing the Gazebo PID and callback count. The measured run executed
929 Python updates and reached 0.03705 rad for a 0.04 rad target.
See [recorded evidence](python_control_result.json).

The reusable C++ bridge handles pluginlib, ROS interfaces, interpreter lifetime
and calls into Python. `cppyy_kit.prebuild()` compiles and caches that bridge;
`control_kit` exports its plugin metadata. A new controller law needs a Python
class, not another C++ plugin or CMake package. Restart Gazebo after editing the
Python module because its interpreter caches imported modules.

This path takes the GIL and allocates Python objects during updates. It provides
no hard real-time guarantee. Invalid return lengths, non-finite commands and
Python exceptions return a controller error before any command writes. Existing
host Python must match the bridge's major/minor version; extension packages must
match the host environment. The demo law uses only the Python standard library.

## Native ros2_control controller

This separate example runs a C++ joint-position controller inside Gazebo's
controller manager. It claims all seven left-arm position interfaces, holds six
joints and moves the last wrist joint by 0.04 rad, then returns it. The example
restores the stock controller and unloads the custom controller, including on
Ctrl-C. It does not change the pick-and-place application's actuator selection.

Compile and export the plugin from this README's directory:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-gazebo --prepare-only
```

Stop any existing simulator. In a second terminal, starting from this directory,
launch the simulator with the exported plugin visible at process startup:

```bash
(
  cd ../tiago_pro_jazzy
  pixi run env \
    AMENT_PREFIX_PATH="$HOME/.cache/cppyy_kit/control_plugin:$PWD/.pixi/envs/default" \
    bash ../simulator_guard.sh -- ros2 launch "$PWD/playground.launch.py" \
    gui:=false rviz:=false moveit:=false navigation:=false
)
```

After `TIAGO_PRO_READY`, run from this README's directory:

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place demo-control-gazebo \
  --report roscon_uk_2026/next_steps/pick_place/native_control_result.json
```

Expect `passed: true` and `stock_controller_restored: true`. The validated run
moved 0.03732 rad for a 0.04 rad goal, with 0.00268 rad final error. The demo sets
the playground's domain 77, Fast DDS and SUBNET discovery profile explicitly.
See [the native-controller measurements](RESULTS.md#native-ros2_control-controller).

The [C++ controller](../../../control_kit/control_kit/native_controller.cpp) owns
its lifecycle, target subscription, feedback reads, bounded proportional position
updates and hardware command writes. `update()` calls no Python. Python uses
`cppyy_kit` to build the library and export its plugin metadata; the demo script
uses ROS services and topics for setup, targets and observation. The overlay
contains its own library copy. After changing the C++ source, prepare the overlay
again and restart the simulator to load the new build.

This controller supplies joint-position commands. It has no Cartesian task solver,
torque control or collision model. The WBC remains external; the existing
pick-and-place commands still use the stock trajectory controllers. A smaller
mock-hardware example is also available:

```bash
pixi run --manifest-path ../../../pixi.toml -e control demo-control-native
```

## Change deployment without changing perception

```bash
pixi run --manifest-path ../../../pixi.toml -e pick-place python roscon_uk_2026/next_steps/pick_place/demo.py \
  --deployment distributed --observe
```

Distributed mode starts cloud filtering, clustering and image detection as three
processes, with the task/controller in a fourth. Composed mode keeps those native
stages and the Python task/controller in one application process. Gazebo remains
external. See [DEPLOYMENT.md](DEPLOYMENT.md) for standalone stage commands,
ROS topics, bounded pending work, lifecycle ownership, and benchmark definitions.

## Prepare the talk

Use the [current speaker notes](../../SPEAKER_NOTES.md) for the suggested
sequence, slide snippets, wording and measured claims.

1. Run the recorded replay and show its NumPy/PCL centroid agreement.
2. Show the native PCL and OpenCV calls in [perception.py](perception.py) and
   [native/](native/). Change the voxel size or image color thresholds.
3. Run `--observe`; show the selected image box and cloud bounds.
4. Show position-only, orientation-only, elbow and gaze task setup.
5. Run the validated manipulation mode described in [RESULTS.md](RESULTS.md).
6. Compare distributed and composed launches using the same algorithms.
7. Explain the measured results and the remaining controller/physics limits.

The [implementation plan](PLAN.md), [controller measurements](controller_evidence.json),
[recorded perception measurements](replay_evidence.json), and
[deployment measurements](evidence/composition/live_summary.json) provide source
material for the slides. Use activated `pixi run` commands so compiler and native
library settings are present before loading cppyy.

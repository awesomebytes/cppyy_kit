# Speaker notes: robot perception, Python control and native computation

Updated 19 October 2026. The current spoken sequence is in
[TALK_SCRIPT.md](TALK_SCRIPT.md), incorporating the robotics opening, language
vote, acceleration alternatives and rclcppyy introduction. Use this file as the
evidence and code-snippet companion for the current robot demonstration.
They supplement the earlier spoken script rather than replacing its historical
MCAP, agent-evaluation and webcam sections.

## Where the other notes live

- [DEEP_DIVE_PRESENTATION.md](DEEP_DIVE_PRESENTATION.md): the longer spoken
  script, prompts and rehearsal commands. Its controller section describes the
  earlier mock-hardware experiment, not the new Gazebo plugin.
- [TALK_FLOW.md](TALK_FLOW.md): the original single-robot narrative and
  audience-selected ordering. Several proposed capabilities are now implemented;
  use the evidence links below for their current status.
- [PLAN.md](PLAN.md): historical work packages and presentation gates.
- [CURRENT_REHEARSAL.md](CURRENT_REHEARSAL.md): rehearsal environment and checks.
- [Pick-and-place results](next_steps/pick_place/RESULTS.md): current robot
  measurements and limitations.

## Current sequence

Follow [the revised timing and script](TALK_SCRIPT.md#timing-and-scope): context,
language vote, acceleration alternatives, cppyy and cppyy_kit, ROS object access,
perception, optional nanoflann, Python control and admittance, pick and place,
whole-body tasks, composition, and the closing discussion. The sections below
are reference material; their order is not the current speaking order.

## Opening: one task, several implementation choices

Say:

> We want the robot to find an object and move it. We will keep the same robot
> and change how we implement and connect its software. Python defines the task.
> Existing native libraries and small C++ functions handle selected operations.

Show the camera image, point cloud and three tabletop objects. Avoid starting a
long manipulation sequence while explaining setup. The validated Placo motion
sequence alone took 116.1 seconds, excluding startup and gripper waits.
Use a prepared recording or run it while discussing task goals.

Explain the tools:

- cppyy provides dynamic access to C++ from Python.
- cppyy_kit supplies build/cache and interoperability helpers.
- Individual kits supply library-specific setup and integration.
- Existing ROS controllers, PCL, OpenCV and Placo still provide their algorithms.

## Perception: give a native operation a concrete job

Say:

> First we reduce the cloud, remove the table and group the remaining points.
> Image colour regions help select which cluster we want to pick.

Show [perception.py](next_steps/pick_place/perception.py) and the
[native stages](next_steps/pick_place/native/). Change voxel size or a colour
threshold. The image labels describe colour and 2D silhouette; they are not a
learned semantic detector or a reliable physical shape classifier.

For the numerical comparison, use the same recorded input: NumPy and PCL
produced 9,181 centroids with an 8 mm grid. Maximum centroid difference was
2.38e-7 m. Median voxel processing was 15.34 ms for NumPy and 1.31 ms for PCL,
including the PCL boundary copies. These are operation measurements, not
end-to-end application timings. [Source](next_steps/pick_place/replay_evidence.json).

## Whole-body tasks: reuse the solver

Say:

> We can express reaching, orientation, posture and gaze goals in Python.
> Placo supplies the solver. cppyy_kit does not replace an existing useful binding.

Use the [WBC setup examples](next_steps/pick_place/README.md#change-whole-body-tasks-from-python).
Explain that the WBC remains outside the ros2_control update loop and sends
commands to the existing controllers. The separate Python plugin demonstration
below does not move the WBC into that loop.

The [Placo contact run](next_steps/pick_place/runs/placo-contact.json) lifted the
object 84.8 mm and placed it within 16.8 mm of the requested centre, without
attachment assistance. Self-collision constraints were enabled. This is one
successful run, not a success-rate measurement or proof of collision safety.
Arm-to-arm transfer remains unvalidated. The custom WBC and Placo backend have
different collision capabilities; do not describe them as interchangeable.

## One slide: the controller-specific Python

Say:

> This is all the controller-specific code. control_kit supplies the reusable
> bridge into the real ros2_control controller manager.

```python
class GentleServo:
    def configure(self, joints, options):
        self.speed = options.get("speed", 0.2)  # rad/s

    def activate(self, measured):
        self.target = list(measured)

    def set_target(self, positions):
        self.target = list(positions)

    def update(self, measured, dt):
        step = self.speed * dt
        return [
            q + max(-step, min(step, goal - q))
            for q, goal in zip(measured, self.target)
        ]
```

This is a reduced teaching example. It bounds the command increment relative to
feedback; it does not guarantee physical joint speed. It omits target validation
and joint-limit checks. Use the validated
[PositionServo](../control_kit/demos/python_position_controller.py) for the demo.

## Next slide: show the glue

Per-controller configuration selects a module, class and joints:

```yaml
gentle:
  ros__parameters:
    type: control_kit_python/PythonController
    joints: [joint1, joint2]
    python_path: /absolute/path/to/controllers
    python_module: gentle_servo
    python_class: GentleServo
    python_options: '{"speed": 0.2}'
```

This YAML is illustrative; use the actual robot joint names and module path.
The reusable export operation is:

```python
from control_kit.python_control import install_python_controller_plugin
install_python_controller_plugin("/absolute/path/to/overlay")
```

The helper calls `cppyy_kit.prebuild()` and writes the plugin metadata. The
manager must inherit the overlay in `AMENT_PREFIX_PATH` before startup. The
standard spawner then loads and activates the controller. Existing interface
owners must be switched out, as the demo does.

```text
ROS target topic → Python set_target()
ros2_control update → C++ bridge → Python update(measured, dt)
                                ← position commands
                    → robot command interfaces
```

Point to the actual
[Python invocation](../control_kit/control_kit/python_controller.cpp:240) and
[export helper](../control_kit/control_kit/python_control.py:55).
The reusable bridge is currently 296 lines of C++ including its header, plus the
Python build/export helper. It handles interpreter lifetime, GIL acquisition,
lifecycle forwarding, interfaces, conversion and errors.

Say:

> CPython makes embedding possible. control_kit supplies the bridge. cppyy_kit
> builds and caches it. A conventional ROS package could provide the same Python
> API; this example reduces the build and integration work we maintain.

Do not claim that cppyy_kit translates the Python law into C++, eliminates the
GIL or makes the controller hard real-time.

## Admittance: push the hand from a browser

Use the [run guide](next_steps/pick_place/ADMITTANCE.md). Start the simulator in
its `kit` environment, which includes cppyy. Compile and warm native calls before
activation. Prepare this before presenting the interactive segment.

Show [the Python update](../control_kit/demos/python_admittance_controller.py:82):

```text
acceleration = (force - damping * velocity - stiffness * offset) / mass
velocity += acceleration * dt
offset += velocity * dt
joint_commands = chain.step(measured_joints, target_pose, ...)
```

This is a schematic excerpt; the implementation adds substeps, bounds and force
expiry. The native `step` evaluates FK, the analytic Jacobian and a damped
least-squares solve. The [C++ kernel](../control_kit/demos/arm_kinematics.hpp)
uses Eigen; the [Python wrapper](../control_kit/demos/arm_kinematics.py) builds,
caches and exposes it through cppyy_kit.

Say:

> The spring behaviour is Python. The kinematics calculation is C++. We can
> change the behaviour without making another C++ controller plugin.

Hold +X at 1 N, then release. The force is synthetic and expressed in
`torso_lift_link`; it is not a physical Gazebo wrench or wrist-sensor reading.
Default stiffness is 80 N/m, so 1 N requests a 12.5 mm equilibrium offset.
The reference offset is bounded to 30 mm. The browser and controller expire
unrefreshed force commands. Show release before trying another direction.

The [recorded live run](next_steps/pick_place/evidence/admittance/live.json)
measured 10.86 mm peak displacement and 1.45 mm return error, with 1,157 Python
updates in the Gazebo process. Native call mean was 31.18 microseconds and maximum
was 3.13 ms. This is not a measured speedup over NumPy or a real-time bound.
There is no collision avoidance in this small admittance controller.

End with Enter or Ctrl-C in the demo terminal. It releases the force, allows a
return phase, restores the stock controller and unloads the demo controller.

## Composition: keep the computation, change its deployment

Show the [deployment guide](next_steps/pick_place/DEPLOYMENT.md). The same cloud
filter, cluster stage and image stage can run in separate processes or alongside
the task application. Gazebo remains a separate process in both cases.

Say:

> Python assembles the native stages. ROS interfaces remain available for
> observation. We can investigate deployment costs without rewriting perception.

The live comparison did not establish a latency speedup. Inputs differed between
runs, and observer scheduling affects the reported latency. Intra-process delivery
also does not remove transformation, PCL conversion or filtering allocations.
Use the [measured deployment table](next_steps/pick_place/RESULTS.md#perception-and-deployment)
if discussing performance.

## Rehearsal and fallback notes

- Follow the current run guides, including their working directories. The nearby
  `next_steps/pixi.toml` is not the root pick-and-place environment.
- Use only one simulator. The admittance launch needs the plugin overlay and the
  simulator `kit` environment. Do not assume the default simulator has cppyy.
- Warm native calls and open the UI before the control segment. Rehearse a full
  stop and controller restoration, not only the successful motion.
- If live perception is unavailable, use the recorded replay and its measured
  output. If live control is unavailable, show the recorded evidence and code;
  label it as a previous run.
- Keep cold compilation, warm computation, ROS transport and full application
  time separate. Retain the mock-hardware label on older controller benchmarks.
- Do not show handover as completed or claim that pick and place uses the new
  admittance controller. They are separate demonstrations.


## Optional Nav2 extension: repeat the minimal-controller story

The new [navigation example](next_steps/navigation/README.md) repeats the Python
controller pattern with a different framework interface. Show the small
[forward-camera policy](../nav2_kit/demos/forward_camera_policy.py), then its
configuration. `nav2_kit` supplies one reusable DWB plugin bridge and exporter;
the author does not write a C++ plugin or CMake package for each policy.

Say:

> We did this for ros2_control. Now a small Python class changes how Nav2 scores
> candidate trajectories. The kit supplies the framework glue, while Nav2 keeps
> its existing controller and collision checks.

The adapter implements the DWB critic interface. Depth input is a separate
opt-in setting. Use the navigation guide for evidence.
This is optional material until the full presentation has been timed.

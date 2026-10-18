# One robot playground, several ways to build its software

Rough narrative for the 45-minute ROSCon UK 2026 deep dive. This captures the
new direction discussed for the talk. It is not an implemented robot stack or
a replacement for the existing [presentation plan](PLAN.md) and its evidence.

## The shared setting

Start with a simulated mobile manipulator: a wheeled base, arm(s), RGB/depth,
IMU, joint state and range/point-cloud data. The current starting point is
[TIAGo PRO on ROS 2 Jazzy and Gazebo Harmonic](next_steps/tiago_pro_jazzy/README.md),
now published as `tiago-pro-playground` 0.2.0 build 2. Fresh installed startup
passed nine controllers, MoveIt, Nav2, sensors and the GUI; the user also
confirmed source-session navigation and MoveIt operation. The optional shared
`kit` environment passed installed `rclcpp_kit` native-node bringup and teardown.
These establish the robot baseline and compatible kit setup, not the robot
processing/composition experiment, acceleration result or autonomous grasping.

The native materials are restored, with a plain floor. One aligned RGBD camera
runs at 30 Hz without duplicate rendering. Quiet-window median real-time factor
improved from 0.589 to 0.832 in the recorded comparison; it remains below 1.0.
The depth-cloud bridge uses the camera body frame matching native XYZ, while
RGB retains its optical frame. Registered cloud geometry places the floor at
z=0 and tabletop at approximately z=1.02 m. Public 0.1.0 remains the earlier
Humble / Gazebo Classic fallback; its wrist-load and motion evidence does not
establish new Harmonic force calibration. Stretch, single-arm TIAGo and TIAGo++
remain earlier alternatives. One shared lock permits one simulator at a time.
The measured cppyy_kit processing and composition demonstration below remains
the next task.

Use one task: find an object, approach it and position the arm near it. Reuse
navigation and basic control software. Grasping and whole-body balance are not
prerequisites. This gives context for concerns shared with humanoids, cars,
drones and quadrupeds: processing, estimation, control, ownership and timing.

Initially show a typical ROS setup: separate processes, topics, parameters,
a viewer and recording. All demonstrations change this same application.

```mermaid
flowchart LR
    sim["Simulated robot and sensors"] --> processing["Perception and preprocessing"]
    processing --> goal["Target or motion reference"]
    goal --> control["Existing navigation and control"]
    control --> sim
    processing -. "Intermediate results and timing" .-> inspect["Viewer, recording and diagnostics"]
```

## Establish the tools before the experiments

Explain the distinction with one small operation:

| Approach | What the developer does |
| --- | --- |
| Without cppyy | Uses available Python bindings, stays in Python, or builds and maintains a native extension. |
| With vanilla cppyy | Calls C++ dynamically, supplies headers and libraries, and handles integration details. |
| With cppyy_kit | Reuses library setup and interoperability helpers, or writes cached C++ kernels through `@cpp`; supplies explicit buffer and lifetime contracts. |

Python remains the high-level language. Native code still needs review and
testing; kits reduce integration work.

## Same content, audience-selected order

Ask whether people would first reuse an established library or ask an agent
to write a tailored implementation. Show their less preferred approach first,
then their preferred approach. Only the order and framing change.

| Audience preference | First | Second |
| --- | --- | --- |
| Established library | Agent-written custom implementation | Established C++ library |
| Agent-written implementation | Established C++ library | Agent-written custom implementation |

Both orders include the same Python baseline, acceleration, library integration,
checks and measurements. Reuse and agents are compatible: an agent can also
integrate the established library.

Choose one sensor-processing operation with a measured bottleneck and an
appropriate C++ library: perhaps depth/point-cloud filtering or neighborhood
queries. Select the exact operation during rehearsal.

The custom section moves measured Python work into a small native kernel. The
library section replaces that operation with a maintained implementation,
without a separate handwritten bindings project. Compare behavior, capability
and performance; do not assume either implementation wins. Use reasonable
Python baselines, including existing vectorized operations and bindings.

## Shared finale: change how the application is assembled

Keep the robot task and algorithm. Explore Python configuring native ROS stages
inside one process or a few coherent subsystems. Keep expensive work and
frequent callbacks native where useful. Make thread scheduling, buffer
ownership, bounded queues and shutdown visible.

Show one readable assembly point, fewer conversions and less deployment
coordination. Retain ROS interfaces for viewers, parameters and recording;
identify which stages remain ROS-visible. Existing ROS composition supplies
the foundation; cppyy_kit helps make it accessible from Python.

Composition is another useful experiment, not a claim that monoliths are the
best architecture. Shared failures, asynchronous ordering and native debugging
remain concerns. A shared process alone does not guarantee zero-copy.

## Timing and participation

| Minutes | Content |
| --- | --- |
| 0–5 | Robot, task, audience vote and initial ROS graph |
| 5–12 | Without cppyy, vanilla cppyy, then cppyy_kit |
| 12–30 | The same two nine-minute implementation sections, in the voted order |
| 30–40 | Composition and the development/debugging workflow |
| 40–45 | Compare results and discuss questions |

Include optional exercises inside these times: change a processing parameter
and check outputs; change overload policy and inspect frame age. Supply prepared
checkpoints. Continue presenting or taking questions while attendees experiment.
Keep recorded runs and checked solutions available.

## Show the working experience and the evidence

Demonstrate a small requirement change, an independent correctness check and
a recorded-input regression. Aim to inspect intermediate results, reproduce a
failure and pause/step the simulation at defined boundaries. These debugging
facilities are proposed work; a Python breakpoint does not pause native workers.

Compare CPU, processing time, frame age, dropped inputs and UI responsiveness.
Separate compilation/startup from warmed execution. Explain ownership and
copies. For agent steps, retain prompts, guidance, failures and corrections;
time to a verified result matters more than time to generated code.

The [existing composition experiment](next_steps/composition/README.md) proves
synthetic native progress, output parity, bounded pending work and joined
shutdown. It does not establish robot performance or zero-copy. Build the
playground in order: verify the stock simulation, select one operation, validate
both implementations, then demonstrate composition and rehearse both orders.
GPU and Zenoh are outside this exploration. EventsCBGExecutor is a separate
upstream dependency upgrade and comparison, not a new executor to implement.

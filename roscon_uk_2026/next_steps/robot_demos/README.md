# Three RViz demos on TIAGo Pro

Run these commands from the **repository root**. Each starts a fresh Gazebo scene and opens RViz. Stop a demo with **Ctrl-C in its terminal** and wait for `Demo stopped.` before starting the next. The launcher holds the simulator lock and stops only its own processes. Add `--gazebo-gui` to also open Gazebo; its additional rendering costs CPU. The simulator and sensors run in either mode.

The default `--scene simple-house` keeps the house walls, corridors, floor, table and three objects. Other furniture is removed, and the navigation map matches the remaining geometry. Use `--scene house` for the original furnished house or `--scene table` for the floor and table only. See [simulation performance measurements](PERFORMANCE.md).

## 1. Original holonomic navigation

```sh
./roscon_uk_2026/run_robot_demo.sh baseline
```

Wait for the RViz Navigation 2 panel to show Navigation as active. Localization remains inactive because this scene uses simulator-truth localization rather than AMCL. Select **Nav2 Goal** and choose a free-space goal east of the initial robot pose, roughly `(x=-1.85, y=1.35)` in `map`, with final yaw 0. The robot initially faces north. The original DWB profile can translate sideways instead of turning first. Use the same goal in the next demo.

## 2. Forward-facing navigation

```sh
./roscon_uk_2026/run_robot_demo.sh forward
```

Send the same goal. The Python `ForwardCameraPolicy` critic rejects translation outside a ±30° forward cone and asks the robot to turn before translating toward a path outside that cone. Nav2's existing DWB planner, collision critics and goal handling remain in use. Near the goal, the stock final-heading critic handles orientation.

Both commands enable the same depth costmap source. The forward profile also enables footprint checking and adjusts velocity samples to include turn-in-place candidates, as documented in [navigation](../navigation/README.md). This comparison demonstrates behavior, not an isolated performance benchmark. Keep the head facing forward for the camera-cone assumption.

## 3. Python admittance controller with an RViz force handle

```sh
./roscon_uk_2026/run_robot_demo.sh admittance
```

The launcher prepares the native plugin, starts the robot, switches the left arm to `CartesianAdmittance`, and prints `RVIZ_FORCE_READY`. RViz opens close to the arm. Choose **Interact** in the toolbar. Drag an axis of the orange marker at the left hand:

- Handle displacement requests virtual force in `torso_lift_link`: `F = 20 N/m × displacement`.
- Force magnitude is limited to 5 N. A 10 cm handle drag requests 2 N.
- The arrow shows the requested force. The label shows its magnitude and measured hand displacement.
- Release the mouse to zero the force and reset the handle. The hand returns toward its starting pose.
- Missing marker feedback for 0.6 seconds zeros the request. The controller independently expires force commands after 0.3 seconds without refresh.

The marker is anchored at the hand's initial pose, not its moving pose. It supplies a synthetic force command, not a physical Gazebo wrench or measured force-sensor reading. The base stays still. The controller limits its Cartesian reference displacement to 30 mm and keeps the reference orientation fixed.

Ctrl-C zeros the force, allows the spring to return, restores the stock arm controller, unloads the Python controller and then shuts down the simulation. Give cleanup time to finish.

## Processes and code

`run_robot_demo.sh` selects the existing simulator Pixi `kit` environment. `run.py` creates the RViz configuration, starts the owned launch process, and starts the controller demo after native controllers are ready. MoveIt is disabled in these three profiles.

Navigation runs the Python critic **inside Nav2's controller_server**, through `nav2_kit_python::PythonCritic`. DWB generates candidate trajectories; Python scores or rejects them. The plugin's C++ adapter and export are supplied by nav2_kit. Read:

- [Python policy](../../../nav2_kit/demos/forward_camera_policy.py)
- [Plugin/profile configuration](../tiago_pro_jazzy/navigation/nav.launch.py)

Admittance runs the Python law **inside Gazebo's ros2_control controller manager**, through `control_kit_python/PythonController`. Python integrates the mass–spring–damper equation. A C++ Eigen kernel computes kinematics and joint commands through cppyy_kit. The separate UI process only turns marker feedback into virtual forces and monitors joint feedback. Read:

- [Python controller](../../../control_kit/demos/python_admittance_controller.py)
- [RViz force handle](../../../control_kit/demos/admittance_rviz.py)
- [Controller registration, switching and cleanup](../pick_place/admittance_demo.py)
- [Native kinematics](../../../control_kit/demos/arm_kinematics.hpp)

The controller adapter and plugin export are reusable `control_kit` code. `CartesianAdmittance`, `arm_kinematics.py` and `arm_kinematics.hpp` are example code under `control_kit/demos`. The `Chain` class parses the arm URDF and calls its cached Eigen kernel; it is not a kit-provided admittance controller.

The helpers build and cache the plugin libraries and generate their pluginlib metadata. These demos require no new per-policy CMake project. Restart the corresponding host after editing the Python implementation.

## Recording and verification

Use the three commands above for video recording. Keep the same camera angle and navigation goal for the before/after clips. For admittance, show drag, sustained displacement, release and return. A 20–30 second clip is sufficient for that interaction.

`--headless` suppresses both windows for validation. `--run-seconds N` stops a run after N wall seconds, followed by normal controller restoration and shutdown. Evidence and logs are written under `evidence/`.

`exercise_marker.py` sends the same feedback messages RViz uses, applies 2 N along X for eight seconds, releases, and checks measured hand motion. Run it in the simulator kit environment while the admittance demo is active. It is a repeatable protocol exercise, not an automated mouse/GUI test.

## Local verification, 22 October 2026

Both navigation runs reached the same goal with position error below 0.1 m.

| Profile | Moving commands | Commands outside ±30° | Measured travel outside ±30° |
| --- | ---: | ---: | ---: |
| Original | 88 | 60 | 0.561 m of 0.722 m |
| Forward critic | 109 | 0 | 0.010 m of 0.708 m |

Sources: [baseline](evidence/baseline.json), [forward](evidence/forward.json). Command direction and measured motion differ because the simulated base accelerates and turns.

The RViz feedback protocol exercise applied 2 N for eight wall seconds, then released. In the run with both GUI windows, measured hand displacement peaked at 7.8 mm and the return error was 3.7 mm. An earlier headless run measured 18.5 mm and 2.4 mm respectively. These wall-time tests depend on simulation speed. The reference displacement limit is 30 mm. The visible RViz handle and robot model were also inspected, and a manual drag produced force commands and visible motion.

A clock failure in one launch left TF unusable. The bridge now selects `/world/default/clock`, generated from the actual world name, rather than Gazebo's optional `/clock` alias. Gazebo documents the alias behavior in its [bridge reference](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md). Subsequent GUI runs received simulation time without the controller-manager clock warnings.

With both Gazebo and RViz windows open, a same-process controller-switch comparison measured RTF 0.537 with Python, 0.575 with the stock arm controller, and 0.553 after restoring Python. Each sample covers seven wall seconds after a two-second settling interval. This short comparison does not explain every transient slowdown. Source: [controller RTF](evidence/controller_rtf.json). A second run with RViz alone measured 0.412 / 0.328 / 0.440 for Python / stock / Python; see [RViz-only samples](evidence/controller_rtf_rviz_only.json). The variability is larger than a consistent controller effect, so these short runs do not establish a stable overhead or a rendering speedup. RTF 0.5 means one simulated second takes two wall seconds.

## Presentation

The working presentation contains the simulator diagram, one video placeholder for each launch, colored Nav2/controller code, registration/configuration, and two controller code-reference slides. Speaker notes contain the launch/capture sequence and source links. Existing slides retain their order. Code excerpts are identified in the notes; use the linked modules for the complete runnable implementation.

After separating the kinematics example from the kit, a headless simplified-house check applied 2 N through RViz feedback messages. Hand displacement peaked at 20.4 mm and returned within 2.0 mm. The adapter now holds the previous command on a zero-duration update instead of deactivating; negative or nonfinite update periods still fail. Source: [force check](evidence/rviz_force.json).

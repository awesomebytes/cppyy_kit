# TIAGo PRO with Jazzy and Gazebo Harmonic

Run the home simulator from this repository checkout:

```bash
cd roscon_uk_2026/next_steps/tiago_pro_jazzy
bash setup.sh
pixi run playground
```

Gazebo and RViz open with the robot facing the original house table and three
physical props. Nine native controllers activate, followed by MoveIt and Nav2.
The shared simulator guard permits one robot simulator at a time.

This is the Jazzy source workspace. The published `tiago-pro-playground 0.1.0`
package remains Humble / Classic; a Jazzy binary package has not been published.
Do not load that package's native libraries into this environment.

The lock selects RoboStack Jazzy and Gazebo Harmonic. Setup reuses the pinned
public PAL descriptions and bundled house from adjacent `tiago_pro`, without
installing Humble. Generated model, MoveIt and navigation files remain local.
The Humble source and its saved RViz layout are unchanged.

## Verified migration

Startup verification found all nine controllers active, MoveIt ready and Nav2
active. Both lasers returned 818 finite ranges; RGB and depth cloud were 640×480;
both wrist force/torque streams and world-pose odometry were available. Read-only
checks are available with `ROS_DOMAIN_ID=77 pixi run python check_startup.py`.

Harmonic publishes depth points with X forward, Y left and Z up. Keep the native
`head_front_camera_depth_frame` header; the Classic optical-frame correction is
incorrect here. Transforming the cloud to `odom` placed 65,274 sampled floor
points at zero and the table-region median at 1.019997 m (expected 1.020 m).
MoveIt uses the point cloud alone; no table or ground collision objects are injected.

The base uses stock Harmonic VelocityControl and world-pose OdometryPublisher.
This remains a planar approximation, with identity `map`→`odom`, not wheel-traction
physics or AMCL. The small native follower plugin applies bounded physical PD
forces. Native kinematic mimics are disabled in the simulator copy, while the ROS
model retains mimic relationships for visualization. The ATI wrist load path and
closed-gripper initial pose are retained. Startup and static sensor behavior are
verified; autonomous navigation and contact/grasp behavior have not been retested.

RViz's one-time indexed-image GLSL warning and unavailable optional
`recognize_objects` action also occurred in Humble. The map renders correctly.

## Optional cppyy_kit experiments

The simulator does not depend on cppyy_kit. The optional `kit` environment shares
the same dependency solve and adds published `ros-jazzy-rclcpp-kit==0.4.1` and
`cppyy-kit==0.4.1`:

```bash
pixi install -e kit --locked
pixi run -e kit playground  # use after stopping the default simulator
pixi run -e kit python -I -c 'import rclcpp_kit; rclcpp_kit.bringup_rclcpp()'
```

Installed-only import, C++ header bringup, native node creation and teardown passed.
Future Python controllers, perception and Nav2 plugins can use this environment;
those experiments are separate from the simulator migration.

## Source provenance

Robot description revisions are pinned in [sources.lock](../tiago_pro/sources.lock),
MoveIt configurations in [moveit/sources.lock](../tiago_pro/moveit/sources.lock),
and house assets in [worlds/home/sources.json](../tiago_pro/worlds/home/sources.json).
The existing [third-party notices](../../../recipe/tiago-pro-playground/THIRD_PARTY.md)
describe their licenses; retain the upstream license files with redistributed assets.

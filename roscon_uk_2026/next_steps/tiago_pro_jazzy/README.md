# TIAGo PRO with Jazzy and Gazebo Harmonic

Install the published Jazzy simulator by copying the
[consumer pixi.toml and pixi.lock](../../../recipe/tiago-pro-playground-jazzy/consumer/)
into an empty directory, then run:

```bash
pixi install --locked
pixi run playground
```

No repository checkout or source build is needed for that consumer environment.
The [package page](https://prefix.dev/channels/awesomebytes/packages/tiago-pro-playground)
contains version `0.2.0` for Jazzy/Harmonic; `0.1.0` remains the Humble/Classic fallback.

For source development from this repository checkout:

```bash
cd roscon_uk_2026/next_steps/tiago_pro_jazzy
bash setup.sh
pixi run playground
```

Gazebo and RViz open with the robot facing the original house table and three
physical props. Nine native controllers activate, followed by MoveIt and Nav2.
The shared simulator guard permits one robot simulator at a time.

This is the Jazzy source workspace for published `tiago-pro-playground 0.2.0`.
Do not load native libraries from the Humble / Classic `0.1.0` package here.

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
closed-gripper initial pose are retained. Startup and static sensor behavior are verified, and the user confirmed navigation
and MoveIt operation. Contact/grasp behavior has not been revalidated.

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

The packaged 0.2.0 model uses the RGBD camera's color stream for the existing RGB
ROS topic, avoiding a second rendered camera at 640×480 and 30 Hz. This is an
aligned RGBD view, rather than a simulation of separate D435 RGB/depth optical
origins. Image and CameraInfo headers use the depth optical frame; point-cloud
XYZ and its header retain the native depth body frame. Aligned-depth topics carry
the depth image, not the RGBD color image. House materials are converted from
Classic Ogre scripts to Harmonic PBR materials while keeping the plain floor,
lighting, collisions and poses.

## Published package footprint and performance

The Linux x86-64 package is 51,253,587 bytes (48.88 MiB). The installed default
consumer has 888 packages and uses about 4.14 GiB logical / 4.47 GiB allocated
workspace space, excluding the shared global package cache. Hardlinks can make
additional physical disk usage lower. Qt6 and the duplicate LLVM23 stack are
excluded; viewers retain Qt5, with LLVM22 and headless OpenCV5.

A short warm sample improved median real-time factor from 0.59 to 0.83 after RGB
render deduplication. This is machine/view dependent and does not guarantee 1.0.

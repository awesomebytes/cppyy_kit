# TIAGo PRO Jazzy home simulator

Version `0.2.1` build `h4aae1cf_0` is published on the
[awesomebytes channel](https://prefix.dev/channels/awesomebytes/packages/tiago-pro-playground).

Copy `pixi.toml` and `pixi.lock` into an empty directory on Linux x86-64:

```bash
pixi install --locked
pixi run playground
```

Gazebo Harmonic and RViz open with the TIAGo PRO home world. Native controllers,
MoveIt and Nav2 start automatically. A working display and OpenGL are required.
The environment uses ROS Jazzy; the older `0.1.0` package remains available for
ROS Humble and Gazebo Classic.

To add the installed cppyy/rclcpp kit for experiments:

```bash
pixi install -e kit --locked
pixi run -e kit playground
```

Stop the default simulator first: both tasks share the single-simulator guard.
The `kit` feature uses the same locked simulator and ROS dependencies. The default
simulator environment does not require cppyy_kit.

RViz settings are initialized once at
`~/.config/tiago-pro-playground/playground.rviz`; existing settings are preserved.
Writable runtime sessions go under `~/.cache/tiago-pro-playground/`.

The base is a planar velocity approximation with world-pose localization. MoveIt
uses the depth cloud, without manually injected table or ground objects. Native
physical gripper followers preserve wrist sensing; this is not a validated grasp
or wheel-traction physics benchmark.

RGB uses the RGBD sensor's aligned color view at 640×480 and 30 Hz, avoiding a
redundant rendered camera. Cloud coordinates retain the native depth body frame;
image headers use the corresponding optical frame. Wood, brick and object
textures are bundled for offline rendering.

The default installed workspace uses approximately 4.14 GiB logical / 4.47 GiB
allocated space, excluding the shared global cache (888 packages). The custom
package download is 48.96 MiB; dependencies are fetched from the listed channels.

Navigation assumes the ready arm pose. Its padded footprint is 0.88 × 0.70 m,
allowing planning through the house map’s 0.80 m doorway.

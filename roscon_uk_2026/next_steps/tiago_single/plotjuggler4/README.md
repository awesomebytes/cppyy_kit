# PlotJuggler 4 with the TIAGo playground

This isolated Pixi environment launches the user's existing PlotJuggler 4 at
`~/apps/plotjuggler` with Humble ROS libraries available to its native ROS 2 Topic
Subscriber extension. It leaves the simulator environment unchanged.

From the parent `tiago_single` directory:

```bash
pixi run --manifest-path plotjuggler4/pixi.toml view
```

Choose **ROS 2 Topic Subscriber** as the source, start streaming, and select
topics such as `/base_imu`, `/joint_states` or `/mobile_base_controller/odom`.
Expand a numeric field and drag it into a plot. The launcher selects ROS domain
161, CycloneDDS and the playground's loopback configuration.

The native extension loads `librclcpp.so` at runtime. Pixi activation alone does
not expose that library to a separately installed AppImage. `view.sh` prepends
the active environment's `lib` directory to `LD_LIBRARY_PATH` for this launch
only. It clears Python/Qt plugin overrides so the AppImage uses its bundled
runtime. No shell startup files or global extension files are changed.

## Verified locally, 18 October 2026

- PlotJuggler **4.0.0**, native ROS 2 Topic Subscriber **1.0.2**.
- Humble extension loads against this RoboStack environment without missing
  libraries or unresolved symbols.
- GUI receives and plots live TIAGo `/base_imu/linear_acceleration/{x,y,z}`.
- Screenshot: `artifacts/pj4-window.png` (local, ignored).

## Optional WebSocket bridge

The environment also includes upstream `pj-bridge-ros2-humble` **0.10.0**.
`pixi run bridge` exposes domain 161 at `ws://127.0.0.1:9090`; select the
**PlotJuggler Bridge** source in the GUI to use it. Topic discovery was verified
against the running TIAGo, but the native subscriber above was the GUI route
tested. The bridge is unnecessary for native subscription and is not started
by `view`. Stop it with Ctrl-C when finished.

If the existing AppImage is absent, `pixi run bootstrap` downloads and extracts
the official 4.0.0 AppImage into ignored `artifacts/`; `view` uses that fallback.

Sources: [PlotJuggler 4.0.0 release](https://github.com/PlotJuggler/PlotJuggler/releases/tag/4.0.0),
[official bridge and Pixi installation](https://github.com/PlotJuggler/plotjuggler_bridge).

# TIAGo PRO home playground

Keep `pixi.toml` and `pixi.lock` together in an otherwise empty directory. No
repository checkout is required. From that directory run:

```bash
pixi install --locked
pixi run playground
```

The published `tiago-pro-playground` 0.1.0 build 2 comes from the public
`https://prefix.dev/awesomebytes` channel. ROS/Gazebo dependencies come from
RoboStack and conda-forge. The lockfile fixes the validated Linux x86-64 stack.

Requires a working display/OpenGL driver. Press Ctrl-C in the launch terminal
to stop. `pixi run playground-headless` disables Gazebo/RViz windows; camera
rendering still requires graphics support. The default includes native
controllers, depth-camera MoveIt occupancy and Nav2 with simulation-truth
localization. Startup sends no autonomous motion.

The installed environment is about **4.15 GiB logical size**; all compressed
package downloads total **1.10 GiB**. Allocated blocks are about **4.48 GiB**,
including data shared with Pixi's cache through hardlinks. Actual additional
storage depends on packages already cached on the machine.

The package uses Qt5 for Gazebo/RViz and a matching headless OpenCV build to
avoid installing a second Qt stack. ROS image conversion and image processing
remain available; OpenCV's `cv2.imshow` windows are not included.

The default RViz layout is copied once to
`~/.config/tiago-pro-playground/playground.rviz`; existing user configuration is
preserved. Generated models, parameters and logs go into per-run directories
under `~/.cache/tiago-pro-playground`. XDG directory overrides are supported.

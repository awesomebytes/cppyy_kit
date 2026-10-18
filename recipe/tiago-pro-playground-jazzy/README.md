# Build the Jazzy playground package

From the cppyy_kit checkout:

```bash
pixi run -e pkg bash recipe/build_tiago_pro_jazzy.sh
```

The stager verifies the existing source pins, copies description/MoveIt/world
resources and their notices, and builds the Harmonic force-only follower plugin.
It excludes source clones, development prefixes, generated models and build output.
The installed command creates a writable cache session and resolves immutable
resources from its own prefix; it does not require this checkout.

`consumer/` contains the small public-channel workspace for installation. The
optional kit environment shares its solve with the default simulator environment.
Headless OpenCV5 and Clang22 constraints avoid an unused Qt6/LLVM23 stack; Gazebo
and RViz remain graphical. OpenCV's `imshow` is not included in that variant.

Version0.2.0 uses ROS Jazzy and Gazebo Harmonic. The separate
`recipe/tiago-pro-playground` recipe and version0.1.0 retain the Humble/Classic
baseline. Publishing a package does not require invoking the all-package release
workflow. Build artifacts and proof logs belong under ignored `output/`.

The installed RGBD-only configuration measured a median real-time factor of 0.83
in a short warm sample, compared with 0.59 before removing the redundant RGB
render camera on the same workstation. This is not a real-time guarantee; GUI
views, cloud rendering and machine load affect it. Resolution and sensor rate
remain 640×480 at 30 Hz.

Gazebo Sensors8 applies `optical_frame_id` to its point-cloud header too, while
keeping XYZ in its native body convention. The bridge therefore overrides only
the cloud's ROS header to `head_front_camera_depth_frame`. Image and CameraInfo
headers remain optical. This separation is covered by the installed sensor check.

Published artifact: `tiago-pro-playground-0.2.0-h4aae1cf_2.conda` (51,253,587 bytes).
SHA256: `10f5a546798358b8721e274fc23899f6b02ca4d7dfc2350aa54b769530f71889`.
Public repodata and the remote consumer lock match this hash; `pixi install --locked`
passed with the public channel manifest.

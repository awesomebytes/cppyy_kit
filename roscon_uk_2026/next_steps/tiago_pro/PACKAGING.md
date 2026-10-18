# Installable TIAGo PRO playground

For the published Jazzy/Harmonic `0.2.0` simulator, see [the Jazzy guide](../tiago_pro_jazzy/README.md). This page retains the Humble/Classic `0.1.0` baseline.

`tiago-pro-playground` **0.1.0 build 2** is published on the public
[awesomebytes channel](https://prefix.dev/channels/awesomebytes/packages/tiago-pro-playground).
It includes pinned robot descriptions, an offline home world, both patched
native Gazebo plugins, ROS launch configuration and the launcher. It has no
cppyy_kit dependency.

## Install without a repository checkout

Copy [consumer/pixi.toml](../../../recipe/tiago-pro-playground/consumer/pixi.toml)
and [consumer/pixi.lock](../../../recipe/tiago-pro-playground/consumer/pixi.lock)
into an empty directory. Run:

```bash
pixi install --locked
pixi run playground
```

The lock fixes the validated Linux x86-64 environment. A working graphical
display and OpenGL driver are needed for Gazebo cameras and RViz. No system
ROS, Git, compiler invocation or source cloning is needed at first launch.
`pixi run playground-headless` disables viewer windows; camera rendering still
requires graphics support. Startup preserves native initial joint positions
and sends no autonomous motion. Press Ctrl-C in the launch terminal to stop.

RViz's supplied layout is copied once to
`$XDG_CONFIG_HOME/tiago-pro-playground/playground.rviz` (default `~/.config`).
Existing user configuration is never overwritten. Generated URDFs, parameters
and logs go into a new session directory under
`$XDG_CACHE_HOME/tiago-pro-playground` (default `~/.cache`); the launcher prints
its path. Finished session directories may be deleted. Immutable assets stay
under the installation prefix. The installed simulator guard uses the same
shared lock as the repository playgrounds.

Native ROS launch arguments remain available:

```bash
pixi run playground gui:=false rviz:=false
pixi run playground rviz_config:=/absolute/path/to/my_layout.rviz
```

## Published artifact and verification

Artifact: `tiago-pro-playground-0.1.0-h41f06ac_2.conda`, **49,450,386 bytes**
(47.16 MiB). SHA256:
`7f40f59a5e2ad515950191f878d246c475a8e370372c8048d48a6b141f8ed6af`.

The public channel's repodata matches that exact filename, build, size and hash.
The consumer lock contains only public remote URLs, with no local file channel
or checkout paths. `pixi install --locked` succeeded with that remote lock.
Only this playground artifact was uploaded; ROS, Gazebo, Nav2, MoveIt and RViz
remain normal dependencies from RoboStack and conda-forge.

An isolated installation generated the corrected robot/MoveIt configuration,
loaded both native patches, and loaded Gazebo's GUI/video libraries and
`cv_bridge`. The coordinated GUI smoke activated all nine controllers, reached
MoveIt's planning-ready state and activated Nav2. Gazebo and RViz windows were
present and left running. No plugin-load failure or autonomous motion occurred.
Generated models/configuration contained no checkout paths. The saved source
RViz layout was preserved exactly. The final package explicitly requires
`libopenblas`, which the fresh install found missing from Gazebo's upstream
video-plugin dependency chain.

| Measurement of the validated consumer | Size |
|---|---:|
| Workspace logical file size | 4.15 GiB (4,453,255,667 bytes) |
| Workspace allocated blocks (`du`) | 4.48 GiB (4,806,332,416 bytes) |
| Environment prefix logical file size | 4.15 GiB (4,452,678,238 bytes) |
| All 826 locked package downloads, compressed | 1.10 GiB (1,181,361,868 bytes) |
| Playground artifact, compressed | 47.16 MiB |

Thus 1–2 GB describes downloads, not the installed stack. Global caches and
per-run user caches are outside this workspace. Allocated blocks include
shared hardlinks, so they do not measure exclusive additional disk usage.
Source development prefixes, compiler environments and staged build inputs
are excluded. Measurements were taken on 2026-10-18; machine-readable evidence
is in ignored `output/tiago-trim-proof.json`, with public artifact metadata in
`output/tiago-published-record.json` and remote install output in
`output/tiago-remote-install.log`.

## Why only one Qt and LLVM stack is installed

Gazebo/RViz/MoveIt require Qt5. Its explicit LLVM21 dependencies coexist with
an unbounded `libclang13 >=21.1.8` requirement, which initially selected
LLVM/Clang23 as well. Build 2 constrains `libclang13 >=21.1.8,<22` to match
Qt's LLVM21.

Qt6 initially came from OpenCV and newer Qwt packages. Build 2 selects Qwt
6.3.0's Qt5 build `h7c222af_0`, matching Gazebo's `libqwt.so.6.3` ABI, and
OpenCV 4.13.0's headless build `headless_py312h6596e09_6`, matching the validated
OpenCV ABI and dependency generation. Image processing and ROS conversion
remain available. OpenCV HighGUI windows such as `cv2.imshow` are unavailable;
Gazebo and RViz remain the graphical viewers.

These three runtime constraints removed duplicate Qt6 and LLVM/Clang23 stacks
and saved **594 MiB (12.3%)** of installed logical size. No ROS, Gazebo, Qt or
OpenCV library was rebuilt or manually removed for that reduction.

## Build from this repository

From the `cppyy_kit` checkout:

```bash
pixi run -e pkg bash recipe/build_tiago_pro.sh
```

Staging verifies pinned description, MoveIt and plugin source commits and
fetches missing repositories at those exact revisions. It includes selected
ament data packages and bundled world data. No development prefix, Git
metadata, generated URDF or checkout symlink enters the artifact.
Rattler-build compiles both native plugins against Humble Gazebo Classic,
installs real ament resource entries and runs `--prepare-only` in a fresh test
prefix. This generates the corrected model and MoveIt configuration without
starting a simulator. Artifacts and build inputs live under ignored `output/`.

This recipe declares compilers/CMake/Ninja only as build requirements, although
upstream aggregate ROS/Gazebo dependencies still pull development components
such as CMake and Boost headers into the consumer environment. The simulator
version is independent of the Jazzy cppyy kit packages.

## Scope and redistribution

Installed `share/tiago_pro_playground/licenses/THIRD_PARTY.md` identifies
licenses and modifications. Original notices, source revisions, native source
copyright headers and local patches are distributed. PAL and RealSense data
and ROBOTIS house assets are Apache-2.0; OSRF furniture models are CC-BY-3.0;
the physical mimic plugin is BSD-3-Clause. Model metadata preserves authors.

The base uses the documented planar chassis approximation. Navigation uses
simulation-truth odometry with identity map-to-odom, not AMCL. MoveIt uses
point-cloud occupancy without injected ground/table/prop collision shapes.
Packaging does not establish wheel traction, grasping or hardware fidelity.

Later Python ros2_control, image/point-cloud processing or Nav2 plugin examples
can depend on this package and chosen cppyy kits in a separate environment or
metapackage. They should use the existing ROS interfaces and explicit
controller ownership; they are not part of this simulator release.

# Packaging (M1c) — rattler-build recipes + release matrix

Ten `noarch: python` conda packages for the cppyy_kit suite, one recipe dir each.
Every kit is pure Python (it JITs C++ at *runtime* via cppyy — nothing is
compiled at build time), so `noarch: python` is correct and verified: one
artifact per package works on any platform/python, and the build is seconds.

## Packages & dependencies (derived from actual imports)

| conda package | import | run deps (beyond `python`) |
|---|---|---|
| `cppyy-kit` | `cppyy_kit` | `cppyy` |
| `ros-jazzy-rclcpp-kit` | `rclcpp_kit` | `cppyy-kit`, `ros-jazzy-rclcpp`, `ros-jazzy-rclpy`, `ros-jazzy-ament-index-python` |
| `ros-jazzy-bt-kit` | `bt_kit` | `cppyy-kit`, `ros-jazzy-behaviortree-cpp` |
| `ros-jazzy-pcl-kit` | `pcl_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `pcl`, `ros-jazzy-pcl-conversions` |
| `ros-jazzy-ompl-kit` | `ompl_kit` | `cppyy-kit`, `ros-jazzy-ompl` |
| `ros-jazzy-nav2-kit` | `nav2_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-nav2-{costmap-2d,navfn-planner,smac-planner,regulated-pure-pursuit-controller,msgs}` |
| `ros-jazzy-moveit-kit` | `moveit_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-moveit`, `ros-jazzy-ament-index-python` |
| `ros-jazzy-control-kit` | `control_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-ros2-control` |
| `ros-jazzy-cv-kit` | `cv_kit` | `cppyy-kit`, `opencv >=4,<5` |
| `ros-jazzy-dbow-kit` | `dbow_kit` | `cppyy-kit`, `ros-jazzy-cv-kit`, `opencv >=4,<5` |

`bt_kit` and `ompl_kit` do **not** import `rclcpp_kit` (only `cppyy_kit`), so they
carry no rclcpp-kit dependency. `cv_kit` uses a `uintptr_t` buffer bridge (no ROS
headers), so it needs only `opencv`. `dbow_kit` imports `cv_kit`; DBoW2 itself is
vendored/user-built (`build-dbow2`), not a conda dep.

## How the build works

Each package has no committed `setup.py`/`pyproject.toml` (in-repo the kits
resolve via PYTHONPATH). `build.sh` sets `PKG_NAME/PKG_IMPORT/PKG_WHERE` and calls
the shared [`_build_kit.sh`](_build_kit.sh), which writes a minimal
`pyproject.toml` into the *throwaway build tree* (never the repo) and
`pip install`s just that one package. `source: path: ../..` + `use_gitignore`
keeps `.pixi/`, `build/`, `output/` out of the copy.

## Build all + prove (local)

```bash
pixi run -e pkg pkg-build-all   # build 10 in dep order into ./output, chaining
                                # the local output as a file:// channel
pixi run -e pkg pkg-prove       # fresh-env artifact proof per package
```

Dependency build order: `cppyy-kit → rclcpp-kit → cv-kit → {bt,ompl,pcl,nav2,
moveit,control} → dbow-kit`. `./output` is gitignored.

## Version

The suite ships lockstep at one version. It lives per-recipe (`context.version`)
plus the `cppyy-kit ==X` / `ros-jazzy-*-kit ==X` pins in dependent recipes —
rattler-build has no clean cross-recipe single-source for per-dir recipes without
collapsing to a single multi-output recipe (which the per-package layout here
deliberately keeps). Bump every occurrence in one step:

```bash
recipe/bump_version.sh 0.2.0
```

## Release

`v*` tag → [`.github/workflows/release.yml`](../.github/workflows/release.yml):
build all → prove all → `rattler-build upload prefix --channel awesomebytes`
(OIDC). **Before the first release**, Sam must authorize this repo on prefix.dev:
`awesomebytes` channel → Repository Access → `awesomebytes/cppyy_kit`,
`release.yml`, read/write. The rclcppyy authorization does not carry over.

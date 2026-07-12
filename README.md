# cppyy_kit

[![CI](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml/badge.svg)](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml)

A suite of **kits** that let Python drive C++ robotics libraries via
[cppyy](https://cppyy.readthedocs.io) — **prototype at Python speed, run at C++
speed, graduate to AOT** — with first-class documentation and LLM-agent
consumability.

> Prototype normally (plain Python) → switch to the kits and it gets
> automatically more efficient → write unit/integration tests (the contract) →
> apply AOT (freeze/lower) → show the benchmark difference, **while the code
> stays the same or changes minimally.**

The suite is extracted and expanded from the patterns proven in
[rclcppyy](https://github.com/awesomebytes/rclcppyy) (7 spikes, 22 documented
patterns, a measured optimization ladder: PCH freeze 890→6 ms header parse, L2
lowering, 14.8×/9.4× PCL showcase, 6.7–14× TF ingest). See
[`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md) for the approved
architecture and [`PLAN.md`](PLAN.md) for the roadmap. **Headline target: ROSCon
UK 2026.**

## Packages

| Package | Depends on | Content |
|---|---|---|
| **`cppyy_kit`** | cppyy | ROS-free base: friction primitives (load / keep_alive / callback / HandleRegistry / warmup / first_use / teardown / probe), `freeze` (PCH + vendored-source tooling), plus the compile cache, `require()`, `@cpp`, `nogil`, stubs, and capability/fallback. |
| **`rclcpp_kit`** | cppyy_kit, rclcpp | The kit for rclcpp (ROS 2 core): bringup, C++ message resolution/conversion, serialization, rosbag2, **tf**, executor/node helpers. Carved out of rclcppyy. |
| **`bt_kit`** | cppyy_kit | BehaviorTree.CPP v4 from Python. |
| **`pcl_kit`** | cppyy_kit (+ rclcpp) | Point Cloud Library; clouds stay in C++ end to end. |
| **`ompl_kit`** | cppyy_kit | Open Motion Planning Library. |
| **`nav2_kit`** | cppyy_kit (+ rclcpp) | Nav2 algorithm cores (Costmap2D + NavFn) composed from Python. |
| **`moveit_kit`** | cppyy_kit (+ rclcpp) | The full MoveIt 2 C++ API from Python. |
| **`control_kit`** | cppyy_kit (+ rclcpp) | A Python ros2_control controller inside the real controller_manager. |
| **`cv_kit`** | cppyy_kit (+ rclcpp) | OpenCV C++ API with a zero-copy `sensor_msgs/Image` → `cv::Mat` bridge. |
| **`dbow_kit`** | cppyy_kit | DBoW2 place recognition / loop closure (no Python binding, not on conda-forge). |
| **`wbc_kit`** | cppyy_kit | Whole-body control: custom Crocoddyl action models authored inline in C++, JIT-compiled with no build system (ROS-free). |

Each kit is a top-level package with its own Python package, `demos/`, `tests/`,
optional `cpp/`, and `SKILL.md` / `WHY.md` / `REPORT.md` docs (kit anatomy —
[`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md) §4.4).

## Quickstart

Requires [pixi](https://pixi.sh). The default env is the ROS/cppyy stack; each
kit's C++ dependency is an additive feature env.

```bash
# lint + the default (auto-skipping) test suite — the CI gate
pixi run lint
pixi run test

# a kit: install its env, run its demo + test suite
pixi run -e bt   demo-bt-t01      # BehaviorTree.CPP first tree, in short Python
pixi run -e bt   test-bt          # bt_kit + base cppyy_kit tests
pixi run -e ompl demo-ompl-plan   # OMPL 2D plan
pixi run -e nav2 test-nav2        # Nav2 cores from Python
pixi run -e control test-control  # a Python controller in the real controller_manager
```

Kit demos/tests are discovered via `PYTHONPATH` (set in `pixi.toml`
`[activation.env]`): the repo root plus each kit dir. ROS-touching kits get the ROS
2 core through **`rclcpp_kit`** (a local package on that path) plus the default
`ros-base` env — no extra per-kit ROS dependency. The `rclcpp_kit` suite + tf demos
run in the `rclcpp` env: `pixi run -e rclcpp test-rclcpp` / `test-tf`.

## Install (conda packages)

> **Published.** The suite ships as 11 rattler-build recipes under
> [`recipe/`](recipe/) via a tag-triggered
> [release workflow](.github/workflows/release.yml); `v0.1.0` is live on the prefix.dev
> `awesomebytes` channel. The snippets below work as-is (or use the repo directly per
> the Quickstart above).

Each package is pure-Python (`noarch`) and installs into any pixi/conda env; its
C++ dependency is declared as a run dependency and pulled by the solver. Add the
`awesomebytes` channel (plus `robostack-jazzy` + `conda-forge` for the ROS/C++
deps):

```toml
# pixi.toml
[workspace]
channels = ["https://prefix.dev/awesomebytes", "robostack-jazzy", "conda-forge"]
platforms = ["linux-64"]

[dependencies]
cppyy-kit = "*"                  # ROS-free base (cppyy only)
wbc-kit = "*"                    # Crocoddyl custom action models (ROS-free)
ros-jazzy-rclcpp-kit = "*"       # rclcpp core: bringup, messages, tf, rosbag2
ros-jazzy-bt-kit = "*"           # BehaviorTree.CPP v4
ros-jazzy-pcl-kit = "*"          # Point Cloud Library
ros-jazzy-ompl-kit = "*"         # Open Motion Planning Library
ros-jazzy-nav2-kit = "*"         # Nav2 algorithm cores
ros-jazzy-moveit-kit = "*"       # MoveIt 2
ros-jazzy-control-kit = "*"      # ros2_control
ros-jazzy-cv-kit = "*"           # OpenCV C++ (zero-copy Image->cv::Mat)
ros-jazzy-dbow-kit = "*"         # DBoW2 loop closure (run build-dbow2 once)
```

Or `pixi add -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge ros-jazzy-bt-kit`.
Install only what you need — every kit pulls `cppyy-kit`, and the ROS-touching
kits pull `ros-jazzy-rclcpp-kit`, transitively.

## Docs

- [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md) — the approved kit-suite architecture.
- [`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md) — the canonical cppyy playbook (35 patterns).
- [`docs/FREEZE.md`](docs/FREEZE.md) — the L0→L1→L2 optimization ladder.
- [`docs/tutorials/`](docs/tutorials/) — end-to-end tutorials (visual loop closure).
- Per kit: `<kit>/SKILL.md` (LLM-facing), `<kit>/WHY.md` (the pitch), `<kit>/REPORT.md` (evidence).

## Make it faster with an LLM: the `cppyy-accelerate` skill

Point a coding agent at slow Python and ask it to speed it up:
[`skills/cppyy-accelerate/SKILL.md`](skills/cppyy-accelerate/SKILL.md) is a
Claude-Code-consumable procedure — **PROFILE** (a cProfile + boundary-tracer
wrapper), **MAP** (a decision tree from hotspot shape to the right kit/pattern, with
an honest DON'T list), **APPLY** (minimal diff per the kit `SKILL.md`), **VERIFY**
(tests-as-contract + a before/after table). The worked example
([`WALKTHROUGH.md`](skills/cppyy-accelerate/WALKTHROUGH.md)) accelerates a naive
Python voxel downsampler **15.6× (47.9 ms → 3.07 ms)** with identical output —
`pixi run -e pcl test-accelerate` (the contract) and `bench-accelerate` (the table).

## Status

**Shipped and published.** The migration off rclcppyy is complete, the base is
enriched (compile cache, `require()`, `@cpp`, `nogil`, stubs, capability/fallback), the
documentation site is live, the `cppyy-accelerate` LLM skill is out, and every demo
lane is delivered. All 11 packages are released as `v0.1.0` on the prefix.dev
`awesomebytes` channel, and rclcppyy is slimmed to thin re-export shims over
`rclcpp_kit` (0.2.0). The suite is extracted from
[rclcppyy](https://github.com/awesomebytes/rclcppyy) with git history; every
ROS-touching kit imports `rclcpp_kit` directly.

Still to come:

- Presentation assets for ROSCon UK 2026.

## License

BSD 3-Clause — see [`LICENSE`](LICENSE).

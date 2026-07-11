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
| **`cppyy_kit`** | cppyy | ROS-free base: friction primitives (load / keep_alive / callback / HandleRegistry / warmup / first_use / teardown / probe), `freeze` (PCH + vendored-source tooling). Enriched in M2 (compile cache, `require()`, `@cpp`, `nogil`, stubs, capability/fallback). |
| **`rclcpp_kit`** | cppyy_kit, rclcpp | The kit for rclcpp (ROS 2 core): bringup, C++ message resolution/conversion, serialization, rosbag2, **tf**, executor/node helpers. **Placeholder — arrives in M1b.** |
| **`bt_kit`** | cppyy_kit | BehaviorTree.CPP v4 from Python. |
| **`pcl_kit`** | cppyy_kit (+ rclcpp) | Point Cloud Library; clouds stay in C++ end to end. |
| **`ompl_kit`** | cppyy_kit | Open Motion Planning Library. |
| **`nav2_kit`** | cppyy_kit (+ rclcpp) | Nav2 algorithm cores (Costmap2D + NavFn) composed from Python. |
| **`moveit_kit`** | cppyy_kit (+ rclcpp) | The full MoveIt 2 C++ API from Python. |
| **`control_kit`** | cppyy_kit (+ rclcpp) | A Python ros2_control controller inside the real controller_manager. |
| **`cv_kit`** | cppyy_kit (+ rclcpp) | OpenCV C++ API with a zero-copy `sensor_msgs/Image` → `cv::Mat` bridge. |
| **`dbow_kit`** | cppyy_kit | DBoW2 place recognition / loop closure (no Python binding, not on conda-forge). |

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
`[activation.env]`): the repo root plus each kit dir. Every ROS-touching kit env
also pulls the **`rclcpp` bridge** — see below.

## Docs

- [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md) — the approved kit-suite architecture.
- [`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md) — the canonical cppyy playbook (22 patterns).
- [`docs/FREEZE.md`](docs/FREEZE.md) — the L0→L1→L2 optimization ladder.
- [`docs/tutorials/`](docs/tutorials/) — end-to-end tutorials (visual loop closure).
- Per kit: `<kit>/SKILL.md` (LLM-facing), `<kit>/WHY.md` (the pitch), `<kit>/REPORT.md` (evidence).

## Status: M1a (migration & bootstrap)

This repo was bootstrapped by migrating the kit suite out of rclcppyy **with git
history** (`git log --follow` traces any migrated file back into rclcppyy).
Still to come:

- **M1b** — carve the real `rclcpp_kit` out of rclcppyy (bringup, messages,
  serialization, rosbag2, tf). Until then, ROS-touching kits import the rclcppyy
  product via the **`rclcpp` bridge**: the `ros-jazzy-rclcppyy` conda package,
  pulled into the ROS-touching feature envs. When M1b lands, those imports switch
  to `rclcpp_kit` and the bridge feature is removed.
- **M1c** — per-package rattler-build recipes + tag-triggered release matrix to
  the prefix.dev `awesomebytes` channel (which replaces the PYTHONPATH mechanism
  with proper editable/conda installs).

## License

BSD 3-Clause — see [`LICENSE`](LICENSE).

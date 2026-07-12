# cppyy_kit

[![CI](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml/badge.svg)](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml)

**Prototype in Python, run at C++ speed — mix Python and C++ with ease.**

cppyy_kit is a suite of *kits* that drive real C++ robotics libraries from short
Python via [cppyy](https://cppyy.readthedocs.io). No bindings to write, no code
generation, no build step: the C++ library you already have installed is called
directly, its own class and method names intact, while your Python does the
orchestration. When a hot path needs C++ speed, you write that path in C++ inline —
in the same file — and the kits handle the data marshaling and object lifetime
across the boundary.

You get the productivity of a Python prototype and the performance of the C++
library underneath it, and the same code climbs an optimization ladder (freeze the
startup cost, cache the compile, lower a hot leaf) without changing shape. Every
number on this page is measured on one machine on one day and linked to the row that
produced it in **[docs/benchmarks.md](docs/benchmarks.md)**.

## Two ways to mix Python and C++

**Drive a whole C++ library from Python** — here the real BehaviorTree.CPP engine
parses the XML, owns the tree, and ticks it; there is no Python binding for it, so
this capability does not otherwise exist:

```python
import bt_kit
bt = bt_kit.bringup_bt()
tree = bt.BehaviorTreeFactory().create_tree_from_text(xml)   # the C++ factory
tree.tickWhileRunning()                                      # the C++ engine ticks
```

**Or drop to inline C++ for a hot kernel** — the decorated function's docstring *is*
its C++ body; its annotations drive the NumPy marshaling; it compiles once and is
cached to a real `.so` thereafter (no first-use JIT after the first build):

```python
import numpy as np
from cppyy_kit import cpp

@cpp
def sum_sq(data: cpp.arr("float")) -> float:      # numpy -> (float* data, size_t data_size)
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

sum_sq(np.array([1, 2, 3], np.float32))            # 14.0 — no ctypes, no build, no bindings
```

The friction that both share — locating and loading the `.so`s, pinning callback
lifetimes, hiding cppyy's template and ownership sharp edges — is factored into the
`cppyy_kit` base, so each domain kit stays thin and each kit's Python mirrors the
library's own API 1:1.

## Install

**Published.** The suite ships as 11 conda packages on the prefix.dev
`awesomebytes` channel (browse: <https://repo.prefix.dev/awesomebytes>). Each package
is pure-Python (`noarch`); its C++ dependency is a run dependency the solver pulls
in. `cppyy-kit` and `wbc-kit` are distro-free; the ROS-touching kits are published as
`ros-jazzy-*`.

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

Or add one at a time:
`pixi add -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge ros-jazzy-bt-kit`.
Install only what you need — every kit pulls `cppyy-kit`, and the ROS-touching kits
pull `ros-jazzy-rclcpp-kit`, transitively. To hack on the suite instead, see
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/).

## Showcase

### The kits — a C++ library each, driven from Python

| Kit | What it drives | Headline |
|---|---|---|
| **[cppyy_kit](docs/COMMON_PATTERNS.md)** (base) | the ROS-free machinery: load / callback / lifetime, `@cpp`, `require`, `nogil`, [freeze & compile-cache](docs/FREEZE.md) | first-use JIT paid once per machine: 632 → 91 ms on the PCL VoxelGrid kernel [↗](docs/benchmarks.md#pcl-compile-cache--frame-0-first-use-jit-vs-cached) |
| **[rclcpp_kit](rclcpp_kit/WHY.md)** | rclcpp (ROS 2 core): bringup, messages, tf, rosbag2, CDR | TF ingest **7.4–16.9×** lower CPU [↗](docs/benchmarks.md#tf-ingest--c-tf2-listener-vs-python-callback) |
| **[bt_kit](bt_kit/WHY.md)** | BehaviorTree.CPP v4 (no Python binding exists) | Groot2-compatible trees from Python; cache 218→62 ms [↗](docs/benchmarks.md#bt_kit-compile-cache--t01-cold-run-adoption) |
| **[pcl_kit](pcl_kit/WHY.md)** | Point Cloud Library (no maintained binding) | **15.1× latency / 7.4× CPU** at 74-LOC parity [↗](docs/benchmarks.md#pcl-showcase--cloud-stays-in-c-end-to-end) |
| **[ompl_kit](ompl_kit/WHY.md)** | Open Motion Planning Library | Python validity-checker in the planner's inner loop, no codegen [↗](ompl_kit/REPORT.md) |
| **[nav2_kit](nav2_kit/WHY.md)** | Nav2 algorithm cores, composed from Python | the real RegulatedPurePursuit with **no lifecycle servers / no pluginlib** [↗](nav2_kit/REPORT.md) |
| **[moveit_kit](moveit_kit/WHY.md)** | the full MoveIt 2 C++ API | the *whole* C++ surface, not `moveit_py`'s curated subset [↗](moveit_kit/REPORT.md) |
| **[control_kit](control_kit/WHY.md)** | ros2_control | a Python controller in the *real* controller_manager (ros2_control has no Python API) [↗](control_kit/REPORT.md) |
| **[cv_kit](cv_kit/WHY.md)** | OpenCV C++ | zero-copy `sensor_msgs/Image` → `cv::Mat`, one CUDA branch point [↗](cv_kit/REPORT.md) |
| **[dbow_kit](dbow_kit/WHY.md)** | DBoW2 place recognition (no binding, not on conda-forge) | vendored, compiled once, loop closure from short Python [↗](dbow_kit/REPORT.md) |
| **[wbc_kit](wbc_kit/WHY.md)** | Crocoddyl custom action models | inline-C++ model, **no build system** [↗](docs/benchmarks.md#wbc--custom-crocoddyl-action-model-python-derived-vs-inline-c) |

Each kit is a package with its own Python module, `demos/`, `tests/`, optional
`cpp/`, and a `WHY.md` (the rationale) / `REPORT.md` (the evidence) / `SKILL.md`
(LLM-facing cheat sheet) trio — the anatomy in [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md).

### Demos & examples

Every headline links to the exact row that produced it in
[docs/benchmarks.md](docs/benchmarks.md).

| Demo | What it proves | Headline number |
|---|---|---|
| [Live webcam A vs B](docs/webcam_demo/REPORT.md) | a hand-written NCC tracker in one inline-C++ kernel vs the identical NumPy loop | **16.18×** @ 640×480 [↗](docs/benchmarks.md#webcam-demo--a-cppyy_kit-c-vs-b-naive-python) |
| [IK 5-solver bench](docs/ik_bench/WHY.md) | benchmark C++-only IK solvers (incl. unpackaged bio_ik/pick_ik) from *one* Python file | pure-Python **10–25× slower**; bio_ik 991 solve/s [↗](docs/benchmarks.md#ik-benchmark--same-panda-same-200-targets-per-solver-subprocess) |
| [WBC inline-C++ model](docs/wbc/REPORT.md) | a custom Crocoddyl action model authored inline, JIT-compiled, no CMake | **22.9×** vs Python-derived, bit-identical cost [↗](docs/benchmarks.md#wbc--custom-crocoddyl-action-model-python-derived-vs-inline-c) |
| [Retargeting teleop rig](docs/retarget_pipeline/REPORT.md) | webcam → body/hand tracking → TF → whole-body retarget onto G1/Talos, live, one Rerun viewer | glue kernel **341.5×**, /tf marshaling **258.9×**, bit-identical [↗](docs/benchmarks.md#retarget-pipeline--perception-tf-marshaling--retarget-glue-kernel) |
| [Visual loop closure](docs/tutorials/vision_loop_closure.md) | ORB + DBoW2 + GTSAM front-end in short Python, pixels never leave C++ | 1080p ingest **135.8×**; 19 loops, precision/recall 1.00/0.95 [↗](docs/benchmarks.md#vision--cv_kit--dbow_kit-synthetic-sequence) |
| [Jitter bench](docs/jitter_bench/REPORT.md) | a ~1 kHz control loop orchestrated from Python on a *stock* kernel | **~2 µs median** period, unprivileged [↗](docs/benchmarks.md#jitter-bench--reduced-reference-set-a1--b--c-idle-60-s-each) |
| [cppyy-accelerate skill](skills/cppyy-accelerate/SKILL.md) | point a coding agent at slow Python; it moves the hot path to a kit | **16.3×** (49.6 → 3.04 ms), output bit-identical [↗](docs/benchmarks.md#accelerate--the-llm-skill-worked-example) |

### Where the speedups apply — and where they don't

The webcam gap is large because the hot per-frame stage is a hand-written per-pixel
NCC tracker with no OpenCV one-liner. When the per-frame work is only
library-provided ops (ORB, RANSAC — `cv2` is already C++), the same A-vs-B
comparison narrows to ~1.1–1.2×
([webcam report](docs/webcam_demo/REPORT.md#the-a-vs-b-table)).

In the retargeting rig, the measured cppyy wins are the `/tf` message marshaling and
the transform/retarget kernel. The IK solve runs on pinocchio's own Python bindings:
instantiating `pinocchio::Model` from headers under Cling trips boost 1.90's variant
template-arity limit (pinocchio's 25-type joint `boost::variant`), so that path
cannot be JIT-parsed
([retarget report](docs/retarget_pipeline/REPORT.md#the-cppyy_kit-win-here-retarget-glue-and-the-honest-boundary-on-the-solve)).

The benchmarks ran on a shared development machine, so the ratios are more repeatable
than the absolute times.

## The optimization ladder

The same code climbs rungs as you need more speed — the kit API does not change:

- **Prototype (L0).** Plain Python driving the kit. Headers are parsed and
  per-signature wrappers JIT-compiled on first use. Fastest to write.
- **Accelerate.** Move the hot path onto C++ via a kit, `@cpp`, or `nogil` — 15.1×
  lower latency in the
  [PCL pipeline benchmark](docs/benchmarks.md#pcl-showcase--cloud-stays-in-c-end-to-end),
  where the cloud stays in C++ end to end at 74-LOC parity.
- **Freeze.** A zero-config Cling PCH of the library headers is built once into
  `~/.cache/cppyy_kit` and auto-loaded thereafter, eliminating the header parse —
  ~27× on rclcpp bringup (~1.73 s → 0.064 s) in the
  [auto-PCH measurement](docs/benchmarks.md#auto-pch--zero-config-cold-vs-warm-bringup).
  The compile cache does the same for `@cpp`/`cppdef` kernels: the first-use JIT —
  632 → 91 ms on the
  [PCL VoxelGrid kernel](docs/benchmarks.md#pcl-compile-cache--frame-0-first-use-jit-vs-cached)
  — is paid once per machine, not once per process.
- **Lower (L2).** A proven-hot leaf is authored as a native C++ node — 22.9× on the
  [WBC Crocoddyl action model](docs/benchmarks.md#wbc--custom-crocoddyl-action-model-python-derived-vs-inline-c)
  (bit-identical cost), removing the per-call cppyy boundary.

Read the full ladder in [`docs/FREEZE.md`](docs/FREEZE.md) and the 36 documented
patterns behind it in [`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md).

## Powers rclcppyy

`rclcpp_kit` is the capability layer under
[**rclcppyy**](https://github.com/awesomebytes/rclcppyy) — the drop-in accelerator
that lets an existing rclpy program run ROS 2's C++ core (rclcpp, tf2, rosbag2, CDR
serialization) with minimal changes. rclcppyy 0.2.0 is now thin re-export shims over
`rclcpp_kit`, and installs from the same channel as `ros-jazzy-rclcppyy`. If you have
an rclpy node paying Python for work that is fundamentally C++ (TF ingest, per-message
marshaling), `rclcpp_kit` moves that work into C++.

## Built for LLM agents

Agent-consumability is a design goal:

- Every kit ships a `SKILL.md` — a compact, LLM-facing cheat sheet of its real API.
- [`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md) is the shared playbook (36
  patterns) a coding agent reads before writing a new kit or a new call.
- The [`cppyy-accelerate`](skills/cppyy-accelerate/SKILL.md) skill is a
  Claude-Code-consumable procedure — **PROFILE** (a cProfile + boundary-tracer),
  **MAP** (hotspot shape → the right kit/pattern, with a list of when not to use it),
  **APPLY** (a minimal diff per the kit `SKILL.md`), **VERIFY** (tests-as-contract +
  a before/after table). Its [worked example](skills/cppyy-accelerate/WALKTHROUGH.md)
  accelerates a naive voxel downsampler **16.3×** with bit-identical output.

## Docs

Full documentation site: **<https://awesomebytes.github.io/cppyy_kit/>**

- [The Patterns](docs/COMMON_PATTERNS.md) — the canonical cppyy playbook (36 patterns).
- [Freeze & Cache](docs/FREEZE.md) — the L0 → L1 → L2 + compile-cache ladder.
- [Benchmarks](docs/benchmarks.md) — every number on this page, one machine, one day, reproducible.
- [Architecture](docs/ARCHITECTURE_V2.md) — how the suite is put together.
- [Tutorials](docs/tutorials/vision_loop_closure.md) — end-to-end walkthroughs.
- Per kit: its **Why** (the pitch), **Report** (the evidence), **Skill** (LLM cheat sheet).

Questions, ideas, and bug reports are welcome on the
[issue tracker](https://github.com/awesomebytes/cppyy_kit/issues).

## License

BSD 3-Clause — see [`LICENSE`](LICENSE).

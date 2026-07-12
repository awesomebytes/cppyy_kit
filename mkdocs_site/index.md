# cppyy_kit

**Prototype in Python, run at C++ speed — mix Python and C++ with ease.**

cppyy_kit is a suite of *kits* that drive real C++ robotics libraries from short
Python via [cppyy](https://cppyy.readthedocs.io). No bindings to write, no code
generation, no build step: the C++ library you already have installed is called
directly, its own class and method names intact, while your Python does the
orchestration. When a hot path needs C++ speed, you write that path in C++ inline —
in the same file — and the kits make the crossing invisible.

You get the productivity of a Python prototype and the performance of the C++ library
underneath it, and the same code climbs an optimization ladder (freeze the startup
cost, cache the compile, lower a hot leaf) without changing shape. Every number on
this site is measured on one machine on one day and linked to the row that produced
it in **[Benchmarks](docs/benchmarks.md)**.

## Two ways to mix Python and C++

**Drive a whole C++ library from Python** — the real BehaviorTree.CPP engine parses
the XML, owns the tree, and ticks it; there is no Python binding for it, so this
capability does not otherwise exist:

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

The friction both share — locating and loading the `.so`s, pinning callback
lifetimes, hiding cppyy's template and ownership sharp edges — is factored into the
`cppyy_kit` base, so each domain kit stays thin and its Python mirrors the library's
own API 1:1.

## The measured story

Every claim here is measured; the [Benchmarks](docs/benchmarks.md) page is the single
consolidated source (one machine, one day, reproducible commands), and each kit's
`REPORT.md` carries the original evidence.

| Lever | Result |
|---|---|
| **Accelerate** — PCL cloud stays in C++ end to end | [**15.1× latency / 7.4× CPU**](docs/benchmarks.md#pcl-showcase-cloud-stays-in-c-end-to-end) at 74-LOC parity |
| **Freeze** — zero-config Cling PCH of the library headers | rclcpp bringup [**~1.73 s → 0.064 s (~27×)**](docs/benchmarks.md#auto-pch-zero-config-cold-vs-warm-bringup), header parse eliminated |
| **Compile cache** — content-hashed `@cpp`/`cppdef` → `.so` | the ~**0.69 s** first-use JIT paid [once per machine, ever](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached) |
| **Lower (L2)** — hot leaf authored as native C++ | inline Crocoddyl model [**22.9×**](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c), bit-identical |
| **TF ingest** — C++ `tf2` listener vs Python callback | [**7.4–16.9×**](docs/benchmarks.md#tf-ingest-c-tf2-listener-vs-python-callback) lower ingest CPU |

## The kits

| Kit | What it drives | Headline |
|---|---|---|
| **[cppyy_kit](kits/cppyy_kit.md)** (base) | the ROS-free machinery: load / callback / lifetime, `@cpp`, `require`, `nogil`, [freeze & compile-cache](docs/FREEZE.md) | eliminates the ~0.69 s first-use JIT [persistently](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached) |
| **[rclcpp_kit](rclcpp_kit/WHY.md)** | rclcpp (ROS 2 core): bringup, messages, tf, rosbag2, CDR | TF ingest **7.4–16.9×** lower CPU |
| **[bt_kit](bt_kit/WHY.md)** | BehaviorTree.CPP v4 (no Python binding exists) | Groot2-compatible trees from Python |
| **[pcl_kit](pcl_kit/WHY.md)** | Point Cloud Library (no maintained binding) | **15.1× latency / 7.4× CPU** at LOC parity |
| **[ompl_kit](ompl_kit/WHY.md)** | Open Motion Planning Library | Python validity-checker in the planner's inner loop, no codegen |
| **[nav2_kit](nav2_kit/WHY.md)** | Nav2 algorithm cores, composed from Python | the real RegulatedPurePursuit with **no lifecycle servers / no pluginlib** |
| **[moveit_kit](moveit_kit/WHY.md)** | the full MoveIt 2 C++ API | the *whole* C++ surface, not `moveit_py`'s subset |
| **[control_kit](control_kit/WHY.md)** | ros2_control | a Python controller in the *real* controller_manager |
| **[cv_kit](cv_kit/WHY.md)** | OpenCV C++ | zero-copy `Image` → `cv::Mat`, one CUDA branch point |
| **[dbow_kit](dbow_kit/WHY.md)** | DBoW2 place recognition (no binding, not on conda-forge) | loop closure from short Python |
| **[wbc_kit](docs/wbc/REPORT.md)** | Crocoddyl custom action models | inline-C++ model, **no build system** |

Each kit is a package with a `WHY.md` (the pitch), `REPORT.md` (the evidence), and
`SKILL.md` (the LLM-facing cheat sheet); the anatomy is in
[Architecture](docs/ARCHITECTURE_V2.md).

## Demos & examples — the thesis, measured

Every headline links to the exact row that produced it in
[Benchmarks](docs/benchmarks.md).

| Demo | What it proves | Headline number |
|---|---|---|
| [Live webcam A vs B](docs/webcam_demo/REPORT.md) | a hand-written NCC tracker in one inline-C++ kernel vs the identical NumPy loop | [**16.18×**](docs/benchmarks.md#webcam-demo-a-cppyy_kit-c-vs-b-naive-python) @ 640×480 |
| [IK 5-solver bench](docs/ik_bench/WHY.md) | benchmark C++-only IK solvers (incl. unpackaged bio_ik/pick_ik) from *one* Python file | pure-Python [**10–25× slower**](docs/benchmarks.md#ik-benchmark-same-panda-same-200-targets-per-solver-subprocess); bio_ik 991 solve/s |
| [WBC inline-C++ model](docs/wbc/REPORT.md) | a custom Crocoddyl action model authored inline, JIT-compiled, no CMake | [**22.9×**](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c) vs Python-derived, bit-identical |
| [Retargeting teleop rig](docs/retarget_pipeline/REPORT.md) | webcam → body/hand tracking → TF → whole-body retarget onto G1/Talos, live, one Rerun viewer | glue kernel [**341.5×**](docs/benchmarks.md#retarget-pipeline-perception-tf-marshaling-retarget-glue-kernel), /tf marshaling 258.9× |
| [Visual loop closure](docs/tutorials/vision_loop_closure.md) | ORB + DBoW2 + GTSAM front-end in short Python, pixels never leave C++ | 1080p ingest [**135.8×**](docs/benchmarks.md#vision-cv_kit-dbow_kit-synthetic-sequence); 19 loops, P/R 1.00/0.95 |
| [Jitter bench](docs/jitter_bench/REPORT.md) | a ~1 kHz control loop orchestrated from Python on a *stock* kernel | [**~2 µs median**](docs/benchmarks.md#jitter-bench-reduced-reference-set-a1-b-c-idle-60-s-each) period, unprivileged |
| [cppyy-accelerate skill](skills/cppyy-accelerate/SKILL.md) | point a coding agent at slow Python; it moves the hot path to a kit | [**16.3×**](docs/benchmarks.md#accelerate-the-llm-skill-worked-example) (49.6 → 3.04 ms), bit-identical |

**Honest by design.** The webcam win is dramatic *because the hot stage is a custom
kernel with no OpenCV one-liner*; when the per-frame work is only library-provided ops
(`cv2` is C++ too) the gap collapses to ~1.1–1.2×, and we say so. In the retargeting
rig the cppyy wins are in the glue (marshaling + the transform kernel) — the IK
*solve* itself is a pinocchio-bindings job where cppyy is blocked by a documented
wall, and that boundary is reported, not hidden. Benchmarks ran on a shared
development machine; treat absolute numbers as directional and ratios as the stabler
signal.

## The optimization ladder

The same code climbs rungs as you need more speed — the kit API does not change:

- **Prototype (L0).** Plain Python driving the kit. Headers parsed and per-signature
  wrappers JIT-compiled on first use. Fastest to write.
- **Accelerate.** Move the hot path onto C++ via a kit, `@cpp`, or `nogil` — the PCL
  showcase keeps the cloud in C++ end to end for
  [15.1× lower latency](docs/benchmarks.md#pcl-showcase-cloud-stays-in-c-end-to-end).
- **Freeze.** A zero-config Cling PCH of the library headers is built once into
  `~/.cache/cppyy_kit` and auto-loaded thereafter, eliminating the header parse
  ([~27×](docs/benchmarks.md#auto-pch-zero-config-cold-vs-warm-bringup) on rclcpp
  bringup). The compile cache pays the ~0.69 s first-use JIT of an `@cpp`/`cppdef`
  kernel [once per machine](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached).
- **Lower (L2).** A proven-hot leaf is authored as a native C++ node — the WBC inline
  Crocoddyl model runs
  [22.9× faster](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c),
  bit-identical, removing the per-call cppyy boundary entirely.

Read the full ladder in **[Freeze & Cache](docs/FREEZE.md)** and the 36 hard-won
patterns behind it in **[The Patterns](docs/COMMON_PATTERNS.md)**.

## Powers rclcppyy

`rclcpp_kit` is the capability layer under
[**rclcppyy**](https://github.com/awesomebytes/rclcppyy) — the drop-in accelerator
that lets an existing rclpy program run ROS 2's C++ core (rclcpp, tf2, rosbag2, CDR
serialization) with minimal changes. rclcppyy 0.2.0 is now thin re-export shims over
`rclcpp_kit`, and installs from the same channel as `ros-jazzy-rclcppyy`.

## Built for LLM agents

Agent-consumability is a first-class design goal: every kit ships a `SKILL.md` (a
compact, LLM-facing cheat sheet of its real API), [The Patterns](docs/COMMON_PATTERNS.md)
is the shared playbook a coding agent reads before writing a new kit or call, and the
[cppyy-accelerate](skills/cppyy-accelerate/SKILL.md) skill is a Claude-Code-consumable
PROFILE → MAP → APPLY → VERIFY procedure whose
[worked example](skills/cppyy-accelerate/WALKTHROUGH.md) accelerates a naive voxel
downsampler **16.3×** with bit-identical output.

## Next steps

- **[Getting Started](getting-started.md)** — install the packages, or develop from the repo.
- **[The Patterns](docs/COMMON_PATTERNS.md)** — the canonical cppyy playbook.
- **[Benchmarks](docs/benchmarks.md)** — every number here, one machine, one day, reproducible.
- **[Architecture](docs/ARCHITECTURE_V2.md)** — how the suite is put together.

---

*Origin: extracted and expanded from [rclcppyy](https://github.com/awesomebytes/rclcppyy),
which it now powers.*

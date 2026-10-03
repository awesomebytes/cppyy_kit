# cppyy_kit

[![CI](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml/badge.svg)](https://github.com/awesomebytes/cppyy_kit/actions/workflows/ci.yml)

**Drive C++ robotics libraries from Python with cppyy.**

**New here? Start with [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/)**
for installation or development instructions.

cppyy_kit provides Python interfaces to C++ robotics libraries through
[cppyy](https://cppyy.readthedocs.io). For supported APIs, cppyy reads installed
headers and JIT-compiles wrappers, so a separate binding or extension build is not
usually needed for each library. Kits expose selected native APIs and provide
helpers for data conversion and object lifetime. The libraries, headers, and other
native dependencies must be installed in the environment.

Here, **cppyy** is the C++-to-Python runtime, **cppyy_kit** is the shared ROS-free
base package, and **domain kits** such as `bt_kit` or `pcl_kit` expose selected APIs
from particular C++ libraries. Installing the base package does not install every
domain kit or its native library.

The suite includes options to reduce repeated header parsing and wrapper compilation,
and to write selected hot paths in C++. Results below link to the corresponding
benchmarks.

<p align="center">
  <img src="docs/media/cppyy_kit_logo.jpg" alt="cppyy_kit logo" width="420">
</p>

## Three ways to mix Python and C++

**Drive BehaviorTree.CPP from Python.** The C++ engine parses the XML, owns the tree,
and ticks it through cppyy:

```python
import bt_kit
bt = bt_kit.bringup_bt()
xml = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree"><AlwaysSuccess/></BehaviorTree>
</root>
"""
factory = bt.BehaviorTreeFactory()
tree = factory.create_tree_from_text(xml)                    # the C++ factory
status = tree.tickWhileRunning()                             # the C++ engine ticks
print(status == bt.NodeStatus.SUCCESS)  # True
```

**Write an inline C++ kernel.** The decorated function's docstring is its C++ body;
its annotations define argument marshaling. By default the compiled artifact is
cached; a compatible cache hit avoids recompiling the wrapper:

```python
import numpy as np
from cppyy_kit import cpp

@cpp
def sum_sq(data: cpp.arr("float")) -> float:      # numpy -> (float* data, size_t data_size)
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

sum_sq(np.array([1, 2, 3], np.float32))            # 14.0; no manual ctypes conversion
```

**Run independent kernels concurrently.** `@cpp(nogil=True)` releases the GIL around
the C++ body, allowing other Python threads to run during that work. On the 16-core
test machine, the eight-job CPU-bound example took about **150 ms with the GIL held
and 20 ms with it released** (about **7.7×**). The
[runnable example](examples/parallel_demo/parallel_demo.py) has the full version:

```python
import threading, numpy as np
from cppyy_kit import cpp

@cpp(nogil=True)                                   # the compiled body runs with the GIL released
def crunch(out: "double*", slot: int, iters: int) -> None:
    "double s = 0; for (std::size_t k = 1; k <= (std::size_t)iters; ++k) s += 1.0/(double(k)*1e-3 + 1.0); out[slot] = s;"

out = np.zeros(8)                                                    # 8 independent jobs, one slot each
threads = [threading.Thread(target=crunch, args=(out, i, 20_000_000)) for i in range(8)]
for t in threads: t.start()
for t in threads: t.join()                                          # wait for all 8 jobs
```

`@cpp(nogil=True)` releases the GIL (Python's interpreter lock) while the C++ function
runs. Python converts arguments before the call and results after it; the C++
computations can run concurrently. These eight jobs are independent and write to
separate output slots.

`cppyy_kit` provides shared utilities for library loading, callback lifetimes, C++
templates, and ownership. Domain kits build on these utilities and document the native
APIs and conversions they support.

## Install

**Published.** The suite ships as 11 conda packages on the prefix.dev
`awesomebytes` channel (browse: <https://repo.prefix.dev/awesomebytes>). Each package
contains a pure-Python (`noarch`) wrapper. Pixi installs the required C++ libraries
and compatible cppyy runtime. The packages currently target Python 3.12 on Linux
x86_64 and ARM64. ARM64 also uses an architecture-specific cppyy bridge, described in
[`recipe/cppyy/README.md`](recipe/cppyy/README.md). `cppyy-kit` and `wbc-kit` are
distro-free; the ROS-touching kits are published as `ros-jazzy-*`.

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
Each kit installs `cppyy-kit`. ROS kits also install `ros-jazzy-rclcpp-kit`
as a dependency. To develop the suite, see
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/).
The Pixi example above targets `linux-64`; use the matching native platform and
ROS dependencies for an ARM64 environment.

## Showcase

### Kits and their C++ libraries

| Kit | C++ library or function | Measured result or example |
|---|---|---|
| **[cppyy_kit](docs/COMMON_PATTERNS.md)** (base) | the ROS-free machinery: loading, callbacks, lifetime, `@cpp`, `require`, `nogil`, [freeze & compile cache](docs/FREEZE.md) | PCL VoxelGrid: 632 ms JIT, 91 ms cache miss, 89-94 ms cache hits [↗](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached) |
| **[rclcpp_kit](rclcpp_kit/WHY.md)** | rclcpp (ROS 2 core): bringup, messages, tf, rosbag2, CDR | Python TF callback used 7.4-16.9× the CPU of the C++ listener in the linked benchmark [↗](docs/benchmarks.md#tf-ingest-c-tf2-listener-vs-python-callback) |
| **[bt_kit](bt_kit/WHY.md)** | BehaviorTree.CPP v4 | Groot2-compatible trees from Python; cache 218→62 ms [↗](docs/benchmarks.md#bt_kit-compile-cache-t01-cold-run-adoption) |
| **[pcl_kit](pcl_kit/WHY.md)** | Point Cloud Library | **15.1× latency / 7.4× CPU** at 74-LOC parity [↗](docs/benchmarks.md#pcl-pipeline-cloud-stays-in-c-end-to-end) |
| **[ompl_kit](ompl_kit/WHY.md)** | Open Motion Planning Library | Python validity-checker in the planner's inner loop, no codegen [↗](ompl_kit/REPORT.md) |
| **[nav2_kit](nav2_kit/WHY.md)** | Nav2 algorithm cores, composed from Python | RegulatedPurePursuit without lifecycle servers or pluginlib [↗](nav2_kit/REPORT.md) |
| **[moveit_kit](moveit_kit/WHY.md)** | MoveIt 2 native APIs through cppyy | robot models, planning, and kinematics [↗](moveit_kit/REPORT.md) |
| **[control_kit](control_kit/WHY.md)** | ros2_control | Python controllers in `controller_manager` [↗](control_kit/REPORT.md) |
| **[cv_kit](cv_kit/WHY.md)** | OpenCV C++ | zero-copy `sensor_msgs/Image` → `cv::Mat`, one CUDA branch point [↗](cv_kit/REPORT.md) |
| **[dbow_kit](dbow_kit/WHY.md)** | DBoW2 place recognition | vendored source, compiled once; used from Python [↗](dbow_kit/REPORT.md) |
| **[wbc_kit](wbc_kit/WHY.md)** | Crocoddyl custom action models | inline-C++ model, **no build system** [↗](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c) |

Each kit has a Python module, demos, tests, and optional C++ sources. Its `WHY.md`
describes its purpose, `REPORT.md` records evidence, and `SKILL.md` documents API
usage for coding agents. See [`docs/ARCHITECTURE_V2.md`](docs/ARCHITECTURE_V2.md).

### Demos & examples

Each result links to the benchmark row that produced it in
[docs/benchmarks.md](docs/benchmarks.md).

| Demo | Description | Result |
|---|---|---|
| [Live webcam A vs B](docs/webcam_demo/REPORT.md) | a hand-written NCC tracker in one inline-C++ kernel vs the identical NumPy loop | **16.18×** @ 640×480 [↗](docs/benchmarks.md#webcam-demo-a-cppyy_kit-c-vs-b-naive-python) |
| [IK 5-solver bench](docs/ik_bench/WHY.md) | benchmark C++-only IK solvers (incl. unpackaged bio_ik/pick_ik) from *one* Python file | pure-Python **10-25× slower**; bio_ik 991 solve/s [↗](docs/benchmarks.md#ik-benchmark-same-panda-same-200-targets-per-solver-subprocess) |
| [WBC inline-C++ model](docs/wbc/REPORT.md) | a custom Crocoddyl action model authored inline, JIT-compiled, no CMake | **22.9×** vs Python-derived, bit-identical cost [↗](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c) |
| [Retargeting teleop rig](docs/retarget_pipeline/REPORT.md) | webcam → body/hand tracking → TF → whole-body retarget onto G1/Talos, live, one Rerun viewer | glue kernel **341.5×**, /tf marshaling **258.9×** [↗](docs/benchmarks.md#retarget-pipeline-perception-tf-marshaling--retarget-glue-kernel) |
| [Visual loop closure](docs/tutorials/vision_loop_closure.md) | ORB + DBoW2 + GTSAM front-end in a Python script; image data stay in C++ throughout this pipeline | 1080p ingest **135.8×**; 19 loops, precision/recall 1.00/0.95 [↗](docs/benchmarks.md#vision-cv_kit--dbow_kit-synthetic-sequence) |
| [RoboPlan vs OMPL on UR5](docs/tutorials/roboplan_ompl_ur5.md) | compare RRT-Connect path validity and measured solve time on one robot scene from Python | run locally for a speed ranking |
| [Jitter bench](docs/jitter_bench/REPORT.md) | a ~1 kHz control loop orchestrated from Python on a stock kernel | **~2 µs median wakeup latency**, unprivileged [↗](docs/benchmarks.md#jitter-bench-reduced-reference-set-a1--b--c-idle-60-s-each) |
| [cppyy-accelerate skill](skills/cppyy-accelerate/SKILL.md) | point a coding agent at slow Python; it moves the hot path to a kit | **16.3×** (49.6 → 3.04 ms), output bit-identical [↗](docs/benchmarks.md#accelerate-example) |

### Choosing what to accelerate

Moving work to C++ can reduce runtime when Python spends time in loops, callbacks, or
data copies. It may have less effect on work already handled by optimized C++ operations,
such as OpenCV's ORB and RANSAC. In the linked webcam comparison, the hand-written
NCC patch-tracking kernel was **16.18×** faster than the equivalent NumPy loop at 640×480.
The ORB/RANSAC comparison measured a **1.1-1.2× speedup**
([webcam report](docs/webcam_demo/REPORT.md#a-vs-b-measurements)).

In the retargeting demo, the measured C++ work covers `/tf` message marshaling and the
transform/retarget kernel; IK uses Pinocchio's existing Python bindings
([retarget report](docs/retarget_pipeline/REPORT.md#retarget-glue-benchmark-and-ik-limitation)).

## Reducing startup and call overhead

Available options include reducing startup work and moving selected hot paths to C++:

- **Prototype (L0).** Use the kit from Python. Headers are parsed and
  per-signature wrappers are JIT-compiled on first use.
- **Accelerate.** Move the hot path to C++ with a kit, `@cpp`, or `nogil`. The
  [PCL pipeline benchmark](docs/benchmarks.md#pcl-pipeline-cloud-stays-in-c-end-to-end)
  measured 15.1× lower latency at 74-LOC parity, with the cloud in C++ end to end.
- **Freeze.** With the auto-PCH hook installed and a matching PCH available, Cling
  loads cached headers at startup instead of parsing them again. In the linked rclcpp
  bringup benchmark, rclcpp initialization took ~1.73 s cold and 0.064 s warm (~27×):
  [auto-PCH measurement](docs/benchmarks.md#auto-pch-zero-config-cold-vs-warm-bringup).
  The compile cache can reuse compatible `@cpp`/`cppdef` artifacts. For the PCL
  VoxelGrid benchmark, JIT took 632 ms, a cache miss took 91 ms, and cache hits took
  89-94 ms [↗](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached).
- **Lower (L2).** Implement one frequently called function as a native C++ node. In the
  [WBC Crocoddyl action model benchmark](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c),
  the inline C++ model ran 22.9× faster than the Python-derived model. The costs
  were bit-identical, and the C++ model removed the per-call cppyy boundary.

See [`docs/FREEZE.md`](docs/FREEZE.md) for compile-cache and PCH details and
[`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md) for 36 usage patterns.

## Using rclcpp_kit with rclcppyy

`rclcpp_kit` provides the C++ APIs used by
[**rclcppyy**](https://github.com/awesomebytes/rclcppyy). rclcppyy lets an
existing rclpy program use ROS 2's C++ core (rclcpp, tf2, rosbag2, and CDR
serialization) with minimal changes. It re-exports APIs from `rclcpp_kit` and
installs from the same channel as `ros-jazzy-rclcppyy`. If an rclpy node spends
time handling TF data or marshaling messages, `rclcpp_kit` can move that work to C++.

## Documentation for coding agents

The documentation also supports coding agents:

- Every kit includes a `SKILL.md` with its API and usage patterns.
- [`docs/COMMON_PATTERNS.md`](docs/COMMON_PATTERNS.md) contains 36 patterns for
  using the APIs and adding a kit.
- The [`cppyy-accelerate`](skills/cppyy-accelerate/SKILL.md) skill gives coding
  agents steps to profile with cProfile and a boundary tracer, select a kit or
  pattern, make a small change based on the kit `SKILL.md`, and verify it with
  tests and a before-and-after table. Its
  [worked example](skills/cppyy-accelerate/WALKTHROUGH.md)
  accelerates a naive voxel downsampler **16.3×** with bit-identical output.

## Docs

Full documentation site: **<https://awesomebytes.github.io/cppyy_kit/>**

- [The Patterns](docs/COMMON_PATTERNS.md): 36 cppyy patterns.
- [Freeze & Cache](docs/FREEZE.md): the L0 → L1 → L2 options and compile cache.
- [Benchmarks](docs/benchmarks.md): results with commands and conditions;
  see linked reports for run-specific context.
- [Architecture](docs/ARCHITECTURE_V2.md): package structure and responsibilities.
- [Tutorials](docs/tutorials/roboplan_ompl_ur5.md): motion-planning comparison and walkthroughs.
- Each kit includes a purpose page, an evidence report, and an API guide for coding agents.

Questions, ideas, and bug reports are welcome on the
[issue tracker](https://github.com/awesomebytes/cppyy_kit/issues).

## License

BSD 3-Clause. See [`LICENSE`](LICENSE).

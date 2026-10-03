# cppyy_kit

**Drive C++ robotics libraries from Python with cppyy.**

**New here? Start with [Getting Started](getting-started.md)** for installation or
development instructions.

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
[runnable example](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/parallel_demo/parallel_demo.py)
has the full version:

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

## Measured results

The [Benchmarks](docs/benchmarks.md) page consolidates results with their commands
and conditions; each kit's `REPORT.md` carries the underlying evidence. Each row
below names its benchmark and links to it.

| Lever | Result |
|---|---|
| **Accelerate**: keep PCL cloud data in C++ | [**15.1× latency / 7.4× CPU**](docs/benchmarks.md#pcl-pipeline-cloud-stays-in-c-end-to-end) at 74-LOC parity in the PCL pipeline benchmark |
| **Freeze**: use a Cling PCH for library headers | rclcpp bringup: [**~1.73 s cold, 0.064 s warm (~27×)**](docs/benchmarks.md#auto-pch-zero-config-cold-vs-warm-bringup) |
| **Compile cache**: reuse content-hashed `@cpp`/`cppdef` artifacts | PCL VoxelGrid: [**632 ms JIT, 91 ms cache miss, 89-94 ms cache hits**](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached) |
| **Lower (L2)**: implement a hot leaf in native C++ | Inline Crocoddyl model: [**22.9×**](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c), bit-identical cost |
| **TF input**: compare C++ `tf2` listener and Python callback | Python callback used [**7.4-16.9× the CPU**](docs/benchmarks.md#tf-ingest-c-tf2-listener-vs-python-callback) of the C++ listener |

## The kits

| Kit | C++ library or function | Result or example |
|---|---|---|
| **[cppyy_kit](kits/cppyy_kit.md)** (base) | the ROS-free machinery: loading, callbacks, lifetime, `@cpp`, `require`, `nogil`, [freeze & compile cache](docs/FREEZE.md) | PCL VoxelGrid: 632 ms JIT, 91 ms cache miss, 89-94 ms cache hits [↗](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached) |
| **[rclcpp_kit](rclcpp_kit/WHY.md)** | rclcpp (ROS 2 core): bringup, messages, tf, rosbag2, CDR | TF ingest **7.4-16.9×** lower CPU |
| **[bt_kit](bt_kit/WHY.md)** | BehaviorTree.CPP v4 | Groot2-compatible trees from Python |
| **[pcl_kit](pcl_kit/WHY.md)** | Point Cloud Library | **15.1× latency / 7.4× CPU** at LOC parity |
| **[ompl_kit](ompl_kit/WHY.md)** | Open Motion Planning Library | Python validity-checker in the planner's inner loop, no codegen |
| **[nav2_kit](nav2_kit/WHY.md)** | Nav2 algorithm cores, composed from Python | RegulatedPurePursuit without lifecycle servers or pluginlib |
| **[moveit_kit](moveit_kit/WHY.md)** | MoveIt 2 native APIs through cppyy | robot models, planning, and kinematics |
| **[control_kit](control_kit/WHY.md)** | ros2_control | Python controllers in `controller_manager` |
| **[cv_kit](cv_kit/WHY.md)** | OpenCV C++ | zero-copy `Image` → `cv::Mat`, one CUDA branch point |
| **[dbow_kit](dbow_kit/WHY.md)** | DBoW2 place recognition | vendored source, compiled once; used from Python |
| **[wbc_kit](docs/wbc/REPORT.md)** | Crocoddyl custom action models | inline-C++ model, **no build system** |

Each kit includes a purpose page (`WHY.md`), an evidence report (`REPORT.md`),
and an API guide for coding agents (`SKILL.md`). See
[Architecture](docs/ARCHITECTURE_V2.md).

## Demos & examples

Each result links to the benchmark row that produced it in
[Benchmarks](docs/benchmarks.md).

| Demo | Description | Result |
|---|---|---|
| [Live webcam A vs B](docs/webcam_demo/REPORT.md) | a hand-written NCC tracker in one inline-C++ kernel vs the identical NumPy loop | [**16.18×**](docs/benchmarks.md#webcam-demo-a-cppyy_kit-c-vs-b-naive-python) @ 640×480 |
| [IK 5-solver bench](docs/ik_bench/WHY.md) | benchmark C++-only IK solvers (incl. unpackaged bio_ik/pick_ik) from *one* Python file | pure-Python [**10-25× slower**](docs/benchmarks.md#ik-benchmark-same-panda-same-200-targets-per-solver-subprocess); bio_ik 991 solve/s |
| [WBC inline-C++ model](docs/wbc/REPORT.md) | a custom Crocoddyl action model authored inline, JIT-compiled, no CMake | [**22.9×**](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c) vs Python-derived, bit-identical |
| [Retargeting teleop rig](docs/retarget_pipeline/REPORT.md) | webcam → body/hand tracking → TF → whole-body retarget onto G1/Talos, live, one Rerun viewer | glue kernel [**341.5×**](docs/benchmarks.md#retarget-pipeline-perception-tf-marshaling-retarget-glue-kernel), /tf marshaling 258.9× |
| [Visual loop closure](docs/tutorials/vision_loop_closure.md) | ORB + DBoW2 + GTSAM front-end in a Python script; image data stay in C++ throughout this pipeline | 1080p ingest [**135.8×**](docs/benchmarks.md#vision-cv_kit-dbow_kit-synthetic-sequence); 19 loops, P/R 1.00/0.95 |
| [Jitter bench](docs/jitter_bench/REPORT.md) | a ~1 kHz control loop orchestrated from Python on a stock kernel | [**~2 µs median wakeup latency**](docs/benchmarks.md#jitter-bench-reduced-reference-set-a1-b-c-idle-60-s-each), unprivileged |
| [cppyy-accelerate skill](skills/cppyy-accelerate/SKILL.md) | point a coding agent at slow Python; it moves the hot path to a kit | [**16.3×**](docs/benchmarks.md#accelerate-example) (49.6 → 3.04 ms), bit-identical |

### Choosing what to accelerate

Moving work to C++ can reduce runtime when Python spends time in loops, callbacks, or
data copies. It may have less effect on work already handled by optimized C++ operations,
such as OpenCV's ORB and RANSAC. In the linked webcam comparison, the hand-written
NCC patch-tracking kernel was **16.18×** faster than the equivalent NumPy loop at 640×480.
The ORB/RANSAC comparison measured a **1.1-1.2× speedup**
([webcam report](docs/webcam_demo/REPORT.md#a-vs-b-measurements)).

In the retargeting demo, the measured C++ work covers `/tf` message marshaling and the
transform/retarget kernel. IK uses Pinocchio's existing Python bindings. See the
[retarget report](docs/retarget_pipeline/REPORT.md#retarget-glue-benchmark-and-ik-limitation).

## Reducing startup and call overhead

Available options include reducing startup work and moving selected hot paths to C++:

- **Prototype (L0).** Use the kit from Python. Headers are parsed and per-signature
  wrappers are JIT-compiled on first use.
- **Accelerate.** Move the hot path to C++ with a kit, `@cpp`, or `nogil`. The PCL
  pipeline measured 15.1× lower latency in the
  [PCL pipeline benchmark](docs/benchmarks.md#pcl-pipeline-cloud-stays-in-c-end-to-end),
  where the cloud stays in C++ end to end.
- **Freeze.** With the auto-PCH hook installed and a matching PCH available, Cling
  loads cached headers at startup instead of parsing them again. In the linked rclcpp
  bringup benchmark, rclcpp initialization took ~1.73 s cold and 0.064 s warm (~27×):
  [auto-PCH measurement](docs/benchmarks.md#auto-pch-zero-config-cold-vs-warm-bringup).
  The compile cache can reuse compatible `@cpp`/`cppdef` artifacts. For the PCL
  VoxelGrid benchmark, JIT took 632 ms, a cache miss took 91 ms, and cache hits took
  89-94 ms [↗](docs/benchmarks.md#pcl-compile-cache-frame-0-first-use-jit-vs-cached).
- **Lower (L2).** Implement one frequently called function as a native C++ node.
  The
  [WBC Crocoddyl action model benchmark](docs/benchmarks.md#wbc-custom-crocoddyl-action-model-python-derived-vs-inline-c)
  measured a 22.9× speedup over the Python-derived model with bit-identical cost.
  This removes the per-call cppyy boundary.

See **[Freeze & Cache](docs/FREEZE.md)** for compile-cache and PCH details and
**[The Patterns](docs/COMMON_PATTERNS.md)** for 36 usage patterns.

## Using rclcpp_kit with rclcppyy

`rclcpp_kit` provides the C++ APIs used by
[**rclcppyy**](https://github.com/awesomebytes/rclcppyy). rclcppyy lets an
existing rclpy program use ROS 2's C++ core (rclcpp, tf2, rosbag2, and CDR
serialization) with minimal changes. It re-exports APIs from `rclcpp_kit` and
installs from the same channel as `ros-jazzy-rclcppyy`.

## Documentation for coding agents

Each kit includes a `SKILL.md` with its API and usage patterns. Coding agents can
use [The Patterns](docs/COMMON_PATTERNS.md) when writing a call or adding a kit. The
[cppyy-accelerate](skills/cppyy-accelerate/SKILL.md) skill describes how to profile,
select a kit or pattern, make a small change, and verify it. Its
[worked example](skills/cppyy-accelerate/WALKTHROUGH.md) accelerates a naive voxel
downsampler **16.3×** with bit-identical output.

## Next steps

- **[Getting Started](getting-started.md)**: install packages or develop from the repo.
- **[The Patterns](docs/COMMON_PATTERNS.md)**: 36 cppyy usage patterns.
- **[Benchmarks](docs/benchmarks.md)**: results, commands, and conditions.
- **[Architecture](docs/ARCHITECTURE_V2.md)**: package structure and responsibilities.

---

*Origin: extracted and expanded from [rclcppyy](https://github.com/awesomebytes/rclcppyy),
which it now powers.*

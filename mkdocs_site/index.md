# cppyy_kit

**Use C++ libraries from Python, or write C++ functions in a Python script.**

`cppyy_kit` helps you call C++ robotics libraries such as BehaviorTree.CPP, PCL,
and OMPL from Python without writing a separate Python binding for each library.
You can also write a C++ function inside your Python code and reuse its compiled
version across runs.

[cppyy](https://cppyy.readthedocs.io) provides access to C++ classes and functions.
The kits handle library setup and provide helpers for Python callbacks, arrays,
and keeping objects alive.

<p align="center">
  <img src="docs/media/cppyy_kit_logo.jpg" alt="cppyy_kit logo" width="420">
</p>

## Try both examples

The numeric annotations and packaged task guides require `cppyy-kit` 0.4.1
or later. Version 0.4.1 is published on the `awesomebytes` channel; its
[release record](https://github.com/awesomebytes/cppyy_kit/blob/main/RELEASE_0.4.1_2026-10-10.md) includes exact public-channel verification
and fresh installed-package examples.

Install [Pixi](https://pixi.sh/latest/installation/), then create an environment
for the two examples below:

```bash
pixi init cppyy-example -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge
cd cppyy-example
pixi add "cppyy-kit>=0.4.1" ros-jazzy-bt-kit numpy
```

Packages are available for Linux x86_64 and ARM64. Pixi installs Python and the
C++ dependencies. See [Getting Started](getting-started.md) for the full walkthrough.

### 1. Call an existing C++ library

This example creates a BehaviorTree.CPP tree and runs it. The C++ engine owns and
ticks the tree; Python calls its API. Custom tree actions can be Python functions.

Save this as `tree.py`:

```python
import bt_kit
bt = bt_kit.bringup_bt()
xml = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree"><AlwaysSuccess/></BehaviorTree>
</root>
"""
factory = bt.BehaviorTreeFactory()
tree = factory.create_tree_from_text(xml)
status = tree.tickWhileRunning()
print(status == bt.NodeStatus.SUCCESS)  # True
```

Run `pixi run python tree.py`. It prints `True`.
See the [bt_kit examples](bt_kit/WHY.md) to add Python actions and conditions.

### 2. Write a C++ function in Python

Use `@cpp` for a function you want to run in C++, such as a loop over array data.
The function's docstring contains the C++ body. Its annotations describe the
arguments and result.

Save this as `kernel.py`:

```python
import numpy as np
from cppyy_kit.numpy_types import NDArray
from cppyy_kit import cpp

@cpp
def sum_sq(data: NDArray[np.float64]) -> float:
    """
    double s = 0;
    for (std::size_t i = 0; i < data_size; ++i) {
        s += data[i] * data[i];
    }
    return s;
    """

print(sum_sq(np.array([1, 2, 3], dtype=np.float64)))  # 14.0
```

Run `pixi run python kernel.py`. It prints `14.0`. The annotation supplies a
writable C++ `double*` pointer and element count. The function compiles on first
use; later runs can load it from the compile cache.

For independent work in Python threads, `@cpp(nogil=True)` releases Python's
interpreter lock during the C++ function. See the [parallel example](https://github.com/awesomebytes/cppyy_kit/blob/main/examples/parallel_demo/parallel_demo.py).

## Choose a library

Install the kit for the library you want to use. Each linked page shows its
Python workflow and examples.

| Library or task | Kit |
|---|---|
| BehaviorTree.CPP trees with Python actions | [bt_kit](bt_kit/WHY.md) |
| PCL point-cloud filtering and conversion | [pcl_kit](pcl_kit/WHY.md) |
| OMPL motion planning with Python validity checks | [ompl_kit](ompl_kit/WHY.md) |
| MoveIt robot models, kinematics, and planning | [moveit_kit](moveit_kit/WHY.md) |
| Nav2 grid planning and mobile-base controllers | [nav2_kit](nav2_kit/WHY.md) |
| ros2_control with Python controllers | [control_kit](control_kit/WHY.md) |
| OpenCV C++ image processing and ROS image buffers | [cv_kit](cv_kit/WHY.md) |
| DBoW2 place recognition | [dbow_kit](dbow_kit/WHY.md) |
| Crocoddyl custom action models | [wbc_kit](wbc_kit/WHY.md) |
| ROS 2 nodes, messages, TF, and rosbag2 through C++ | [rclcpp_kit](rclcpp_kit/README.md) |

`cppyy-kit` provides the shared tools, including `@cpp`; library kits add the
corresponding C++ library. [Getting Started](getting-started.md#install-another-kit)
lists their package names.

## Tutorials and measured results

- [RoboPlan and OMPL on UR5](docs/tutorials/roboplan_ompl_ur5.md): compare path
  validity and planning speed from one Python script.
- [Visual loop closure](docs/tutorials/vision_loop_closure.md): combine OpenCV,
  DBoW2, and pose-graph optimization from Python.
- [Webcam tracker comparison](docs/webcam_demo/REPORT.md): a custom C++ tracker
  and a Python/NumPy implementation process the same frames. The recorded 640×480
  benchmark measured a 16.18× speedup for that tracker.
- [PCL pipeline benchmark](docs/benchmarks.md#pcl-pipeline-cloud-stays-in-c-end-to-end):
  the C++ pipeline had 15.1× lower latency than the rclpy/NumPy baseline.
- [All benchmark results](docs/benchmarks.md): measurements and commands for
  startup, data processing, callbacks, and control loops.

## Learn more

Read the packaged instructions for a task or installed kit:

```bash
pixi run python -m cppyy_kit guide accelerate
pixi run python -m cppyy_kit guide ompl_kit api
```

These commands are included in published 0.4.1 packages. See
[task guides and environment checks](docs/GUIDES.md).

- [Using callbacks, arrays, and C++ objects](docs/COMMON_PATTERNS.md).
- [Reusing compiled code and cached headers](docs/FREEZE.md) to reduce startup work.
- [rclcppyy](https://github.com/awesomebytes/rclcppyy) to use these ROS C++ APIs
  through an rclpy-style interface.
- [Package architecture](docs/ARCHITECTURE_V2.md) for contributors.
- [cppyy-accelerate](skills/cppyy-accelerate/SKILL.md) for coding agents that profile
  a Python program and move selected work to C++.

Report problems or suggest examples on the
[issue tracker](https://github.com/awesomebytes/cppyy_kit/issues).

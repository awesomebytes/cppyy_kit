# Getting Started

Run an existing C++ library and an inline C++ function from Python. The two
examples below use one Pixi environment.

The numeric `@cpp` annotations below are new in `cppyy-kit` 0.4.0. Published
0.3.x packages do not support them. Before 0.4.1 is published, use a clone of
this repository: first save the tree and kernel examples below as `tree.py` and
`kernel.py` in the checkout root, then run `pixi run -e bt python tree.py` and
`pixi run python kernel.py` from that root. See
[Run repository demos or develop the kits](#run-repository-demos-or-develop-the-kits)
for checkout setup. After publication, create a standalone project and add
`cppyy-kit>=0.4.1` with `pixi add`.

## Set up the environment

These packages support Linux x86_64 and ARM64 and use Python 3.12. Pixi installs
Python and the C++ libraries in the project environment.

Install [Pixi](https://pixi.sh/latest/installation/), then run:

```bash
pixi init cppyy-example -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge
cd cppyy-example
pixi add "cppyy-kit>=0.4.1" ros-jazzy-bt-kit numpy
```

`cppyy-kit` supplies the shared tools, `ros-jazzy-bt-kit` adds BehaviorTree.CPP,
and `numpy` supplies the array used in the second example.

## Call BehaviorTree.CPP

Save this as `tree.py` in the project directory:

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

Run it:

```bash
pixi run python tree.py
```

The script prints `True`: the tree's `AlwaysSuccess` node returns the C++
`NodeStatus.SUCCESS` value. See [bt_kit](bt_kit/WHY.md) to register Python actions
and conditions.

## Write an inline C++ function

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

Run it:

```bash
pixi run python kernel.py
```

The script prints `14.0`, the sum of the squared array elements. `@cpp` compiles
the C++ body in the docstring. The `NDArray[np.float64]` annotation supplies a
writable `double*` pointer and an element count; `data_size` is available inside
the C++ body.

On first use, cppyy compiles the required code. Compatible cached code can be
reused on later runs. See [Compile cache and cached headers](docs/FREEZE.md) when
you want to inspect or manage startup compilation.

## Install another kit

Choose a package for the library you want to use and add it to the same project.
For example:

```bash
pixi add ros-jazzy-ompl-kit
```

| Library or task | Package | Python import |
|---|---|---|
| Inline C++ and shared helpers | `cppyy-kit` | `cppyy_kit` |
| BehaviorTree.CPP | `ros-jazzy-bt-kit` | `bt_kit` |
| PCL | `ros-jazzy-pcl-kit` | `pcl_kit` |
| OMPL | `ros-jazzy-ompl-kit` | `ompl_kit` |
| MoveIt | `ros-jazzy-moveit-kit` | `moveit_kit` |
| Nav2 | `ros-jazzy-nav2-kit` | `nav2_kit` |
| ros2_control | `ros-jazzy-control-kit` | `control_kit` |
| OpenCV C++ | `ros-jazzy-cv-kit` | `cv_kit` |
| DBoW2 | `ros-jazzy-dbow-kit` | `dbow_kit` |
| Crocoddyl custom action models | `wbc-kit` | `wbc_kit` |
| ROS 2 C++ APIs | `ros-jazzy-rclcpp-kit` | `rclcpp_kit` |

Pixi installs each kit's dependencies. The kit's usage page describes any
additional setup, such as building DBoW2 from source.

## Run repository demos or develop the kits

The tutorial commands with named environments, such as `pixi run -e ompl`, use
this repository's Pixi project. Clone it to run those demos or change the kits:

```bash
git clone https://github.com/awesomebytes/cppyy_kit
cd cppyy_kit
pixi run -e bt demo-bt-t01
pixi run -e ompl demo-ompl-plan
```

Each environment installs the dependencies for its demos. The repository configures
Python's import paths so the commands use the local kit sources.

For code changes, run the corresponding kit tests and the shared lint check:

```bash
pixi run -e bt test-bt
pixi run lint
```

## Read task and library guides

The current checkout and forthcoming 0.4.1 packages provide guides for Python acceleration,
loading a new C++ library and working with an existing compiled component:

```bash
pixi run python -m cppyy_kit guide
pixi run python -m cppyy_kit guide existing-cpp
pixi run python -m cppyy_kit guide bt_kit api
pixi run python -m cppyy_kit status --environment
```

See [task guides and environment checks](docs/GUIDES.md) for the reading order.

## Preview the documentation

From the repository checkout:

```bash
pixi run -e docs docs-serve    # http://127.0.0.1:8000
pixi run -e docs docs-build    # build into ./site
```

## What to try next

- [RoboPlan and OMPL on UR5](docs/tutorials/roboplan_ompl_ur5.md): compare path
  validity and planning speed from Python.
- [Visual loop closure](docs/tutorials/vision_loop_closure.md): build an image
  processing and place-recognition pipeline.
- [Callbacks, arrays, and C++ object lifetimes](docs/COMMON_PATTERNS.md): look up
  a usage pattern as you need it.

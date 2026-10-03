# Why bt_kit: BehaviorTree.CPP from Python via cppyy

`bt_kit` lets Python callbacks run as leaves in a tree parsed and ticked by the
BehaviorTree.CPP v4 C++ engine. You can use BT.CPP XML and its tree factory from
Python without writing a binding or building a wrapper for the Python callbacks.

Install `ros-jazzy-bt-kit` with `pixi add -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge ros-jazzy-bt-kit`.
See the [Getting Started guide](https://awesomebytes.github.io/cppyy_kit/getting-started/)
for setup, or the [project repository](https://github.com/awesomebytes/cppyy_kit).
The `bt` Pixi environment and `demo-bt-t01` task below are for a source checkout.
For the API, see [SKILL.md](SKILL.md); for implementation details and measurements,
see [REPORT.md](REPORT.md).

## Try a BT.CPP tree from Python

The existing first-tree example registers Python functions as leaves, then runs
the tree with BT.CPP:

### Python example

```python
#!/usr/bin/env python
"""
BehaviorTree.CPP official tutorial 1 ("Your first behavior tree"), in Python via
bt_kit. It uses the same factory, registration functions, and tree methods as the
C++ tutorial. The leaf callbacks are Python functions.
"""
import bt_kit

bt = bt_kit.bringup_bt()

XML = """
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <CheckBattery   name="check_battery"/>
      <OpenGripper    name="open_gripper"/>
      <ApproachObject name="approach_object"/>
      <CloseGripper   name="close_gripper"/>
    </Sequence>
  </BehaviorTree>
</root>
"""


def check_battery(node):
    print("[ Battery: OK ]")
    return bt.NodeStatus.SUCCESS


def open_gripper(node):
    print("GripperInterface::open")
    return bt.NodeStatus.SUCCESS


def approach_object(node):
    print("ApproachObject: approach_object")
    return bt.NodeStatus.SUCCESS


def close_gripper(node):
    print("GripperInterface::close")
    return bt.NodeStatus.SUCCESS


def main():
    factory = bt.BehaviorTreeFactory()
    factory.registerSimpleCondition("CheckBattery", check_battery)
    factory.registerSimpleAction("OpenGripper", open_gripper)
    factory.registerSimpleAction("ApproachObject", approach_object)
    factory.registerSimpleAction("CloseGripper", close_gripper)

    tree = factory.createTreeFromText(XML)
    tree.tickWhileRunning()


if __name__ == "__main__":
    main()
```

From a source checkout, run `pixi run -e bt demo-bt-t01`. It prints:

```text
[ Battery: OK ]
GripperInterface::open
ApproachObject: approach_object
GripperInterface::close
```

## Compare with the complete C++ tutorial

These are complete programs. The left side shows the official
["first tree" tutorial](https://www.behaviortree.dev/docs/tutorial-basics/tutorial_01_first_tree)
from behaviortree.dev. The right side shows the runnable file
`bt_kit/demos/t01_first_tree.py`. The Python code uses the same API names
as the C++ tutorial and omits the C++ build files and class definition.

### C++, `first_tree.cpp` (official tutorial)

```cpp
#include "behaviortree_cpp/bt_factory.h"
using namespace BT;

// A custom SyncActionNode, created by inheritance (the recommended way).
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string& name) : BT::SyncActionNode(name, {}) {}

  // You must override the virtual function tick().
  BT::NodeStatus tick() override
  {
    std::cout << "ApproachObject: " << this->name() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};

// A plain function used as a condition.
BT::NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

// Methods of an existing class, wrapped as actions.
class GripperInterface
{
public:
  GripperInterface() : _open(true) {}

  NodeStatus open()
  {
    _open = true;
    std::cout << "GripperInterface::open" << std::endl;
    return NodeStatus::SUCCESS;
  }
  NodeStatus close()
  {
    std::cout << "GripperInterface::close" << std::endl;
    _open = false;
    return NodeStatus::SUCCESS;
  }
private:
  bool _open;
};

static const char* xml_text = R"(
 <root BTCPP_format="4">
   <BehaviorTree ID="MainTree">
     <Sequence name="root_sequence">
       <CheckBattery   name="check_battery"/>
       <OpenGripper    name="open_gripper"/>
       <ApproachObject name="approach_object"/>
       <CloseGripper   name="close_gripper"/>
     </Sequence>
   </BehaviorTree>
 </root>
 )";

int main()
{
  BehaviorTreeFactory factory;

  factory.registerNodeType<ApproachObject>("ApproachObject");
  factory.registerSimpleCondition("CheckBattery", [&](TreeNode&) { return CheckBattery(); });

  GripperInterface gripper;
  factory.registerSimpleAction("OpenGripper",  [&](TreeNode&) { return gripper.open();  });
  factory.registerSimpleAction("CloseGripper", [&](TreeNode&) { return gripper.close(); });

  auto tree = factory.createTreeFromText(xml_text);
  tree.tickWhileRunning();
  return 0;
}
```

This C++ example still needs a `CMakeLists.txt`
(`find_package(behaviortree_cpp REQUIRED)`, `add_executable(first_tree ...)`,
`target_link_libraries(first_tree BT::behaviortree_cpp)`) and a build. Run
`colcon build` (or `cmake . && make`), to compile and link a binary before you
can execute it.

### Differences shown in the examples

- **No compile step for the Python demo.** The C++ program needs a CMakeLists and a `colcon build`
  before it can run. The Python file can run directly. Its one-time
  startup cost is a one-time **~0.85 s** cppyy bringup (JIT-including the header +
  loading the `.so`), and only what you touch is JIT-compiled.
- **No wrapper or code generation.** Nothing is generated. `factory`,
  `registerSimpleAction`, `registerSimpleCondition`, `createTreeFromText`,
  `tickWhileRunning` are the library's own names. The Python uses those names.
- **Same XML, same engine.** The tree text is identical and is parsed and ticked
  by `libbehaviortree_cpp.so`; BT.CPP XML remains usable with BT.CPP tools.
- **Mixed C++/Python leaves in one tree.** You can keep some leaves in Python and
  move other leaves to C++ (JIT-compiled or in existing libraries) in the same tree. See
  Mode B and `t03_mixed_tree.py` below.

**What the C++ version provides.**
`registerNodeType<ApproachObject>` uses a *compile-time* C++ type, which gives
static type checking and a full node *manifest*, the metadata Groot2 uses to
populate its editor palette and that BT.CPP writes into a `TreeNodesModel`. From
Python you register via functors / `register_stateful` instead (a Python class
can't be a C++ template argument), so Python-defined nodes don't yet contribute a
manifest, and custom control/decorator node *types* still have to be written in
C++. See [REPORT.md](REPORT.md) §5 for the full list.

---

## Two ways to use it

### Mode A: use BehaviorTree.CPP from Python
Prototype whole trees, with the leaves in Python, at Python speed of iteration.
This mode runs the maintained C++ engine and official XML from a Python script.
Tutorials 1 and 2 (`t01_first_tree.py`, `t02_ports.py`) use this mode. It supports
tree setup, behavior changes, and tests with short edit-run cycles.

### Mode B: call existing C++ from a Python tree
The mixed-tree demo combines Python leaves, a JIT-compiled C++ leaf, and a call to
an installed rclcpp library. `bt_kit/demos/t03_mixed_tree.py` demonstrates this in
a single `Sequence`:

1. `CheckSensors`, a **Python** leaf;
2. `ComputePlan`, a **JIT-compiled C++** functor (no Python on that tick);
3. `PublishStatus`, a leaf that drives **existing C++**: it publishes a ROS 2
   `std_msgs/String` through **rclcpp** (via rclcppyy) from inside the tick.

The third leaf calls rclcpp from a behavior-tree tick. The demo runs a subscriber
in the same process and confirms that all three messages arrived (`RESULT: OK`).
This lets Python combine existing C++ controllers, planners, and drivers in a tree.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md) §3–4):

- **No code generation, no wrapper build.** Nothing is generated or compiled
  ahead of time. Contrast the C++ tutorial's `cmake && make`; here `python x.py`
  is the whole workflow. Bringup is a one-time ~0.85 s JIT.
- **Header-following, so it tracks the installed version.** cppyy reads the
  library's own headers at runtime, so bt_kit automatically matches whatever
  BehaviorTree.CPP is installed (4.9.0 here). No separate binding needs to track
  library updates.
- **On-demand template instantiation.** Templated members like
  `getInput<std::string>` are instantiated when called from Python. No need to pre-declare which specializations a binding exposes.
- **BT.CPP XML and engine.** Trees use BT.CPP's XML format and the installed C++
  engine. Compatibility with a particular BT.CPP tool or plugin depends on that
  component's requirements.
- **Mixed-language leaves in one tree.** Python, JIT'd C++, and calls into
  existing `.so`s coexist in a single tree (Mode B). You pay a language boundary
  only for Python leaves, about 0.3 µs per leaf tick (~2x a C++
  leaf), i.e. ~630k Python-leaf ticks/s, far above any robot control rate.
- **A prototype-to-native lowering path.** The kit is the L0 rung of a
  staged implementation: **L0**, prototype with cppyy JIT (what runs today:
  Python leaves, JIT'd, ~0.85 s bringup); **L1**, *freeze*: ship a precompiled
  cppyy dictionary (`.pcm`/`.so`) so startup pays no JIT; **L2**, *lower*: emit
  native C++ for the hot leaves (or the whole tree) compiled ahead of time, same
  XML, same structure, the Python leaves become C++. You prototype in Python and
  later compile selected leaves or the full tree. L1/L2 are planned work; see
  PLAN.md's Experiments section.

---

## Limits

A 2026-07-11 follow-up review addressed several v0 limits: typed ports
(int/float/bool/vectors), per-node stateful instances, loggers + Groot2 publishing
+ a tick observer, readable XML errors, and subtrees/scripting all work now (see
[REPORT.md](REPORT.md) §5). Python leaves run under the GIL on the tree thread, so
`ParallelNode` does not run them concurrently. Custom control and decorator node
types still need C++. Direction-specific and struct/JSON ports are not supported.
Startup includes about 0.85 s of header parsing. The PCH work and raw cppyy limits
are described in §5. The kit handles the cppyy operations tested here.

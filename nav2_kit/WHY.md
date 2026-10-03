# nav2_kit: use Nav2 algorithm cores from Python

`nav2_kit` lets you build a navigation stack by driving [Nav2](https://nav2.org)'s
**algorithm cores directly** from Python: the real C++ code owns the costmap grid and
runs the planner (NavFn or Smac 2D) and the RegulatedPurePursuit controller. Python
controls the loop and provides the world data. The kit uses the installed Nav2
headers. It does not require lifecycle servers, pluginlib, code generation, or a build
step.

Nav2's Python interface is client-side: it configures C++ servers with YAML and sends
them goals. This document shows how to build a small stack from Nav2's C++ classes
with Python. For the API, see [SKILL.md](SKILL.md). For implementation evidence,
limitations, and benchmarks, see [REPORT.md](REPORT.md).

---

## Implementing a custom planning loop with stock Nav2

Suppose you just want to try your own idea: "take this occupancy grid, plan across it
with NavFn, and drive along the result." In stock Nav2, the supported way to run a
*custom* planner or controller is to make it a **C++ pluginlib plugin inside a
lifecycle server**. Concretely, per the
[Nav2 "writing a new planner plugin" docs](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html):

- **Write a C++ class** deriving `nav2_core::GlobalPlanner`, implementing
  `configure() / activate() / deactivate() / cleanup() / createPlan()`, taking a
  `LifecycleNode`, a `tf2_ros::Buffer`, and a `Costmap2DROS`.
- **Export it as a plugin**, `PLUGINLIB_EXPORT_CLASS`, a `plugins.xml`, `ament`
  registration, and a `CMakeLists.txt` that builds a shared library.
- **Wire the lifecycle bringup**, a `planner_server` with a params YAML naming your
  plugin, then launch the lifecycle manager to `configure`→`activate` it.
- **Provide tf + a costmap**, the `Costmap2DROS` needs a transform tree
  (`map`→`odom`→`base_link`) and sensor/static layers to populate the grid.

The stock setup requires `colcon build`, plugin XML, YAML configuration, a launch file,
a lifecycle manager, and a tf tree before you can call the planner. This setup suits
production fleets, but adds steps when testing a planner on a grid.

Contrast the cppyy "after": `pixi install -e nav2`, then `python your_plan.py`,
JIT-including the installed Nav2 headers in ~70 ms at startup.

---

## Side by side: a custom planning loop, stock Nav2 vs nav2_kit

### Stock Nav2, the shape of a custom global planner

```cpp
// my_planner.hpp / .cpp: a nav2_core::GlobalPlanner plugin
class MyPlanner : public nav2_core::GlobalPlanner {
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override; void deactivate() override; void cleanup() override;
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
                                 const geometry_msgs::msg::PoseStamped & goal, ...) override;
};
PLUGINLIB_EXPORT_CLASS(MyPlanner, nav2_core::GlobalPlanner)
```
```xml
<!-- my_planner_plugin.xml -->
<library path="my_planner"><class type="MyPlanner" base_class_type="nav2_core::GlobalPlanner"/></library>
```
```yaml
# nav2_params.yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased: {plugin: "MyPlanner"}
```
…plus a `CMakeLists.txt` building the plugin, a launch file bringing up the
`planner_server` + lifecycle manager, and a tf tree feeding a `Costmap2DROS`. Then a
`colcon build` and a lifecycle bringup, before the planner runs once.

### nav2_kit, the complete runnable file this repo ships

```python
#!/usr/bin/env python
import numpy as np
import nav2_kit
nav2_kit.bringup_nav2()

grid = np.zeros((100, 100), dtype=np.uint8)                 # your world
grid[:, 50] = nav2_kit.LETHAL_OBSTACLE                      # a wall
grid[44:56, 50] = nav2_kit.FREE_SPACE                       # ... with a doorway
costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05)

path = nav2_kit.plan_navfn(costmap, start=(20, 50), goal=(80, 50))  # NavFn (C++)
print(f"Planned {len(path)} waypoints from {tuple(path[0])} to {tuple(path[-1])}")
```

Run it: `pixi run -e nav2 demo-nav2-plan`. It plans across the grid with **Nav2's
real NavFn algorithm**, the same C++ `nav2_navfn_planner::NavFn` the
`planner_server` runs, and prints the path, with no server, no plugin XML, no YAML,
no tf, no build.

### What we gain (from the comparison above)

- **No plugin, lifecycle, YAML, tf, or build setup.** The stock path needs a C++
  plugin, `plugins.xml`, params YAML, a launch file + lifecycle manager, and a tf
  tree; nav2_kit runs the moment you invoke it (~70 ms one-time cppyy bringup).
- **The world and the loop are just Python.** The occupancy grid is a NumPy array;
  the follow controller is a Python function you can breakpoint and edit. You iterate
  in seconds, not `colcon build` cycles.
- **It is the same `libnav2_*.so`.** `Costmap2D` and `NavFn` are Nav2's own classes,
  header-following, so nav2_kit tracks whatever Nav2 is installed, no binding to fall
  behind.
- **A prototype-to-native path.** Prototype with cppyy JIT, then compile the planner
  as a Nav2 plugin when needed. The Nav2 calls stay the same.

**What stock Nav2 buys that this does not.** A production stack: lifecycle
management, dynamic costmap layers from live sensors, tf/localization, recovery
behaviors, the full planner/controller/behavior-tree ecosystem, and the operational
maturity of the servers. nav2_kit is for *composing and prototyping from the cores*,
not for running a robot in production.

---

## Supported components and limitations

The supported components and their requirements are documented in [REPORT.md](REPORT.md):

- **Pure cores (no rclcpp): `Costmap2D` and `NavFn`.** Use `Costmap2D(w, h, res,
  ox, oy)` and `NavFn(nx, ny)` with a raw `unsigned char*` cost array.
  No node, no tf, no pluginlib. Directly drivable.
- **Lifecycle-coupled components: Smac 2D and RegulatedPurePursuit.** They take a
  `LifecycleNode`; RPP also takes a `Costmap2DROS` and `tf2_ros::Buffer`. The kit
  constructs the `LifecycleNode` in-process from Python, like the `rclcpp::Node` used
  elsewhere in the kit. It also constructs a plugin-free `Costmap2DROS`. No lifecycle
  server, pluginlib, or YAML setup is needed. You can use Nav2's RPP
  (`--controller rpp`) or the Python pure-pursuit controller.
- **Smac Hybrid-A\* (SE(2)) is unavailable.** It constructs, but its OMPL-backed
  distance heuristic crashes intermittently under Cling. The kit does not expose it.

---

## Two ways to use it

### Mode A, plan from Python on your own grid
Synthesize or load an occupancy grid, build a `Costmap2D`, plan with `NavFn`, and use
the path however you like (`d01_plan_grid.py`). Good for planner experiments,
map-based reasoning, and dataset generation where edit-run speed matters.

### Mode B: a small navigation stack in rviz2
`nav2_kit/demos/d02_own_nav_stack.py` plans and follows a simulated differential-drive
robot. It publishes `nav_msgs/OccupancyGrid` +
`nav_msgs/Path` + `geometry_msgs/TwistStamped` via rclcppyy, so an rviz2 (Fixed Frame
`map`) shows the map, plan, and commanded velocity as the robot drives to the goal.
**Pick the pieces:** `--planner navfn|smac` and `--controller pursuit|rpp`. All four
combinations reach the goal; `--planner smac --controller rpp` runs Nav2's real Smac 2D
planner **and** its real RegulatedPurePursuit controller, both C++, driven from one
self-contained Python file.

---

## cppyy features

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md)):

- **No plugin, YAML, lifecycle, or build setup.** `python x.py` is the workflow; bringup
  is a one-time ~70 ms JIT.
- **Header-following, tracks the installed Nav2.** No hand-maintained binding.
- **Bulk data stays fast.** A NumPy grid → `Costmap2D` is a single `memcpy`
  (~600–3600× a per-cell Python loop); the plan never leaves C++ (NavFn on 1024² in
  tens of ms vs ~2 s for a pure-Python A\* in this benchmark).
- **A prototype-to-native lowering path**, as with bt_kit / pcl_kit / ompl_kit: the
  same calls become a compiled Nav2 plugin when you deploy.

---

## Limits

nav2_kit is deliberately **not a Nav2 stack**: no lifecycle *servers*/manager, no
pluginlib-by-name loading, no tf tree/localization, no dynamic obstacle/inflation
layers, no recovery behaviors. The kit exposes `Costmap2D` + `NavFn` and Smac **2D** +
the RPP controller (via an in-process `LifecycleNode` +
plugin-free `Costmap2DROS`, still no servers). Smac **Hybrid-A\*** remains out (a flaky
OMPL-under-Cling crash). Loading a **Python
planner/controller plugin *inside* a real Nav2 server**, is a separate planned spike.
See [REPORT.md](REPORT.md) §6 for the full list.

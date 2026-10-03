# nav2_kit: use Nav2 algorithm cores from Python

`nav2_kit` is for mobile-base navigation on an occupancy grid. Python provides the
world and controls the loop; Nav2's C++ classes provide the costmap and planners
(NavFn or Smac 2D), and optionally the RegulatedPurePursuit controller. Its
planning domain is 2D grid navigation for mobile bases.

The kit uses cppyy to expose classes from the installed Nav2 headers. Helpers
bridge a NumPy grid to the C++ costmap and return planner paths as NumPy arrays.

From a source checkout, run the included grid plan from the `cppyy_kit` directory:

```bash
pixi run -e nav2 demo-nav2-plan
```

The output includes a waypoint count and confirms that the route passes through
the doorway in the wall. These commands use the source checkout's Pixi environment.
For installed use, see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/)
and the published package `ros-jazzy-nav2-kit`.

For the API, see [SKILL.md](SKILL.md). For implementation evidence and detailed
limitations, see [REPORT.md](REPORT.md).

---

## Plan on a grid with Nav2's C++ NavFn

The included example builds a costmap from a NumPy occupancy grid, with a wall
and doorway, then calls Nav2's `NavFn` planner directly.

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

The command runs Nav2's C++ `nav2_navfn_planner::NavFn`, the same planner used by
`planner_server`, and prints the waypoint count and confirms the route crosses the
divider through its doorway. This grid example needs no server, plugin XML, YAML,
tf, or build.

---

## What a custom planning loop takes in stock Nav2

In stock Nav2, the supported way to run a custom planner or controller is to make
it a C++ pluginlib plugin inside a lifecycle server. See the
[Nav2 writing a new planner plugin docs](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html).

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

## What this gives you

- **The world and the loop are just Python.** The occupancy grid is a NumPy array;
  the follow controller is a Python function you can breakpoint and edit. You iterate
  in seconds, not `colcon build` cycles.
- **C++ object access from Python.** cppyy exposes Nav2 classes from the installed
  headers; kit helpers handle the NumPy-to-costmap copy and planner result arrays.
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

## Limits

nav2_kit is deliberately **not a Nav2 stack**: no lifecycle *servers*/manager, no
pluginlib-by-name loading, no tf tree/localization, no dynamic obstacle/inflation
layers, no recovery behaviors. The kit exposes `Costmap2D` + `NavFn` and Smac **2D** +
the RPP controller (via an in-process `LifecycleNode` +
plugin-free `Costmap2DROS`, still no servers). Smac **Hybrid-A\*** remains out (a flaky
OMPL-under-Cling crash). Loading a **Python
planner/controller plugin *inside* a real Nav2 server**, is a separate planned spike.
See [REPORT.md](REPORT.md) §6 for the full list.

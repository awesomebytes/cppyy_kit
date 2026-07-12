# Why nav2_kit — your own Nav stack from Nav2's cores, in Python via cppyy

`nav2_kit` lets you build a navigation stack by driving [Nav2](https://nav2.org)'s
**algorithm cores directly** from Python: the real C++ code owns the costmap grid and
runs the planner (NavFn or Smac 2D) and, since the M6d lifecycle unlock, the real
RegulatedPurePursuit controller — while your Python owns the loop and the world. It
does this against the Nav2 that is already installed, with **no lifecycle servers and
no pluginlib** — and with no code generation and no build step.

That framing is the whole point. Nav2 is a superb, production navigation system — but
its Python surface is deliberately *client-side*: you configure C++ servers with YAML
and send them goals. This doc shows what cppyy gives you when you want the opposite:
to compose your own miniature stack from Nav2's building blocks. For the API, see
[SKILL.md](SKILL.md); for the feasibility evidence, the honest coupling
boundary, and benchmarks, see [REPORT.md](REPORT.md).

---

## The thing stock Nav2 makes heavy: a *custom* planning loop

Suppose you just want to try your own idea: "take this occupancy grid, plan across it
with NavFn, and drive along the result." In stock Nav2, the supported way to run a
*custom* planner or controller is to make it a **C++ pluginlib plugin inside a
lifecycle server**. Concretely, per the
[Nav2 "writing a new planner plugin" docs](https://docs.nav2.org/plugin_tutorials/docs/writing_new_nav2planner_plugin.html):

- **Write a C++ class** deriving `nav2_core::GlobalPlanner`, implementing
  `configure() / activate() / deactivate() / cleanup() / createPlan()`, taking a
  `LifecycleNode`, a `tf2_ros::Buffer`, and a `Costmap2DROS`.
- **Export it as a plugin** — `PLUGINLIB_EXPORT_CLASS`, a `plugins.xml`, `ament`
  registration, and a `CMakeLists.txt` that builds a shared library.
- **Wire the lifecycle bringup** — a `planner_server` with a params YAML naming your
  plugin, then launch the lifecycle manager to `configure`→`activate` it.
- **Provide tf + a costmap** — the `Costmap2DROS` needs a transform tree
  (`map`→`odom`→`base_link`) and sensor/static layers to populate the grid.

That is: `colcon build`, a plugin XML, a YAML config, a launch file, a lifecycle
manager, and a tf tree — before you can call your planner once. It is the right
architecture for a fleet in production; it is a lot of ceremony for "try my idea on a
grid."

Contrast the cppyy "after": `pixi install -e nav2`, then `python your_plan.py`,
JIT-including the installed Nav2 headers in ~70 ms at startup.

---

## Side by side: a custom planning loop, stock Nav2 vs nav2_kit

### Stock Nav2 — the shape of a custom global planner

```cpp
// my_planner.hpp / .cpp — a nav2_core::GlobalPlanner plugin
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
`colcon build` and a lifecycle bringup — before the planner runs once.

### nav2_kit — the complete runnable file this repo ships

```python
#!/usr/bin/env python
import numpy as np
from rclcppyy.kits import nav2_kit
nav2_kit.bringup_nav2()

grid = np.zeros((100, 100), dtype=np.uint8)                 # your world
grid[:, 50] = nav2_kit.LETHAL_OBSTACLE                      # a wall
grid[44:56, 50] = nav2_kit.FREE_SPACE                       # ... with a doorway
costmap = nav2_kit.costmap_from_numpy(grid, resolution=0.05)

path = nav2_kit.plan_navfn(costmap, start=(20, 50), goal=(80, 50))  # NavFn (C++)
print(f"Planned {len(path)} waypoints from {tuple(path[0])} to {tuple(path[-1])}")
```

Run it: `pixi run -e nav2 demo-nav2-plan`. It plans across the grid with **Nav2's
real NavFn algorithm** — the same C++ `nav2_navfn_planner::NavFn` the
`planner_server` runs — and prints the path, with no server, no plugin XML, no YAML,
no tf, no build.

### What we gain (from the comparison above)

- **No plugin/lifecycle/YAML/tf ceremony, no build.** The stock path needs a C++
  plugin, `plugins.xml`, params YAML, a launch file + lifecycle manager, and a tf
  tree; nav2_kit runs the moment you invoke it (~70 ms one-time cppyy bringup).
- **The world and the loop are just Python.** The occupancy grid is a NumPy array;
  the follow controller is a Python function you can breakpoint and edit. You iterate
  in seconds, not `colcon build` cycles.
- **It is the same `libnav2_*.so`.** `Costmap2D` and `NavFn` are Nav2's own classes,
  header-following, so nav2_kit tracks whatever Nav2 is installed — no binding to fall
  behind.
- **A prototype-to-native path.** As with the other kits, this is the L0 rung:
  prototype the stack with cppyy JIT today; the same calls lower to a compiled Nav2
  plugin when you want to deploy inside the real servers.

**What stock Nav2 buys that this does not.** A production stack: lifecycle
management, dynamic costmap layers from live sensors, tf/localization, recovery
behaviors, the full planner/controller/behavior-tree ecosystem, and the operational
maturity of the servers. nav2_kit is for *composing and prototyping from the cores*,
not for running a robot in production.

---

## The honest part: what is a clean core, and what is not

nav2_kit draws the line where the evidence does, and the **M6d lifecycle unlock** moved
it (full detail in [REPORT.md](REPORT.md)):

- **Pure cores (surfaced, no rclcpp at all): `Costmap2D`, `NavFn`.** Plain classes —
  `Costmap2D(w, h, res, ox, oy)`, `NavFn(nx, ny)` on a raw `unsigned char*` cost array.
  No node, no tf, no pluginlib. Directly drivable.
- **Lifecycle-coupled cores (NOW surfaced, M6d): Smac 2D + the real RegulatedPurePursuit
  controller.** These take a `LifecycleNode` (and RPP a `Costmap2DROS` + `tf2_ros::Buffer`)
  — and the key insight is that **a `LifecycleNode` is a plain class you construct
  in-process from Python**, exactly like the `rclcpp::Node` we already build. So
  nav2_kit builds the node object (and a plugin-free `Costmap2DROS`) the ctors ask for —
  **no lifecycle server, no pluginlib, no YAML.** The showcase's follow controller can
  now be Nav2's *actual* RPP (`--controller rpp`); the ~30-line Python pure-pursuit is a
  lightweight *choice*, no longer a forced limitation.
- **Still walled: Smac Hybrid-A\* (SE(2)).** Not a coupling problem — it constructs
  fine — but its OMPL-backed distance heuristic segfaults non-deterministically under
  Cling. A documented flaky partial, not shipped.

This honesty is the point: a real, working core road — now including the
lifecycle-coupled planners/controllers — with the one remaining wall (a runtime OMPL
instability, not "it needs a node") clearly marked.

---

## Two ways to use it

### Mode A — plan from Python on your own grid
Synthesize or load an occupancy grid, build a `Costmap2D`, plan with `NavFn`, and use
the path however you like (`d01_plan_grid.py`). Good for planner experiments,
map-based reasoning, and dataset generation where edit-run speed matters.

### Mode B — a whole miniature nav stack, live to rviz2
`nav2_kit/demos/d02_own_nav_stack.py` (the showcase) plans, follows over simulated
diff-drive kinematics, and publishes a live `nav_msgs/OccupancyGrid` +
`nav_msgs/Path` + `geometry_msgs/TwistStamped` via rclcppyy — so an rviz2 (Fixed Frame
`map`) shows the map, plan, and commanded velocity as the robot drives to the goal.
**Pick the pieces:** `--planner navfn|smac` and `--controller pursuit|rpp`. All four
combinations reach the goal; `--planner smac --controller rpp` runs Nav2's real Smac 2D
planner **and** its real RegulatedPurePursuit controller — both C++, driven from one
self-contained Python file.

---

## Advantages of the cppyy approach

Grounded in the spike's measured numbers (see [REPORT.md](REPORT.md)):

- **No plugin/YAML/lifecycle/build ceremony.** `python x.py` is the workflow; bringup
  is a one-time ~70 ms JIT.
- **Header-following, tracks the installed Nav2.** No hand-maintained binding.
- **Bulk data stays fast.** A NumPy grid → `Costmap2D` is a single `memcpy`
  (~600–3600× a per-cell Python loop); the plan never leaves C++ (NavFn on 1024² in
  tens of ms vs ~2 s for a pure-Python A\* — the orchestration story).
- **A prototype-to-native lowering path**, as with bt_kit / pcl_kit / ompl_kit: the
  same calls become a compiled Nav2 plugin when you deploy.

---

## Limits

nav2_kit is deliberately **not a Nav2 stack**: no lifecycle *servers*/manager, no
pluginlib-by-name loading, no tf tree/localization, no dynamic obstacle/inflation
layers, no recovery behaviors. Surfaced: `Costmap2D` + `NavFn` (pure cores) and — since
M6d — Smac **2D** + the real RPP controller (via an in-process `LifecycleNode` +
plugin-free `Costmap2DROS`, still no servers). Smac **Hybrid-A\*** remains out (a flaky
OMPL-under-Cling crash). The complementary direction — loading a **Python
planner/controller plugin *inside* a real Nav2 server** — is a separate planned spike.
The full, honest list is in [REPORT.md](REPORT.md) §6.

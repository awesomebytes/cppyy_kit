# moveit_kit: use MoveIt 2 from Python via cppyy

`moveit_kit` is for planning robot-arm motions with MoveIt 2 from Python. MoveIt's
C++ code provides the robot model, FCL planning scene, KDL kinematics solver, and
OMPL planner. Python sets the problem, can provide validity callbacks, and reads
the resulting trajectory.

cppyy exposes MoveIt's installed C++ classes and methods to Python. `moveit_kit`
adds the setup needed to build a model and load the kinematics and planning plugins.

For a first motion plan, run the Panda pose-goal demo from a source checkout, in
the `cppyy_kit` directory:

```bash
pixi run -e moveit demo-moveit-plan
```

The demo prints a solved waypoint count, a trajectory duration of `0.00s`, and
joint start/goal values, then publishes a `DisplayTrajectory` on
`/display_planned_path` for RViz. The trajectory is geometric and has no time
parameterization, so it is not ready for controller execution. These commands use
the source checkout's Pixi environment.
For installed use, see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/)
and the published package `ros-jazzy-moveit-kit`.

For the API, see [SKILL.md](SKILL.md); for feasibility evidence and detailed
limitations, see [REPORT.md](REPORT.md).

---

## Plan the Panda arm with MoveIt's OMPL plugin

`moveit_kit/demos/d02_plan_pose_goal.py` plans the Panda arm to a Cartesian pose
goal using MoveIt's real OMPL `PlannerManager` plugin, then publishes the result as
a `moveit_msgs/DisplayTrajectory` on `/display_planned_path` for RViz. The full
runnable command is above; this excerpt shows the API sequence:

```python
moveit_kit.bringup_moveit(with_kinematics=True, with_planning=True)
node = moveit_kit.make_node("plan", moveit_kit.parameter_overrides(cfg.ompl, "ompl"))
moveit_kit.load_kinematics_solver(node, model, "panda_arm")
planner = moveit_kit.load_planner(node, model)                # OMPL, via pluginlib
result = moveit_kit.plan_pose_goal(planner, scene, "panda_arm", "panda_link8", target)
pub.publish(moveit_kit.display_trajectory(result, scene))     # rviz-compatible
```

The Panda config selects `geometric::RRTConnect`. The snippet shows the plan call;
the complete model, scene, and publisher setup is in the runnable demo.

---

## A callback missing from moveit_py

MoveIt's C++ `RobotState::setFromIK` has an overload that takes a validity callback
(`GroupStateValidityCallbackFn`). The solver calls it for IK candidates so the
application can reject collisions. The `moveit_py` API inspected in
[the report](REPORT.md) exposes `set_from_ik(group_name, pose, timeout)` without
that callback parameter.

cppyy exposes the C++ overload from MoveIt's headers. The kit supplies a helper
to pass a Python collision check to it:

```python
cb = moveit_kit.state_validity_callback(
    lambda rs, group, values: not scene.isStateColliding(rs, group.getName()))
state.setFromIK(jmg, target_pose, 0.2, cb)   # the C++ KDL solver calls your Python
```

In the recorded test, the C++ solver called this Python collision check
**70–122 times** during one `setFromIK` call and rejected colliding candidates.

---

## Side by side: build a model, do FK + IK, C++ vs Python

On the left, the shape of a MoveIt C++ program (RobotModelLoader + KDL IK) **and its
build system**. On the right, the runnable file this repo ships,
`moveit_kit/demos/d01_robot_state.py`.

### C++, `robot_state.cpp` + `CMakeLists.txt` (+ ament package)

```cpp
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ik");   // needs robot_description +
                                                       // robot_description_kinematics
                                                       // params (launch file / YAML)
  robot_model_loader::RobotModelLoader loader(node);   // reads params, loads KDL plugin
  auto model = loader.getModel();
  moveit::core::RobotState state(model);
  const auto* jmg = model->getJointModelGroup("panda_arm");
  state.setToDefaultValues(jmg, "ready");
  state.update();
  Eigen::Isometry3d target = state.getGlobalLinkTransform("panda_link8");
  state.setToRandomPositions(jmg);
  bool ok = state.setFromIK(jmg, target, 0.1);
  RCLCPP_INFO(node->get_logger(), "IK %s", ok ? "solved" : "failed");
}
```

```cmake
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
add_executable(ik robot_state.cpp)
ament_target_dependencies(ik moveit_core moveit_ros_planning rclcpp)
```

…and this does not run yet: you need the `CMakeLists.txt`, a `package.xml`, `colcon
build`, a launch file that puts `robot_description` + `robot_description_kinematics` on
the node's parameter server, and a rebuild on every edit.

### Python, `d01_robot_state.py` (moveit_kit, shipped in this repo)

```python
import rclcpp_kit
import moveit_kit

moveit = moveit_kit.bringup_moveit(with_kinematics=True)   # parse + KDL plugin
cfg = moveit_kit.panda_config()
model = moveit_kit.build_robot_model(cfg.urdf, cfg.srdf)   # from URDF+SRDF strings
jmg = model.getJointModelGroup("panda_arm")

state = moveit.core.RobotState(model)
state.setToDefaultValues(jmg, "ready"); state.update()
target = state.getGlobalLinkTransform("panda_link8")       # FK

rclcpp = bringup_rclcpp(); rclcpp.ok() or rclcpp.init()
node = moveit_kit.make_node("ik")
moveit_kit.load_kinematics_solver(node, model, "panda_arm")  # KDL via pluginlib
state.setToRandomPositions(jmg); state.update()
print("IK", "solved" if state.setFromIK(jmg, target, 0.1) else "failed")   # IK
```

Run it directly: `pixi run -e moveit demo-moveit-state`. `RobotState`, `setFromIK`,
`getGlobalLinkTransform` are MoveIt's **own** C++ methods, the kit only assembles the
node parameters and loads the KDL plugin for you (the plugin/parameter bootstrap that
would otherwise be a launch file; see REPORT.md §2).

### What this gives you

- **No compile step, no CMake, no launch file.** `python x.py` is the workflow. The
  model comes from URDF+SRDF *strings*, and the kit boots MoveIt's plugin/parameter stack
  in-process (the bit that normally forces a launch file).
- **C++ methods beyond moveit_py bindings.** The `setFromIK` validity-callback overload
  omitted by moveit_py is available through the installed MoveIt headers. cppyy reads the
  installed 2.12.4 headers, so the kit tracks whatever MoveIt is installed.
- **Python in the loop where it helps.** A collision check or custom constraint can be
  a Python function called by the C++ solver. See [REPORT.md](REPORT.md) §3 for
  measurements.

---

## Limits

moveit_kit is a v0 spike focused on **planning**, not execution. It plans but does not
drive controllers (that is the ros2_control kit); the trajectory is geometric (no time
parameterization, because the full `PlanningPipeline` header crashes Cling and only the
planner plugin is used); pose-goal planning needs the kinematics solver loaded; the
config helper is panda-specific; and MoveItServo / CHOMP/STOMP/Pilz are reachable via the
same pluginlib method but are not surfaced. See the full list in
[REPORT.md](REPORT.md) §5.

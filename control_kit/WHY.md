# Why control_kit: Python and C++ ros2_control workflows

`control_kit` runs Python or compiled C++ implementations of ros2_control's
`ControllerInterface`. Use a Python implementation to prototype the update law,
or compile a native controller with `cppyy_kit` so its update calls no Python.
Both can run inside an in-process `controller_manager`. A reusable plugin bridge also runs a Python law in a separately launched manager;
the native controller can be exported directly.

Install `ros-jazzy-control-kit` with `pixi add -c https://prefix.dev/awesomebytes -c robostack-jazzy -c conda-forge ros-jazzy-control-kit`.
See the [Getting Started guide](https://awesomebytes.github.io/cppyy_kit/getting-started/)
for setup, or the [project repository](https://github.com/awesomebytes/cppyy_kit).
The `control` Pixi environment and `demo-control-python` task below require a
source checkout. See [SKILL.md](SKILL.md) for API and ordering rules, and
[REPORT.md](REPORT.md) for implementation details and measurements.

## Python controller in an external manager

The repository includes a [Python position servo](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/python_position_controller.py)
loaded into Gazebo's ros2_control manager through a reusable bridge. Its
`update(measured_positions, dt)` computes commands in Python. The bridge handles
ROS interfaces and plugin loading. `cppyy_kit` compiles and caches the bridge,
and `control_kit` exports the plugin overlay without a per-controller CMake package.

Follow the [external-manager instructions](EXTERNAL_CONTROLLERS.md).
This requires the source checkout. It is separate from the cross-inherited
in-process example below. Python allocation and GIL acquisition remain in the
update path, so this is not a hard real-time controller.

The [admittance example](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/python_admittance_controller.py)
accepts force targets. Its spring dynamics remain Python while a cached
native Eigen kernel computes the Jacobian and damped inverse-kinematics step.

## Native C++ controller example

This example requires the repository checkout. From this guide's directory:

```bash
pixi run --manifest-path ../pixi.toml -e control demo-control-native
```

The demo compiles [PositionServo](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/control_kit/native_controller.cpp) through
`cppyy_kit`, adds an owning C++ instance to the real controller manager, and moves
two mock joints to `[0.6, -0.35]` radians. It prints `OK` when the maximum measured
error is below 0.002 rad. All lifecycle methods and `update()` execute C++.
Python configures the controller and supplies goals; it does not implement the
update callback.

The controller applies a proportional position step bounded by per-joint speed
and position limits. Activation starts from measured positions. Invalid target
arrays are rejected without changing the previous target. The update uses
preallocated buffers and a nonblocking target handoff. This does not establish
hard real-time scheduling on the host.

`ControlRig.add_native_controller(controller, name, parameters=...)` retains
normal C++ shared ownership. The [native helper](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/control_kit/native_control.py)
also exports the cached build as a pluginlib overlay for an external manager.
See the [external-manager guide](EXTERNAL_CONTROLLERS.md) for plugin export and loading.

## Python controller example

The controller below uses mock hardware and the real read/update/write loop.

```python
import rclcpp_kit
import control_kit as ck

rclcpp_kit.bringup_rclcpp().init()
ck.bringup_control()

class MyPD(ck.ControllerInterface):                 # derive the REAL base class
    def __init__(self):
        super().__init__(); self.target = [0.5, -0.3]; self.kp = 0.4
    def on_init(self):                       return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self):
        return ck.interface_config(["joint1/position", "joint2/position"])
    def state_interface_configuration(self):
        return ck.interface_config(["joint1/position", "joint2/position"])
    def on_configure(self, prev):            return ck.CallbackReturn.SUCCESS
    def on_activate(self, prev):             return ck.CallbackReturn.SUCCESS
    def on_deactivate(self, prev):           return ck.CallbackReturn.SUCCESS
    def update(self, time, period):                  # the framework calls this each cycle
        for i in range(ck.n_command_interfaces(self)):
            cur = ck.read_state(self, i)
            ck.write_command(self, i, cur + self.kp * (self.target[i] - cur))
        return ck.return_type.OK

rig = ck.make_controller_manager(ck.mock_system_urdf(["joint1", "joint2"]))
rig.add_python_controller(MyPD(), "pd")             # inject, no plugin xml, no .so
rig.configure("pd"); rig.activate(["pd"])
rig.run(seconds=2.0, rate_hz=100)                   # the REAL read/update/write loop
```

This example needs no `plugin_description.xml`, `CMakeLists.txt`, `package.xml`, colcon
build, launch file, spawner, or second process. `MyPD` derives from
`controller_interface::ControllerInterface`. A real
`controller_manager::ControllerManager` calls its `update()` method in the control loop.
The manager uses `mock_components/GenericSystem` hardware created from a URDF string.

From a source checkout, run `pixi run -e control demo-control-python`. The demo reports successful configure and activation, runs about 300 cycles at 100 Hz, and ends with `OK` when measured tracking error is below 0.1 rad.

## The stock plugin workflow

To add a controller you write, minimum:

1. **A C++ class** deriving `controller_interface::ControllerInterface`, implementing the
   pure virtuals `on_init`, `command_interface_configuration`, `state_interface_configuration`,
   `update`, plus the lifecycle `on_configure`/`on_activate`/`on_deactivate`, in a
   `.hpp` + `.cpp` pair, with visibility macros.
2. **A `plugin_description.xml`** declaring the class as a `pluginlib` plugin with its
   `base_class_type`.
3. **A `CMakeLists.txt`**, `ament_cmake` project, `pluginlib_export_plugin_description_file`,
   `generate_parameter_library` for the params, link `controller_interface` /
   `hardware_interface` / `rclcpp_lifecycle`, install targets.
4. **A `package.xml`** with the build/exec deps.
5. **A colcon build** of the workspace to produce the `.so` and register the plugin in the
   ament index.
6. **A YAML** with the controller's parameters and type, and a **launch file** starting
   `ros2_control_node` (or `controller_manager`) with the robot description + that YAML.
7. **A spawner** (`ros2 run controller_manager spawner my_controller`) to load, configure
   and activate it into the running manager.

Each change to the control law requires a rebuild and relaunch. This takes longer than
rerunning a Python script.

## Side-by-side: Python injection

| | stock ros2_control | control_kit |
|---|---|---|
| controller definition | C++ `.hpp`+`.cpp` deriving `ControllerInterface` | Python class deriving the *same* `ControllerInterface` |
| plugin registration | `plugin_description.xml` + `pluginlib` export | none (injected via `add_controller`) |
| build | `CMakeLists.txt` + `package.xml` + colcon | none (cppyy JITs the glue) |
| run | launch `ros2_control_node` + YAML + spawner | `make_controller_manager()` + 3 calls, in-process |
| iterate | rebuild + relaunch + respawn (minutes) | rerun the script (seconds) |
| hardware | real / a `SystemInterface` plugin | `mock_components/GenericSystem` from a URDF string |
| the loop | CM's RT thread | `rig.run()` in your Python process (the real `read/update/write`) |

## What this is for (and what it isn't)

**Use control_kit** to prototype a control law against ros2_control, teach the API, or
test a controller with mock hardware before building a plugin. See [REPORT.md](REPORT.md)
§4 for measurements. In that test, both controllers completed 100 Hz cycles without
late cycles. At 1 kHz, both reached the average rate, but Python garbage collection and
GIL pauses caused late cycles. This test does not establish hard real-time behavior.

The jitter benchmark measured this rig at 1 kHz with `prctl(PR_SET_TIMERSLACK, 1)`,
`mlockall`, and CPU pinning. It measured about 2.4 µs median wakeup latency for the
in-process `ControllerManager` loop on a stock kernel. See the
[jitter benchmark report](../docs/jitter_bench/REPORT.md). Tail spikes under load remain
until `SCHED_FIFO` and preemption settings are applied.

**Use a compiled plugin** when a separately launched `controller_manager` must load the
controller by type name. `install_native_controller_plugin(prefix)` compiles the
native position controller and creates its pluginlib overlay. It builds without
loading the exported plugin into the Python process. The in-process example uses
a separate build without plugin registration. The controller interface is the same.

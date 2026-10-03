# Why control_kit: Python and C++ ros2_control workflows

ros2_control provides no Python controller API. A stock controller needs a C++ class,
plugin metadata, a build, and a running `controller_manager`. control_kit lets Python
code add a controller instance to a manager in the same process.

## Stock ros2_control: what a new controller costs

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

## The same controller in Python

```python
import rclcpp_kit
import control_kit as ck

bringup_rclcpp().init()
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

## Side-by-side

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
controller by type name. See REPORT §3, Route B. A prototype can be written in Python
and then ported to a native C++ plugin with the L2 direct-compile procedure. The interface
contract is the same.

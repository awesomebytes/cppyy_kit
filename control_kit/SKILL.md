# control_kit API reference

Use `control_kit` to write a ros2_control controller in Python and run it inside a
`controller_manager::ControllerManager` in-process. `control_kit.bringup_control()`
returns the C++ `controller_interface` namespace. Initialize rclcpp before creating
a controller manager.

In a Pixi project configured with the channels in
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/), install
`pixi add ros-jazzy-control-kit` and run scripts with
`pixi run python your_script.py`. Stock controller examples also need
`pixi add ros-jazzy-ros2-controllers`. See the
[kit overview](https://awesomebytes.github.io/cppyy_kit/control_kit/WHY/) and
[binding report](https://awesomebytes.github.io/cppyy_kit/control_kit/REPORT/).
Commands using `pixi run -e control` require this repository checkout.

## Bring-up

```python
import rclcpp_kit
import control_kit as ck

rclcpp_kit.bringup_rclcpp().init()                 # rclcpp must be up
ns = ck.bringup_control()               # JIT-include CM/controller headers, load .so, cppdef glue
                                        # returns cppyy.gbl.controller_interface; idempotent
ck.load_message_support()               # OPTIONAL: std_msgs typesupport for a command topic
ck.warmup()                             # OPTIONAL: front-load the CM-construction JIT
```

## The mock hardware rig

```python
urdf = ck.mock_system_urdf(["joint1", "joint2"])   # mock_components/GenericSystem, 2 joints
#   mock_system_urdf(joints, command_interfaces=("position",),
#                    state_interfaces=("position","velocity"),
#                    robot_name="mock_bot", initial_value=0.0) -> URDF str
rig = ck.make_controller_manager(urdf, update_rate=100)   # real ControllerManager -> ControlRig
#   make_controller_manager(urdf, update_rate=100, node_name="controller_manager",
#                           parameters=None) -> ControlRig
```

`rig.cm` is the C++ `controller_manager::ControllerManager`. Use its native methods
(`is_resource_manager_initialized()`, `get_update_rate()`, `get_loaded_controllers()`,
`read`/`update`/`write`, ...).

## ControlRig methods

| method | does |
|---|---|
| `rig.load_controller(name, type, parameters=None)` | load a **stock C++** controller by pluginlib type; `parameters` (dict) set on its node. **Before the loop.** |
| `rig.add_python_controller(controller, name, type_label="python_controller")` | inject a **Python** controller instance (a `ControllerInterface` subclass). **Before the loop.** |
| `rig.set_controller_parameters(ctrl, {...})` | set params on a loaded controller's node |
| `rig.configure(name) -> bool` | configure (INACTIVE); True on OK |
| `rig.activate(names, timeout_s=5.0) -> bool` | activate (ACTIVE); off-thread switch + pumped `update()` |
| `rig.deactivate(names, timeout_s=5.0) -> bool` | deactivate (INACTIVE) |
| `rig.update(period=None)` | one `read`→`update`→`write` cycle |
| `rig.spin()` | `executor.spin_some()`, service controller-node topics (not during a switch) |
| `rig.run(seconds, rate_hz=None, spin=False, on_cycle=None) -> int` | hold the loop at `rate_hz` for `seconds` |
| `rig.add_node(node)` | add an rclcpp node (e.g. a command publisher) to the CM executor |

**Ordering rule:** load/configure/add every controller **before** the first
`update()`/`activate()`/`run()`. After the loop has run, `load_*`/`add_*` raise (the CM's
real-time-safe list swap would deadlock a synchronous load), mirrors `ros2_control_node`.

## Writing a Python controller

Derive `ck.ControllerInterface` and override the real C++ virtuals by name:

```python
class MyController(ck.ControllerInterface):
    def __init__(self):
        super().__init__()                                   # REQUIRED
    def on_init(self):                       return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self):
        return ck.interface_config(["joint1/position"])      # what to claim
    def state_interface_configuration(self):
        return ck.interface_config(["joint1/position"])
    def on_configure(self, prev_state):      return ck.CallbackReturn.SUCCESS
    def on_activate(self, prev_state):       return ck.CallbackReturn.SUCCESS
    def on_deactivate(self, prev_state):     return ck.CallbackReturn.SUCCESS
    def update(self, time, period):                          # called every cycle
        n = ck.n_command_interfaces(self)
        for i in range(n):
            x = ck.read_state(self, i)                       # read hardware state
            ck.write_command(self, i, ...)                   # write hardware command
        return ck.return_type.OK
```

### Enums and helpers

| name | note |
|---|---|
| `ck.CallbackReturn` | `.SUCCESS` / `.FAILURE` / `.ERROR`, return from `on_init`/`on_*` |
| `ck.return_type` | `.OK` / `.ERROR`, return from `update` |
| `ck.ok(value) -> bool` | True if a returned `return_type` is OK (handles the uint8→1-char-str crossing) |
| `ck.interface_config(names, config_type="individual")` | build an `InterfaceConfiguration` (`"individual"`/`"all"`/`"none"`) |
| `ck.interface_configuration_type` | the `INDIVIDUAL`/`ALL`/`NONE` enum |

### Interface accessors (call inside `update`, after activation)

| function | returns / does |
|---|---|
| `ck.n_state_interfaces(self)` / `ck.n_command_interfaces(self)` | count of claimed interfaces |
| `ck.read_state(self, i)` | value of the i-th claimed **state** interface (float) |
| `ck.read_command(self, i)` | value in the i-th claimed **command** interface |
| `ck.write_command(self, i, value)` | write the i-th claimed **command** interface |

Interface order matches the `*_interface_configuration` names.

## Full example (self-commanding PD)

```python
rclcpp_kit.bringup_rclcpp().init(); ck.bringup_control()

class PD(ck.ControllerInterface):
    def __init__(self): super().__init__(); self.target=[0.5,-0.3]; self.kp=0.4
    def on_init(self): return ck.CallbackReturn.SUCCESS
    def command_interface_configuration(self): return ck.interface_config(["j1/position","j2/position"])
    def state_interface_configuration(self):   return ck.interface_config(["j1/position","j2/position"])
    def on_configure(self,s): return ck.CallbackReturn.SUCCESS
    def on_activate(self,s):  return ck.CallbackReturn.SUCCESS
    def on_deactivate(self,s):return ck.CallbackReturn.SUCCESS
    def update(self,t,p):
        for i in range(ck.n_command_interfaces(self)):
            c = ck.read_state(self,i); ck.write_command(self,i, c+self.kp*(self.target[i]-c))
        return ck.return_type.OK

rig = ck.make_controller_manager(ck.mock_system_urdf(["j1","j2"]))
rig.add_python_controller(PD(), "pd"); rig.configure("pd"); rig.activate(["pd"])
rig.run(seconds=2.0, rate_hz=100)
# teardown is automatic (registered atexit): deactivate + drop the CM before rclcpp shutdown
```

## Repository tasks

Run these from this repository checkout:

```
pixi run -e control demo-control-rig      # d01: CM + mock HW + stock C++ controller (Stages 1-2)
pixi run -e control demo-control-python   # d02: the Python PD controller showcase (Stage 3)
pixi run -e control bench-control         # bench: Python vs C++ loop numbers (Stage 4)
pixi run -e control test-control          # the test suite (auto-skips without ros2_control)
```

## Fewer missed deadlines (real-time knobs)

This example requires the repository checkout's `jitter_bench` module. The jitter
benchmark measured these unprivileged knobs on this rig's 1 kHz loop
(~2.4 µs median wakeup on a stock kernel,
[report](https://awesomebytes.github.io/cppyy_kit/docs/jitter_bench/REPORT/)):

```python
from jitter_bench.harness import apply_timerslack, try_mlockall, apply_affinity
apply_timerslack(1)   # Linux default slack is 50 us; this alone moved p50 52.4 -> 2.4 us
try_mlockall()        # lock pages, avoid page-fault stalls (works unprivileged)
apply_affinity(2)     # pin the loop to one core
# apply_scheduling("fifo", 50)  # needs an rtprio grant; see jitter report, Stage 1
```

Apply them before `rig.run()`. A `nogil` C++ loop body additionally holds its median
under machine load where a Python loop degrades (2.1–2.3 µs vs ~5 µs p50 in the same
benchmark).

## Common errors (see REPORT §2)

- **Load before the loop** (above). **`super().__init__()` is required** in your controller.
- Derive `ck.ControllerInterface` **directly** (not a further Python/JIT base) or cppyy's
  override dispatcher fails.
- A returned `return_type` is a 1-char `str`, use `ck.ok(...)`, not `int(...)`.
- Clean exit is automatic via the teardown registry; don't `unload_controller` a Python
  controller yourself.

## Position task callback

`make_position_task_controller(joints, update_positions)` creates a real
`ControllerInterface`. The callback receives a position dictionary and the update
period in seconds. It returns a position dictionary. Omitted joints hold their
current position. Nonfinite commands hold all positions and return a controller
error. `controller.last_error` records callback failures.

`control_kit.task_control.TaskModel` reads a URDF, computes link transforms with a
cached native Eigen kernel, and solves weighted damped least squares. Callers
assemble task residuals and Jacobians and provide weights and damping.

## Native position servo

Use `control_kit.native_control.make_native_position_controller()` for a compiled
C++ `ControllerInterface`. Python submits goals; `update()` reads and writes the
loaned hardware interfaces in C++. It does not call Python or acquire the GIL.

```python
from control_kit.native_control import make_native_position_controller
controller = make_native_position_controller()
rig.add_native_controller(controller, "servo", parameters={
    "joints": ["joint1", "joint2"], "gain": 6.0,
    "speed_limits": [0.5, 0.5],
    "lower_limits": [-1.0, -1.0], "upper_limits": [1.0, 1.0],
})
assert rig.configure("servo")
assert rig.activate(["servo"])
assert controller.set_target([0.4, -0.2])
rig.run(seconds=2.0)
```

Call `add_native_controller` before the update loop. It preserves the factory's
owning `shared_ptr`. Activation holds measured positions and clears prior goals.
The target order matches `joints`. Revolute positions use radians; prismatic
positions use metres. Gain uses s^-1 and speed limits use joint units/s. Parameters
are read on configure. Missing speed limits default to 1 joint unit/s. Missing
position limits leave positions unbounded.

`set_target` rejects wrong sizes, nonfinite values and out-of-bounds targets. Its
handoff uses a mutex; the update uses `try_lock` and retains the previous target
if a writer holds the mutex. Update storage is allocated during configuration.
The update requires a finite period with `0 < dt <= 0.1 s` and finite measured
positions inside the configured limits. Invalid input returns controller ERROR.
This mechanism alone does not provide a hard real-time execution guarantee.

`install_native_controller_plugin(prefix)` builds a pluginlib export without
loading the controller into Python. It copies the library and discovery metadata
into `prefix`. Prepend that directory to `AMENT_PREFIX_PATH` before starting an
external controller manager. Load type `control_kit_native/PositionServo`. Send
`std_msgs/msg/Float64MultiArray` goals to `/<controller_name>/commands`. The native
subscriber uses reliable QoS with depth 1. The overlay survives cache cleanup.

Repository demo: `pixi run -e control demo-control-native` runs the native servo
inside a real in-process controller manager with `mock_components/GenericSystem`.
The two measured positions should reach their goals within 0.002 rad after 3 s.


## Python law in an external controller manager

`control_kit.python_control.install_python_controller_plugin(prefix)` builds and
exports the reusable `control_kit_python/PythonController` plugin. Prepend the
returned overlay to `AMENT_PREFIX_PATH` before launching the manager. Parameters
are `joints`, `python_module`, `python_class`, absolute `python_path`, and
`python_options` (a JSON object encoded as a string).

The selected Python class supplies `configure(joints, options)`,
`set_target(values)`, and `update(measured_positions, dt_seconds)`. It may supply
`activate(measured_positions)` and `deactivate()`. The update returns one finite
position command per joint. The bridge validates the whole result before writing
commands. Target messages on `~/commands` are `Float64MultiArray`; their meaning
belongs to the selected Python class. For the position servo they are joint goals;
for the admittance example they are virtual force components in newtons.

Python runs inside the manager process. The bridge serializes calls and acquires
the GIL. It does not provide hard real-time timing. Compile native helpers and
warm their call signatures during configuration, before activation. Restart the
host after editing imported Python modules. Dependencies must be available in the
host interpreter, which must match the bridge's Python major/minor version.

See the repository [Gazebo examples](EXTERNAL_CONTROLLERS.md).


For Eigen-heavy kernels, call `cppyy_kit.prebuild(**build)` before
`cppyy_kit.cppdef_cached(**build)` with the same code, declarations and options.
This compiles the implementation outside Cling, then exposes only declarations.
The [demo kinematics wrapper](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/arm_kinematics.py) follows this sequence and warms
its FK and damped-IK entry points before the controller activates.

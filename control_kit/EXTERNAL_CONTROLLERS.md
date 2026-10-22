# Load a Python controller into ros2_control

Use the reusable adapter when an existing Gazebo or `controller_manager` process
must load your Python control law. The adapter handles pluginlib, lifecycle calls,
ROS commands and joint interfaces. Your class returns ordered position commands.
These APIs are available in release 0.4.2; the package recipe includes
all native sources for the next build. Use the activated Linux ROS Jazzy compiler
and Python environment for both the build and host process.

Save this as `hold_position.py` in your application directory:

```python
class HoldPosition:
    def configure(self, joints, options):
        pass

    def activate(self, positions):
        self.reference = list(positions)

    def update(self, positions, dt):
        return self.reference

    def set_target(self, values):
        return False
```

In your launcher, before starting the controller manager:

```python
import os
from control_kit.python_control import install_python_controller_plugin

prefix = install_python_controller_plugin("./controller_overlay")
os.environ["AMENT_PREFIX_PATH"] = prefix + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
```

Save `hold.yaml`, replacing the joint names and absolute Python path:

```yaml
hold:
  ros__parameters:
    type: control_kit_python/PythonController
    joints: [joint_1, joint_2]
    python_module: hold_position
    python_class: HoldPosition
    python_path: /absolute/path/to/application
    python_options: '{}'
```

Start the manager with your existing robot description and hardware setup, then:

```sh
ros2 run controller_manager spawner hold -p hold.yaml
```

The controller holds the measured positions captured on activation. Interfaces
must be free. If another controller owns them, load with `--inactive`, then use
`ros2 control switch_controllers --strict` to deactivate the old controller and
activate the new one in a single request. Restore the previous controller before
unloading the example.

`configure(joints, options)`, `set_target(values)` and `update(positions, dt)` are
required; `activate(positions)` and `deactivate()` are optional. `dt` is seconds;
positions use each joint's units. Returning `False` rejects a target. A class can
interpret reliable depth-1 `Float64MultiArray` messages on `~/commands` as joint
goals, virtual forces or another application-specific vector. Update must return
one finite position per joint. The adapter validates the complete result before
writing interfaces. A zero-duration update holds the previous command and skips
Python; negative/nonfinite periods and callback failures return controller ERROR.

Python runs inside the manager process. Calls are serialized and acquire the GIL.
This does not provide hard-real-time scheduling. Compile and warm native helpers
before activation. Restart the host after editing an imported controller module.
The adapter links the building environment's Python; keep that environment and
its compatible extension dependencies available to the host.

For compiled C++ updates, use `install_native_controller_plugin(prefix)` from
`control_kit.native_control`, type `control_kit_native/PositionServo`. Configure
`joints`, `gain`, `speed_limits`, `lower_limits` and `upper_limits`; send joint
position targets to `~/commands`. In an in-process `ControlRig`, use
`make_native_position_controller()` and `rig.add_native_controller(...)` instead.
The factory and manager share C++ ownership.

The example algorithms remain separate from the adapter:
[Python servo](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/python_position_controller.py),
[admittance law](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/python_admittance_controller.py), and
[Eigen kinematics](https://github.com/awesomebytes/cppyy_kit/blob/main/control_kit/demos/arm_kinematics.py). These files require a repository
checkout. The native position-servo demonstration runs with
`pixi run -e control demo-control-native` from the repository root.

# Add a Python trajectory critic to Nav2

Use this adapter to score DWB trajectories inside the existing Nav2
`controller_server`. Keep the robot's map, sensors, planner, collision critics
and normal launch configuration. These APIs are source additions after release
0.4.1. The next package build includes DWB and the adapter's C++ sources. Build
and launch from the same activated Linux ROS Jazzy/Python environment.

In your launch setup, before starting `controller_server`:

```python
import os
from nav2_kit.python_critic import install_python_critic_plugin

prefix = install_python_critic_plugin("./nav_overlay")
os.environ["AMENT_PREFIX_PATH"] = prefix + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
```

Save a policy as `forward_policy.py` in your application directory:

```python
import math

class ForwardOnly:
    def configure(self, options):
        self.half_angle = math.pi / 6

    def prepare(self, yaw, path_heading, goal_distance):
        pass

    def score(self, vx, vy, wz, final_heading):
        if math.hypot(vx, vy) > 0.0001:
            if abs(math.atan2(vy, vx)) > self.half_angle:
                return None
        return 0.0
```

Append `ForwardOnly` to the existing `FollowPath.critics` list. Add these fields
under `controller_server.ros__parameters.FollowPath`:

```yaml
ForwardOnly.class: nav2_kit_python::PythonCritic
ForwardOnly.scale: 1.0
ForwardOnly.python_module: forward_policy
ForwardOnly.python_class: ForwardOnly
ForwardOnly.python_path: /absolute/path/to/application
ForwardOnly.python_options: '{}'
```

When Nav2 loads the critic, the adapter imports the class in `controller_server`.
This policy rejects sideways/backward translation outside a 30-degree cone.
Keep zero-translation samples available so DWB can consider turning in place.
The repository's [forward-camera policy](https://github.com/awesomebytes/cppyy_kit/blob/main/nav2_kit/demos/forward_camera_policy.py) also
scores heading and waits until the robot faces the path before translating.

`configure(options)` receives the decoded JSON object. `prepare` runs once per
DWB cycle with yaw, direction to a path look-ahead point, and distance to goal.
`lookahead_distance` selects the look-ahead distance in metres. `score` receives
base-frame linear velocities in m/s, angular velocity in rad/s and the candidate's
final yaw in radians. Return `None` to reject a candidate, or a finite nonnegative
cost. Python failures and invalid costs become DWB trajectory errors. DWB still
selects the command using the complete critic set.

The kit builds and caches a reusable shared library, then writes pluginlib and
ament discovery metadata into the overlay. It copies the library so the overlay
survives cache cleanup. Python callbacks acquire the GIL inside the scoring loop;
keep them short. Restart the host after editing the imported policy. This route
does not provide a hard-real-time execution guarantee.

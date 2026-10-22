# Face the direction of travel with a Python Nav2 plugin

This source-checkout experiment adds a Python trajectory critic to the existing
DWB controller. It favours path-facing headings and rejects translation outside
a forward camera cone. DWB still generates trajectories, checks obstacles and
chooses the base command. The default simulator profile is unchanged.

## Run the forward profile

Start in this document's `navigation` directory. Stop any existing simulator.
Use the source simulator's `kit` environment, which contains cppyy and Nav2:

```bash
(
  cd ../tiago_pro_jazzy
  pixi run -e kit env \
    PYTHONPATH="$PWD/../../..:$PWD/../../../nav2_kit:$PWD/../../../rclcpp_kit" \
    TIAGO_PRO_NAV_PROFILE=forward TIAGO_PRO_NAV_DEPTH=1 \
    bash ../simulator_guard.sh -- ros2 launch "$PWD/playground.launch.py" \
    gui:=true rviz:=true moveit:=false navigation:=true
)
```

The launch builds/caches the plugin and exports its metadata before starting
Nav2. Use an RViz Nav2 goal in known free space. Restart Nav2 after changing the
Python policy, since the host caches its imported module. Ctrl-C in the launch
terminal stops the owned stack.

`TIAGO_PRO_NAV_PROFILE=default` retains the original DWB configuration.
`TIAGO_PRO_NAV_DEPTH=1` independently enables the depth obstacle source in both
costmaps; omit it to retain laser-only sensing. These launch settings require this repository checkout.

## What a Nav2 plugin is

Nav2 servers load C++ implementations of interfaces through pluginlib. A planner
produces a path; a controller produces base velocity commands; a DWB critic
scores or rejects candidate trajectories inside that controller. This experiment
uses the smaller critic interface, rather than replacing the controller.
[Nav2 plugin overview](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/).

The new [PythonCritic bridge](../../../nav2_kit/nav2_kit/python_critic.cpp) is a
real `dwb_core::TrajectoryCritic` loaded inside `controller_server`. It imports
[ForwardCameraPolicy](../../../nav2_kit/demos/forward_camera_policy.py) and calls:

```python
policy.configure(options)
policy.prepare(robot_yaw, path_heading, goal_distance)
score = policy.score(vx, vy, wz, predicted_final_heading)
```

Angles use radians; velocities are in the base frame, in m/s and rad/s.
`None` rejects a candidate. A finite nonnegative float contributes to its cost.
Python errors and malformed scores reject the candidate. `prepare` runs once
per controller cycle; `score` runs for each candidate reaching this critic.
This acquires the GIL and is not a hard real-time implementation.

The policy allows limited holonomic travel inside a +/-30 degree cone. It
requests a turn before translation when the path heading is outside that cone. The adapter computes that heading from the robot to a look-ahead point on the path, avoiding changes between individual grid-segment tangents.
Near the goal, it releases path-heading scoring so the stock final-yaw critic
can act, while retaining the translation cone. The cone assumes the head camera
is facing forward relative to the base. Moving the head invalidates that
assumption; the policy does not track head pan or guarantee sensor visibility.

The profile retains stock DWB critics and adds `ObstacleFootprint` so predicted
poses are checked against the polygon. The original `BaseObstacle` critic alone
uses centre-cell costs and inflation. The profile is not merely a change to
one scalar heading weight. Nav2 recovery behaviours are separate from this
critic and can command motions outside its constraints.

The [export helper](../../../nav2_kit/nav2_kit/python_critic.py) uses
`cppyy_kit.prebuild()` and writes the DWB plugin XML and ament resource. Python
provides the policy; the reusable C++ bridge supplies the framework boundary.
This follows the same integration pattern as the ros2_control Python bridge.

## Minimal application code is the intended interface

The author supplies a Python policy class and a small configuration. The kit
supplies the C++ bridge, interpreter handling, plugin export and cache. There
is no per-policy CMake package, handwritten binding or C++ plugin subclass.
Changing the policy requires restarting its host, not writing another bridge.

The implemented policy has three methods and no ROS imports:

```python
class MyPolicy:
    def configure(self, options):
        ...  # Read policy parameters.

    def prepare(self, yaw, path_heading, goal_distance):
        ...  # Store context once per controller cycle.

    def score(self, vx, vy, wz, final_heading):
        ...  # Return a nonnegative cost, or None to reject.
```

This is the same authoring goal as the minimal Python ros2_control example.
The [complete forward policy](../../../nav2_kit/demos/forward_camera_policy.py)
is the executable example. Its repetitive framework work is in the kit.

## Why depth needs its own configuration

The original costmaps observe only `/scan_front_raw` and `/scan_rear_raw`.
Turning the robot does not make an unconfigured depth stream affect navigation.
The opt-in profile adds `/head_front_camera/depth/color/points` as a PointCloud2
observation source, with marking and clearing. The source simulator's cloud uses
`head_front_camera_depth_frame`, with native XYZ conventions already corrected
for this scene. Obstacles are accepted between 0.15 and 2 m height, within 0.3 to
2.8 m range. Lasers continue to provide surrounding coverage.

[Depth-only evidence](evidence/depth_only.json) disables the static layer and both
laser marking sources, clears the global costmap, and observes fresh depth marks.
This checks depth integration; it is not an end-to-end obstacle-avoidance success
rate or proof of full 3D collision clearance.

## Live heading and depth results

Both navigation runs started at the same measured pose to within 2e-8 m and
requested a goal 0.8 m east while initially facing north. Both reached their
Nav2 goal. The profile comparison is not an isolated performance benchmark:
the forward profile also adds footprint checking and tunes velocity samples.

| Measurement | Original profile | Forward profile |
|---|---:|---:|
| Translation commands outside +/-30 degrees | 77 / 112 | 0 / 113 |
| Measured distance outside that cone | 0.562 m | 0.00592 m |
| Total measured distance | 0.721 m | 0.711 m |

The forward endpoint was 0.09775 m from the requested goal, within the configured
0.1 m position tolerance. Command statistics include speeds above 0.03 m/s;
pose statistics discard increments below 0.1 mm. A velocity constraint does
not guarantee identical physical motion. See [baseline](evidence/baseline.json),
[forward](evidence/forward.json), and [measurement code](verify_forward.py).

The first tuning runs stalled and are retained as `forward_*_aborted.json`.
The successful profile samples exact zero X/Y translation for turn-in-place,
uses a heading weight of 80, and releases path-heading scoring within 0.25 m
of the goal while retaining the translation cone. A threshold of 0.0001 m/s
prevents small creep commands bypassing the cone. Reusing these values on a
different robot or controller frequency still needs validation.

With depth as the sole obstacle marking source, 3,796 of 3,910 sampled cloud
points between 0.5 and 1.5 m height landed in lethal costmap cells.
[Recorded observation](evidence/depth_only.json). This is sensor integration
proof, not an obstacle-avoidance success-rate measurement.

The [native plugin probe](evidence/plugin.json) checked pluginlib discovery and
rejection of malformed scores and Python exceptions, including SystemExit.
Its bridge-plus-policy measurement averaged 0.406 microseconds over 100,000
calls with an existing interpreter. It excludes full DWB scoring, obstacle
checks and ROS scheduling, and supplies no real-time bound.

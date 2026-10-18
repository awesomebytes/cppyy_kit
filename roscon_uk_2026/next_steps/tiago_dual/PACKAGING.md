# TIAGo++ package and integration ledger

Use `pixi.lock` for binary package versions and `sources.lock.repos` for exact
Git revisions. `build.sh` selects the 19 source packages needed by this baseline.
No source repository is patched. No channel publication has been performed.

| Capability | Existing dependency | Local integration |
| --- | --- | --- |
| Wrist F/T physics | PAL Schunk force_torque sensor macros and Gazebo Classic | Set generated sensor always_on=true so headless measurements update. |
| F/T state interfaces | RoboStack gazebo_ros2_control 0.4.10 | Existing six force/torque interfaces per wrist. |
| ROS wrench messages | RoboStack force_torque_sensor_broadcaster 2.53.1 | Spawn both existing PAL ft_sensor controller YAML files. |
| Known-load validation | gazebo_ros stock force-system plugin and gazebo_msgs ApplyLinkWrench | Apply bounded 5 N world-Z force using current simulation start time, then verify expiration and recovery. |

The F/T extension requires no rebuild, compiler invocation, additional package,
new plugin, or custom sensor implementation. Future recipes should install the
PAL descriptions/controller configurations and keep generated-model activation
in the launch adapter or propose the always_on setting upstream.

`validation.json` contains all measured force vectors, applied force magnitude,
observed change magnitude, post-release residual and service success for each
wrist. `shutdown.json` records whether the smoke session needed a forced kill.
Read the recorded pass fields before claiming a verified force response.

The trial validates a known applied force through existing simulator APIs. It
does not validate a physical sensor calibration or a contact manipulation task.

## Recorded headless validation, 18 October 2026

| Measurement | Left wrist | Right wrist |
| --- | --- | --- |
| Applied world-Z force | 5 N | 5 N |
| Measured force-vector change magnitude | 4.704 N | 4.665 N |
| Force residual after release | 0.0667 N | 0.0644 N |
| Received WrenchStamped messages | 2,409 | 2,395 |
| Six finite force/torque axes | Passed | Passed |

Acceptance required force-change error below 0.5 N and post-release force
residual below 0.5 N. Both passed. Base odometry moved 0.210 m; both arm joint 1
responses were approximately +0.150 rad. RGB, depth, cloud, scan, IMU, joints,
odometry and clock checks passed. The headless simulator shut down with launch
exit code 0 and no forced kill; no dual simulation was left running.

An initial load test correctly failed because its request start time was zero,
so Gazebo immediately expired the already-past three-second wrench. The final
test uses the observed simulation clock. This is a test-client correction, not
a simulator or sensor implementation change.

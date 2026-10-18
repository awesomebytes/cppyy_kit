# Navigate TIAGo PRO with Nav2

From this repository's `tiago_pro` directory, `pixi run playground` starts the
home simulator, native controllers, Nav2, MoveIt and RViz together. In RViz,
select **Nav2 Goal**, then click and drag on the map to choose a destination
and final heading. Keep the arms in the ready posture while driving.

To add navigation to an already running bare simulator, use
`pixi run navigation`. This task uses the same unified launch file with
simulation, MoveIt and RViz disabled. Do not launch a second navigator or base
command source alongside an existing navigation stack. Ctrl-C stops the
processes owned by the launch you started.

## Map, frames and displays

The home default loads `worlds/home/map.yaml`. Its 0.05 m raster projects known
house collision primitives; live front and rear lidar feed the costmaps.
It does not claim complete mesh coverage or validated doorway clearance.
Use the home map with the home world. `navigation/nav.launch.py` also generates
an obstacle-free 20 × 20 m map for explicit empty-world experiments; that map
is unsuitable for the furnished house.

Localization uses **simulation truth**. The native base plugin supplies world
coordinates in `odom → base_footprint`; navigation publishes identity
`map → odom`. There is no AMCL or SLAM, and no second node should publish that
same transform. The nonzero home spawn is already represented in odometry.

Keep RViz's fixed frame `map`. Relevant native topics are:

| Data | Topic |
|---|---|
| Static map | `/map` |
| Front/rear scans | `/scan_front_raw`, `/scan_rear_raw` |
| Global/local costmaps | `/global_costmap/costmap`, `/local_costmap/costmap` |
| Global path | `/plan` |
| Local controller path | `/local_plan` |
| Base command | `/mobile_base_controller/cmd_vel_unstamped` |
| Odometry | `/mobile_base_controller/odom` |

A fresh local path requires an active controller and a subscriber; an idle
robot need not publish one. The raw depth cloud and MoveIt Octomap are separate
from these lidar-driven Nav2 costmaps.

## Control and clearance

Stock NavFn planning, omnidirectional DWB control, costmaps, inflation,
behavior trees and lifecycle management use the installed Nav2 packages.
There is no private PAL navigation dependency. The planner commands ordinary
`Twist` messages through the native planar base plugin.

The padded working footprint is 1.10 × 1.00 m. It covers the ready endpoint's
measured arm/hand envelope, approximately x=[-0.0612,0.4953] m and
y=[-0.4016,0.4024] m, with extra margin. It does not cover arbitrary arm paths
or provide coordinated arm/base collision planning. Wheel traction and slip
are not validated; the plugin commands the chassis directly.

## Optional maintenance evidence

These checkout commands require an existing navigation stack. The first is
plan-only; the second moves the real simulated base. Keep other clients idle:

```bash
ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp pixi run python navigation/display_plan.py
ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp pixi run python navigation/verify_goal.py
```

They are diagnostics, not required for the normal RViz goal workflow. The
arrival client requests an offset goal and checks native action success,
actual commands and fresh odometry. Its earlier empty-world run finished
within 0.0973 m and 0.0887 rad; two subsequent user RViz goals also succeeded.
Reports are in `navigation/artifacts/` and the parent's `artifacts/`.
Those results establish the checked arrivals, not full house obstacle-avoidance
coverage. No autonomous goal is sent by startup.

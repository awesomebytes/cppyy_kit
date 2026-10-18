# MoveIt: plan and command arms, torso, head and grippers

Start the [complete playground](../README.md) with `pixi run playground` from
this repository's `tiago_pro` directory. The unified `playground.launch.py`
starts MoveIt after native controller activation. It does not execute a homing
trajectory or run motion checks during startup.

## Plan in RViz

Select a planning group in **MotionPlanning**, set its goal, then **Plan**.
Inspect the preview before **Execute**. The normal Plan button publishes the
preview itself; no helper publisher is required.

| Group | Goal interface |
|---|---|
| `arm_left`, `arm_right` | Cartesian tool marker or joint targets |
| `both_arms` | Both-arm joint targets; historical named `compact` pose |
| `both_arms_torso` | Joint targets including torso; named `ready` pose |
| `gripper_left`, `gripper_right` | Named `open` and `closed` or primary screw position |
| `head` | Named `center`, `look_left`, `look_right`, or the two Joints-tab values |

The ready pose uses PAL's public spherical-wrist home arm values with a
0.20 m torso lift. It is the simulator's direct initial state. Use its named
goal only when you deliberately want to return there later. The head has no
Cartesian IK solver.

Each gripper has one active screw joint. Its full travel is 0.000 m closed to
0.070 m open. The remaining joints follow native physical mimic PID couplings.
MoveIt's stock GripperCommand handle sends the final planned position to the
existing effort controller; it does not stream trajectory points or set jaw
speed from their timestamps. The coordinate is screw travel, not total aperture.
Free-space movement is not validation of contact grasping or object attachment.

## Native preview settings

The working playback preset uses **State Display Time = `0.05 s`**, loop and
interrupt enabled, and **Show Trail** disabled. The `1x` mode jumped between
endpoints in this local build even though manual scrubbing showed valid
intermediate states. Fixed-step playback resolved the user-visible problem;
the internal real-time timing defect remains unproven. Preview timing does
not alter controller timestamps.

The planned-path topic is `/display_planned_path`. The saved per-link mask
shows both arms, wrists, grippers and head while leaving the main Scene Robot
complete. This mask is static; it does not follow the selected planning group.
Use the native trajectory-slider panel to pause and scrub. Save preferred
layout/display changes in RViz; startup preserves the selected config file.

## Model, controllers and perception

The generated model comes from the simulator's canonical
`../generated/robot_description.urdf`, including both corrected ATI sensing
chains, package mesh URIs and physical gripper relationships. Pinned public
PAL semantic/collision configuration, stock KDL, OMPL and the simple controller
manager provide the planning integration. Setup fetches the exact revisions
in [sources.lock](sources.lock); no private PAL bringup is loaded.

Native controller mappings are:

- `arm_left_controller`, `arm_right_controller`, `torso_controller` and
  `head_controller`: `FollowJointTrajectory`, action suffix `follow_joint_trajectory`.
- `gripper_left_controller`, `gripper_right_controller`: `GripperCommand`,
  action suffix `gripper_cmd`, with a 10 N command-effort limit.

A standard floating virtual joint connects fixed `odom` to `base_footprint`.
TF supplies the current base pose; no planning group commands that virtual
joint. This is necessary because MoveIt 2.5.9 accumulates its native Octomap in
the model's planning frame. A robot-rooted map would accumulate observations
in the moving base frame after driving.

The stock `PointCloudOctomapUpdater` consumes
`/head_front_camera/depth/color/points`, using 2.5 cm voxels, 3 m maximum range,
2 Hz updates, point subsampling and a 2 cm robot self-filter offset.
`/moveit/filtered_head_depth` exposes the native filtered cloud. Enable scene
geometry to inspect collision occupancy. No manual ground/table/prop shapes
are added. Gazebo's physical floor is independent of this planning setup.

The updater retains observed ground and does not perform semantic ground
segmentation. Only visible depth surfaces contribute. Small props can be missed
or merged at the selected resolution, and occluded surfaces remain unknown.
Later free-space rays clear old observations; **Clear Octomap** discards the
map after a reset. This is collision occupancy, not object recognition or a
guarantee that a planned grasp will work.

## Optional maintenance checks

These commands require this checkout and the running stack. They cause real
simulated motion. Keep other command sources idle and use clear space:

```bash
# Native MoveIt gripper close/open, with action results and observed feedback.
ROS_DOMAIN_ID=77 pixi run python moveit/check_devices.py
# Native head named goals and return to center.
ROS_DOMAIN_ID=77 pixi run python moveit/check_devices.py --groups head
# Deliberately return torso and both arms to the ready pose.
ROS_DOMAIN_ID=77 pixi run python moveit/check.py --ready-only
```

`check.py --plan-only --ready-only` plans without execution. These optional
clients additionally audit stored arm/hand mesh heights against 5 cm above
z=0 before executing a native trajectory. That client-side audit is not a
manual ground collision object and is not added to the ordinary RViz workflow.
The normal UI uses joint limits, self-collisions and observed Octomap geometry.

`test_configure.py` checks generated semantic/controller references and named
joint bounds without running ROS. `test_plan_audit.py` checks rejection before
execution in the optional client. They are not startup dependencies.

Recorded evidence under `../artifacts/` includes `moveit_motion.json`,
`moveit_devices.json` and `ready_execution.json`. The device run completed four
full-stroke gripper goals and three head goals through native MoveIt execution;
worst settled errors were 3.30 micrometres for screws and 0.000938 rad for the
head. Earlier explicit-ground path experiments describe historical conditions;
they do not imply that the current cloud-only scene contains a manual floor.

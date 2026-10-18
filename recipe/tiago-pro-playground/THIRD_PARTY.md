# Distributed sources and local modifications

Source URLs and exact revisions are in provenance/sources.lock,
provenance/moveit/sources.lock, provenance/gripper/sources.lock and
provenance/worlds/home/sources.json. Original package manifests and license
notices accompany the installed resources.

- PAL Robotics robot descriptions, controller and MoveIt configuration:
  Apache-2.0. Original package data are installed without modification.
- Intel RealSense description: Apache-2.0. Meshes retain their source notices.
- ROBOTIS TurtleBot3 House: Apache-2.0. House table is explicitly static and its
  collision surface matches the visible top at 1.020 m. Integration adds a plain
  bright floor, balanced lighting and three dynamic primitive props. Map data
  are generated from static collision primitives.
- Open Source Robotics Foundation gazebo_models: CC-BY-3.0. Source:
  https://github.com/osrf/gazebo_models at the revision in sources.json.
  Included cafe_table, first_2015_trash_can, ground_plane, mailbox, sun and
  table_marble models retain their model.config author attribution. The trash
  can COLLADA ambient material is corrected to its original dark green.
  License: https://creativecommons.org/licenses/by/3.0/ . The upstream license
  statement is worlds/home/GAZEBO_MODELS_LICENSE.
- gazebo_ros_planar_move.cpp: Apache-2.0, original copyright header retained.
  Local patch applies velocity to base_footprint only, preserving articulated
  wrist force reactions. See provenance/patches.
- roboticsgroup_upatras_gazebo_plugins: BSD-3-Clause, original copyright and
  license retained. Local patch uses physical PID forces for mimic followers
  without position teleportation. See provenance/gripper/patches.

Runtime model generation applies the documented ATI force-sensor chain, native
initial posture, CPU ray sensor, optical frame and gripper control corrections.
MoveIt uses a floating odom virtual joint and depth-only Octomap integration.
These modifications are in the installed Python runtime sources.

# Distributed sources and local modifications

Source URLs and exact revisions are in provenance/sources.lock,
provenance/moveit/sources.lock and
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
- The Harmonic force-only follower system is part of cppyy_kit (BSD-3-Clause).
  It applies bounded PD joint forces; the original Classic plugin binaries are
  not distributed in this version. Source is retained in provenance/native.

Runtime generation applies the ATI sensing-chain correction, native initial pose,
GPU lidar sensors and gripper controls. Harmonic's cloud retains its native body
frame; no Classic optical-frame override is applied. Material scripts are converted
into Harmonic material parameters; texture notices accompany runtime/world_assets.
MoveIt uses a floating odom virtual joint and depth-only Octomap integration.
These modifications are in the installed Python runtime sources.

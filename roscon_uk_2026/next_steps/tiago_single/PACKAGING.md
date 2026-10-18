# Packaging ledger

No package was uploaded. The source pins are in `source-lock.repos`; the binary
versions and hashes are in `pixi.lock`.

## Source overlay used by the minimal launch

| Repository | Packages built |
| --- | --- |
| launch_pal | launch_pal |
| pal_urdf_utils | pal_urdf_utils |
| urdf_test | urdf_test (needed by the source dependency graph) |
| pmb2_robot | pmb2_description, pmb2_controller_configuration |
| omni_base_robot | omni_base_description, omni_base_controller_configuration |
| pal_hey5 | pal_hey5_description, pal_hey5_controller_configuration |
| pal_gripper | pal_gripper_description, pal_gripper_controller_configuration |
| pal_robotiq_gripper | pal_robotiq_description, pal_robotiq_controller_configuration |
| tiago_robot | tiago_description, tiago_controller_configuration |
| pal_gazebo_worlds | pal_gazebo_worlds (available for later world selection) |
| pal_gazebo_plugins | pal_gazebo_plugins |

The official public repos file omits `omni_base_robot`, although current launch
and model sources reference it. It also omits `pal_gazebo_plugins`, which supplies
the ground-truth odometry plugin requested by the PMB2 model. Both additional
repositories are explicitly pinned. Their sources built without modification.
The other original public repos are pinned but not all their packages are built.

## Narrow upstream patch

`patches/pal_urdf_utils-classic-lidar.patch` changes only the selected SICK Tim571
sensor tag. Classic uses `ray`; modern Gazebo retains `gpu_lidar`. Both use the
existing upstream sensor geometry, scan settings and ROS plugin. This avoids
relying on an unsupported modern sensor name in Classic. The local smoke used
this patch. No algorithm or kinematics was replaced.

## Environment integration

- Leave `PAL_DISTRO` unset. Setting it even to an empty string trips private
  PAL module registration in description CMake scripts. Build and run wrappers
  explicitly unset it.
- Pin `setuptools<80`. Current colcon's legacy `setup.py develop --editable`
  invocation failed with setuptools 80.9 and 84; 79 works.
- Use Cyclone DDS with the checked-in loopback configuration and up to 100 auto
  participant indices. The default small range was insufficient for peer trials.
- Own ROS domain is 161 and Classic master port is 11361.
- Simulator-starting entrypoints use the shared `../simulator_guard.sh` lock.
  Nested launches preserve descriptor 9 through the Python manager. A second
  playground start exits 75. Client-only smoke checks and stop commands do not
  take the simulator lock. The isolated
  viewer and full smoke subsequently passed; descriptor inheritance and a
  competing benign command returning 75 were verified at runtime.
- Untested world-name and Gazebo-IP changes were reverted. Keep the tested
  `default` world and launch sequentially; do not infer concurrent isolation
  from distinct Classic master ports.
- The local world is self-contained, with a ground plane, light and target box.
  Online model downloading is disabled. Package mesh search paths are explicit.

## Future channel build

The current evidence establishes a usable Humble/Classic source overlay and
identifies candidate recipes, not a publishable supported-Gazebo distribution.
Before packaging, validate the latest managed shutdown flow, select a
supported Gazebo/ROS combination, trim unused public repos, build overlay recipes
against that environment and run the same sensor/motion tests against the
installed packages. Full PAL bringup, Nav2, MoveIt and play_motion2 require a
separate dependency and integration pass.

## Viewer environment

RViz2 is installed separately in `tools/` using a second Pixi manifest and lock.
The launch wrapper adds only the source overlay package prefixes to
`AMENT_PREFIX_PATH` for mesh lookup, and uses domain 161 plus the same Cyclone
configuration. It does not source or update the simulator's environment.
The live GUI rendered the robot, RGB image, point cloud and lidar without
plugin errors. `tiago.rviz` contains the reusable display configuration.

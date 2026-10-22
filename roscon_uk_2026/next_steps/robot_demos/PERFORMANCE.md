# Run the lighter simulation scene

From the repository root:

```sh
./roscon_uk_2026/run_robot_demo.sh forward --gazebo-gui
```

The default `simple-house` scene keeps the walls, corridors, floor, original table and three objects. It removes the other furniture and generates a matching static navigation map. The command above opens both Gazebo and RViz.

The optional `--scene table` scene keeps the floor, robot, original table and three objects. It removes the house walls and other furniture. Physics remains at a 1 ms timestep, ros2_control at 100 Hz, RGB-D at 640×480 and 30 simulated Hz, and lasers at 10 Hz. The original scene is available with `--scene house`.

The scene option also selects an empty static navigation map for the table scene. Laser/depth observations provide its obstacles. The house scene retains its original map.

## Measurements on this laptop

Measured on 22 October 2026, with the same robot and Python admittance controller. RTF is elapsed ROS simulation time divided by elapsed wall time. RTF 0.9 means ten wall seconds advance approximately nine simulation seconds. These are short local measurements, not a guaranteed rate.

| Scene and setting | Windows | RTF |
| --- | --- | ---: |
| Full house, Intel GPU, 1 ms physics, camera 30 Hz | None | 0.67 |
| Full house, Intel GPU, 1 ms physics, camera reduced to 5 Hz | None | 0.69 |
| Full house, NVIDIA GPU, 1 ms physics, camera 30 Hz | None | 0.60 |
| Table scene, Intel GPU, 1 ms physics, camera 30 Hz | None | 0.91 |

The table scene was approximately 35% faster than the full-house baseline, with no timestep or sensor-rate change. The original house contributes 108 collision shapes and 104 visuals. The retained table contributes five collision shapes. The robot has 40 mesh, 12 cylinder, five box and four sphere collision shapes. Removing the house changes both collision and rendering work; this comparison does not isolate their individual costs.

A five-second `/proc` CPU sample put Gazebo's main thread at approximately 81% of one CPU core, with other individual threads below 7%. The simulation update loop is a substantial CPU cost. Having 16 CPU cores does not make this loop 16 times faster.

Ogre's log confirmed that the original renderer was accelerated Intel graphics, not software rendering. Explicit NVIDIA offload changed the renderer successfully but did not improve the measured full-house run. The [Gazebo hybrid-GPU documentation](https://github.com/gazebosim/docs/blob/master/harmonic/troubleshooting.md) describes those offload variables. No system-wide GPU or power settings were changed.

With RViz and Nav2 active in the table scene, a separate NVIDIA-rendered run measured 0.78 RTF over 15 seconds during navigation. A later Intel-rendered run with the navigation heading fix measured 0.66 RTF over 15 seconds while navigating. These runs have more work than the headless arm comparison above and do not form a controlled GPU comparison.

Increasing the physics step to 2 ms raised raw throughput, but the controller later deactivated and the arm-motion check failed. The shipped launcher retains the original 1 ms step. Changing the timestep during a run also disrupted the controller. These diagnostic samples are retained with qualifications in the evidence directory.

The table-scene arm check applied 2 N for eight wall seconds and then released. Measured hand displacement peaked at 20.0 mm and returned within 2.1 mm. The controller remained active and restored the stock controller on shutdown.

The table-scene navigation check reached its goal within 0.098 m and 0.119 rad. All 114 moving commands stayed within the 30-degree forward cone. This check exposed a heading oscillation when the adapter used individual grid-path segment tangents. The adapter now uses the direction from the robot to the look-ahead point. Source: [navigation evidence](evidence/performance/table_forward_fixed.json). The original house also passed after this change: 117 moving commands, zero outside the cone, and 0.098 m position error; see [house regression](evidence/performance/house_forward_fixed.json).

## Reproduce an RTF measurement

Start a demo and wait for its ready message. In another terminal, from the repository root:

```sh
cd roscon_uk_2026/next_steps/tiago_pro_jazzy
pixi run -e kit python ../robot_demos/measure_rtf.py --seconds 15
```

Compare the same demo, window configuration, sensor rates and robot motion. Initialization, compiling the controller, active navigation and dragging the hand can change the load. `--headless` starts the simulator without either GUI; `--gazebo-gui` adds Gazebo's window alongside RViz.

Raw measurements and conditions: [performance evidence](evidence/performance/conditions.json). The individual JSON files retain the actual wall and simulation intervals. Diagnostic samples that lost the active controller are flagged and excluded from the comparison table.

## Simplified house with both windows open

The simplified house retains 59 house/table collision shapes, compared with 108 in the furnished house. Physics remains at 1 ms and sensor rates are unchanged. Sensor bridges now subscribe to Gazebo only when ROS subscribers exist. Both Nav2 costmaps subscribed to the depth pointcloud during validation. Unused image streams therefore avoid conversion and publication. RViz laser, cloud and global-costmap displays start disabled using both configuration flags.

On 22 October 2026, the forward-navigation demo with Gazebo GUI and RViz open measured **0.926 RTF** over 15 seconds, followed by **0.850 RTF** over a second 15-second interval in the same session. These windows include different robot activity and are not an isolated measurement of furniture removal. The navigation check succeeded with 0.099 m position error; all 76 moving commands were inside the 30-degree forward cone.

Evidence: [first interval](evidence/performance/simple_house_gui.json), [second interval](evidence/performance/simple_house_gui_repeat.json), [navigation](evidence/performance/simple_house_forward.json).

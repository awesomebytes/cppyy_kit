# Record the table scene

These commands require this repository checkout and the prepared Jazzy environment.
Start from the repository root. The launch uses the default table-facing spawn
and native ready pose. It starts nine controllers without commanding joint motion.

```bash
mkdir -p roscon_uk_2026/next_steps/agent_session/data
cd roscon_uk_2026/next_steps/tiago_pro_jazzy
pixi run env ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  bash ../simulator_guard.sh -- ros2 launch "$PWD/playground.launch.py" \
  gui:=false rviz:=false moveit:=false navigation:=false \
  > ../agent_session/data/simulator.log 2>&1
```

Wait for `TIAGO_PRO_READY` in the log. The launch sets
`GZ_PARTITION=tiago_pro_jazzy`. In a second terminal at the repository root,
record a new output directory. The directory must not already exist.

```bash
timeout --signal=INT 15s pixi run \
  --manifest-path roscon_uk_2026/next_steps/tiago_pro_jazzy/pixi.toml \
  env ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET \
  ros2 bag record -s mcap --storage-preset-profile zstd_fast \
  -o roscon_uk_2026/next_steps/agent_session/data/table_scene \
  --topics /head_front_camera/color/image_raw \
  /head_front_camera/color/camera_info /head_front_camera/depth/color/points \
  /head_front_camera/depth/image_rect_raw /head_front_camera/depth/camera_info \
  /tf /tf_static /clock /joint_states /mobile_base_controller/odom \
  > roscon_uk_2026/next_steps/agent_session/data/recording.log 2>&1
```

Exit status 124 is expected from `timeout`; SIGINT closes the MCAP writer.
The captured bag contains 14.574 seconds of wall-clock data and 273 RGB frames and 273 cloud
frames. [The manifest](capture_manifest.json) records counts, frame IDs, camera
calibration, timestamps, source hashes and the actual simulator environment.
`data/bag_info.txt` contains the recorded bag summary.

The inspected `data/rgb_first.png` shows red, blue and yellow objects on the
table. The first cloud contains 147,131 finite points out of 307,200. Cloud XYZ
uses the native `head_front_camera_depth_frame`: X forward, Y left, Z up.
Image and CameraInfo use the depth optical frame. Bag record timestamps use
wall time; message headers and `/clock` use simulation time.

The recorder reported two transport message losses. The MCAP and extracted
local data remain under the repository's ignored `data/` directory.

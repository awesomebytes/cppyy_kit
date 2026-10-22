# Run perception with the robot task

Start the playground, then select an object without moving the robot.
These commands require this repository checkout. Run them at the repository root.

```bash
pixi run -e pick-place python roscon_uk_2026/next_steps/pick_place/demo.py \
  --deployment composed --observe
```

The expected result is a selected colored object with its measured centre and
bounds in `odom`. The image stage detects color regions throughout the scene.
The task projects the cloud clusters into the image to select a table object.
Image regions alone are not pick targets.

Use `--deployment distributed` to run the same native stages in three separate
processes. `--deployment external` uses stages started separately. Both forms
keep the task controller in the `demo.py` process. The composed form also keeps
all perception stages in that process.

## Run only perception

```bash
pixi run -e pick-place env ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python roscon_uk_2026/next_steps/pick_place/pipeline.py --duration 20 \
  --report /tmp/pick-place-pipeline.json
```

`--stage filter`, `--stage cluster` and `--stage image` select one stage. Run
all three commands for a separate-process deployment. `--prefix` changes the
output topic prefix. The default topics are:

| Stage | Input | Output |
|---|---|---|
| Filter | `/head_front_camera/depth/color/points` | `/pick_place/filtered_points` |
| Cluster | `/pick_place/filtered_points` | `/pick_place/objects` |
| Image | `/head_front_camera/color/image_raw` | `/pick_place/image_objects` |

The filter transforms native cloud XYZ fields at the sensor timestamp, removes
nonfinite points and computes PCL voxel centroids. It preserves the sensor stamp
and sets the frame to `odom`. It waits at most 100 ms of wall time for the exact
transform. A missing transform suppresses that frame. The cluster stage removes
the table and extracts object clusters. It disables a second voxel pass.
`--roi XMIN YMIN ZMIN XMAX YMAX ZMAX` sets its bounds in metres in the fixed
frame. The defaults cover the playground table.

The image stage wraps the native image buffer in an OpenCV matrix and runs the
same color-mask and silhouette code used by the recorded-frame example.
Both summaries use `std_msgs/msg/String` containing JSON. Geometry summaries
include the source stamp, frame, table height, object centres and bounds.

Each native stage holds at most one pending input and uses a ROS history depth
of one. A worker owns the current input while processing it. New inputs replace
the pending input when the worker is occupied. Large cloud data stays in native
ROS messages throughout the filter-to-cluster path. In one process this path
uses rclcpp intra-process delivery. This is not a claim of zero-copy processing:
frame transformation, PCL conversion and voxel filtering allocate or copy data.

## Embed the stages

With this directory on `sys.path`:

```python
from pipeline import PerceptionPipeline

with PerceptionPipeline(prefix="/pick_place") as perception:
    run_task_controller()
```

`build_pipeline(ros, args)` also accepts an existing managed native session and
returns its executor, executor thread, stage handles and node owners. Stage
handles expose `stats().to_dict()`. The session cancels and joins its executor,
then joins stage workers before releasing the TF listener and its buffer.
The TF buffer pointer remains valid until those workers stop. Restart the
pipeline after resetting simulator time.

`rclcpp_kit.native.create_fused_pipeline` now accepts `include_paths`,
`library_paths` and `libraries` for external C++ transforms. These arguments
participate in the compilation cache identity. `reliability="best_effort"`
selects sensor-compatible input QoS. Output publishers remain reliable.

## Measure both deployments

```bash
pixi run -e pick-place env ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  python roscon_uk_2026/next_steps/pick_place/benchmark.py --mode both \
  --warmup 3 --duration 15 --output /tmp/pick-place-benchmark
```

The runner reports compilation and import startup separately from each
process's ready interval. CPU is process CPU seconds divided by wall seconds.
The separate deployment reports the sum of its three process ratios. The
external observer is outside those CPU totals. Reports also include output
counts, native queue counters, exceptions and joined shutdown.

Wall latency starts when an external observer receives the source frame and
ends when it receives the matching summary. It excludes sensor generation and
includes observer scheduling. Only matched arrivals contribute samples.
Simulation frame age uses the source stamp and the observer's simulation clock.
It has different units from wall latency when the simulator runs below real time.

Live runs are sequential and can receive different input frames. Use `--bag
/path/to/recording` to replay the same bag for each deployment. The bag must
include camera topics, `/tf` and `/tf_static`. Run replay in a ROS domain without
another simulator or `/clock` publisher. Keep the robot stationary during a live
comparison and restart the benchmark if the simulator is reset.

The saved [live evidence](evidence/composition/live_summary.json) records three
cloud objects in both deployments and joined shutdown for every process. It
also records different image output counts between runs. These measurements do
not establish a processing-latency speedup from composition.

# rclcpp_kit (placeholder — arrives in M1b)

`rclcpp_kit` is the kit **for rclcpp** (ROS 2 core), following the same naming
rule as every other kit. Its body is carved out of the
[rclcppyy](https://github.com/awesomebytes/rclcppyy) product in **M1b**:

- rclcpp bringup (`bringup_rclcpp`, `add_ros2_include_paths`, executor/node helpers)
- C++ message resolution/conversion + serialization
- rosbag2_cpp (+ the rosbag2_py compat shim)
- **tf** (`rclcppyy.tf` moves here)
- the rclcpp PCH freeze recipe

## What is here now

The Python package (`rclcpp_kit/`) is an empty placeholder. The tf material that
will live here is already staged so it moves with its future home:

- `REPORT.md` — the tf evidence (tf2 C++ transform stack via cppyy).
- `demos/` — tf demos (`d01_lookup_example`, `tf_storm_publisher`, benches).
- `tests/test_tf.py` — the tf test suite.

**M1b-temporary bridge.** These demos/tests still import the rclcppyy product
(`import rclcppyy`, `from rclcppyy.bringup_rclcpp import …`, `from rclcppyy import tf`),
which is provided in the relevant pixi envs by the `ros-jazzy-rclcppyy` conda
package (the `rclcpp` feature). When M1b lands the real `rclcpp_kit`, these
imports switch to `rclcpp_kit` and the bridge feature is removed.

The tf test is deliberately **excluded from the default `pixi run test`**: tf2
headers are present in the default ROS env, so the test would try to import the
rclcppyy bridge (absent from the default env) rather than skip. Run it with
`pixi run -e rclcpp test-tf`.

**tf vs. the released bridge.** `tf` was added to rclcppyy *after* its 0.1.0
release, so the `ros-jazzy-rclcppyy` conda package that provides the bridge does
**not** ship `rclcppyy.tf`. On that bridge, `test-tf` / `demo-tf-*` **skip
cleanly** (the guard probes the installed package for `tf.py`). They run once tf
is available — i.e. when M1b carves the real `rclcpp_kit` (tf then lives here, no
bridge needed) or a post-0.1.0 rclcppyy is installed. Every *other* kit only
needs `bringup_rclcpp` / `add_ros2_include_paths`, which the 0.1.0 bridge does
provide (nav2 / control / moveit suites pass against it).

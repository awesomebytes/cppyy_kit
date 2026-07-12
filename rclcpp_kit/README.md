# rclcpp_kit

`rclcpp_kit` is the kit **for rclcpp** (ROS 2 core), following the same naming
rule as every other kit. It is the rclcpp core **capability layer** that every
ROS-touching kit — and the [rclcppyy](https://github.com/awesomebytes/rclcppyy)
product — builds on. It was carved out of rclcppyy's core **with git history**
(`git log --follow` traces any module back into rclcppyy).

It sits between the ROS-free [`cppyy_kit`](../cppyy_kit) base (load_libraries /
keep_alive / register_teardown / pretty_cpp_error) and the domain kits.

## What's here

| Module | Surface |
|---|---|
| `bringup_rclcpp` | `bringup_rclcpp()` (JIT `rclcpp/rclcpp.hpp` + load core libs), `add_ros2_include_paths()`, `shutdown_rclcpp()`, the rclpy-style `rclcpp.Node` adapters (create_publisher / create_subscription / create_timer / destroy_node), C++ message resolution + the shared recursive `convert_python_msg_to_cpp` |
| `serialization` | CDR serialize/deserialize of C++ messages, byte-compatible with `rclpy.serialization`; bytes ⇄ `rclcpp::SerializedMessage` |
| `rosbag2_cpp` | the C++ `rosbag2_cpp` reader/writer (open_reader / open_writer / iterate) |
| `rosbag2_py_compat` | a `rosbag2_py`-compatible shim (SequentialReader/Writer, StorageOptions, …) backed by `rosbag2_cpp` |
| `tf` | the tf2 C++ transform stack: a `tf2_ros::TransformListener` ingesting `/tf` wholly in C++ on its own thread (`TransformListener.lookup_transform` / `can_transform` / `set_transform`) |

```python
import rclcpp_kit
rclcpp = rclcpp_kit.bringup_rclcpp()             # rclcpp up under cppyy

from rclcpp_kit import tf
listener = tf.TransformListener()                # own node + own C++ spin thread
ts = listener.lookup_transform("world", "sensor", timeout=1.0)
x = ts.transform.translation.x                   # the real geometry_msgs message

from rclcpp_kit import serialization as ser
blob = ser.serialized_message_to_bytes(ser.serialize_message(cpp_msg))
```

The surface mirrors the names the rclcppyy product exposed, so rclcppyy is slimmed
to thin re-export shims over this package and stays a drop-in rclpy accelerator.

## Running it

`rclcpp_kit` needs only the ROS core (rclcpp + tf2, both in the default `ros-base`
env); it carries no opt-in C++ dependency of its own. Its env is `rclcpp`:

```bash
pixi run -e rclcpp test-rclcpp     # full suite: bringup, pub/sub, serialization, tf
pixi run -e rclcpp test-tf         # the tf gate (8 tests)
pixi run -e rclcpp demo-tf-lookup  # C++ listener ingests /tf; Python looks it up
pixi run -e rclcpp demo-tf-storm   # synthetic TF storm publisher
pixi run -e rclcpp bench-tf        # stock rclpy listener vs rclcpp_kit C++ listener
```

Unlike the domain kits, `rclcpp_kit`'s tests genuinely bring up rclcpp + DDS (they
do not auto-skip in the default env), so they run in the `rclcpp` env rather than
the default `pixi run test` collect-and-skip smoke.

## Docs

- [`SKILL.md`](SKILL.md) — LLM-facing: when to use, copy-paste patterns, gotchas.
- [`WHY.md`](WHY.md) — the pitch (why drive rclcpp/tf from Python via cppyy).
- [`REPORT.md`](REPORT.md) — the tf spike evidence (mechanism + benchmark).

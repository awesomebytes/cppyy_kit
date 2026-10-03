# Why rclcpp_kit

Run ROS 2 C++ libraries such as rclcpp, tf2, rosbag2, and CDR serialization from
Python. Selected message work can then run in C++, outside the GIL, while Python
handles orchestration. Domain kits and the rclcppyy accelerator use
`rclcpp_kit` for ROS support.

## Python work in the default path

The stock rclpy path uses Python for these operations:

- **TF ingest uses Python.** `tf2_ros`' Python `TransformListener`
  subscribes to `/tf` with a **Python** callback, so every `TFMessage` is
  deserialized into Python objects and fed **one transform at a time** across the
  Python-to-C boundary into the buffer. This runs on a Python thread that holds
  the GIL.
- **Every publish/subscribe** crosses a Python message object; **every**
  `lookup_transform` builds a fresh Python message out.

`rclcpp_kit` uses tf2's **C++** `TransformListener` to ingest `/tf` on a dedicated
thread. Publishers and subscribers pass C++ messages, and serialization uses
rclcpp's CDR implementation.

## TF characterization

The following raw values came from one pass per variant on a shared development
machine. The host, software versions, and method are listed in [REPORT.md](REPORT.md).
Use the reproduction command there to measure the target host and workload.

| scenario | ingest CPU% py / cpp | lookup µs median py / cpp | observed py/cpp ratio |
|---|---|---|---|
| idle (no storm) | 0.0 / 0.0 | 7.5 / 1.4 | lookup 5.4× |
| 1 k tf/s | 4.0 / 0.6 | 7.0 / 1.4 | ingest 6.7×; lookup 5.0× |
| 10 k tf/s | 19.3 / 1.4 | 13.5 / 4.5 | ingest 14×; lookup 3.0× |

The C++ listener decodes and inserts transforms in C++. The Python path crosses the
language boundary for each transform while holding the GIL. This difference does not
depend on the timing measurements. The measurements are consistent with a possible
benefit for busy trees and frequent lookups. Repeat the measurements with the target
host, TF rates, and tree sizes to estimate the effect for that workload.

## Features and limits

- `lookup_transform` returns the real
  `geometry_msgs::msg::TransformStamped`; a subscription callback gets the real C++
  message. You can use the C++ API from Python through cppyy.
- **Byte-for-byte serialization parity** with `rclpy.serialization` (tested), so
  bags and wire bytes interoperate.
- At interpreter exit, `cppyy_kit` releases the rclcpp context and DDS layer in
  order. The process exits normally without `os._exit`.
- In the idle case, both variants recorded 0.0% CPU. Measure representative
  workloads repeatedly before choosing a path for performance.

For copy-paste patterns see [SKILL.md](SKILL.md); for the base primitives it builds
on, [`cppyy_kit`](../docs/COMMON_PATTERNS.md).

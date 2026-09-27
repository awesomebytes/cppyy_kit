# Why rclcpp_kit

**The one-liner:** run ROS 2's *C++* core — rclcpp, tf2, rosbag2, CDR
serialization — from Python, so selected per-message work happens in C++
(off the GIL) while your orchestration stays in short Python. `rclcpp_kit` is the
capability layer that makes that ergonomic; every ROS-touching kit (and the
rclcppyy drop-in accelerator) is built on it.

## The problem it removes

The stock rclpy path pays Python for work that is fundamentally C++:

- **TF ingest is entirely Python.** `tf2_ros`' Python `TransformListener`
  subscribes to `/tf` with a **Python** callback, so every `TFMessage` is
  deserialized into Python objects and fed **one transform at a time** across the
  Python→C boundary into the buffer — all on a Python thread holding the GIL.
- **Every publish/subscribe** crosses a Python message object; **every**
  `lookup_transform` builds a fresh Python message out.

`rclcpp_kit` runs the real C++ machinery instead: the tf2 **C++**
`TransformListener` ingests `/tf` wholly in C++ on its own dedicated thread;
publishers/subscribers move the C++ message; serialization is rclcpp's own CDR.

## TF characterization

The following raw values came from one pass per variant on a shared development
machine. They characterize that run only: they are not a portable performance
claim, regression budget, or declaration of a winner. The full environment,
method, reproduction command, and limitations are in [REPORT.md](REPORT.md).

| scenario | ingest CPU% py / cpp | lookup µs median py / cpp | observed py/cpp ratio |
|---|---|---|---|
| idle (no storm) | 0.0 / 0.0 | 7.5 / 1.4 | lookup 5.4× |
| 1 k tf/s | 4.0 / 0.6 | 7.0 / 1.4 | ingest 6.7×; lookup 5.0× |
| 10 k tf/s | 19.3 / 1.4 | 13.5 / 4.5 | ingest 14×; lookup 3.0× |

The C++ listener decodes and inserts wholly in C++, while the Python path crosses
each transform under the GIL. That mechanism is verified independently of timing.
The raw separation above is consistent with the hypothesis that this matters for
busy trees and frequent lookups, but this single shared-host pass does not establish
the size, stability, or portability of an improvement.

## What you get, and the honest boundary

- **Mirror-don't-sugar.** `lookup_transform` returns the real
  `geometry_msgs::msg::TransformStamped`; a subscription callback gets the real C++
  message. You use the C++ API, minus the cppyy friction.
- **Byte-for-byte serialization parity** with `rclpy.serialization` (tested), so
  bags and wire bytes interoperate.
- **Clean teardown** — the rclcpp context and DDS layer are released in a defined
  order at exit (via `cppyy_kit`'s ordered teardown), no `os._exit` hacks.
- **Where the raw run was close in absolute CPU:** the idle row recorded 0.0% for
  both variants. Workload-specific repeated measurement is required before choosing
  a path for performance.

For copy-paste patterns see [SKILL.md](SKILL.md); for the base primitives it builds
on, [`cppyy_kit`](../kits/cppyy_kit.md).

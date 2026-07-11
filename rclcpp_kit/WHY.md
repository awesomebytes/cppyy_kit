# Why rclcpp_kit

**The one-liner:** run ROS 2's *C++* core — rclcpp, tf2, rosbag2, CDR
serialization — from Python, so the expensive per-message work happens in C++
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

## The evidence (TF, measured)

Same synthetic TF storm, one variant at a time (full method + table in
[REPORT.md](REPORT.md)):

| scenario | ingest CPU% py / cpp | lookup µs med py / cpp |
|---|---|---|
| idle (no storm) | 0.0 / 0.0 | 7.5 / 1.4  (5.4×) |
| 1 k tf/s | 4.0 / 0.6  (6.7×) | 7.0 / 1.4 |
| 10 k tf/s | 19.3 / 1.4  (**14×**) | 13.5 / 4.5  (3×) |

**Ingest is the headline and the win grows with load** — ~7× at 1 k tf/s, ~14× at
10 k — because the C++ listener decodes and inserts wholly in C++ while the Python
one crosses each transform under the GIL. Lookups are ~5× cheaper too, even idle.
The *math* is identical (both call the same `tf2::BufferCore`), so the win shows up
precisely where TF cost shows up in a profile: busy trees, frequent lookups.

## What you get, and the honest boundary

- **Mirror-don't-sugar.** `lookup_transform` returns the real
  `geometry_msgs::msg::TransformStamped`; a subscription callback gets the real C++
  message. You use the C++ API, minus the cppyy friction.
- **Byte-for-byte serialization parity** with `rclpy.serialization` (tested), so
  bags and wire bytes interoperate.
- **Clean teardown** — the rclcpp context and DDS layer are released in a defined
  order at exit (via `cppyy_kit`'s ordered teardown), no `os._exit` hacks.
- **Where it's marginal:** a quiet tf tree with occasional lookups is sub-1% CPU
  either way. This is an efficiency layer for the hot paths, not a free rewrite.

For copy-paste patterns see [SKILL.md](SKILL.md); for the base primitives it builds
on, [`cppyy_kit`](../cppyy_kit).

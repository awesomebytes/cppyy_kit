# rclcpp_kit — cheat sheet for a coding agent

You are writing Python that drives **ROS 2 core (rclcpp + tf2 + rosbag2)** through
`rclcpp_kit`, via cppyy. The kit **mirrors the C++ API** and hides only the cppyy
friction (bringup, symbol resolution, message conversion, ordered teardown). It is
the capability layer every ROS-touching kit builds on; it is **not** the rclcppyy
drop-in accelerator (that's the separate `rclcppyy` product, which re-exports this).

(For *why* this exists and the measured TF numbers, see [WHY.md](WHY.md) /
[REPORT.md](REPORT.md).)

**Requires** the ROS core, present in the default `ros-base` env. Its own env is
`rclcpp`: `pixi run -e rclcpp python your_script.py`.

**Golden rules**
- Call `rclcpp = rclcpp_kit.bringup_rclcpp()` once; it returns the real `rclcpp`
  namespace and is idempotent. The first call JITs `rclcpp/rclcpp.hpp` (a few
  seconds) — do it at startup, not on a hot path.
- A plain `rclcpp.Node` accepts **both** calling conventions: rclpy-style
  (`node.create_publisher(String, "topic", 10)`, Python messages auto-converted to
  C++) and native rclcpp template syntax (`node.create_publisher[CppMsgT]("topic", 10)`,
  zero-overhead). Same for `create_subscription` / `create_timer`.
- Message classes: hand either a Python message class (`std_msgs.msg.String`) or a
  cppyy C++ class (`cppyy.gbl.std_msgs.msg.String`) — the kit resolves both.
- Let teardown happen: `cppyy_kit.shutdown()` runs at interpreter exit and releases
  the rclcpp context in order (no `os._exit` needed). A normal `return`/`sys.exit`
  is clean.

---

## Pattern 1 — bring up rclcpp, publish/subscribe the rclpy way
*Use for:* any node that needs the C++ backend with familiar rclpy calls.

```python
import rclcpp_kit
from std_msgs.msg import String

rclcpp = rclcpp_kit.bringup_rclcpp()
node = rclcpp.Node("demo")
pub = node.create_publisher(String, "chatter", 10)       # Python msg auto-converts

def on_msg(msg):                                         # msg is the C++ message
    print(msg.data)
sub = node.create_subscription(String, "chatter", on_msg, 10)

msg = String(); msg.data = "hi"
pub.publish(msg)                                         # converted to C++ then sent
rclcpp.spin_some(node)
```
Gotcha: the callback receives the **C++** message proxy (read `.data` directly). The
Python callable is auto-pinned (via `cppyy_kit.keep_alive`) so it is not collected.

## Pattern 2 — tf2 transforms, ingested entirely in C++
*Use for:* looking up transforms without the stock rclpy listener's per-message
Python cost. The C++ `tf2_ros::TransformListener` ingests `/tf` on its own thread.

```python
import rclcpp_kit
from rclcpp_kit import tf

rclcpp_kit.bringup_rclcpp()
listener = tf.TransformListener()                        # own node + own C++ thread
# ... transforms arrive on /tf ...
ts = listener.lookup_transform("world", "sensor", timeout=1.0)
x, y = ts.transform.translation.x, ts.transform.translation.y
ok = listener.can_transform("world", "sensor")
listener.set_transform(a_transform_stamped, is_static=True)   # seed directly
```
`time=` accepts `None` (latest) / seconds / an rclpy·rclcpp `Time`. Missing frames or
a timeout raise `tf.TransformException`. `get_frame_names()` returns `str`s.

## Pattern 3 — CDR serialization, byte-compatible with rclpy
*Use for:* wire bytes / bag round-trips.

```python
from rclcpp_kit import serialization as ser
from std_msgs.msg import String

_, Cpp = ser.cpp_message_type_from_python(String)
m = Cpp(); m.data = "payload"
blob = ser.serialized_message_to_bytes(ser.serialize_message(m))   # == rclpy bytes
back = ser.deserialize_message(ser.serialized_message_from_bytes(blob), String)
```

## Pattern 4 — rosbag2 from Python (C++ reader/writer)
*Use for:* reading/writing bags with the C++ `rosbag2_cpp` stack, or as a
`rosbag2_py` drop-in.

```python
from rclcpp_kit import rosbag2_cpp
reader = rosbag2_cpp.open_reader("/path/to/bag", storage_id="mcap")
for md in rosbag2_cpp.iter_topics(reader):
    print(md.name, md.type)
for sbm in rosbag2_cpp.iter_messages(reader):            # C++ SerializedBagMessage
    ...

from rclcpp_kit import rosbag2_py_compat as rosbag2_py    # rosbag2_py-shaped API
```

---

## Gotchas (the cppyy friction this kit hides, so you know the boundary)
- **Bringup is a header-parse cost, once.** `bringup_rclcpp()` JITs the rclcpp
  headers on the first call; subsequent calls are no-ops. Freeze (PCH) removes the
  parse, not the per-signature JIT — see `docs/FREEZE.md`.
- **`tf2_ros::Buffer` is deliberately avoided.** Its overloaded lookup/canTransform
  mis-resolve under cppyy and crash; the kit uses the plain `tf2::BufferCore` with
  unambiguous `cppdef` accessors. Use `tf.TransformListener`, not raw `tf2_ros::Buffer`.
- **Objects that own C++ threads/executors must be released before shutdown.**
  `tf.TransformListener.close()` (also auto-registered) drops the listener before
  `rclcpp::shutdown()`; don't hold one past teardown.
- **Symbols resolve by soname at call time.** If you reach past the kit into another
  ROS library, `cppyy_kit.load_libraries([...])` it first (see cppyy_kit's SKILL).

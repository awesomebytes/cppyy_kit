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

## Pattern 5 — typed async clients with C++-owned futures
*Use for:* calling a stock ROS service while keeping `rclcpp` template and future
ownership out of Python.

```python
import time

from rclcpp_kit.native import native
from std_srvs.srv import SetBool

with native(["client"]) as ros:
    node = ros.create_node("client")
    executor = ros.create_executor()
    executor.add_node(node)
    ros.start_executor(executor)

    group = ros.create_callback_group(node, "reentrant")
    client = ros.create_native_client(
        node, SetBool, "set_bool", callback_group=group)
    assert client.wait_for_service(1.0)
    request = client.make_request()
    request.data = True
    token = client.send(request)
    while not client.ready(token):
        time.sleep(0.001)
    response = client.take(token)  # real C++ Response
```

`client.raw_client` is the original typed `rclcpp::Client<ServiceT>`. Calls sent
through the adapter must also be taken or canceled through it. The session cancels
outstanding calls during ordered teardown. `send()` accepts only the owning C++
request returned by `make_request()`; use `send_cpp_value()` for an existing
generated C++ request value. Generated Python requests are never converted. For
raw publisher/subscription creation,
use `ros.create_publisher_options(group)` and
`ros.create_subscription_options(group)` because cppyy cannot assign the shared
callback-group member directly.

## Pattern 6 — real lifecycle nodes from Python
*Use for:* lifecycle publishers, transitions, and other
`rclcpp_lifecycle::LifecycleNode` facilities without a custom binding package.

```python
from rclcpp_kit.native import native

with native(["lifecycle"]) as ros:
    lifecycle = ros.create_native_lifecycle_node("worker")
    executor = ros.create_executor()
    lifecycle.attach_executor(executor)
    ros.start_executor(executor)

    raw_node = lifecycle.raw_node  # real rclcpp_lifecycle shared pointer
    assert str(raw_node.get_current_state().label()) == "unconfigured"
```

The standard lifecycle services are enabled by default and become responsive after
executor attachment. Use `raw_node` for the actual lifecycle API; the adapter owns
only construction, executor membership, and teardown.

## Pattern 7 — typed action clients with C++-owned goal state
*Use for:* action goals, feedback, results, and cancellation when the typed
`rclcpp_action` state should remain in C++.

```python
import time

from tf2_msgs.action import LookupTransform

client = ros.create_native_action_client(
    node, LookupTransform, "lookup", feedback_capacity=16)
assert client.wait_for_server(2.0)
goal = client.make_goal()
goal.target_frame = "map"
goal.source_frame = "base"
token = client.send_goal(goal)
while not client.goal_response_ready(token):
    time.sleep(0.001)
if client.goal_accepted(token):
    while not client.result_ready(token):
        time.sleep(0.001)
    result = client.take_result(token)
```

Use `request_cancel(token)` for remote cancellation and `forget(token)` only to
release local state. The feedback queue is bounded and drop-oldest; inspect
`client.stats()` rather than assuming every feedback sample was retained.
`send_goal()` accepts only the owning C++ goal returned by `make_goal()`; use
`send_cpp_value()` for an existing generated C++ goal value. Generated Python
goals are never converted.

## Pattern 8 — load registered AOT components
*Use for:* standard ROS composition from Python while the container, loaded nodes,
context, and executor remain C++ owned.

```python
from rclcpp_kit.native import native

with native(["container"]) as ros:
    executor = ros.create_executor("multi_threaded", threads=2)
    container = ros.create_native_component_manager(
        executor, name="native_container")
    ros.start_executor(executor)

    # Use ordinary composition_interfaces LoadNode/ListNodes/UnloadNode clients.
    raw_manager = container.raw_manager
```

Only AOT C++ plugins registered in the ament resource index are loadable. Install
each component package separately; the adapter preserves the stock composition
service protocol rather than mirroring it.

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
- **A managed client is asynchronous.** Its executor must spin before `ready(token)`
  can become true. `take(token)` is single-use and rejects an unready token; call
  `cancel(token)` when abandoning work so `rclcpp` pending state is released.
- **Cold service/client coexistence needs a compiler or a warm artifact.** The kit
  compiles glue first and loads declarations into Cling, avoiding conflicting
  `std::call_once` TLS state. Without a runtime compiler, one adapter retains its
  Cling fallback, but multiple cold glue facilities in one interpreter are not a
  guaranteed combination.
- **Raw lifecycle access transfers lifetime responsibility.** Retaining
  `lifecycle.raw_node` keeps its shared C++ node alive after adapter close. Drop raw
  owners before session teardown when deterministic destruction matters.
- **Managed actions are client-only and polling-oriented.** Adapter-submitted
  goals should be completed or forgotten through the adapter. First use of a type
  performs a synchronous content-addressed AOT glue build; raw generated `uint8`
  fields can appear as one-character strings through cppyy.
- **Component close is not a concurrent composition operation.** Stop or quiesce
  load/unload requests before explicit close. Ordered session teardown already
  stops native executor threads before releasing the container and loaded nodes.
- **Symbols resolve by soname at call time.** If you reach past the kit into another
  ROS library, `cppyy_kit.load_libraries([...])` it first (see cppyy_kit's SKILL).

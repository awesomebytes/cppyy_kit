# TF through rclcpp_kit: comparison with the Python implementation

**Date:** 2026-07-11. **Environment:** pixi `default` (robostack-jazzy `ros-base` + conda-forge),
`cppyy 3.5.0`, Python 3.12.13, `tf2`/`tf2_ros` 0.36.x, cyclonedds, linux-64.
`ROS_DOMAIN_ID=51`. Each variant ran once on a shared machine. A parallel vision
job used domain 52 during measurement. The table records the results from that setup.
Repeated runs on a dedicated machine are needed to estimate results for a target
workload.

**Hypothesis:** The C++ `TransformListener`, called through cppyy, will use less
CPU for TF ingestion than the Python implementation.

**Result:** The C++ listener ingested and decoded transforms on its own thread, and
the numeric results matched the expected values. In this shared-host run, the raw
Python/C++ CPU ratios ranged from 6.7 to 14 during the TF storm. Idle lookup medians
were 7.5 µs and 1.4 µs. The helper is available as `rclcpp_kit.tf`; it returns the
`geometry_msgs::msg::TransformStamped`. The `rclcpp` environment has 8 tests, which
took about 4 seconds in this run and include a network-ingest test. Demos and the
benchmark are available as pixi tasks.

> The TF code was first part of the rclcppyy product as `rclcppyy.tf`. It was moved
> to `rclcpp_kit` with its bringup, serialization, and rosbag2 modules. The mechanism,
> measurements, and analysis below are unchanged. The import path is now
> `rclcpp_kit.tf`. Run it with
> `pixi run -e rclcpp test-tf`.

---

## TF ingestion in the Python implementation

The stock **Python** listener/buffer path, cited to the site-packages sources:

**`tf2_ros/transform_listener.py`:** `TransformListener.__init__` creates two **rclpy**
subscriptions, on `/tf` (QoS depth 100, volatile) and `/tf_static` (depth 100,
transient-local), whose callbacks are **Python methods** `self.callback` /
`self.static_callback` (lines 85-88). With `spin_thread=True`, it spins its own
`SingleThreadedExecutor` in a Python `threading.Thread` (lines 90-99). The callback
(lines 114-118) processes each message:

```python
def callback(self, data: TFMessage) -> None:
    who = 'default_authority'
    for transform in data.transforms:          # Python for-loop
        self.buffer.set_transform(transform, who)   # one Python->C crossing each
```

By the time this runs, rclpy has already **deserialized the whole `TFMessage` into
Python objects:** a Python list of Python `TransformStamped`, each containing Python
`Header`/`Transform`/`Vector3`/`Quaternion`. Then a **Python loop** hands each
transform, **one at a time**, into the buffer.

**`tf2_ros/buffer.py`:** `Buffer` subclasses `tf2_py.BufferCore` (a C extension,
`tf2_py/_tf2_py.so`, wrapping the C++ `tf2::BufferCore`) and `BufferInterface`
(line 56). `set_transform` (lines 111-117) calls `super().set_transform(...)`.
This passes each Python `TransformStamped` to the C extension, which converts it to a
C++ `geometry_msgs::msg::TransformStamped` and inserts it. The method then runs
`_call_new_data_callbacks()` (a Python `RLock` + list iterate) **on every insert**.
`lookup_transform` (lines 133-150) calls `can_transform` (which, with the default
zero timeout, immediately calls the C-extension `can_transform_core`) and then
`lookup_transform_core` (C extension); the TF math is C++, but every call marshals
the string/`Time`/`Duration` args in and **builds a fresh Python `TransformStamped`
out**.

**So per `/tf` message with N transforms the Python path pays, all on a Python thread
holding the GIL:** (1) full rclpy deserialization into Python message objects,
(2) a Python for-loop, (3) N Python→C `set_transform` calls each re-converting a
Python message to C++, (4) N `_call_new_data_callbacks` iterations. The actual cache
and interpolation are C++ (in `tf2_py`), but *feeding* the cache is entirely Python.

**The C++ path (`tf2_ros/transform_listener.hpp`)** subscribes with a **C++** callback
`subscription_callback(TFMessage::ConstSharedPtr, is_static)`; rclcpp delivers a **C++**
`TFMessage`, the callback iterates and calls `buffer_.setTransform(...)` in C++. With
`spin_thread=true` it builds its **own `SingleThreadedExecutor` on a dedicated
`std::thread`** and sets `buffer_.setUsingDedicatedThread(true)`. Ingest is therefore
wholly in C++, off the GIL; Python only crosses the boundary when it calls
`lookup_transform`. The benchmark measures this difference.

---

## TF through rclcppyy

First, check the constructor signatures, as described in nav2 REPORT §2.
`tf2_ros::TransformListener` has three constructors: `(tf2::BufferCore&,
bool spin_thread=true)` (makes its own node), `(BufferCore&, NodeT&& node, ...)`
templated on the node, and a node-interfaces form. It takes a
**`tf2::BufferCore&`**, so Python can construct it without a lifecycle node or a
pluginlib base. `tf2::BufferCore` contains the cache and transform operations; it
does not own a node.

The following checks were completed (scratch probes; see the commit's scratch history):

| # | Capability | Result | Evidence |
|---|---|:--:|---|
| A | **Bringup + JIT**: include `tf2/buffer_core.hpp` + `tf2_ros/transform_listener.h` + TFMessage, load `libtf2`/`libtf2_ros` | **WORKS** | headers JIT-parse in **~0.05 s + ~0.10 s** on top of the rclcpp include (bringup_rclcpp ~1.8 s) |
| B | **Numeric correctness** (plain `BufferCore`, no net): set a known 2-hop tree, `lookupTransform` | **WORKS** | `world<-sensor` returns exactly `(1, 2, 0)`; `canTransform` True/False correct; `allFramesAsString` reads the tree |
| C | **Network ingest by the C++ listener** (no Python per-message crossing) | **WORKS** | publish `/tf` via rclcppyy → C++ `TransformListener` (own thread) ingests → Python `lookup` returns the composed transform; asserted numerically |
| D | **canTransform / timeouts** | **WORKS** | a C++ `can_wait` poll helper (steady-clock deadline, the listener's dedicated thread keeps ingesting) returns on availability or times out; missing-frame lookups raise a clean `TransformException` |
| E | **Clean teardown** | **WORKS** | releasing the listener (dtor cancels its executor + joins its thread) before `rclcpp::shutdown()` via `register_teardown` → exit 0 |

**Two cppyy binding issues required workarounds:**

1. **`tf2_ros::Buffer` mis-resolves under cppyy and crashes.** Its `lookupTransform`/
   `canTransform` are heavily overloaded (a `tf2::TimePoint` form via `using`, plus
   `rclcpp::Time`+`Duration` timeout forms). A 3-arg `lookupTransform(target, source,
   TimePointZero)` from Python resolved into the **timeout-path `canTransform`**, which
   calls `rclcpp::Clock::now()` and **bus-errors** (confirmed by the fault backtrace:
   `Clock::now()` ← `tf2_ros::Buffer::canTransform(...)`). Also its ctor is a template
   with a universal-reference default (`NodeT&& node = NodeT()`) which cppyy rejects
   ("class has no public constructors"). **Fix:** use the plain `tf2::BufferCore` (the
   listener accepts `BufferCore&` directly) whose single `TimePoint` overloads resolve
   cleanly, and route lookups through **unambiguous `cppdef` free functions**.
2. **Build the listener in a `cppdef` factory.** `make_shared<TransformListener>(buf,
   node, spin)` compiles in C++ but does not resolve from Python. As in control_kit
   and nav2, a C++ factory constructs the object. The glue contains a one-line factory.

The kit is small: `rclcppyy/tf.py` is **~120 lines of Python + ~35 lines of embedded
C++ glue** (a factory, a `TimePoint`-from-nanos converter, and `can`/`lookup`/
`can_wait` accessors).

---

## Benchmark method and results

The benchmark uses the same synthetic TF storm: a chain
`world -> link_0 -> ... -> link_{N-1}` is published as one `TFMessage` at rate R.
The aggregate load is N times R transforms/s. It runs one variant at a time on the
same machine (`scripts/tf_demos/bench_tf.py`; run it with `pixi run bench-tf`):

* **(a) py** = stock rclpy path: `tf2_ros.Buffer` + `tf2_ros.TransformListener`.
* **(b) cpp** = rclcppyy: `tf2::BufferCore` + C++ `tf2_ros::TransformListener`.
* **(c) idle** = no storm, lookup-only (isolates lookup-call overhead).

**ingest CPU%** = process-wide CPU (all threads, `time.process_time`) to keep the buffer
fed over a 3 s window with the main thread idle. **lookup** rows in the storm scenarios
are measured **under ingest load**. Each table row is one observation from this host.

| scenario | ingest CPU% py / cpp | lookup µs median py / cpp | lookups/s py / cpp | observed py/cpp ratios |
|---|---:|---:|---:|---:|
| idle (no storm) | 0.0 / 0.0 | 7.5 / 1.4 | 131 791 / 563 265 | lookup 5.4× |
| 1 k tf/s | 4.0 / 0.6 | 7.0 / 1.4 | 131 663 / 531 871 | ingest 6.7×; lookup 5.0× |
| 5 k tf/s | 12.1 / 1.1 | 9.4 / 2.5 | 93 330 / 326 391 | ingest 11×; lookup 3.8× |
| 10 k tf/s | 19.3 / 1.4 | 13.5 / 4.5 | 59 194 / 192 204 | ingest 14×; lookup 3.0× |

(For example, idle p99 lookup was 9.5 µs for Python and 1.6 µs for C++.)

**What the measurements show:**

- The raw ingest separation increased across the 1 k, 5 k, and 10 k tf/s rows. That
  is consistent with the verified ownership difference: the Python path deserializes
  and crosses each transform under the GIL, while the C++ path decodes and inserts in
  C++. One pass cannot quantify the effect or separate it from host interference.
- The raw lookup medians were lower for the C++ path in every row. The Python path
  performs two extension calls and builds a Python message, while the C++ path returns
  a cppyy proxy. Repeated controlled samples are still needed before attributing or
  generalizing the observed ratio.
- The idle ingest row recorded 0.0% for both paths. Absolute CPU differences and the
  usefulness of either path remain workload-specific.

---

## Implementation

The TF module is available as `rclcpp_kit.tf`. Domain kits (bt/pcl/ompl/nav2/moveit/
control/cv/dbow) wrap third-party or opt-in libraries in separate pixi feature
environments. tf2 is part of ROS 2 core and is included in the default `ros-base`
environment with rclcpp, so the module belongs in
`rclcpp_kit` alongside `bringup_rclcpp` / `serialization` / `rosbag2_cpp`, not behind
an opt-in env. Surface:

```python
import rclcpp_kit
from rclcpp_kit import tf

rclcpp_kit.bringup_rclcpp()
listener = tf.TransformListener()                 # own node + own C++ spin thread
# or tf.TransformListener(node=my_node)           # attach to an existing node
ts = listener.lookup_transform("world", "sensor", timeout=1.0)
x = ts.transform.translation.x                     # the real geometry_msgs message
ok = listener.can_transform("world", "sensor")
```

- `tf.bringup_tf()`: loads headers, libraries, and glue once; returns `(tf2, glue)`.
- `tf.TransformListener(node=None, *, spin_thread=True, cache_time_sec=None)`:
  `lookup_transform`, `can_transform` (both with an optional `timeout=`),
  `set_transform` (seed the buffer directly), `get_frame_names`, `all_frames_as_string`/
  `_yaml`, and `close`. `time=` accepts `None` (latest), seconds, or an rclpy or
  rclcpp `Time`.
- `tf.time_from_sec` / `tf.duration_from_sec`; `tf.TransformException`.
- `lookup_transform` returns the
  `geometry_msgs::msg::TransformStamped` (the same cppyy proxy the rest of rclcpp_kit
  uses); the raw `tf2` namespace is available for advanced use.

Demos (`rclcpp_kit/demos/`, pixi tasks in the `rclcpp` env): `demo-tf-lookup` (minimal
lookup example), `demo-tf-storm` (the storm publisher), `bench-tf` (the table above).
Tests (`rclcpp_kit/tests/test_tf.py`, 8 tests, `pixi run -e rclcpp test-tf`): numeric
composition, can/timeout, chain, time helpers, and a real network-ingest test.

**Not implemented:** `node.tf_listener()` on a node wrapper; a
`TransformBroadcaster` helper for publishing; `lookup_transform_full`
(fixed-frame/advanced API); `transform()` of a stamped datatype (needs
`tf2_geometry_msgs` converters).

---

## Possible additions to COMMON_PATTERNS

1. **Cppyy can select a wrong C++ overload that compiles but crashes at runtime
   (see §9/§17).** `tf2_ros::Buffer`'s
   `lookupTransform(target, source, TimePoint)` resolved into the `rclcpp::Time`+timeout
   `canTransform`, which called `rclcpp::Clock::now()` and raised a bus error. When a
   class has many overloads (a `using`-imported base form and timeout/clock forms),
   prefer the base class with the single unambiguous signature (here `tf2::BufferCore`),
   or wrap the exact call in a `cppdef` free function. A wrong-overload crash has no
   Python traceback. Test overload resolution directly.
2. **A template constructor with a universal-reference default (`NodeT&& node = NodeT()`)
   does not resolve from Python** ("class has no public constructors"). This is another case of
   "build the object in a small C++ factory" (§6 make_shared, control_kit, nav2). Add it
   to the make_shared bullet.
3. **A library that already spins its own C++ thread may suit cppyy use
   (see §13).** `tf2_ros::TransformListener(spin_thread=true)` ingests `/tf` on its
   own `std::thread`, without using the GIL; Python crosses on `lookup`. Measured
   raw Python/C++ ingest CPU ratios ranged from 6.7 to 14 in this shared-host pass.
   To estimate the effect on another workload, repeat the measurements on its target
   host and with representative TF rates and tree sizes.
4. **Teardown: a C++ object owning an executor + `std::thread` must be released before
   `rclcpp::shutdown()` (third instance of §14/§19).** `register_teardown` a callback
   that drops the listener (its dtor cancels the executor + joins the thread); it runs
   LIFO-before `shutdown_rclcpp`, the correct order. Exit 0 confirmed.
5. **`std::string` inside a returned `std::vector<std::string>` can appear as Python
   `bytes`, not `str` (see §11).** `getAllFrameNames()` returned a
   list of `bytes`; decode at the kit boundary.

---

## Conclusion

The ownership hypothesis is confirmed: the stock rclpy TransformListener feeds its
buffer through Python, while the C++ listener does that work on its own C++ thread.
The retained shared-host measurements showed Python/C++ ingest CPU ratios from
6.7 to 14 and lower raw lookup medians for the C++ path. To estimate the effect on
another workload, repeat the measurements on its target host. The module is
available as `rclcpp_kit.tf`; it provides a small wrapper around the C++ API, with
demos, a reproducible benchmark, and tests that include network ingestion. The
wrapper uses about 35 lines of C++ glue to handle overloaded methods and
construction from Python.

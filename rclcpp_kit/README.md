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
| `native` | Managed custom Context, real Node/NodeOptions, single- and multi-threaded executors, callback groups and callback-group entity options, intra-process selection, per-publisher loaning capability queries, deterministic shutdown, and the raw `rclcpp` namespace |
| `native_pipeline` | Content-addressed editable C++ subscription callbacks and fused subscription-transform-publisher objects; zero Python callback crossings, structured counters, explicit every/latest/bounded-batch delivery, and fresh/reused/loaned output memory |
| `type_adapter` | Value-only extension contract for domain-kit ROS/native conversions, including copy semantics, owner retention, alias mutability, and limitations |
| `native_service` | Content-addressed editable C++ service callbacks with stock and AOT-client interoperability, counters, zero Python request crossings, and managed teardown |
| `native_client` | Cached typed C++ clients with C++-owned async futures, stock and AOT-server interoperability, cancellation/counters, raw-client access, and managed teardown |
| `native_action` | Cached typed C++ action clients with C++-owned goal/result/cancel state, stock and AOT-server interoperability, bounded feedback, raw handles, counters, and managed teardown |
| `native_component` | Real `rclcpp_components::ComponentManager` containers on the managed context, stock composition services, AOT component loading, raw-manager access, and ordered teardown |
| `native_lifecycle` | Managed real `rclcpp_lifecycle::LifecycleNode` objects, executor attachment, stock and AOT-client lifecycle services, raw-node access, and ordered teardown |
| `serialization` | CDR serialize/deserialize of C++ messages, byte-compatible with `rclpy.serialization`; bytes ⇄ `rclcpp::SerializedMessage` |
| `rosbag2_cpp` | the C++ `rosbag2_cpp` reader/writer (open_reader / open_writer / iterate) |
| `rosbag2_py_compat` | a `rosbag2_py`-compatible shim (SequentialReader/Writer, StorageOptions, …) backed by `rosbag2_cpp` |
| `tf` | the tf2 C++ transform stack: a `tf2_ros::TransformListener` ingesting `/tf` wholly in C++ on its own thread (`TransformListener.lookup_transform` / `can_transform` / `set_transform`) |

Native lowering stays inside the managed ownership boundary:

```python
with rclcpp_kit.native(["pipeline"]) as ros:
    node = ros.create_node("pipeline")
    relay = ros.create_fused_pipeline(
        node, String, String, "input", "output",
        'output.data = input.data + ":native";',
        delivery="latest",
        output_memory="loaned",
    )
    stats = relay.stats()
    assert stats.python_boundary_crossings == 0
    assert stats.middleware_loaned_messages + stats.allocator_fallbacks \
        == stats.output_instances
```

`output_memory="fresh"` constructs an output for every transform and is the
default. `"reuse"` keeps one output behind a mutex, so the transform must fully
overwrite it and concurrent transforms are serialized. `"loaned"` uses
`rclcpp::LoanedMessage` RAII; the counters distinguish middleware loans from the
publisher allocator fallback. A loan is not by itself a zero-copy guarantee.

Managed clients keep only the template and future-lifetime friction behind a
small adapter. They accept ordinary generated Python requests or direct C++
requests, return opaque call tokens, and leave the original typed client exposed:

```python
client = ros.create_native_client(node, SetBool, "set_bool")
token = client.send(SetBool.Request(data=True))
if client.ready(token):
    response = client.take(token)  # the C++ response object
raw_client = client.raw_client
```

Calls submitted through the adapter must be taken or canceled through it. Session
teardown cancels any calls still pending before releasing the client.

Cold native service and client glue is compiled to a content-addressed DSO before
its declarations are loaded into Cling. This permits both facilities to coexist in
one interpreter without conflicting `std::call_once` TLS state. If no runtime
compiler is available, an individual adapter retains the original Cling fallback,
but cold same-interpreter multi-glue coexistence is not guaranteed; query
`ros.capabilities.native_service_client_coexistence`.

Managed action clients apply the same narrow rule to `rclcpp_action` template and
future state. They expose opaque goal tokens plus the original typed client and
accepted goal handles. Feedback is an explicit bounded drop-oldest queue, and
`forget(token)` releases local state without canceling the remote goal. The first
use of an action type synchronously builds content-addressed glue because this
toolchain cannot safely instantiate its type support through Cling alone.

Lifecycle nodes keep only ownership and executor attachment behind the adapter;
transitions and lifecycle-specific facilities remain the real C++ API:

```python
with rclcpp_kit.native(["lifecycle"]) as ros:
    lifecycle = ros.create_native_lifecycle_node("worker")
    executor = ros.create_executor()
    lifecycle.attach_executor(executor)
    thread = ros.start_executor(executor)
    raw_node = lifecycle.raw_node
```

The raw node is a shared-pointer escape hatch. Retaining it after `close()` extends
the C++ node lifetime by design, so deterministic managed teardown assumes callers
do not retain an extra raw owner.

Managed component containers load ordinary AOT C++ components registered in the
ament resource index and keep the standard `composition_interfaces` services:

```python
with rclcpp_kit.native(["container"]) as ros:
    executor = ros.create_executor("multi_threaded", threads=2)
    container = ros.create_native_component_manager(
        executor, name="native_container")
    ros.start_executor(executor)
    raw_manager = container.raw_manager
```

Component packages remain separate runtime dependencies. The adapter deliberately
does not turn Python classes into components, and explicit close must not race an
active load or unload request.

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
pixi run -e rclcpp bench-native-choices-smoke  # verified eight-case raw smoke
pixi run -e rclcpp bench-native-choices        # repeated raw characterization
```

Unlike the domain kits, `rclcpp_kit`'s tests genuinely bring up rclcpp + DDS (they
do not auto-skip in the default env), so they run in the `rclcpp` env rather than
the default `pixi run test` collect-and-skip smoke.

## Docs

- [`SKILL.md`](SKILL.md) — LLM-facing: when to use, copy-paste patterns, gotchas.
- [`WHY.md`](WHY.md) — the pitch (why drive rclcpp/tf from Python via cppyy).
- [`REPORT.md`](REPORT.md) — the tf spike evidence (mechanism + benchmark).
- [`NATIVE_CHOICES_BENCHMARK.md`](NATIVE_CHOICES_BENCHMARK.md) — correctness
  contract, reproduction commands, raw native-choice measurements, and limits.

# control_kit: writing a Python controller for controller_manager

**Date:** 2026-07-11 · **Env:** pixi `control` (robostack-jazzy + conda-forge),
`ros-jazzy-ros2-control` (controller_manager / controller_interface / hardware_interface),
`ros-jazzy-ros2-controllers`, `cppyy 3.5.0`, Python 3.12, linux-64. ROS_DOMAIN_ID=49.
**Question:** ros2_control does not define a Python controller API. A controller is a C++
class derived from `controller_interface::ControllerInterface`, exported via a pluginlib
`plugin_description.xml`, built by CMake/ament into a `.so`, and spawned into a
`controller_manager` process. Can we instead write the controller in **Python** and run
it **inside a real `controller_manager::ControllerManager`** in a single process, against
mock hardware through the `read`→`update`→`write` loop. This was listed as
unproven in the exposure note.

**Result:** All four stages passed. A `ControllerManager` is
constructed in-process from a URDF string with `mock_components/GenericSystem` hardware
(Stage 1); a **stock C++ controller** (`forward_command_controller`) is loaded via
pluginlib, configured, activated, and commanded through the mock hardware (Stage 2); and
**a PD controller written as a plain Python class that derives the real
`ControllerInterface`** is injected into that CM and driven by the real update loop,
tracking a moving reference (Stage 3). The Python controller's `update()` costs **2.57
µs/cycle** vs **0.85 µs** for the C++ baseline (3.0×) and holds **100 Hz solidly / 1 kHz
with occasional GC-induced deadline misses** (Stage 4). Route A uses cross-language
inheritance from the framework base class. Route B was not needed for the in-process test.
The ros2_control manager and controller-interface headers parsed in Cling. The
MoveIt headers tested in this comparison include generate_parameter_library output
and did not parse.

(See [WHY.md](WHY.md) for the stock ros2_control workflow and [SKILL.md](SKILL.md) for the API reference.)

---

## How the kit works

```mermaid
flowchart TD
    U["Your Python: a class deriving controller_interface::ControllerInterface,
       overriding on_init / *_interface_configuration / on_activate / update"]
    subgraph KIT["control_kit package: control_kit/"]
      B["bringup_control(): JIT-include controller_manager.hpp + controller_interface.hpp
         (parse in Cling); load the manager and pluginlib libraries; define C++ helpers"]
      R["make_controller_manager(urdf): real ControllerManager in-process w/ mock HW"]
      I["add_python_controller(): inject the Python instance via add_controller
         (no-op-deleter shared_ptr); read_state/write_command reach protected interfaces"]
      A["activate(): switch_controller on a std::thread while update() is pumped
         (switch blocks on the loop); ord()-decode uint8 return_type; ordered teardown"]
    end
    J["cppyy / Cling JIT"]
    E["libcontroller_manager.so + libhardware_interface.so + pluginlib-dlopen'd
       mock_components/GenericSystem + stock controllers: ros2_control C++"]
    U --> KIT --> J --> E
    E -. "cm.update() -> your Python update() every cycle" .-> U
```

`bringup_control()` includes the controller-manager and controller-interface headers and loads the shared libraries. Code then uses ros2_control classes **directly**
(`controller_manager::ControllerManager`, `controller_interface::ControllerInterface`).
The kit provides helpers for four operations (§2): constructing the manager, injecting
a Python subclass, accessing protected interfaces, and activating controllers from
another thread. The Python class derives from the framework base. The control loop
calls its `update()` method.
control_kit is **654 lines** (heavy docstrings per the mirror convention), of which **~94
are embedded C++ glue** (executor factory, threaded switch, interface accessors, the
injector).

---

## 1. Test stages

| # | Stage | Result | Evidence |
|---|---|:--:|---|
| 1 | **In-process ControllerManager** from a URDF string + `mock_components/GenericSystem`, RT loop spinning | **WORKS** | `ControllerManager(executor, urdf, activate_all=true, ...)` builds the ResourceManager internally; `is_resource_manager_initialized()==true`, `get_update_rate()==100`; mock system `configure`+`activate`d; `read`/`update`/`write` loop ran, exit 0. |
| 2 | **Stock C++ controller** via the rig | **WORKS** | `forward_command_controller/ForwardCommandController` loaded by pluginlib (`load_controller`), params set on its node, `configure`→INACTIVE, `activate`→**ACTIVE (lifecycle id 3)**; a `[0.5,-0.3]` command published on `/fwd/commands` flowed topic→controller→hardware (read back off the command interface = `[0.5000,-0.3000]`). `demo-control-rig` exit 0. |
| 3 | Python controller (Route A, cross-inheritance) | **WORKS** | `PythonPDController(control_kit.ControllerInterface)` overrides `on_init` / `command_interface_configuration` / `state_interface_configuration` / `on_configure` / `on_activate` / `on_deactivate` / `update`. The CM called each override and ran `update()` at 100 Hz. The controller tracked a moving cosine reference with maximum error 0.050 rad on the mock joints. `demo-control-python` exited 0. |
| 4 | **Measure** (Python vs C++ under the same rig) | **DONE** | §4 numbers. Python `update()` 2.57 µs vs C++ 0.85 µs; both hold 100 Hz (0 late) and 1 kHz (~0.45 % late); GC pause 2.29 ms is the RT hazard. |

All four stages exercised ros2_control C++.

---

## 2. Bring-up details

### 2.1 Header parsing
MoveIt's convenience headers SIGSEGV Cling because they pull `generate_parameter_library`
output (`*_parameters.hpp` → fmt/rsl), docs/moveit_kit/REPORT.md §2.1. **ros2_control does
not have this problem at the surface we use.** Probed out-of-process (moveit lesson), these
all JIT-parse **cleanly**: `controller_interface/controller_interface_base.hpp`,
`controller_interface/controller_interface.hpp`, `hardware_interface/resource_manager.hpp`,
`controller_manager/controller_manager.hpp`. The CM's own parameters use a
forward-declared `ParamListener`/`Params` defined in the `.cpp` (not the header), and
`controller_interface_params.hpp` is a **hand-written struct** (pulls `joint_limits.hpp`,
which is also plain), so the full `ControllerManager` class is directly reachable. The
per-controller parameter headers (for example, `forward_command_controller_parameters.hpp`)
are generated. They are inside the controller `.so` loaded by pluginlib, so cppyy does
not parse them. The manager headers tested here parse in Cling without these headers.

### 2.2 Route A: cross-inheritance and injection
A Python class derives `controller_interface::ControllerInterface` and overrides its
plain virtuals (`on_init`, `update`, `command_interface_configuration`,
`state_interface_configuration` are pure virtual and **not `final`**; `on_configure` etc.
are plain virtuals from `LifecycleNodeInterface`). The kit handles four cppyy limitations:

- **Derive the *compiled* base, not a JIT'd one.** Deriving an intermediate class defined
  in a `cppyy.cppdef` breaks cppyy's override-dispatcher generation. The `on_init` return
  type (a `using CallbackReturn` alias) resolves to `<unknown>`. Class creation then fails
  (`no python-side overrides supported`). Deriving the **compiled** `ControllerInterface`
  directly works: the CM calls every Python override.
- **`CallbackReturn` is a `using` alias that cppyy resolves to Python `int`.** The real
  enum is `rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn`
  (SUCCESS=97, FAILURE=98, ERROR=99); the kit exposes it as `control_kit.CallbackReturn`
  from that full path (accessing `controller_interface::CallbackReturn` gives `int` and
  `.SUCCESS` then raises `AttributeError`).
- **Injection via a no-op-deleter `shared_ptr`.** `ControllerManager::add_controller` (a
  public method, grepped from the header) takes a `ControllerSpec` whose `.c` is a
  `ControllerInterfaceBaseSharedPtr`. Assigning that from Python fails (`C++ type cannot
  be converted to memory`, cppyy cannot copy a `shared_ptr` aliasing a cross-inherited
  object). The kit assembles the spec **in C++** and wraps the raw base pointer in a
  `shared_ptr` with a **no-op deleter**, Python owns the object (pinned by the rig), the
  CM must not free it. `add_controller` does **not** consult pluginlib (the instance is
  supplied), so no `.so`/xml is needed.
- **Protected interfaces via a same-layout accessor.** A controller reads/writes hardware
  through the *protected* `state_interfaces_` / `command_interfaces_` members, invisible
  to a Python subclass. `ControllerInterfaceBase` is the offset-0 base of every controller,
  so a `struct IfaceAccessor : ControllerInterfaceBase` reads those members at their true
  offsets; the kit exposes `read_state` / `read_command` / `write_command` /
  `n_state`/`n_command` as free functions taking the controller (Python passes `self`,
  which cppyy upcasts to the base pointer).

### 2.3 Off-thread activation for blocking switches
`switch_controller` blocks until the `update()` loop applies the switch (it is done in
`ControllerManager::update()`), so it cannot be called from the loop-driving thread.
Two cppyy facts shaped the fix: **`std::async` does not JIT in Cling** (symbol
materialization failure for the async-state template instantiation), but a **plain-function
`std::thread` does**; and **cppyy does not release the GIL on a blocking C++ call**
(measured: a background Python thread advanced its counter only once during a 2 s C++
`sleep`), so the switch cannot run on a Python thread while the main thread pumps `update()`.
The kit therefore runs `switch_controller` on a C++ `std::thread` (a static request struct
carries the args; atomics report completion) and pumps `update()` from Python until it
completes. Only `read`/`update`/`write` are pumped during the switch, spinning the
executor concurrently races the switch's mutation of the executor's node set.

### 2.4 uint8_t enums cross as a 1-char str
`controller_interface::return_type` is `enum class : std::uint8_t`. A **returned** value
(from `configure_controller`, `switch_controller`, a controller's `update`) crosses as a
1-char Python `str` (`'\x00'`==OK, `'\x01'`==ERROR), `int('\x00')` raises. The enum
**member** `return_type.OK` is a proxy that `int()`s fine. `control_kit.ok(v)` handles both
(`ord` on a str, else `int`). The same applies to a uint8 enum read back from a **struct
member** (`InterfaceConfiguration.type`).

### 2.5 Load before the loop; and teardown
- **Load/configure controllers *before* the first `update()`.** Once `update()` has run,
  the CM manages its controller list with real-time-safe swaps that block a synchronous
  `load_controller`/`add_controller` until the loop pumps again, which **deadlocks** a
  load issued from the (now-stopped) loop thread. This mirrors how `ros2_control_node`
  works (configure the stack, then spin the RT loop). The rig **guards** this: `load_*` /
  `add_*` after `update()`/`activate()` raise a clear error instead of hanging.
- **Teardown.** The CM owns a `pal_statistics` **async publisher thread**; if the CM
  outlives the rclcpp context the process cores at exit (`context cannot be slept with
  because it's invalid` → SIGSEGV, reproduced). The rig registers a teardown (via
  `cppyy_kit.register_teardown`, so it runs at `atexit` **before** rclcpp shutdown) that
  deactivates active controllers and resets the CM (C++ static ref + Python ref), the CM
  destructor then joins the pal_statistics thread while the context is still valid →
  deterministic exit 0. **Never `unload_controller` a cross-inherited controller at
  teardown** (it hung in probing); let the CM destructor clean it up. A benign
  `class_loader` "SEVERE WARNING … will NOT be unloaded" and `pal_statistics` "registry
  not found" warnings still print; neither affects the exit code.

### 2.6 Miscellany
`std::make_shared<rclcpp::executors::SingleThreadedExecutor>()` from Python hit cppyy
overload-cache flakiness (worked in one probe, failed verbatim in the next); building it in
a C++ factory is reliable (Pattern 6). Controller params (`joints`, `interface_name`) must
be **set** on the controller's own node after load (the controller's ParamListener
*declares* them empty in `on_init`; Jazzy controller nodes do **not** auto-declare from CM
overrides, `define_custom_node_options` only sets `enable_logger_service`), so the kit
`set_parameter`s them before `configure`.

---

## 3. Route A and Route B

Route A uses cross-inheritance. Route B uses a compiled pluginlib `.so` whose
`update()` calls a Python function through a C ABI. Route A was tested end to end
(§1/§2). A Python controller can be injected in three lines without compiling a
plugin or writing CMake files. Route B was not built because the in-process rig
did not need a compiled plugin.

Route A injects a Python controller with `add_controller`. Pluginlib cannot load it
by type name into a separately launched `ros2_control_node`. It runs only while the
Python process drives the loop. Use Route B if a stock controller manager must load
the controller by type name. Route B needs a compiled pluginlib `.so`; the L2
direct-compile procedure and C ABI setter pattern are in `scripts/freeze/build_l2_node.py`
and COMMON_PATTERNS §5. Section 4 describes the path from a Python prototype to a
native controller.

---

## 4. Stage 4: real-time results

> **Follow-up measurement (2026-07-12):** the jitter benchmark re-ran this rig's 1 kHz
> loop with unprivileged real-time knobs (timer slack 1 ns, `mlockall`, CPU pinning) and
> measured ~2.4 µs median wakeup latency on a stock kernel, see
> [docs/jitter_bench/REPORT.md](../docs/jitter_bench/REPORT.md) for the full matrix and
> the `SCHED_FIFO` and preemption steps used to address the remaining tail. The table
> below shows the original untuned measurements.

Update loop held for 8 s at each rate; per-cycle wall-clock intervals; "late" = interval >
1.5× the target period. Python controller (cross-inherited PD) vs stock C++
`forward_command_controller`, on the same rig and mock hardware. Another kit benchmark
used the machine during measurement, so treat these results as provisional.

| controller | pure `cm.update()` | 100 Hz achieved / late | 100 Hz interval ms (p50/p95/p99/max) | 1 kHz achieved / late | 1 kHz interval ms (p50/p95/p99/max) |
|---|--:|:--:|:--:|:--:|:--:|
| **C++ baseline** (forward_command) | **0.85 µs** | 100.0 Hz / 0 of 800 | 10.0 / 10.19 / 10.40 / 10.55 | 1000.0 Hz / 37 of 8000 | 1.00 / 1.07 / 1.33 / 2.09 |
| **Python** (cross-inherited PD) | **2.57 µs** | 100.0 Hz / 0 of 800 | 10.0 / 10.30 / 10.45 / 11.11 | 1000.0 Hz / 36 of 8000 | 1.00 / 1.09 / 1.28 / 3.53 |

- **update() dispatch: Python 2.57 µs vs C++ 0.85 µs (3.0×).** The difference
  comes from the cross-language call and Python arithmetic. The Python call took
  2.57 µs: 0.026% of a 10 ms (100 Hz) period and 0.26% of a 1 ms (1 kHz) period.
- **100 Hz: no late cycles for both.** Zero late cycles; the ~0.2–0.5 ms jitter is `time.sleep`
  / OS scheduling, indistinguishable between Python and C++. A 100 Hz Python controller is
  suitable for 100 Hz operation under this test setup. This test used mock hardware.
- **1 kHz: works on average, misses occasionally.** Both hit 1000 Hz mean with ~0.45 % late
  cycles (36–37 / 8000). The Python **tail is worse** (max 3.53 ms vs 2.09 ms): a
  **`gc.collect()` pause measured at 2.29 ms** exceeds a 1 ms period, so a GC that fires
  mid-loop exceeds one period, as shown by the Python maximum. The loop is
  single-threaded, so there is no GIL *contention*; the RT hazard is GIL-holding **pauses**
  (GC, allocation, any background Python thread) rather than throughput.
- These results support prototyping and soft real-time use at the tested rates. They do
  not establish hard real-time behavior. Python garbage collection, allocation, and GIL
  pauses can exceed a 1 ms period.

Prototype and validate the control law in Python against the controller manager. Then
compile `update()` as a native C++ pluginlib controller using the L2 direct-compile path
(`scripts/freeze/build_l2_node.py`) for hard real-time deployment in a stock
`ros2_control_node`. The Python and C++ controllers use the same interface contract.

---

## 5. Gaps

1. **In-process only / not spawner-loadable.** Route A's controller runs while the Python
   process drives the loop and is injected with `add_controller`. Pluginlib cannot load
   it by name into a separately launched `ros2_control_node` (see Route B in §3).
2. **First-`update()` JIT stall.** The first cross into a Python `update()` JIT-compiles the
   call wrapper, a **one-time ~29 ms** cost that shows as a single CM "Overrun might occur"
   warning on cycle 0. It does not recur; a per-controller warmup (exercise one `update()`
   before the timed loop) would move it. `warmup()` currently front-loads CM construction,
   not the per-controller dispatch.
3. **Not hard-real-time** (§4): GC/GIL pauses can miss a 1 kHz deadline.
4. **Mock hardware only in the demos.** `mock_components/GenericSystem` mirrors command→state;
   a real `SystemInterface` plugin loads the same way (pluginlib), untested here.
5. **Chainable controllers** (`ChainableControllerInterface`, reference interfaces) not
   probed, the same cross-inheritance mechanic should apply.
6. **Deprecated `get_value()`** is used by the interface accessor (returns `double`
   directly); `get_optional<T>()` is the non-deprecated form if warnings matter.
7. **First-run PCH rebuild** (~a minute, once/machine) and the rclcpp header JIT dominate
   cold start; freeze (FREEZE.md) would help.

---

## 6. Lessons for cppyy_kit

- **Cross-inheritance of a *framework* base + injection via a no-op-deleter `shared_ptr`.**
  Sharpens Pattern 16/17a: to hand a cross-inherited Python instance to a C++ container that
  stores it by `shared_ptr`, wrap the raw base pointer **in C++** with a no-op deleter
  (Python keeps ownership; pin the instance), assigning a `shared_ptr` aliasing a
  cross-inherited object *from Python* fails (`C++ type cannot be converted to memory`).
- **Derive the *compiled* base, never a JIT'd (`cppdef`) intermediate.** cppyy's override
  dispatcher fails to resolve return types (`<unknown>`) when the base is JIT-defined.
  Access protected base members via a **same-layout accessor** (`reinterpret_cast` to a
  `struct : Base` that reads them) exposed as free functions.
- **`using`-alias enums resolve to Python `int`.** Reference the real nested enum type
  (`Outer::Inner::Enum`), the alias loses the enum-ness.
- **uint8_t-backed `enum class`: a returned value / struct-member read crosses as a 1-char
  `str` (`ord` it); the enum member is a proxy (`int()`s).** Generalises Pattern 11 to the
  unsigned-char underlying type.
- **`std::async` does not JIT in Cling; a plain-function `std::thread` does.** For a blocking
  C++ API that must run off the calling thread.
- **cppyy does not release the GIL on a blocking C++ call** (measured). A blocking C++ call
  must run on a **C++** thread if Python work must proceed concurrently, not a Python thread.
- **`std::make_shared<T>()` of some classes is flaky from Python** (overload-cache
  sensitivity); build in a C++ factory (Pattern 6).

---

## 7. Recommendation

The test shows that a Python controller can run inside a `controller_manager` using the
real update loop and mock hardware. This addresses the item marked unproven in the
exposure note. Route A derives from `ControllerInterface`. The ros2_control headers
tested here parse in Cling. The kit handles injection, protected-interface access, the
blocking switch call, uint8 enums, and teardown with about 94 lines of C++ glue. The
tests do not establish hard real-time behavior. Loading a controller by pluginlib type
name requires Route B.

The same cross-inheritance method may apply to Nav2 controller, planner, and behavior
plugins if their managers expose an API for injecting plugin instances. If a manager
loads plugins only by type name, use Route B with a compiled `.so` and C ABI setter.
**Recommendation:** continue development of the kit and this method.

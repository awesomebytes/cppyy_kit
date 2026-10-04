# Common patterns for using C++ libraries from Python

This is an advanced usage and kit-author reference for using C++ libraries from
Python through cppyy. For installation and a first working example, start with
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/).

To find a practical pattern, see
[callbacks](#python-callbacks) and
[callback and buffer lifetime](#callback-lifetimes),
[writing an inline C++ kernel](#inline-cpp),
or [reusing compiled C++ with the cache](#compile-cache).
The catalog draws on BehaviorTree.CPP, PCL, and other domain kits. Each example
describes behavior tested with its named library; coverage and requirements vary
by library. Shared `cppyy_kit` utilities cover loading, conversion, callbacks, and
lifetime management.

## Three integration steps

A typical integration includes three steps:

1. **Bringup:** locate the install, add include paths, JIT-include the headers,
   and `load_library` the `.so` set.
2. **Handle cppyy-specific requirements:** account for container construction,
   ownership, lifetime, and template attributes.
3. **Expose selected native operations:** preserve class and method names where
   practical. Add wrappers for bringup, conversion, or lifetime management as needed.

For example, `bringup_bt()` and `bringup_pcl()` both load their library and prepare
its headers. Calls after bringup use native operations such as
`factory.registerSimpleAction` and `pcl.VoxelGrid[pcl.PointXYZ]`.

---

## Pattern catalog

### 1. Bringup: `load_library` is mandatory; `add_library_path` is not enough
cppyy resolves a symbol by finding its **owning `.so` at call time** by scanning
its own library search path. Adding a path is not enough. Every library you call
into must be `load_library`'d by soname. `cppyy_kit.load_libraries(sonames,
search_paths)` centralizes this.
- **bt:** one lib, `libbehaviortree_cpp.so`.
- **pcl:** a set, `libpcl_common/octree/kdtree/search/sample_consensus/filters`
  (filters pulls the rest transitively at runtime).
- Do not rely on `LD_LIBRARY_PATH`; an installed package has no activation hook,
  and cppyy uses its own search path for call-time resolution.

### 2. Bringup cost & staging: gate the expensive includes
Bringup time is dominated by the **header JIT-parse**. Split it into cheap and
expensive stages and let the caller skip what they don't need.
- **bt:** ~0.85 s, ~89% of it the single `cppyy.include("bt_factory.h")`; logger
  headers (which pull zmq/flatbuffers) are included lazily only when an
  observability helper is first called.
- **pcl:** core ~1.3 s; adding the ROS message headers (`pcl_conversions`) costs
  another ~1.9 s, gated behind `bringup_pcl(with_ros=False)`.
- **ompl:** ~538 ms. This is lower than bt or pcl, despite using boost. Parse time
  depends on the headers included transitively. Measured parse times were ompl+boost
  (538 ms) < bt.CPP (0.9 s) < pcl (1.3 s). Measure the actual `include(...)` call
  before estimating bringup time from the library size.

<a id="python-callbacks"></a>

### 3. Crossing a Python function **into** C++ (`callback`)
Pass a Python callable to C++ with `cppyy_kit.callback(fn)`. The helper infers the
signature and retains the callable for the required lifetime.
```python
def on_value(x: int, y: float) -> bool:      # hints -> "bool(int, double)"
    return x > y
fn = cppyy_kit.callback(on_value)            # ready to pass to any C++ std::function slot
```
- **Inference** maps `int`->int, `float`->double, `bool`->bool, `str`->std::string,
  `None`->void (return), and any cppyy C++ class (via `__cpp_name__`) as a
  reference. Parameters with Python defaults / `*args` are ignored (not C++ args).
- **A class hint can only infer `T&`**, but cppyy will bind a
  `std::function<...(T&)>` even where the API wants `const T*` (ompl's
  `setStateValidityChecker`), and the mismatch then fails *later* at the call site.
  `callback` warns once and names the required fix. For the exact form, annotate the
  parameter with the **C++ type string**, used verbatim:
  `def check(s: "const ompl::base::State*") -> bool: ...` →
  `bool(const ompl::base::State*)`. (flake8/pyflakes flags such a string annotation
  as `F722`, a forward-reference false positive. Add `# noqa: F722`, or use
  `signature=` instead.)
- **Explicit `signature=` wins** for anything, e.g.
  `cppyy_kit.callback(tick, signature="BT::NodeStatus(BT::TreeNode&)",
  owner=factory)`, exactly how bt_kit registers leaf/stateful hooks and ompl_kit
  fixes the validity-checker pointer form.
- **Threading:** the callback runs in whatever C++ thread invokes it (cppyy takes
  the GIL); a single-threaded driver (a tick loop, a `spin_some`) never contends.
- `cppyy_kit.std_function(sig, fn)` is the low-level option (raw wrapper, you
  handle lifetime yourself); prefer `callback`.

<a id="callback-lifetimes"></a>

### 4. Python callback and buffer lifetime
cppyy does **not** keep a Python callable (nor its `std::function` wrapper, nor a
buffer backing a view) alive just because C++ holds it. If Python collects a callback,
calling it from C++ raises `TypeError: callable was deleted`. A test reproduced this
when a temporary `lambda` passed to raw `std_function` was collected before the call.
- **`callback()` retains the Python callable and wrapper.** With `owner=` they live
  as long as that object; without `owner=` they are
  pinned in a module-level registry for the process lifetime. Call
  `cppyy_kit.release_callbacks()` after C++ no longer needs them.
- For **non-callback** objects (a buffer backing a zero-copy view, a logger),
  `cppyy_kit.keep_alive(owner, *objs)` is the primitive. **pcl:** the source cloud
  is pinned on the ctypes buffer backing a NumPy view so it can't outlive its
  storage. `keep_alive` raises `TypeError` if the owner cannot store its lifetime
  pins; don't ignore that failure. **bt:** leaf callbacks are pinned on the factory
  (via `callback(owner=)`) and carried onto the tree.

### C++ → Python direction (no helper needed)
C++ functions are already callable from Python, so no helper is needed:
```python
cppyy.gbl.mylib.some_fn(21)          # a cppdef'd/loaded C++ function IS a Python callable
holder.store(cppyy.gbl.mylib.some_fn)  # and can be handed straight back into a C++ API
```
A Python `callback()` stored in a C++ `std::function` can also call back into Python
when C++ invokes it. This is tested in
`test/test_cppyy_kit.py`.

### 5. Crossing objects **out** without crossing ownership (`HandleRegistry`)
Returning a `std::unique_ptr<T>` **from** a Python `std::function` fails
(`C++ type cannot be converted to memory`). To let C++ create per-instance Python
state, keep the ownership-creating lambda entirely in C++ and have it call a
Python **builder that returns an integer handle**; dispatch later callbacks by
that handle. `cppyy_kit.HandleRegistry` is the table.
- **bt:** the per-tree-node stateful builder, C++ builds each node, calls the
  Python builder and receives a handle. The shim's onStart/onRunning/onHalted dispatch by
  handle. This is what makes two nodes of the same registered ID keep independent
  state.

### 6. Containers and bulk data: use C++ helpers for unsupported conversions
In tested configurations, some STL construction and insertion operations from
Python terminated the process without a Python traceback. Build affected containers
in a C++ helper; pass raw addresses for bulk buffers when that avoids per-element
conversion.
- **bt:** the `PortsList` (`unordered_map<string,PortInfo>`) is built in C++ from
  two parallel `vector<string>` values (names, types); constructing the map or a
  `vector<pair>` from Python triggered the failure in this test.
- **pcl:** a `cppdef` helper copies between NumPy and a cloud using the array address
  (`arr.ctypes.data` as `uintptr_t`) and `memcpy` or a strided copy in C++. A
  per-element Python loop is about 90x slower.
- **nav2:** the same method copies the `unsigned char*` costmap buffer. It is about
  600 to 3600x faster than a Python loop. NavFn returns `getPathX()` and `getPathY()`
  as `float*` values with a length. A C++ helper copies these arrays before returning
  them to Python.
- **Build the message once, refill per frame (retarget).** For a ROS message
  re-published every cycle, construct it **once** in C++ (fixed structure, such as a
  `TFMessage`'s 75 frame names) and each cycle refill only its numeric fields from one
  flat address, rather than reconstructing the message's proxies field-by-field in
  Python. Measured **265×** for a 75-frame `/tf` message (0.0005 vs 0.144 ms/message);
  this reuses the message because it stays in C++. A Python broadcaster typically
  rebuilds the message for each frame.
- **Copy-in vs alias-in (vision).** The above all *own* storage, so one copy in is
  unavoidable. When the C++ type can **alias** an external buffer it is genuinely
  **zero-copy**: `cv::Mat(rows, cols, type, void* data, step)` wraps a ROS
  `Image` buffer pointer-identically. Still a `cppdef` helper (cppyy rejects a Python
  int as `void*`; pass `uintptr_t`), and you **must keep the source buffer alive**
  for the Mat's lifetime (a lifetime guard, not a copy). Distinguish "buffer you can
  alias" (zero-copy) from "storage you must own" (one copy).
- **Build the object in a small C++ factory when Python can't construct it.** Three
  instances now: `std::make_shared<T>()` flaky from Python (overload-cache
  sensitivity, control_kit); `make_shared` of a specific class (control_kit); and a
  **template ctor with a universal-reference default**, `Cls(NodeT&& node =
  NodeT())` reports "class has no public constructors" from Python (tf). One-line
  `cppdef` factory returning the object sidesteps all three.

### 7. On-demand templates: include the `impl` headers
A precompiled `.so` only carries the specializations its authors compiled. To let
Cling instantiate `Template<UserType>` at JIT time, include the library's
`impl/*.hpp` (or `*.hxx`). This is the difference between a fixed-surface binding
and cppyy. Cling can instantiate other types on demand.
- **pcl:** including `pcl/impl/pcl_base.hpp` + `filters/impl/voxel_grid.hpp` lets
  `PointCloud<T>` / `VoxelGrid<T>` instantiate for point types no binding shipped.
- **bt:** template **member** calls work directly, `node.getInput[T](key)` from
  Python; unwrap the returned `Expected<T>` with `has_value()`/`value()`
  (`cppyy_kit.unwrap_expected`).
- **vision:** a **dependent-type** template member (a templated member accessed on a
  value whose type depends on a template parameter, inside a patched header) needs
  the explicit `.template` disambiguator required by clang's two-phase lookup:
  `obj.member.template ptr<T>()`. It is not needed for concrete types.

### 8. Call C++ function templates directly; let cppyy deduce
When a template argument can be deduced from a runtime argument, call the function
straight from Python without an explicit `[T]` or a wrapper.
- **pcl:** `pcl.toROSMsg(cloud, msg)` / `pcl.fromROSMsg(msg, cloud)` deduce
  `PointT` from the cloud. Reach for a `cppdef` helper only when a template arg
  can't be deduced or ownership must not cross (Pattern 5).

### 9. Cling parser compatibility and failed declarations
Cling may reject constructs accepted by newer clang versions. In tested failures, a
`cppdef` error terminated the interpreter during transaction rollback without a
Python traceback. Probe risky declarations and includes in a subprocess.
- **pcl:** Cling rejects a trailing type attribute (`struct { ... } EIGEN_ALIGN16;`).
  Use a prefix form (`struct alignas(16) X`). Custom point types must be declared
  that way.
- **A failed `cppyy.include` can leave Cling in a bad state (nav2).** In one test, a
  header parse failure caused a later, unrelated include to fail in the same process;
  the latter header parsed in a fresh process. Probe headers with uncertain
  dependencies in a subprocess.
- **A tested MoveIt parameter header crashes Cling during parsing.** The
  `generate_parameter_library`-generated `kinematics_parameters.hpp` and
  `planning_pipeline_parameters.hpp` failed in the tested MoveIt environment. Avoid
  including these headers in the main process; load the compiled plugin or probe the
  header in a subprocess. This result is specific to the tested headers and Cling
  version; test other generated headers separately.
- **The parser limitation does not imply a plugin build failure (ik_bench).** The
  tested pick_ik plugin builds its generated parameter code with CMake and loads
  through pluginlib without parsing those headers through Cling. Load the compiled
  plugin rather than including its parameter headers in Cling (§19).
- **ORC static-initializer failure: parsing succeeds, execution fails (vision/gtsam).**
  A header can `cppyy.include` successfully but the first use fault later, when Cling's ORC
  JIT must materialize a **namespace-scope internal-linkage static** it cannot emit.
  gtsam's `static const KeyFormatter DefaultKeyFormatter` (`Key.h`, a non-exported
  `std::function` global; each TU emits its own init). In this test, the failure was
  specific to Cling's JIT rather than a missing dependency. It differs from a parse
  error; when includes pass but a symbol does not materialize, use §20 (the library's
  Python binding for batch steps) for this path.
- **`boost::variant` template-arity incompatibility (wbc/pinocchio).**
  A big `boost::variant` that a **precompiled** `.so` carries fine can be
  **un-JIT-able from headers** under a newer boost whose preprocessed arity limit it
  exceeds. pinocchio's `JointModelVariant` is a **25-type `boost::variant`**;
  re-instantiating `ModelTpl<Scalar>` for a new scalar hits **boost 1.90**'s
  `make_variant_list` limit (`wrong number of template arguments (25, should be at
  least 0)`), while the shipped `double`/casadi libraries sidestep it by being
  precompiled. This is a **parse** failure, distinct from the ORC static-initializer
  (an *execution* failure). Reproduce with `g++` to confirm it is not a Cling quirk;
  suspect it when a template *class* re-instantiation for a new parameter fails but
  the shipped specialization works. A second test in the retarget pipeline also
  failed for the default-`double` `Model`. Instantiating `pinocchio::Model` from
  headers for URDF parsing, FK, or a Crocoddyl `StateMultibody` reaches the same
  `make_variant_list` limit. The subprocess probe reported a compile error at
  `JointModelTpl<double>`. Use pinocchio's
  rigid-body **multibody core via its Python bindings**; cppyy is useful in this stack for
  the abstract/custom-model path (crocoddyl action models, §31) and *non*-pinocchio glue
  kernels, not the `Model` itself.
- **Mitigation:** probe risky glue out of process first:
  `cppyy_kit.probe_cppdef(code, include_paths=(), library_paths=(), headers=(),
  libraries=(), timeout=60)` compiles it in a throwaway subprocess and returns
  `(ok, message)` without risking the main interpreter. The timeout is in seconds;
  nonpositive values raise `ValueError`, and an expired probe returns
  `(False, diagnostic)`. Pass it the **full
  ament include-path set** (every package's include dir, via
  `get_packages_with_prefixes`), not just the target library's. A header
  that transitively pulls the ROS message tree can fail on a missing transitive header.

### 10. Error messages: remove the C++ signature prefix
cppyy prefixes a C++ exception with the mangled call signature and ` => `. Split
on ` => ` and collapse whitespace for a readable one-line message, re-raised as a
kit exception (`cppyy_kit.pretty_cpp_error`, `CppyyKitError`).
- **bt:** `BtXmlError` removes the `createTreeFromText(...) =>` signature prefix, yielding
  `RuntimeError: Error at line 4: -> Node not recognized: Nope`.

### 11. Values that don't cross as ints: enums, `unsigned char`, macros
C++ enums behave like their int values across the boundary
(`BT.NodeStatus.SUCCESS == 2`). Expose plain-int constants for convenience while
keeping the real enum available (`bt_kit.SUCCESS` and `bt.NodeStatus.SUCCESS`).
Five related conversion cases need explicit handling:
- **`unsigned char` (and `uint8_t`-backed `enum class`) crosses as a length-1
  Python `str`, not an int** (nav2, control). `Costmap2D::getCost()` and its
  `static constexpr unsigned char` cost constants come back as `'\xfe'`, and
  `'\xfe' == 254` is `False`. Read with `ord(...)`, and expose **plain-int**
  constants from the kit. (The enum *member* is still an int-able proxy; it's a
  *returned value / struct-member read* of the uint8 type that becomes a `str`.)
- **The same 8-bit types cross as a length-1 `str` a SECOND, independent way:
  as a `std::function` callback ARGUMENT when C++ calls into Python**
  (rclcpp_kit: the lifecycle transition-callback bridge). This is a different
  code path from the first case. In one test, this typedef's plain return values and
  struct-member reads converted to integers, but the callback argument did not. Symptom: a state id of `1` arrives as
  `'\x01'`, so `int(state_id)` raises `ValueError: invalid literal for int()
  with base 10: '\x01'`. Fix: widen the *crossing* signature from `uint8_t`/
  `int8_t` to `int` (e.g. `std::function<int(int, std::string)>`), and cast
  back to the real 8-bit type or enum only on the C++ side of the shim. Rule
  of thumb: never spell an 8-bit integer type in a `std::function` signature
  that C++ uses to call a Python callable. Use `int` at that boundary.
- **A `using`-alias of an enum resolves to plain Python `int`** (control), losing
  the enum-ness. Reference the **real nested enum type** (`Outer::Inner::Enum`), not
  the alias.
- **Type-constant `#define` macros are invisible to cppyy** (vision: `CV_8UC1`,
  `CV_8U`). Re-expose the few you need as real `const int` in a `cppdef` block.
- **A `std::string` inside a returned `std::vector<std::string>` can surface as
  Python `bytes`, not `str`** (tf: `getAllFrameNames()`). Decode at the kit
  boundary (`b.decode()`).

### 12. Mirror, don't sugar
Patch/return the library's real classes so methods keep their C++ names (add
snake_case aliases). A bespoke DSL was prototyped for bt_kit but not kept. It
needed a module-global registry, creating risks across trees, re-imports, and tests, and
forced knowledge that doesn't transfer. Both kits ship the mirror.

### 13. GIL / concurrency (what "parallel" means)
Kit callbacks run in the **calling C++ thread**. A single-threaded engine serializes
its callbacks; concurrency with other threads depends on the driver.
- **bt:** `ParallelNode` is cooperative bookkeeping, not OS threads, Python leaves
  under it run sequentially (no true parallelism, no contention). A leaf that
  sleeps / does I/O releases the GIL, so spinning the tree from a background thread
  does not deadlock; a busy-blocking leaf would. Don't expose `ThreadedAction`
  (real C++ worker thread) without explicit GIL handling.
- **cppyy does NOT release the GIL on a blocking C++ call (control, measured).** So
  a blocking C++ call cannot be overlapped with Python work by putting it on a
  *Python* thread, it holds the GIL the whole time. Run it on a **C++** thread
  instead: a plain-function `std::thread` in a `cppdef` helper (note `std::async`
  does **not** JIT in Cling, use `std::thread`). control_kit's blocking
  controller-switch does exactly this.
- **Let C++ own the loop and call into Python only when needed (tf).** A
  library that already spins its own C++ thread can be a useful cppyy target.
  `tf2_ros::TransformListener(spin_thread=
  true)` ingests `/tf` on its own `std::thread`, entirely off the GIL; Python only
  crosses on `lookup`. One shared-host pass observed Python/C++ ingest CPU ratios of
  6.7 to 14 as traffic increased. Measure the library's loop and target workload
  before replacing it with a Python thread.

### 14. Teardown: release global-state C++ objects before Python finalizes
A process may run Python finalizers and cppyy's Cling teardown without a defined
order. Python finalizers can destroy C++ objects. Objects that own **process-global
or static state**, such as an `rclcpp` Context, DDS participant, or ZMQ-backed BT
logger, may fail if destroyed after Cling or while a DDS thread still uses their state.
The process may then exit with SIGSEGV and no Python traceback.
- **History and evidence:** Some rclcppyy scripts called `os._exit(0)` after printing
  results. This skipped destructors and returned success without cleaning up objects.
  Tests on cppyy 3.5, ROS Jazzy, CycloneDDS, and Fast DDS did not reproduce a crash in
  about 8 scenarios. They covered plain pub/sub, mixed bt+rclcpp and pcl+rclcpp use,
  module-global entity lifetimes, and one or two calls to `rclcpp::shutdown()`.
- **Fix:** `cppyy_kit.register_teardown(cb)` and `cppyy_kit.shutdown()` provide a LIFO,
  idempotent, best-effort registry connected to `atexit`. Python runs these handlers
  after `main` returns and before clearing module globals. The registry runs before
  cppyy tears down Cling, while Python and cppyy are available. rclcppyy registers
  `shutdown_rclcpp()`, a guarded once-only `rclcpp::shutdown()`; the guard also
  closes the historical **double-`rcl_shutdown`** race (`rclcpp` installs its own
  SIGINT/SIGTERM handler that can call shutdown a second time). Kits with no
  process-global C++ state (bt, pcl) register nothing: their objects are
  per-instance and RAII-released on scope exit, and the JIT'd namespaces are
  Cling's to tear down. `test/test_clean_exit.py` checks this behavior.
- **A C++ object owning an executor + `std::thread` is the same hazard (tf, 3rd
  instance).** `rclcppyy.tf`'s C++ TransformListener owns a spinning executor +
  thread; `register_teardown` a callback that drops it (its dtor cancels the
  executor and joins the thread) so it releases **before** `shutdown_rclcpp` (LIFO
  order is correct). Exit 0 confirmed. Same rule as the pluginlib instance/loader
  `reset()` in §19. Objects that own threads, executors, or global state need an
  ordered teardown.

### 15. First-use JIT: report and warm wrapper compilation
The first time a given C++ signature is crossed, cppyy JIT-compiles a call wrapper
for it. This is a one-time cost for each signature. It can be large at kit entry
points (bt_kit's first `registerSimpleAction` ~0.4 s to codegen the
`std::function<NodeStatus(TreeNode&)>` thunk + the register call wrapper; the first
pcl NumPy→VoxelGrid→NumPy frame ~0.45 s). A **freeze/PCH does not remove it** (the
PCH is an AST, this is call-wrapper codegen triggered by the Python call), and
`-O0`/`-O1` make no difference (it is Clang front-end instantiation, not LLVM
optimisation). So a script's *first live call* stalls unexpectedly.

cppyy_kit provides two functions for this:
- **Make it visible.** Wrap a known-expensive kit entry point in
  `with cppyy_kit.first_use(label, warmup_hint):`. On the first call that exceeds a
  threshold it prints a one-time message to stderr with a suggested action, e.g. *"bt_kit.
  register_simple_action compiled a call wrapper on first use (408 ms). Call
  bt_kit.warmup() during initialization. Set RCLCPPYY_JIT_NOTICE=0 to silence this message."* Thereafter
  (and when disabled or warming) it is a bare passthrough, zero overhead.
- **Move it.** A kit's `warmup()` exercises its expensive signatures on throwaway
  objects (under `cppyy_kit.suppress_first_use_notice()`, via the
  `cppyy_kit.warmup(*thunks)` building block) so the wrappers are JIT'd and cached
  process-globally during init. Measured: bt_kit `time-to-first-tick` 678 → 98 ms
  (the spike moves into a ~0.9 s init); pcl frame-0 630 → 4 ms.

*Scope choice:* instrument the **kit-owned entry points** (registration, bringup),
not every cppyy call (too broad, adds overhead) nor a generic opt-in context (can't
name the API/warmup in the notice). This is reliable where the kit owns the entry
point (bt registration); for kits that mirror a raw algorithm API whose first-use
cost is *inside* un-wrapped library calls (pcl's VoxelGrid), the notice is
best-effort. Use `warmup()` for those calls. `warmup()` stays per-kit (only the
kit knows what to exercise); the notice/suppress/runner are shared.

### 16. Cross-language inheritance (Python derives a C++ virtual base)
A frequently used crossing, first tested in ompl_kit: a Python class *derives* a C++
class and C++ calls its overrides in a hot loop (RRT\* calls a Python
`StateValidityChecker` millions of times/solve). Use these rules:
- Derive the cppyy class directly; **`super().__init__(base_args)` is mandatory**
  (the C++ base must be constructed, e.g. with its `SpaceInformation`).
- Override the virtual by its **exact C++ name** (`isValid`, `stateCost`), cppyy
  matches on the name.
- **Only plain virtuals** can be overridden across the boundary. A `final` (or
  non-virtual) member cannot, this is exactly why bt_kit's `final` `tick()` needed
  a C++ shim instead (Pattern 5 / bt REPORT). Check the base before promising it.
- **Watch for versioned pure-virtual creep (wbc/Crocoddyl).** A library minor version
  can **add pure virtuals to a base you subclass** and silently make your override
  abstract, Crocoddyl 3.2's `CROCODDYL_BASE_CAST` macro added
  `cloneAsDouble`/`cloneAsFloat` to `ActionModelAbstract`. In C++ that is a compile
  error; **in cppyy it is a failed-`cppdef` crash** with no traceback (§9). Before
  authoring, `nm`/grep the base for **every** pure virtual (including macro-injected
  ones) and probe the subclass `cppdef` out-of-process; a kit should ship the mandatory
  boilerplate as a constant (`wbc_kit.ACTION_MODEL_CLONES` is the worked example).
- **Pin the subclass instance** with `keep_alive` (or an `owner`): the "callable
  was deleted" risk (Pattern 4) applies to override *instances* too, C++ holds
  the object, cppyy won't keep it alive for you.
- Pointer arguments arrive **auto-downcast** (Pattern 17b) so member access on the
  concrete type works with no explicit cast.
Cost: ~350 ns per override call, 1 to 3 M dispatches/s. This is small when the
override is a small part of the work and significant when it dominates (then lower it to C++, the L2 level).

**Deriving a *framework* base and injecting it (control_kit also uses this pattern):**
- **Derive the *compiled* base, never a `cppdef`'d intermediate.** cppyy's override
  dispatcher fails to resolve return types (`<unknown>`) when the base was itself
  JIT-defined; subclass the real library class directly.
- **Inject the instance where C++ stores it by `shared_ptr` via a C++-built no-op-
  deleter `shared_ptr`.** Assigning a `shared_ptr` that aliases a cross-inherited
  Python object *from Python* fails (`C++ type cannot be converted to memory`);
  build the aliasing `shared_ptr` (no-op deleter, so Python keeps ownership, pin
  the instance) in a `cppdef` helper. This is how control_kit hands a Python
  `ControllerInterface` subclass to the real `controller_manager`.
- **Reach protected base members through a same-layout accessor**, a
  `struct : Base` that `reinterpret_cast`s and reads them, exposed as free
  functions, since cppyy can't touch `protected` across the boundary.

### 17. `shared_ptr` ownership + RTTI downcast (two cppyy conveniences)
- **(a) Wrapping a raw pointer in the library's `shared_ptr` transfers ownership.**
  Constructing a `SomethingPtr(raw)` from a cppyy-owned raw object flips the raw's
  `__python_owns__` to `False`, cppyy yields ownership to the `shared_ptr`, so
  there is **no double-free**. This makes the pervasive "wrap the raw in the
  library's Ptr and hand it on" idiom (`ob.StateSpacePtr(space)`) safe, and mirrors
  `make_shared`.
- **(b) Pointer arguments are auto-downcast by RTTI.** cppyy presents a base-typed
  pointer argument (a callback's `const State*`) as its **concrete runtime type**,
  so `state[0]` / `state[1]` work without an explicit downcast. You rarely need a
  cast helper; when you do (a stored base pointer), use `getattr(obj, "as")[T]()`
  (Pattern 18).
- **(c) cppyy dereferences a `shared_ptr` to bind a `const T&` parameter** (moveit:
  `srdf::Model::initString` took a `ModelInterfaceSharedPtr` directly), smart-
  pointer forwarding is reliable for reference params, not just member access.
- **(d) Eigen block/coeff assignment does NOT cross** (moveit: `iso.translation()[i]
  = v` → "object does not support item assignment"). Build Eigen objects in a
  `cppdef` helper (assemble the whole vector/matrix in C++), not element-by-element
  from Python. Eigen is everywhere in robotics C++, so this recurs.

### 18. Reserved-word method names: `getattr(obj, "as")[T]()` for C++ `as<T>()`
The pervasive C++ idiom `obj->as<T>()` is a Python `SyntaxError` (`as` is a
keyword, even `obj.as` won't parse). The spelling is `getattr(obj, "as")[T]()`
(fetch the attribute by string, then subscript the template arg). Use `getattr` when
a C++ method name is a Python keyword such as `as`, `from`, or `class`.

### 19. In-process pluginlib + a parameterized node (the ROS 2 plugin/param bootstrap)
Modern ROS 2 stacks load their algorithms as pluginlib plugins configured by node
parameters. Both work in-process (moveit_kit proved it; control_kit reuses it):
- **pluginlib:** `load_library("libclass_loader.so")` + **the plugin base-class
  library** (for its typeinfo), construct `pluginlib::ClassLoader<Base>(pkg,
  "Base::type")` in a `cppdef`, then `createUniqueInstance(lookup_name)`, pluginlib
  `dlopen`s the plugin `.so` itself via the ament index (do **not** cppyy-load the
  plugin). The "add the library named in the JIT link error" loop resolves the rest.
- **A dlopen'd plugin's sibling libs need `LD_LIBRARY_PATH` set *before* the process
  starts (ik_bench).** A vendored plugin `.so` depends on its co-installed core lib
  (`libpick_ik_plugin.so` → `libbio_ik.so`) in the same private prefix. The dynamic
  linker reads `LD_LIBRARY_PATH` **at process start**, so setting it inside the running
  worker is too late for pluginlib's `dlopen`. Prepend the vendored `lib/` dir to the
  **child's** `LD_LIBRARY_PATH` before spawning (or `$ORIGIN`-RPATH the install);
  `AMENT_PREFIX_PATH`, which *is* read at runtime, carries the plugin discovery.
- **parameterized node:** `NodeOptions().automatically_declare_parameters_from_overrides(true)
  .parameter_overrides(vec)` + `make_shared<rclcpp::Node>(name, options)`, fed by a
  **YAML → dotted-`rclcpp::Parameter` flattener** (nested dict → dotted names,
  homogeneous lists → typed arrays), the reusable primitive.
- **Teardown (Pattern 14, also applies):** a pluginlib instance/loader must be
  `reset()` before Cling teardown or the process cores at exit, `register_teardown`
  it. The `class_loader` "will NOT be unloaded" warning is benign/expected.

### 20. Kit-authoring triage: is the C++ core drivable, and when to fall back
Before investing in a kit, a couple of one-line greps tell you what's separable:
- **Lifecycle coupling (nav2).** Grep the class's ctor / `configure` signatures: if
  it takes plain data it's drivable; if it takes a `LifecycleNode` / `*ROS` wrapper /
  a pluginlib base, it needs the server (out of scope for a "drive the core" road, or
  use the pluginlib bootstrap §19). `nm -DC` / a header grep up front beats
  discovering it after the JIT investment.
- **Missing transitive headers (vision/gtsam).** A header-heavy library can be
  **un-JIT-able** if a transitive include is absent from the env (gtsam →
  `boost/optional.hpp`). Grep the target's transitive includes for env-absent deps
  first. When blocked *and* the work is **batch** (not a hot loop), the library's own
  **Python binding is a legitimate fallback** for that step, cppyy is not the only
  tool, and a kit can mix (drive the hot C++ path via cppyy, use the binding for a
  one-shot batch step). (2nd instance of this rule after the gtsam batch step below.)
- **Probe layered blockers one at a time, and know when to stop.** gtsam via cppyy
  is the worked example: fixing the boost blocker (add headers) only exposed a
  `GTSAM_USE_TBB` → tbb-headers blocker, which when fixed exposed the Cling **ORC
  static-initializer limitation** (§9), which adding dependencies does not resolve. Check one layer,
  re-probe out-of-process; when the bottom layer is a Cling limitation rather than a
  missing dep, stop and take the Python-binding fallback for that batch step. Don't
  continue adding dependencies for an issue caused by Cling's parser.
- **Distrust environment shims, prefer the native binary.** A library's console-
  script entry point can be broken in an env while its native binary works (vision:
  the `rerun` console script vs spawning the viewer binary by its executable path).
  When a Python-package CLI shim misbehaves, resolve and invoke the real executable
  directly rather than assuming the library is broken.

### 21. Vendored-source direct-compile (when no package is available)
For a small, well-understood subset of a library that ships no conda package
(DBoW2), clone it + apply a **documented, marker-guarded in-place patch** + compile
with a direct `$CXX` invocation into a `.so`. This avoids the library's
CMake/ExternalProject. It generalizes the L2 lowering recipe (`build_l2_node` →
`build_dbow2`): a reproducible build script, artifact gitignored, env-version tagged.

**A second case: a vendored ROS/MoveIt *plugin* needs its ament install layout,
which includes more than a bare `.so` (ik_bench).** The direct-`$CXX`→`.so` recipe does *not* suffice for a
pluginlib plugin, because discovery is via the **ament index**, which only the
package's own `ament_package()` + `pluginlib_export_plugin_description_file` produce
(the `<pkg>` marker, the plugin description XML, the `.so`). So for a plugin,
"vendored build" = **run its own CMake with a plain `cmake` configure/build/install
into a private prefix**, then put that prefix on `AMENT_PREFIX_PATH`, pluginlib then
finds it by lookup name, no different from a packaged one. The two unpackaged IK
solvers (bio_ik, pick_ik) both built first try this way; pick_ik needed only one extra
header-only dep (`range-v3`) added to the env, no source patches.

### 22. Overload mis-resolution: a compilable-but-WRONG overload that crashes
Distinct from the parse/execution faults (§9): with a **large set of overloads**, cppyy
can pick one that **compiles and runs but is the wrong one**, crashing at runtime
(bus error, no Python traceback). tf: `tf2_ros::Buffer::lookupTransform(target,
source, TimePoint)` resolved into the `rclcpp::Time`+timeout `canTransform` path,
which called `rclcpp::Clock::now()` and bus-errored. The trap is worst when a class
mixes a `using`-imported base form with timeout/clock forms of the same name.
- **Rule:** prefer the **single-signature base class** (here `tf2::BufferCore`, one
  unambiguous `lookupTransform`) over the overload-heavy derived one; or wrap the
  exact call in a `cppdef` free function, letting C++ overload resolution pick it. Probe a suspicious overloaded call out-of-process; a
  wrong-overload crash gives you nothing to read in Python.

<a id="compile-cache"></a>

### 23. Reuse compiled wrapper artifacts (`cppdef_cached`)
Use `@cpp` (§26) for a C++ kernel you write in Python. Use
`cppyy_kit.cppdef_cached(code, decls=...)` for reusable C++ helpers or trampolines:
the definitions compile into a `.so`, and `decls` lets Cling call them without
JIT-compiling those function bodies again. The first compatible cache miss builds
the `.so`; later runs in the same compatible environment can reuse it. This caches
kit-authored glue, not arbitrary template calls made directly by application code
(see **Cache scope** below).

The compile cache reduces first-use call-wrapper JIT (§15). The separate PCH cache
reduces header parsing (see
[FREEZE.md, Automatic PCH setup](FREEZE.md#8-automatic-pch-setup-cppyy_kitautopch)).
Measured with bt_kit adopted (t01): first-use register
**~233 ms → ~60 ms**, and freeze + cache compose to **~1.77 s → ~0.43 s** end-to-end
(FREEZE.md §4); pcl_kit's d02 frame-0 **~681 ms → ~88 ms**. Run 1 pays a one-time
`.so` compile in its cache directory; later runs can reuse it when the cache key and
environment match. A kit can include a prebuilt artifact with
`cppyy_kit.cache.prebuild`.
- **Declarations are mandatory for the speedup.** Cling emits any function *body*
  it can see (inline or not), ignoring the `.so` copy, so the fast path must give
  Cling **bodiless declarations** (`decls=`) and let the definitions live only in
  the `.so`. Without `decls` the call safely degrades to a plain `cppyy.cppdef`
  (correct, uncached) and says so once. `extern "C"` and free functions / classes
  with out-of-line methods are the supported subset.
- **Cache the crossing as well as the glue.** The ~0.4 s isn't
  cppyy internals you can intercept. It is the `std::function<Ret(Args)>` thunk +
  the register call wrapper. Build **both in compiled code**: a trampoline whose
  `.so` constructs the `std::function` wrapping the Python callable and does the
  registration, converting the C++ argument to its Python proxy with cppyy's
  public `CPyCppyy::Instance_FromVoidPtr(&obj, "Cpp::Type")` (header under
  `$CONDA_PREFIX/include/pythonX.Y/CPyCppyy/API.h`; link `libcppyy`). Pass the
  Python callable to a `PyObject*` parameter, cppyy hands it across directly.
  `cppdef_cached(..., trampoline=True)` adds the Python + CPyCppyy include and the
  `libcppyy` link. bt's `BT::NodeStatus(BT::TreeNode&)` is the worked example
  (`scripts/cache/validate_cache_bt.py`, the kit-adoption reference).
- **Cache scope:** this caches the glue/trampolines the *kit* authors. cppyy's
  on-demand template member instantiations from arbitrary user calls
  (`node.getInput[T]` for a new `T`) are not cached, they stay JIT unless routed
  through their own cached helper. Artifacts are env-version-tagged + gitignored
  and include source, declarations, compile options, include paths and library
  paths in their identity. Included header contents and compiler/runtime changes
  are not discovered automatically. Clear the compile cache after those changes
  and restart the process. A library that fails to load is rebuilt when possible.
- **Disable the cache for debugging.** To rule the cache out when a kernel
  misbehaves, bypass it so `cppdef_cached` is a plain in-memory `cppyy.cppdef` (no
  `.so` read/write): per call `cppdef_cached(..., cached=False)` / `@cpp(cached=False)`;
  process-wide `cppyy_kit.disable_caching()` (or `with cppyy_kit.caching_disabled():`);
  or the `CPPYY_KIT_NO_CACHE=1` env var. Remove artifacts with
  `cppyy_kit.clear_cache()`.
  The PCH has its own switch (`CPPYY_KIT_NO_AUTOPCH=1`). Full decision tree + artifact
  locations: **FREEZE.md §9, "Debugging: turning the caches off".**

**Kit adoption example.** Both bt_kit and pcl_kit use this pattern;
it is what a new kit (and the `cppyy-accelerate` skill) should apply. Split the glue into
bodiless *declarations* and out-of-line *definitions* (or a trampoline), cache with
`cppdef_cached`, and branch the hot call site on a `_CACHED` flag with a graceful
JIT fallback:

```python
_CACHED = False

_DEFS = r"""                         # compiled into the .so (out-of-line, or a
#include <lib/thing.h>               # PyObject* trampoline that builds the
namespace mykit {                    # std::function + does the call in C++)
  void do_thing(Thing& t, PyObject* fn) { /* ... CPyCppyy::Instance_FromVoidPtr ... */ }
}"""
_DECLS = r"""                        # bodiless: what Cling needs on a cache hit
#include <lib/thing.h>
namespace mykit { void do_thing(Thing&, PyObject*); }"""

def _adopt(prefix):
    global _CACHED
    if os.environ.get("CPPYY_KIT_NO_CACHE") == "1":
        cppyy.cppdef(_FALLBACK_GLUE); _CACHED = False; return
    try:
        cppyy_kit.cppdef_cached(_DEFS, decls=_DECLS, name="mykit_glue",
                                trampoline=True,                     # adds Python+CPyCppyy+libcppyy
                                include_paths=[os.path.join(prefix, "include")],
                                library_paths=[os.path.join(prefix, "lib")],
                                libraries=["thing"])
        _ = cppyy.gbl.mykit.do_thing          # confirm it's callable before committing
        _CACHED = True
    except Exception as exc:                  # no compiler/CPyCppyy -> JIT path + one notice
        _CACHED = False
        cppyy_kit._compile._stderr("[mykit] compile cache unavailable (%s); JIT path." % exc)
        cppyy.cppdef(_FALLBACK_GLUE)

def call_it(t, fn):
    if _CACHED:
        cppyy.gbl.mykit.do_thing(t, fn)       # ~ms; the .so already carries the wrapper
    else:
        with cppyy_kit.first_use("mykit.call_it", "mykit.warmup()"):
            ... the cppyy callback()/template path (warmup-movable) ...
```

Rules that make it safe: the `.so` translation unit must `#include` the library
headers itself (a standalone compile doesn't inherit bringup's includes) and add
`$CONDA_PREFIX/include` for transitive deps (boost etc.); pass caller `include_paths`
so the miss/hit `cppdef` resolves the same headers; keep the cache a pure
optimisation (never a correctness dependency) via the fallback. Worked references:
`bt_kit._adopt_glue` + `scripts/cache/validate_cache_bt.py` (callback trampoline),
`pcl_kit._adopt_glue` (a library template, `VoxelGrid<PointXYZ>`, compiled into
the `.so`).

### 24. Boundary tracer: a typed manifest of every crossing (`cppyy_kit.trace`)
Python crosses into C++ through cppyy_kit. Instrumenting cppyy_kit yields a small, typed record of what a kit app loaded, compiled and
wrapped, with the C++ signatures, counts and timings. `cppyy_kit.trace.start()` /
`stop()` (or `CPPYY_KIT_TRACE=1` before import) turns it on; the crossing points
(`load_libraries`, `cppdef_cached`, `callback`/`std_function`) record automatically.
Tracing is off by default. While disabled, `trace.span(...)` returns a shared no-op
timer without making a timing system call or recording an event.
- **The manifest records the results.** `stop()` returns (and optionally writes) JSON with
  a per-kind summary and an **instantiation manifest**: the distinct C++ signatures
  crossed, sorted by cost, i.e. exactly what a freeze PCH or the compile cache (§23)
  should cover, and data for hotspot analysis with `cppyy-accelerate`. `python -m cppyy_kit trace report trace.json` pretty-prints it.
- **Use it to decide what to cache.** Trace a workload once; the top instantiation
  lines (e.g. `std_function` at ~100 ms for `BT::NodeStatus(BT::TreeNode&)`) name the
  crossings worth routing through a cached trampoline (§23) or baking into the PCH.

### 25. `require()` a header-only library, conda-first, fetch only if unpackaged
`cppyy_kit.require(name, header, url=, sha256=)` registers a header-only C++ library.
It first searches the environment's include paths. If the header is installed,
it registers that directory and returns without downloading anything. If the
header is absent and both `url` and `sha256` are provided, it downloads the archive,
verifies the checksum, unpacks it, and registers the cache directory. Later calls
can reuse the cached headers offline. Supported inputs are a single header, a
`.zip` archive, or a `.tar.gz` archive; use `strip_prefix` to remove an archive's
top-level directory.

Fetched entries are keyed by URL, checksum, header and strip prefix. Downloads
and extraction use staging and a shared lock; complete entries are published
atomically. Offline hits validate the recorded file hashes and can be read from
read-only storage. Different source pins coexist. The include directory is now
`<cache>/<name>/<request hash>/include`; old unverified entries are fetched again.
Unsafe archive paths, links and special entries are rejected. Installed headers
still take priority and are not checked against a requested download checksum.

- **Policy, not convenience:** prefer the conda-forge package for anything on it
  (Eigen, fmt, nlohmann_json). Reach for `url=` only for the unpackaged or an exact
  pinned version, the same version-pinning practice as the §21 vendored-source builds, without
  compiling the source. `require` fetches *sources*; pair it with `cppdef_cached` (§23) when you
  need a compiled `.so`, not just headers.
- **Integrity + reproducibility:** `sha256` is mandatory for a fetch (a mismatch
  raises, the partial download is removed). Point `$CPPYY_KIT_REQUIRE_DIR` at a
  persistent dir (e.g. `~/.cache`) for a machine-wide header cache.

<a id="inline-cpp"></a>

### 26. `@cpp`, write a C++ kernel in Python, compiled + cached + auto-marshaled
For a small hot kernel you'd otherwise hand-write as a `cppdef` helper plus manual
`uintptr_t` marshaling (§6), `cppyy_kit.cpp` does both. The decorated function's
**docstring is the C++ body** (its Python body never runs) and its **annotations
drive marshaling**; on first call it compiles once into a cached `.so`
(`cppdef_cached`, §23) and loads it thereafter.

The numeric annotation API is new in `cppyy-kit` 0.4.0. Published 0.3.x
packages do not support it; use the source checkout until 0.4.0 is published
(see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/#run-repository-demos-or-develop-the-kits)).

```python
import numpy as np
from cppyy_kit.numpy_types import NDArray
from cppyy_kit import cpp

@cpp
def sum_sq(data: NDArray[np.float64]) -> float:
    """
    double s = 0;
    for (std::size_t i = 0; i < data_size; ++i) {
        s += data[i] * data[i];
    }
    return s;
    """

sum_sq(np.array([1, 2, 3], dtype=np.float64))  # 14.0
```
- **Input annotation forms.** `cppyy_kit.numpy_types` groups NumPy typing names:
  `NDArray`, `ArrayLike`, `DTypeLike`, `ConstNDArray`, and supported numeric
  scalar dtype names. For explicit array dtypes in `@cpp`, use `NDArray[T]` for
  writable arrays or `ConstNDArray[T]` for read-only pointers. `ArrayLike` and
  `DTypeLike` are general NumPy hints, not marshaled `@cpp` parameter types. A
  dtype-specific `np.ndarray` annotation works too. Numeric inputs may also be
  annotated as `list[T]`/`typing.List[T]`, homogeneous `tuple[T, ...]`, or
  `Sequence[T]` from `collections.abc`/`typing`. An omitted input annotation or
  bare `np.ndarray` infers a supported numeric type from the value. Use an
  explicit type when the C++ type should be clear at the function definition.
- **Type mapping.** Python `int` maps to the existing C++ `int` (`np.intc`),
  `float` to `double`, `bool` to `bool`, and `complex` to
  `std::complex<double>`. NumPy `float32`/`float64`, fixed-width integers,
  booleans, and `complex64`/`complex128` are supported. Integer sequence values
  are range-checked before conversion to `np.intc`. Sequence values are converted
  according to their annotation: for example, `list[np.float32]` converts Python
  floats to 32-bit values and can round them; a `float` annotation accepts integer
  values and converts them to C++ `double`.
- **Sequence ownership.** Typed numeric sequences are copied into native storage
  on each call and kept alive until the C++ function returns, including through
  `@cpp(nogil=True)`. C++ mutations to that temporary buffer are not copied back
  to the Python list or tuple. Typed empty sequences work; an untyped empty,
  heterogeneous or nested sequence, and an unsupported dtype raise clear errors.
- **Array layout and ownership.** Typed `NDArray` inputs borrow existing storage
  without a copy only when it is native-endian, aligned, and C-contiguous.
  Multidimensional C-contiguous arrays are accepted and exposed as one flat typed
  pointer plus the total element count; shape and strides are not passed to C++.
  Dtype or layout mismatches raise errors; arrays are never silently copied to
  satisfy the annotation. `NDArray[T]` emits a mutable pointer and requires
  writable storage.
  `std::string` arguments remain supported as well.
- **Read-only array input.** For a kernel that only reads its buffer, import
  `ConstNDArray` from `cppyy_kit.numpy_types` and annotate the array as
  `ConstNDArray[T]`. It emits `const T*`, accepts read-only or writable arrays,
  and enforces the same dtype and layout checks without copying:

  ```python
  import numpy as np
  from cppyy_kit import cpp
  from cppyy_kit.numpy_types import ConstNDArray

  @cpp
  def sum_readonly(data: ConstNDArray[np.float64]) -> float:
      """
      double s = 0;
      for (std::size_t i = 0; i < data_size; ++i) {
          s += data[i] * data[i];
      }
      return s;
      """

  values = np.array([1, 2, 3], dtype=np.float64)
  values.flags.writeable = False
  print(sum_readonly(values))  # 14.0
  ```
- **Specialization and returns.** A concrete argument specialization is compiled
  and cached once, then reused across calls, array lengths, and later sessions.
  Keep a return annotation explicit when returning a value; omitting it returns
  `void`.
- **The low-level pointer form remains available.** A verbatim `"T*"` annotation
  takes a NumPy array address or an integer address and hands the body a typed
  pointer (the `reinterpret_cast` is injected). This advanced form does not
  validate a NumPy buffer's dtype or layout. Add a separate length parameter
  when the C++ body needs the element count.
- **Calls follow Python binding rules.** Defaults and keyword arguments work;
  missing, extra, duplicate, or unexpected arguments raise `TypeError` before the
  C++ kernel is compiled. Keyword-only and variadic parameters are rejected when
  decorating the function. For example, a parameter declared `factor: float = 2.0`
  can be omitted or passed by name.
- **It composes with the cache.** With caching enabled, a compatible `@cpp` artifact
  can be reused across runs, avoiding wrapper JIT on a cache hit. Pass
  `@cpp(include_paths=..., libraries=...)` to call into a real library from the body.
- **`@cpp(nogil=True)` releases the GIL around only the compiled body** (the ergonomic
  form of §27), plain Python threads calling the kernel run on N cores. **`cached=False`**
  compiles the kernel in-memory and skips the `.so` cache (the §23 debugging escape
  hatch, also `cppyy_kit.disable_caching()` / `CPPYY_KIT_NO_CACHE=1`; see FREEZE.md
  "Debugging: turning the caches off").
- **Performance depends on the workload.** A hand-written NCC patch tracker measured
  **~12 to 15×** faster in the C++ kernel (4.32 ms vs 66.3 ms/frame at 640×480). For
  per-frame OpenCV library operations (ORB/match/RANSAC), the gap was **~1.1 to 1.2×**
  in orchestration. A `@cpp`/`cppdef` kernel is suited to per-element Python loops
  without a vectorized NumPy or library equivalent, such as custom trackers, cost
  functions, and estimators. Chaining library primitives alone may give little
  benefit when those operations are already implemented in C++.
  For A-vs-B CPU measurements, record `time.process_time()` deltas for sequential,
  single-threaded calls: `cpu% = 100 * Δcpu/Δwall`.

### 27. `nogil()`, release the GIL around a blocking C++ call
§13's rule ("cppyy does not release the GIL on a blocking C++ call") has a fix:
`cppyy_kit.nogil(fn)` runs a **C++** nullary callable through a compiled shim that
drops the GIL around it, so concurrent Python threads run during the call. Measured
(test_nogil.py): a 500 ms C++ sleep called directly lets a
co-thread advance ~1 tick; through `nogil` it advances **~470**, the co-thread runs
the whole time. The shim restores the GIL on both normal return and C++ exception
unwinding before cppyy translates an exception back into Python.
- **The ergonomic front-end: `@cpp(nogil=True)` (§26).** When the C++ you want to run
  GIL-free is a kernel you're writing anyway, skip the `std::function` ceremony, add
  `nogil=True` to `@cpp` and the decorated call releases the GIL around the compiled
  body directly. `@cpp` compiles a small wrapper (in the same cached `.so`) that
  forwards the already-marshaled POD arguments into the kernel while the GIL is
  released, so the GIL is dropped for **only** the C++ body, cppyy's argument/result
  marshaling stays under the lock on either side. An RAII guard restores the GIL on
  normal return and C++ exception unwinding. The wrapper adds ~0.04 µs/call over
  `nogil=False` (a trivial `add`; measured). Plain Python threads calling the kernel
  run on N cores: eight jobs, **7.7× faster**
  than GIL-held on a 16-core box (`examples/parallel_demo`, the front-page snippet).
  Reach for the raw `nogil(fn)` below only for a *pre-existing* C++ callable you did not
  write with `@cpp` (a library's blocking `spin()`/`wait()`).
- **`fn` must be C++, not Python.** A Python callable would re-acquire the GIL to run
  (cppyy takes it to enter Python), so there is no benefit. Bind args/results in C++ (a
  `cppdef`/`@cpp` nullary wrapper writing its result into a C++ object you read
  after). This follows §13's guidance to put blocking work on a C++ path rather than
  a Python thread.
- **`run_async(fn)`** is the asyncio form: `await`s the blocking C++ work on an
  executor thread *with the GIL released*, so the event loop keeps running.
- **Callback caveat:** if `fn` calls back into Python while the GIL is released, that
  callback must re-take the GIL first, a cppyy Python callback does so automatically;
  hand-written C++ touching `PyObject*` under `nogil` must `PyGILState_Ensure()`.
- **Beyond "other threads run": the loop itself jitters less (jitter_bench).** Running
  the *whole* hot loop (wait + compute) in C++ via `nogil`+`cppdef_cached` also tightens
  its *scheduling determinism*. A 1 kHz control loop held its **~2 µs median wakeup
  latency under load** where the equivalent pure-Python loops rose to ~5 µs, and its p99
  under load was the lowest of the four variants tested: the C++ loop never re-enters the
  interpreter between wake and next sleep, so the scheduler sees one long-running C++
  thread rather than a Python thread cycling the interpreter, and load perturbs it less.
  So for a periodic loop `nogil` is not only "let other threads run", it is "the loop
  jitters less." (Full jitter matrix + the bigger unprivileged lever: §35.)

### 28. `.pyi` stubs for a kit's mirror surface (IDE/mypy corridor)
A kit assembles its mirror API at runtime, so editors see nothing. `python -m
cppyy_kit stubgen <module> -o <module>.pyi` emits a static `.pyi` for the module's
**public Python surface**, functions, classes (with methods) and scalar constants,
including names re-exported from submodules, giving name + arity completion and a
mypy corridor. Committed pilots: `cppyy_kit/__init__.pyi`, `bt_kit/bt_kit/__init__.pyi`.
- **Scope:** it stubs the statically knowable *kit* API, not the C++ namespace
  a bringup returns (`cppyy.gbl.BT.*` are dynamic cppyy proxies with no static
  signature, a bringup's return is `Any`). Signatures are names + arity with `Any`
  types (always-valid, loose) rather than guessed C++ types; tighten by hand where a
  kit wants richer hints. Regenerate when the surface changes.

### 29. Capability checks, fallback paths, and status reports
Kits need to check optional features such as OpenCV CUDA support, a compile-cache
compiler, or a frozen PCH. They can report the result and select a fallback path.
`cppyy_kit.capability` provides a shared API:
```python
capability.register("cuda", probe_cuda, "OpenCV built with CUDA")  # probed once, cached
if capability.available("cuda"):        # detect
    gpu_path()
else:
    cpu_path()                          # fallback
print(capability.report())              # introspect (also: python -m cppyy_kit status)
```
- A detect callable returns `bool` or `(bool, detail)`; a raise is caught and recorded
  as unavailable-with-reason (so a probe can't break bringup). `set_state(name, ok,
  detail)` records a capability decided by an *adoption attempt* rather than a probe.
- **Reference adoption:** `bt_kit._adopt_glue` (§23) now asks
  `capability.available("compile_cache")` before attempting the trampoline and
  `set_state("bt_kit.compile_cache", ...)` with the outcome, so `python -m cppyy_kit
  status` shows whether the base capability is available and whether bt_kit used the
  cache. Other kits can use this API for CUDA, lifecycle, and binding checks instead
  of separate `try/except` blocks.

### 30. In-process lifecycle bootstrap: build the node the coupled ctor asks for
Modern ROS 2 cores often take a `rclcpp_lifecycle::LifecycleNode` / a `*ROS` wrapper /
a pluginlib base in their ctor or `configure`, so they *look* like they need the
server. They don't: those objects are **plain classes you construct in-process from
Python**, no lifecycle servers, no manager, no YAML, no action interface. This is the
third instance of the "in-process ROS 2 node/manager" family after moveit_kit's
parameterized `Node` and control_kit's `ControllerManager` (§19). nav2_kit uses the
same approach for lifecycle integration.
- **Construct a `LifecycleNode`.** `make_shared["rclcpp_lifecycle::
  LifecycleNode"](name, ns, NodeOptions)`, then walk `configure()` (UNCONFIGURED→
  INACTIVE) and `activate()` (→ACTIVE); `get_clock()`/`get_logger()` are live
  immediately. `lifecycle_node.hpp` **parses successfully** (no generate_parameter_
  library parsing failure, as in MoveIt's convenience headers). This object fits
  lifecycle-coupled constructors that accept a `LifecycleNode`.
- **A plugin-free `*ROS` wrapper runs in-process too.** `make_shared<Costmap2DROS>(
  NodeOptions with parameter_overrides)` + `configure()` → a blank fillable master grid
  (fill it from NumPy, §6). Its `NodeOptions` ctor names the node and sets
  `is_lifecycle_follower_=false` (a standalone node you drive). Do **not** `activate()`
  unless you want its background update thread.
- **`NodeOptions` auto-declare is a trap for self-declaring nodes.**
  `automatically_declare_parameters_from_overrides(True)` is right for a node that
  declares nothing (it turns your overrides into real params) but **wrong for a node
  that calls `declare_parameter` itself** (`Costmap2DROS`): it double-declares and
  throws `ParameterAlreadyDeclaredException`. Rule: auto-declare only for nodes that
  declare nothing; otherwise pass overrides *without* it and let the node's own
  `declare_parameter(name, default)` pick them up.
- **"The header comments the parameter name" ≠ "the parameter is unused".** RPP's
  `computeVelocityCommands(..., nav2_core::GoalChecker * /*goal_checker*/)` reads as
  unused, but the *definition* dereferences it (`goal_checker->getTolerances()`) →
  `nullptr` crashes. When a coupled API takes an interface pointer, supply a **minimal
  C++ stub subclass** (a `cppdef` `struct : Base`), not `nullptr`, even when the
  signature suggests it is ignored. Check the `.so`, not just the header.
- **Construction and runtime dependencies fail in different ways.** A LifecycleNode
  enabled Smac 2D (`AStarAlgorithm<Node2D>`) planning from Python, but not Hybrid-A\*.
  Hybrid-A\* segfaulted in `precomputeDistanceHeuristic` in about 2 of 3 runs while
  calling OMPL through Cling. `Node2D` is stable because its search does not call
  OMPL at runtime, although its header includes OMPL headers. Constructing an object
  does not establish that all its runtime paths are safe to call.
- **Teardown (§14, applied).** These objects own DDS entities (+ a bond timer); their
  destructors must run **before** `rclcpp` shutdown. `register_teardown` a callback that
  drops each one so it runs *before* `shutdown_rclcpp` (LIFO). Verified: nav2_kit's
  14-test suite and all four planner×controller demo combinations exit 0.

For the authoring guidance in §20, inspect the constructor and `configure` signatures.
Plain data classes such as `Costmap2D(w,h,...)` and `NavFn(nx,ny)` can be used
directly. A `LifecycleNode`, `*ROS` wrapper, or pluginlib base can use this in-process
bootstrap. Remaining limitations come from missing or unstable runtime dependencies.

### 31. Lower the hot virtual: a suitable cppyy target
A framework whose hot loop repeatedly calls a **user-authored virtual** is a suitable
cppyy target. Examples include OMPL's
`StateValidityChecker::isValid` (RRT\* calls it millions of times/solve, §16),
ros2_control's `update` (control_kit), and Crocoddyl's `calc`/`calcDiff` (the DDP
solver calls them per node per iteration plus line-search rollouts).
- **Prototype the virtual in Python** (the binding's supported path), then **lower it
  to an inline-C++ subclass in the *same script*** via `cppdef`, JIT-compiled at
  runtime, so the solver calls native C++ in the hot loop with **no build system**.
- cppyy is the *only* tool that offers the *fast* authoring path without the framework's
  usual "write a CMake project linking the library" rebuild. The framework's own
  workflow is "prototype in Python (slow) or ship a C++ model (needs a build)"; cppyy
  fills the missing "fast **and** no build system, one file" cell.
- **Measured (Crocoddyl, wbc):** the inline-C++ custom action model runs at the
  compiled built-in's speed (**0.32 vs 0.34 ms**) and **~21.7×** the Python-derived
  model, converging to a **bit-identical** cost (250.039320, 8 iters, the numeric
  match is the regression gate). ompl_kit's Python validity checker was ~350 ns/override
  call, 1 to 3 M dispatches/s (§16); the difference is small for small problems and larger when
  the override dominates the loop.
- This uses the L2 native compilation described in [FREEZE.md](FREEZE.md): keep the
  crossing out of the hot loop by putting the *whole* per-iteration virtual in C++.
  Check for versioned pure-virtual changes (§16) and failed `cppdef` declarations (§9) when
  authoring the subclass; probe it out-of-process first.

### 32. A library's own Python binding and cppyy coexist in one process
Many robotics libraries ship their own binding (boost::python, pybind, `cv2`) *and* can
be driven by cppyy. They can run in one process if these rules are followed:
- **Both load the same `.so`; the C++ objects are separate.** The separation of
  labour is **prototype with the library's own binding, lower the hot path with cppyy,
  in one script** (the Crocoddyl example, §31). But a **cppyy-created C++ object cannot be
  handed to a boost::python API** (two proxy runtimes, wbc-verified), so the cppyy path
  must build its own containers/solve in C++ (§6), not feed the binding's objects. Don't
  pass objects *between* the two runtimes.
- **Same build only.** Two loaders of *one* build coexist, `cv2` (the CPU
  `libopencv`) and a cppyy-loaded CPU OpenCV share the same `.so`, no corruption
  (webcam). The hazard is **mixing two builds of the same soname** in one process (a
  CUDA `libopencv` alongside the CPU one corrupts it): a GPU-vs-CPU comparison must be
  single-pipeline or two processes, never a same-process A-vs-B.

### 33. Schema-derived C++ structs, validate at the boundary, compute in C++
*(Design + probe RFC; prototype `cppyy_kit/pydantic_structs.py`. Numbers below measured
on cppyy 3.5.0 / pydantic 2.13.4, 1M `Detection`.)*

You already describe your data with a Pydantic v2 model for edge validation, that
schema *is* a struct layout. `pydantic_structs` emits the equivalent C++ `struct`
(compiled + cached), so the same data lives as a `std::vector<Struct>` instead of a
`list` of model instances: compact, typed, and zero-copy-viewable as NumPy on its
numeric columns. Validate input with Pydantic, compute in C++, then validate the result with Pydantic. `to_model()` re-runs the validators so the
C++ excursion can't silently violate the model.
- **A struct is a *parse* cost, not a call-wrapper-JIT cost, so cache the kernels, not
  the struct.** A struct is a type *declaration*; cppyy learns its layout by parsing it
  once per process (~7 ms for a small set, the domain of the freeze PCH, §2/L1), and
  there is no function body to compile into a `.so`. What genuinely recurs is (a) the
  `std::vector<Struct>` template first-use JIT (~46 ms) and (b) the **consumer kernels +
  marshaling glue**, which have function bodies and can be persisted with `cppdef_cached` (§23).
- **Compact storage:** `list[Detection]` (Pydantic instances) **1112 MB** →
  `std::vector<Struct>` **70 MB** (16× smaller); numpy columns 49 MB.
- **Compute: NumPy remains faster for flat reductions.** For `sum(score)` (a pure
  contiguous reduction), NumPy is **136×** faster than Python; the struct loop is
  12× faster (it walks the AoS with a 64-B stride). For pure columnar numeric
  reductions, use NumPy. The zero-copy column view can provide the input. For fused,
  branch-heavy logic, the C++ struct kernel is faster: a `score>0.5` filter and
  centroid is 7× faster than Python, compared with NumPy's 3×. NumPy's mask and gather
  allocate intermediate arrays; the C++ loop handles both operations in one pass.
  The struct also keeps the model's nested and mixed fields, which a flat array cannot
  represent.
- **"Free" type checks:** consumer kernels compile *against* the struct, so a misused
  field is a Cling compile error that names it (`no member named 'scoree' … did you mean
  'score'?`; `invalid operands ('double' and 'std::string')`). Run that check
  **out of process** (`probe_cppdef`) because a failed `cppdef` can contaminate the live
  interpreter (§9).
- **Crossing traps:** a `std::string` inside a returned `std::vector<std::string>`
  crosses as **`bytes`**, `to_model()` must `.decode()` string fields (§11). The
  zero-copy numeric column view is **strided/non-contiguous** (stride = `sizeof(Struct)`),
  a read/mutate-in-place convenience, not a free numpy pipeline; contiguous SoA columns
  are just NumPy. The view aliases the vector's buffer, so the vector must outlive it and
  any `resize`/`push_back` invalidates it (`column()` pins the vector on its ctypes
  backing buffer via `keep_alive`, which raises if pinning fails).

Use this approach when you already maintain Pydantic models and need compact hot-path computation.
It does not replace a wire format or NumPy.

### 34. Hybrid pipelines: a Python ML front end, a cppyy hot path, two envs
A realistic robotics pipeline mixes a Python ML library (its inference *is* a library
primitive, don't wrap it, as §26's benchmark comparison shows) with a cppyy_kit hot path, and the
two halves can have **incompatible native dependencies**. The retarget capture rig was the
worked example: MediaPipe perception feeds a pinocchio retarget solve, and the two halves
were originally split across two envs because the ROS stack pinned libboost 1.90 and pinocchio's conda stack pinned 1.86.

> **Dated correction (2026-07-12):** that *specific* clash dissolved, conda-forge rebuilt
> pinocchio 4.x against libboost 1.90, so pinocchio now co-solves with the robostack ROS
> stack in one `solve-group`, and the retarget half runs in a ROS-capable env consuming the
> landmark frames straight off `/tf` (rclcpp_kit's C++ listener). The **two-env pattern
> below remains the general rule** for any incompatible pair. It is no
> longer required for this pinocchio+ROS case. Note this is the *solve/ABI* boundary only -
> the Cling header-parse limitation on `pinocchio::Model` (§9, the 25-type `boost::variant`)
> is **unchanged**: it trips on boost 1.90 too, so the IK solve stays a bindings job either
> way.

The pattern for building such a system (still valid whenever two halves truly can't share):
- **Split at the env boundary; couple with a replayable stream.** When a hard dependency
  conflict forces two processes, make the seam a **tailable/replayable file** (here a
  JSONL landmark stream): live coupling = tail it, CI/rehearsal = replay it, so the same
  live code path runs headless. Record/replay is a design stance from day one, not a mode
  bolted on. Give the stream a `format` tag and **refuse a mismatched/renamed tag** with
  an error naming both values (a stale recording is a clear failure, not a silent
  mismap). The shared **coordinate-frame/contract module** imports only stdlib+numpy so it
  loads in both environments.
- **The first pip dependency in a conda/pixi repo needs discipline.** Put it in a
  dedicated feature env with a `[pypi-dependencies]` section (keep the ROS-free base
  minimal). Two rules that avoid an ABI split: **verify the pip deps' numpy equals the
  conda numpy** (mediapipe brought numpy 2.5.1, matching conda's, no split), and
  **exclude any conda package the pip dep re-provides** (do not compose a conda `opencv`
  with mediapipe's pip `opencv-contrib-python`). Compose with the ROS default via
  `solve-group="default"` so the shared stack stays one solve. Pin ML model bundles by
  URL **and SHA-256** (verify-after-download, refuse a mismatch), the same supply-chain
  hygiene as §25's `require(..., sha256=)`.
- **Measured speedups came from the glue code.** In this rig they were
  the /tf marshaling (§6 build-once-refill, 265×) and the per-frame retarget glue kernel
  (coord transform + target map + a sequential One-Euro filter in one `cppdef` pass,
  **303.8×**, bit-identical), both §6/§26, both with numeric-agreement checks. The IK
  *solve* stays a pinocchio-bindings job (§9's `Model` limitation); record this blocked
  integration with the specific cause.

### 35. Low-jitter timed loops from Python: tune timer slack
A control/HIL loop *orchestrated from Python* can hit a µs-scale period median on a
**stock (non-PREEMPT_RT) kernel** with only unprivileged tuning, the orchestration
language is not what sets the median.
- **`prctl(PR_SET_TIMERSLACK, 1)` is the effective unprivileged option.** Linux' default timer slack is
  **50 µs**, the kernel may defer any `clock_nanosleep`/`futex`/`poll` wakeup by up to
  that to batch wakeups, and at 1 kHz that slack *is* the median wakeup latency. One
  unprivileged `prctl` call drops the median from **~52 µs → ~2.4 µs (~22×)**; the removed
  ~52 µs was timer slack, not Python overhead. Set it once at loop start.
- **Then `mlockall` + CPU pinning + `clock_nanosleep(TIMER_ABSTIME)`.** With slack tuned,
  a bare-Python loop, a cppyy_kit C++ loop, and a real ros2_control loop all sit at
  **p50 ~2 µs / 1000.0 Hz / <1 % late cycles** idle. `clock_nanosleep` beats
  deadline-corrected `time.sleep` (thinner tail). Driving a *real* ros2_control
  `read→update→write` from Python (cross-inherited PD controller) adds negligible median
  jitter over a bare timer loop.
- Under load, the difference appears in the tail rather than the median. A `nogil`+`cppdef_cached`
  C++ loop (§27) keeps its ~2 µs median under load where pure-Python loops rise to ~5 µs.
- **The tail is a scheduling problem, not a Python problem.** Idle p99.9 ≈ 2 ms and rare
  multi-hundred-ms spikes on a busy shared machine are CFS preemption on a non-isolated
  core, collapsed by privileged tuning (`SCHED_FIFO` + `preempt=full` +
  `isolcpus`/`nohz_full`/`rcu_nocbs`), and only *bounded* under adversarial load by
  `CONFIG_PREEMPT_RT`; the stock kernel already ships every soft-RT primitive. The measurements support
  soft real-time (prototyping / HIL / sim / teleop) from Python now; hard-RT is a tuning
  path on the same kernel, and the graduation to a native `update()` (§31) is unchanged.

### 36. Automatic PCH setup
The Cling PCH that reduces a kit's header-parse cost (FREEZE.md) can be built manually
or through `cppyy_kit.autopch`. Auto-PCH schedules a build on first use and stores it
under `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch`. A compatible artifact can be loaded
in later processes when the startup hook is installed and the cache is enabled.

Importing `cppyy_kit`, printing packaged guides and inspecting an environment
do not initialize Cling or install a startup hook. Native setup runs when a C++
helper first needs it. Register native resource cleanup after constructing the
resource; runtime setup then preserves cleanup-before-Cling teardown order.

- **Startup activation uses a `.pth` file.** In environments that process the hook,
  it runs before user imports. The `.pth` calls a standalone bootstrap
  (`cppyy_kit._autopch_boot`,
  stdlib-only, never-raising) that binds `CLING_STANDARD_PCH` from the env's manifest.
  cppyy_kit self-installs it on first import (`python -m cppyy_kit.autopch --uninstall`
  to remove). This matters: `import cppyy` early in a program sets `CLING_STANDARD_PCH`
  to cppyy's *own* std PCH, so an in-process `setup()` after that import is too
  late. The startup hook sets the PCH before a later user import of cppyy. cppyy_kit's
  own import then prints the one `Cling PCH loaded from …` line from a marker the `.pth`
  set (a print at every `python` start would be noise).
- **A kit registers the headers it parses**, once, at bringup:
  `cppyy_kit.register_pch_headers(headers, include_paths=..., force_symbols=None)`. Warm
  run whose PCH already bakes them → cheap no-op; otherwise the set is folded into the
  env manifest and a **detached background build** runs at exit (lockfile-guarded, atomic
  write), so the next run loads it. `force_symbols` is the §1-FREEZE option for
  internal-linkage statics, applied only on the warm path (the JIT parse defines them
  otherwise); rclcpp needs none.
- **Keys invalidate naturally; the cache self-prunes.** The `.pch` filename hashes the
  env prefix + cppyy versions + the baked header set; a rebuilt env or upgraded cppyy is a
  cache miss (fall back to JIT), never a silent ABI mismatch. After each build the cache
  is trimmed to the newest few artifacts per environment (keeping any a live manifest
  references) so artifacts from many environments do not accumulate. Opt out with
  `CPPYY_KIT_NO_AUTOPCH=1` (or `python -m cppyy_kit.autopch --prune` / `--uninstall`);
  never committed. When debugging, this is the PCH's "off" switch, the compile cache
  (§23) has its own; see **FREEZE.md §9, "Debugging: turning the caches off"** for the
  full behavior.
- **Characterized (rclcpp, one shared host):** for `bringup_rclcpp()`, with a fresh
  cache directory and the background build complete between first and warm run, the
  `rclcpp C++ headers loaded (…)` line was ~1.9 s cold and ~0.0 s warm, while the
  whole call was ~1.9 s and ~0.06 s (an observed ~30× ratio). This verifies cache
  pickup, including when the program imports `cppyy` before `cppyy_kit`. It removes the **parse** only (cppyy's
  first-use call-wrapper JIT is the separate §23 cost).

### 37. Cache the subscription template instantiation (`rclcpp_kit.subscription_cache`)
Creating an rclcpp subscription from Python makes cppyy JIT-instantiate
`rclcpp::create_subscription<MsgT>` on first use, per message type. The shared-host
characterization observed ~2.8 s for `sensor_msgs::msg::Image`; the PCH (§36) does not
touch it (that removes the header *parse*, not template instantiation). This is the
§23 compile cache applied to a template cppyy instantiates on your behalf: a tiny
trampoline that calls
`create_subscription<MsgT>` is compiled once into a `.so` per type (the template is
instantiated at compile time), then `load_library`'d thereafter.
- **A miss retains the plain path.** On a cache miss the rclpy-style
  `node.create_subscription(MsgType, topic, cb, qos)` uses the plain template call for
  that run, and the `.so` is compiled in a detached
  background process at interpreter exit; the next run loads it. The trampoline is used
  only when its `.so` exists, and any failure falls back to the plain call. The cache is
  an optional startup optimization, never a correctness dependency (verified: the
  pub/sub roundtrip suite passes on both the plain and cached paths).
- **Machine-persistent, cwd-independent.** Artifacts live under
  `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/subs/<version-tag>` (not the compile cache's
  default `<cwd>/build`), so a CLI run from any directory reuses them; env-version-tagged
  like the other caches. Opt out with `RCLCPP_KIT_NO_SUB_CACHE=1`.
- **Characterized (rclcpp, Image, PCH warm, one shared host).** Time-to-ready for
  `import rclcpp_kit; bringup_rclcpp(); node.create_subscription(Image, …)` was
  ~3.26 s without the `.so` and ~0.56 s with it; the `create_subscription` call itself
  was ~2.9 s and ~0.22 s. The ~0.14 s residual in that call is cppyy's per-signature
  `std::function` thunk (the Python→C++ callable wrapper), the same non-cacheable boundary
  as the §23 residual, generated at the call from Python, not carried in the `.so`.
  These values were measured on the stated host and environment.

---

## L0 and L1 status

**Today (L0, JIT):** everything above runs by JIT-compiling the library's headers
at bringup, a one-time, idempotent per-process cost (bt ~0.9 s, pcl ~1.3 s).
Correct and fast at steady state; the only downside is startup latency.

**L1 uses a Cling PCH.** This is now applied
**automatically** by the zero-config auto-PCH (§36 above); the mechanism below is what
it wraps and remains the manual path for explicit control. The full recipe, artifact
lifecycle, numbers and limitations live in [FREEZE.md](FREEZE.md); the short version:

- The bringup cost is ~89 % header JIT-parse. A `rootcling`/`genreflex` dictionary
  does **not** help. It supplies reflection/autoload metadata, not a parsed AST,
  so Cling still lazily re-parses on first class use (measured ~0.8 s).
- Use the mechanism **cppyy already uses for its own std headers:** a
  **Cling precompiled header**. Build a PCH that bakes the kit's headers on top of
  cppyy's std set (`rootcling -generate-pch`, reusing `etc/dictpch/makepch.py`'s
  command), and point `CLING_STANDARD_PCH` at it. Cling materialises the header AST
  from the PCH at interpreter start, so `cppyy.include(...)` becomes a ~6 ms lookup
  instead of a ~0.9 s parse. **Measured: `include(bt_factory.h)` ~890 ms → ~6 ms
  (~140×); bringup total ~950 ms → ~90 ms (~10.7×).** Same 16-test suite green on
  the frozen path (`pixi run -e bt test-bt-frozen`).
- **Two rules make it real.** (1) `CLING_STANDARD_PCH` must be set *before the
  first `import cppyy`* (Cling binds its PCH at interpreter init; `import rclcppyy`
  imports cppyy transitively), so activation is via a launcher that sets the env
  and `exec`s the target (`scripts/freeze/run_frozen.py`). (2) The AST-only PCH
  doesn't emit the header's *internal-linkage statics* (bt: `BT::UndefinedAnyType`)
  and the library's copy is a non-exported local symbol, so on the frozen path the
  kit emits one strong definition under the exact mangled name; applied only when
  frozen (in L0 the live parse already defines it).
- **What freezing does NOT remove:** the first-use JIT of cppyy's per-signature
  call wrappers (`registerSimpleAction`'s `std::function` thunk etc., ~0.7 s for
  t01, unchanged L0↔L1). A header PCH only removes the parse step. Cutting the first-use
  JIT is a separate step (L2 native lowering, or caching the instantiations).
- **rclcpp measurement:** on the same host, the PCH reduced the measured
  `rclcpp/rclcpp.hpp` include time from ~1.71 s to ~6 ms. This verifies that the mechanism
  loads a non-BT header set from a PCH.

**What a kit should do now:** make bringup idempotent and staged, and register its
headers once via `cppyy_kit.register_pch_headers(...)` (§36) so the zero-config
auto-PCH removes the header parse automatically on the second run. No per-kit PCH build or launcher is needed. The only manual residue is the occasional `force_symbols`
entry when a freeze surfaces an unresolved internal-linkage static (§1, FREEZE.md);
the explicit `freeze-<kit>-build` path stays available for CI or full control.

---

*Evidence lives in the per-kit reports: `docs/bt_kit/REPORT.md` (capability matrix,
deep-pass results, AOT probe) and `docs/pcl_kit/REPORT.md` (copy accounting,
benchmark). This document describes patterns that apply across libraries.*

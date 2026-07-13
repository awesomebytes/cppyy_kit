# cppyy_kit — common patterns for driving a C++ library from Python

This is the shared playbook behind the cppyy_kit suite (`bt_kit` for
BehaviorTree.CPP, `pcl_kit` for the Point Cloud Library, and eight more). A kit
wraps a C++ library so it can be driven from short Python that **mirrors the
library's own API**, hiding only the cppyy friction. That friction is the same from
one library to the next, so it is factored into `cppyy_kit`; this document is the
narrative and the evidence behind it, for the next kit author (human or LLM).

Two independent kits confirm every pattern below: BehaviorTree.CPP (a callback /
tree engine) and PCL (templated bulk-data algorithms) stress different edges, and
the union is what a general `cppyy_kit` must cover.

## The three-ingredient recipe

Every kit is the same three moves:

1. **Bringup** — locate the install, add include paths, JIT-include the headers,
   and `load_library` the `.so` set.
2. **Hide that library's cppyy sharp edges** — the traps below (containers,
   ownership, lifetime, template attributes).
3. **Mirror the library's own API** — expose the real class/method names so a
   user's (or an LLM's training on the) library transfers 1:1; no DSL, no hidden
   state.

`bringup_bt()` and `bringup_pcl()` are the same shape; the leaf/algorithm calls
after bringup are the library's own (`factory.registerSimpleAction`,
`pcl.VoxelGrid[pcl.PointXYZ]`).

---

## Pattern catalog

### 1. Bringup: `load_library` is mandatory; `add_library_path` is not enough
cppyy resolves a symbol by finding its **owning `.so` at call time** by scanning
its own library search path. Adding a path is not enough — every library you call
into must be `load_library`'d by soname. `cppyy_kit.load_libraries(sonames,
search_paths)` centralizes this.
- **bt:** one lib — `libbehaviortree_cpp.so`.
- **pcl:** a set — `libpcl_common/octree/kdtree/search/sample_consensus/filters`
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
- **ompl:** ~538 ms — *lower than bt or pcl despite pulling in boost*. The cost
  tracks the **transitively-included header stack**, not the library's "size" or
  reputation: ompl+boost (538 ms) < bt.CPP (0.9 s) < pcl (1.3 s). Measure the
  actual `include(...)` before assuming a big library means a slow bringup.

### 3. Crossing a Python function **into** C++ (`callback`)
Hand a Python callable to C++ in one line — `cppyy_kit.callback(fn)` — with the
signature inferred and the lifetime pinned for you:
```python
def on_value(x: int, y: float) -> bool:      # hints -> "bool(int, double)"
    return x > y
fn = cppyy_kit.callback(on_value)            # ready to pass to any C++ std::function slot
```
- **Inference** maps `int`->int, `float`->double, `bool`->bool, `str`->std::string,
  `None`->void (return), and any cppyy C++ class (via `__cpp_name__`) as a
  reference. Parameters with Python defaults / `*args` are ignored (not C++ args).
- **A class hint can only infer `T&`** — but cppyy will bind a
  `std::function<...(T&)>` even where the API wants `const T*` (ompl's
  `setStateValidityChecker`), and the mismatch then fails *later* at the call site.
  callback **warns once** naming the fix. For the exact form, annotate the
  parameter with the **C++ type string**, used verbatim:
  `def check(s: "const ompl::base::State*") -> bool: ...` →
  `bool(const ompl::base::State*)`. (flake8/pyflakes flags such a string annotation
  as `F722`, a forward-ref false positive — add `# noqa: F722`, or use
  `signature=` instead.)
- **Explicit `signature=` wins** for anything, e.g.
  `cppyy_kit.callback(tick, signature="BT::NodeStatus(BT::TreeNode&)",
  owner=factory)` — exactly how bt_kit registers leaf/stateful hooks and ompl_kit
  fixes the validity-checker pointer form.
- **Threading:** the callback runs in whatever C++ thread invokes it (cppyy takes
  the GIL); a single-threaded driver (a tick loop, a `spin_some`) never contends.
- `cppyy_kit.std_function(sig, fn)` is the low-level escape hatch (raw wrapper, you
  handle lifetime yourself); prefer `callback`.

### 4. Lifetime: the "callable was deleted" footgun (now handled)
cppyy does **not** keep a Python callable (nor its `std::function` wrapper, nor a
buffer backing a view) alive just because C++ holds it — a collected callback
raises `TypeError: callable was deleted` when fired. This bit us for real: a
throwaway `lambda` handed to the raw `std_function` was collected before the call.
- **`callback()` makes it impossible to hit silently:** it always pins. With
  `owner=` the wrapper + fn live as long as that object; without `owner=` they are
  pinned in a module-level registry for the process lifetime
  (`cppyy_kit.release_callbacks()` drops those when you're sure C++ is done).
- For **non-callback** objects (a buffer backing a zero-copy view, a logger),
  `cppyy_kit.keep_alive(owner, *objs)` is the primitive. **pcl:** the source cloud
  is pinned on the ctypes buffer backing a NumPy view so it can't outlive its
  storage. **bt:** leaf callbacks are pinned on the factory (via `callback(owner=)`)
  and carried onto the tree.

### C++ → Python direction (no helper needed)
The reverse crossing is already one line, so it is documented, not wrapped
(mirror-don't-sugar):
```python
cppyy.gbl.mylib.some_fn(21)          # a cppdef'd/loaded C++ function IS a Python callable
holder.store(cppyy.gbl.mylib.some_fn)  # and can be handed straight back into a C++ API
```
Round-trip works too: a Python `callback()` stored in a C++ `std::function`,
invoked from C++, can call back into further Python — verified in
`test/test_cppyy_kit.py`.

### 5. Crossing objects **out** without crossing ownership (`HandleRegistry`)
Returning a `std::unique_ptr<T>` **from** a Python `std::function` fails
(`C++ type cannot be converted to memory`). To let C++ create per-instance Python
state, keep the ownership-creating lambda entirely in C++ and have it call a
Python **builder that returns an integer handle**; dispatch later callbacks by
that handle. `cppyy_kit.HandleRegistry` is the table.
- **bt:** the per-tree-node stateful builder — C++ builds each node, calls the
  Python builder (→ handle), and the shim's onStart/onRunning/onHalted dispatch by
  handle. This is what makes two nodes of the same registered ID keep independent
  state.

### 6. Containers & bulk data: build in C++, pass raw addresses
Constructing/inserting STL containers **from Python** can **SIGSEGV with no
traceback** (cppyy's `MapFromPairs` on a map, aligned-storage construction). Keep
all container/buffer work inside a `cppyy.cppdef` helper.
- **bt:** the `PortsList` (`unordered_map<string,PortInfo>`) is built in C++ from
  two **parallel `vector<string>`** (names, types) — passing a `vector<pair>` or
  the map itself from Python is what crashes.
- **pcl:** NumPy↔cloud copies are a `cppdef` helper taking `reinterpret_cast`-able
  integer addresses (`arr.ctypes.data` as `uintptr_t`) and doing the
  `memcpy`/strided copy in C++ — a per-element Python loop is ~90x slower.
- **nav2 (3rd instance):** the `unsigned char*` costmap buffer, same recipe
  (address as `uintptr_t`, `memcpy` in `cppdef`), ~600–3600× a Python loop.
  Output-by-pointer-array (NavFn's `getPathX()/getPathY()` `float*` + length) is
  the same "keep it in C++": one helper that `memcpy`s the outputs, not marshalling
  C arrays across the boundary.
- **Build the message once, refill per frame (retarget).** For a ROS message
  re-published every cycle, construct it **once** in C++ (fixed structure — e.g. a
  `TFMessage`'s 75 frame names) and each cycle refill only its numeric fields from one
  flat address, rather than reconstructing the message's proxies field-by-field in
  Python. Measured **265×** for a 75-frame `/tf` message (0.0005 vs 0.144 ms/message);
  the reuse is only possible because the message lives in C++ — a Python broadcaster
  typically rebuilds per frame.
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
  **template ctor with a universal-reference default** — `Cls(NodeT&& node =
  NodeT())` reports "class has no public constructors" from Python (tf). One-line
  `cppdef` factory returning the object sidesteps all three.

### 7. On-demand templates: include the `impl` headers
A precompiled `.so` only carries the specializations its authors compiled. To let
Cling instantiate `Template<UserType>` at JIT time, include the library's
`impl/*.hpp` (or `*.hxx`). This is the difference between a fixed-surface binding
and cppyy — **any** type works on demand.
- **pcl:** including `pcl/impl/pcl_base.hpp` + `filters/impl/voxel_grid.hpp` lets
  `PointCloud<T>` / `VoxelGrid<T>` instantiate for point types no binding shipped.
- **bt:** template **member** calls work directly — `node.getInput[T](key)` from
  Python; unwrap the returned `Expected<T>` with `has_value()`/`value()`
  (`cppyy_kit.unwrap_expected`).
- **vision:** a **dependent-type** template member (a templated member accessed on a
  value whose type depends on a template parameter, inside a patched header) needs
  the explicit `.template` disambiguator — `obj.member.template ptr<T>()` — the
  clang two-phase-lookup requirement; not needed when the same call is on a concrete
  type.

### 8. Call C++ function templates directly; let cppyy deduce
When a template argument can be deduced from a runtime argument, call the function
straight from Python — no explicit `[T]`, no wrapper.
- **pcl:** `pcl.toROSMsg(cloud, msg)` / `pcl.fromROSMsg(msg, cloud)` deduce
  `PointT` from the cloud. Reach for a `cppdef` helper only when a template arg
  can't be deduced or ownership must not cross (Pattern 5).

### 9. Cling is an older clang: attributes & failed-`cppdef` crashes
Cling's parser trips on some modern constructs, and a **failed `cppdef` can crash
during transaction revert** (no Python traceback).
- **pcl:** a trailing type-attribute (`struct { ... } EIGEN_ALIGN16;`) parse-errors
  — use a prefix form (`struct alignas(16) X`). Custom point types must be declared
  that way.
- **A failed `cppyy.include` contaminates the interpreter too, not just `cppdef`
  (nav2).** When one header failed mid-parse (a missing transitive dep), the *next,
  unrelated* `cppyy.include` in the same process failed spuriously — though it
  includes cleanly in a fresh process. So probe a **risky include** (heavy/uncertain
  transitive deps) out-of-process, exactly as for `cppdef`.
- **`generate_parameter_library` headers SIGSEGV the Cling parser (moveit).** Any
  `*_parameters.hpp` (fmt + rsl + validators) crashes on include — and modern ROS 2
  packages all generate one. Never `cppyy.include` it; find the *clean* base header
  and load the class/plugin directly. (ros2_control's headers happen to be
  Cling-clean — this wall is per-package, so probe.)
- **Confirmed a *parse*-wall only — the compiled artifact is fine (ik_bench).**
  pick_ik is the positive control: a `generate_parameter_library`-heavy MoveIt IK
  plugin whose generated `*_parameters.hpp` is exactly the SIGSEGV header, yet it
  **builds and `dlopen`s cleanly** — because its own CMake compiles the g_p_l code into
  `libpick_ik_plugin.so` and pluginlib loads the finished `.so`; nothing ever
  `cppyy.include`s a pick_ik header. Rule stands: load the plugin, never parse its
  params header (§19).
- **The ORC static-initializer wall: parse succeeds, *execution* fails (vision/gtsam).**
  A header can `cppyy.include` cleanly yet the first use fault later, when Cling's ORC
  JIT must materialize a **namespace-scope internal-linkage static** it can't emit —
  gtsam's `static const KeyFormatter DefaultKeyFormatter` (`Key.h`, a non-exported
  `std::function` global; each TU emits its own init). No dependency change fixes it
  (it is a Cling limitation, not a missing dep). This is a distinct failure *stage*
  from a parse error — worth suspecting when includes pass but a symbol won't
  materialize; the honest fallback is §20 (the library's own Python binding for batch
  steps).
- **`boost::variant` template-arity is an env-version parse wall (wbc/pinocchio).**
  A big `boost::variant` that a **precompiled** `.so` carries fine can be
  **un-JIT-able from headers** under a newer boost whose preprocessed arity limit it
  exceeds. pinocchio's `JointModelVariant` is a **25-type `boost::variant`**;
  re-instantiating `ModelTpl<Scalar>` for a new scalar hits **boost 1.90**'s
  `make_variant_list` limit (`wrong number of template arguments (25, should be at
  least 0)`) — while the shipped `double`/casadi libraries sidestep it by being
  precompiled. This is a **parse** failure, distinct from the ORC static-init wall
  (an *execution* failure). Reproduce with `g++` to confirm it is not a Cling quirk;
  suspect it when a template *class* re-instantiation for a new parameter fails but
  the shipped specialization works. **2nd instance (retarget): it also blocks the
  default-`double` `Model`, not just exotic scalars** — instantiating
  `pinocchio::Model` from headers at all (a URDF parse, FK on a real robot, a crocoddyl
  `StateMultibody`) trips the same `make_variant_list` limit, a clean compile error at
  `JointModelTpl<double>` (probed out-of-process, not a crash). So drive pinocchio's
  rigid-body **multibody core via its Python bindings**; cppyy's win for this stack is
  the abstract/custom-model path (crocoddyl action models, §31) and *non*-pinocchio glue
  kernels, not the `Model` itself.
- **Mitigation:** probe risky glue out-of-process first —
  `cppyy_kit.probe_cppdef(code, include_paths=, headers=, libraries=)` compiles it
  in a throwaway subprocess and returns `(ok, message)` without risking the main
  interpreter. Pass it the **full ament include-path set** (every package's include
  dir, via `get_packages_with_prefixes`), not just the target library's — else a
  header that transitively pulls the ROS message tree fails on a missing transitive
  header (a false negative).

### 10. Error ergonomics: strip the signature wall
cppyy prefixes a C++ exception with the mangled call signature and ` => `. Split
on ` => ` and collapse whitespace for a readable one-line message, re-raised as a
kit exception (`cppyy_kit.pretty_cpp_error`, `CppyyKitError`).
- **bt:** `BtXmlError` turns the `createTreeFromText(...) =>` wall into
  `RuntimeError: Error at line 4: -> Node not recognized: Nope`.

### 11. Values that don't cross as ints: enums, `unsigned char`, macros
C++ enums behave like their int values across the boundary
(`BT.NodeStatus.SUCCESS == 2`). Expose plain-int constants for convenience while
keeping the real enum available (`bt_kit.SUCCESS` and `bt.NodeStatus.SUCCESS`).
But three neighbours are silent traps:
- **`unsigned char` (and `uint8_t`-backed `enum class`) crosses as a length-1
  Python `str`, not an int** (nav2, control). `Costmap2D::getCost()` and its
  `static constexpr unsigned char` cost constants come back as `'\xfe'`, and
  `'\xfe' == 254` is `False`. Read with `ord(...)`, and expose **plain-int**
  constants from the kit. (The enum *member* is still an int-able proxy; it's a
  *returned value / struct-member read* of the uint8 type that becomes a `str`.)
- **A `using`-alias of an enum resolves to plain Python `int`** (control) — losing
  the enum-ness. Reference the **real nested enum type** (`Outer::Inner::Enum`), not
  the alias.
- **Type-constant `#define` macros are invisible to cppyy** (vision: `CV_8UC1`,
  `CV_8U`). Re-expose the few you need as real `const int` in a `cppdef` block.
- **A `std::string` inside a returned `std::vector<std::string>` can surface as
  Python `bytes`, not `str`** (tf: `getAllFrameNames()`). Decode at the kit
  boundary (`b.decode()`).

### 12. Mirror, don't sugar
Patch/return the library's real classes so methods keep their C++ names (add
snake_case aliases). A bespoke DSL was prototyped for bt_kit and **rejected**: it
needed a module-global registry (a footgun across trees/re-imports/tests) and
forced knowledge that doesn't transfer. Both kits ship the mirror.

### 13. GIL / concurrency (what "parallel" means)
Kit callbacks run in the **calling C++ thread**. A single-threaded engine (a tick
loop, a spin) never contends.
- **bt:** `ParallelNode` is cooperative bookkeeping, not OS threads — Python leaves
  under it run sequentially (no true parallelism, no contention). A leaf that
  sleeps / does I/O releases the GIL, so spinning the tree from a background thread
  does not deadlock; a busy-blocking leaf would. Don't expose `ThreadedAction`
  (real C++ worker thread) without explicit GIL handling.
- **cppyy does NOT release the GIL on a blocking C++ call (control, measured).** So
  a blocking C++ call cannot be overlapped with Python work by putting it on a
  *Python* thread — it holds the GIL the whole time. Run it on a **C++** thread
  instead: a plain-function `std::thread` in a `cppdef` helper (note `std::async`
  does **not** JIT in Cling — use `std::thread`). control_kit's blocking
  controller-switch does exactly this.
- **The efficiency face — "let C++ own the loop; cross only on demand" (tf).** The
  flip side of the GIL rule is a *design win*: a library that already spins its own
  C++ thread is an **ideal** cppyy target. `tf2_ros::TransformListener(spin_thread=
  true)` ingests `/tf` on its own `std::thread`, entirely off the GIL; Python only
  crosses on `lookup`. Measured **~7–14× less ingest CPU** than an equivalent Python
  listener (whose callback runs under the GIL), and the win **compounds with
  traffic**. Prefer wrapping the library's own loop over re-implementing it in a
  Python thread.

### 14. Teardown: release global-state C++ objects before Python finalizes
A cppyy process ends by running two teardown mechanisms with **no ordering
contract** between them: Python finalization (which drops the last references to
cppyy-proxied C++ objects, running their destructors) and cppyy's own atexit hook
(which tears down Cling / the JIT). A C++ object that owns **process-global or
static state** — an `rclcpp` Context and the DDS participant / background threads
it owns, a ZMQ-backed BT logger — is the hazard: if its destructor runs after
Cling is gone (or a DDS thread touches freed state), the process can **SIGSEGV
with no Python traceback**, *after* all useful work is done.
- **History / evidence:** rclcppyy scripts long papered over this with
  `os._exit(0)` right after printing their results — a hard exit skips every
  destructor, so the return code is deterministic but nothing is actually cleaned
  up. A root-cause pass on the current stack (cppyy 3.5, ROS Jazzy, cyclonedds
  *and* fastrtps) could **not reproduce a crash** in ~8 scenarios — plain
  pub/sub, the bt+rclcpp mixed tree, the pcl+rclcpp pipeline, module-global entity
  lifetimes, and single/double explicit `rclcpp::shutdown()` — so the dodges were
  vestigial (present since each file's first commit). The hazard is nonetheless
  real in principle, so the fix makes teardown **ordered and explicit** instead of
  relying on the accident that end-of-`main` locals get RAII-released while the
  interpreter is still healthy.
- **Fix:** `cppyy_kit.register_teardown(cb)` + `cppyy_kit.shutdown()` — a LIFO,
  idempotent, best-effort registry, wired to `atexit`. atexit runs after `main`
  returns but **before** module globals are cleared and before cppyy's
  (earlier-registered, therefore later-running) Cling teardown — the correct
  window, with both Python and cppyy still healthy. rclcppyy registers
  `shutdown_rclcpp()`, a guarded once-only `rclcpp::shutdown()`; the guard also
  closes the historical **double-`rcl_shutdown`** race (`rclcpp` installs its own
  SIGINT/SIGTERM handler that can call shutdown a second time). Kits with no
  process-global C++ state (bt, pcl) register nothing: their objects are
  per-instance and RAII-released on scope exit, and the JIT'd namespaces are
  Cling's to tear down. `test/test_clean_exit.py` is the regression tripwire.
- **A C++ object owning an executor + `std::thread` is the same hazard (tf, 3rd
  instance).** `rclcppyy.tf`'s C++ TransformListener owns a spinning executor +
  thread; `register_teardown` a callback that drops it (its dtor cancels the
  executor and joins the thread) so it releases **before** `shutdown_rclcpp` (LIFO
  order is correct). Exit 0 confirmed. Same rule as the pluginlib instance/loader
  `reset()` in §19: anything owning threads/executors/global state gets an ordered
  teardown.

### 15. First-use JIT: make it visible, move it with `warmup()`
The first time a given C++ signature is crossed, cppyy JIT-compiles a call wrapper
for it. It is a one-time, **per-signature** cost — and a big one at a kit's entry
points (bt_kit's first `registerSimpleAction` ~0.4 s to codegen the
`std::function<NodeStatus(TreeNode&)>` thunk + the register call wrapper; the first
pcl NumPy→VoxelGrid→NumPy frame ~0.45 s). A **freeze/PCH does not remove it** (the
PCH is an AST, this is call-wrapper codegen triggered by the Python call), and
`-O0`/`-O1` make no difference (it is Clang front-end instantiation, not LLVM
optimisation). So a script's *first live call* stalls unexpectedly.

Two moves, both in `cppyy_kit`:
- **Make it visible.** Wrap a known-expensive kit entry point in
  `with cppyy_kit.first_use(label, warmup_hint):`. On the first call that exceeds a
  threshold it prints a one-time, LLM-actionable line to stderr — e.g. *"bt_kit.
  register_simple_action JIT-compiled a call wrapper on first use (408 ms). Call
  bt_kit.warmup() once during init… Silence: RCLCPPYY_JIT_NOTICE=0."* Thereafter
  (and when disabled or warming) it is a bare passthrough — zero overhead.
- **Move it.** A kit's `warmup()` exercises its expensive signatures on throwaway
  objects (under `cppyy_kit.suppress_first_use_notice()`, via the
  `cppyy_kit.warmup(*thunks)` building block) so the wrappers are JIT'd and cached
  process-globally during init. Measured: bt_kit `time-to-first-tick` 678 → 98 ms
  (the spike moves into a ~0.9 s init); pcl showcase frame-0 630 → 4 ms.

*Scope choice:* instrument the **kit-owned entry points** (registration, bringup),
not every cppyy call (too broad, adds overhead) nor a generic opt-in context (can't
name the API/warmup in the notice). This is reliable where the kit owns the entry
point (bt registration); for kits that mirror a raw algorithm API whose first-use
cost is *inside* un-wrapped library calls (pcl's VoxelGrid), the notice is
best-effort and `warmup()` is the primary tool. `warmup()` stays per-kit (only the
kit knows what to exercise); the notice/suppress/runner are shared.

### 16. Cross-language inheritance (Python derives a C++ virtual base)
The heaviest crossing, first proven in ompl_kit: a Python class *derives* a C++
class and C++ calls its overrides in a hot loop (RRT\* calls a Python
`StateValidityChecker` millions of times/solve). It works — with rules:
- Derive the cppyy class directly; **`super().__init__(base_args)` is mandatory**
  (the C++ base must be constructed, e.g. with its `SpaceInformation`).
- Override the virtual by its **exact C++ name** (`isValid`, `stateCost`) — cppyy
  matches on the name.
- **Only plain virtuals** can be overridden across the boundary. A `final` (or
  non-virtual) member cannot — this is exactly why bt_kit's `final` `tick()` needed
  a C++ shim instead (Pattern 5 / bt REPORT). Check the base before promising it.
- **Watch for versioned pure-virtual creep (wbc/Crocoddyl).** A library minor version
  can **add pure virtuals to a base you subclass** and silently make your override
  abstract — Crocoddyl 3.2's `CROCODDYL_BASE_CAST` macro added
  `cloneAsDouble`/`cloneAsFloat` to `ActionModelAbstract`. In C++ that is a compile
  error; **in cppyy it is a failed-`cppdef` crash** with no traceback (§9). Before
  authoring, `nm`/grep the base for **every** pure virtual (including macro-injected
  ones) and probe the subclass `cppdef` out-of-process; a kit should ship the mandatory
  boilerplate as a constant (`wbc_kit.ACTION_MODEL_CLONES` is the worked example).
- **Pin the subclass instance** with `keep_alive` (or an `owner`): the "callable
  was deleted" footgun (Pattern 4) applies to override *instances* too — C++ holds
  the object, cppyy won't keep it alive for you.
- Pointer arguments arrive **auto-downcast** (Pattern 17b) so member access on the
  concrete type works with no explicit cast.
Cost here: ~350 ns/override call, 1–3 M dispatches/s — invisible for small problems,
material when the override dominates (then lower it to C++, the L2 rung).

**Deriving a *framework* base and injecting it (control_kit sharpens this):**
- **Derive the *compiled* base, never a `cppdef`'d intermediate.** cppyy's override
  dispatcher fails to resolve return types (`<unknown>`) when the base was itself
  JIT-defined; subclass the real library class directly.
- **Inject the instance where C++ stores it by `shared_ptr` via a C++-built no-op-
  deleter `shared_ptr`.** Assigning a `shared_ptr` that aliases a cross-inherited
  Python object *from Python* fails (`C++ type cannot be converted to memory`);
  build the aliasing `shared_ptr` (no-op deleter, so Python keeps ownership — pin
  the instance) in a `cppdef` helper. This is how control_kit hands a Python
  `ControllerInterface` subclass to the real `controller_manager`.
- **Reach protected base members through a same-layout accessor** — a
  `struct : Base` that `reinterpret_cast`s and reads them, exposed as free
  functions — since cppyy can't touch `protected` across the boundary.

### 17. `shared_ptr` ownership + RTTI downcast (two cppyy conveniences)
- **(a) Wrapping a raw pointer in the library's `shared_ptr` transfers ownership.**
  Constructing a `SomethingPtr(raw)` from a cppyy-owned raw object flips the raw's
  `__python_owns__` to `False` — cppyy yields ownership to the `shared_ptr`, so
  there is **no double-free**. This makes the pervasive "wrap the raw in the
  library's Ptr and hand it on" idiom (`ob.StateSpacePtr(space)`) safe, and mirrors
  `make_shared`.
- **(b) Pointer arguments are auto-downcast by RTTI.** cppyy presents a base-typed
  pointer argument (a callback's `const State*`) as its **concrete runtime type**,
  so `state[0]` / `state[1]` work without an explicit downcast. You rarely need a
  cast helper; when you do (a stored base pointer), use `getattr(obj, "as")[T]()`
  (Pattern 18).
- **(c) cppyy dereferences a `shared_ptr` to bind a `const T&` parameter** (moveit:
  `srdf::Model::initString` took a `ModelInterfaceSharedPtr` directly) — smart-
  pointer forwarding is reliable for reference params, not just member access.
- **(d) Eigen block/coeff assignment does NOT cross** (moveit: `iso.translation()[i]
  = v` → "object does not support item assignment"). Build Eigen objects in a
  `cppdef` helper (assemble the whole vector/matrix in C++), not element-by-element
  from Python. Eigen is everywhere in robotics C++, so this recurs.

### 18. Reserved-word method names: `getattr(obj, "as")[T]()` for C++ `as<T>()`
The pervasive C++ idiom `obj->as<T>()` is a Python `SyntaxError` (`as` is a
keyword — even `obj.as` won't parse). The spelling is `getattr(obj, "as")[T]()`
(fetch the attribute by string, then subscript the template arg). A one-liner, but
a guaranteed stumble for any library with a method named `as`, `from`, `import`,
`class`, `global`, … — reach for `getattr` when a C++ name collides with a keyword.

### 19. In-process pluginlib + a parameterized node (the ROS 2 plugin/param bootstrap)
Modern ROS 2 stacks load their algorithms as pluginlib plugins configured by node
parameters. Both work in-process (moveit_kit proved it; control_kit reuses it):
- **pluginlib:** `load_library("libclass_loader.so")` + **the plugin base-class
  library** (for its typeinfo), construct `pluginlib::ClassLoader<Base>(pkg,
  "Base::type")` in a `cppdef`, then `createUniqueInstance(lookup_name)` — pluginlib
  `dlopen`s the plugin `.so` itself via the ament index (do **not** cppyy-load the
  plugin). The "add the library named in the JIT link error" loop resolves the rest.
- **A dlopen'd plugin's sibling libs need `LD_LIBRARY_PATH` set *before* the process
  starts (ik_bench).** A vendored plugin `.so` depends on its co-installed core lib
  (`libpick_ik_plugin.so` → `libbio_ik.so`) in the same private prefix. The dynamic
  linker reads `LD_LIBRARY_PATH` **at process start**, so setting it inside the running
  worker is too late for pluginlib's `dlopen`. Prepend the vendored `lib/` dir to the
  **child's** `LD_LIBRARY_PATH` before spawning (or `$ORIGIN`-RPATH the install);
  `AMENT_PREFIX_PATH` — which *is* read at runtime — carries the plugin discovery.
- **parameterized node:** `NodeOptions().automatically_declare_parameters_from_overrides(true)
  .parameter_overrides(vec)` + `make_shared<rclcpp::Node>(name, options)`, fed by a
  **YAML → dotted-`rclcpp::Parameter` flattener** (nested dict → dotted names,
  homogeneous lists → typed arrays) — the reusable primitive.
- **Teardown (Pattern 14, sharpened):** a pluginlib instance/loader must be
  `reset()` before Cling teardown or the process cores at exit — `register_teardown`
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
  **Python binding is a legitimate fallback** for that step — cppyy is not the only
  tool, and a kit can mix (drive the hot C++ path via cppyy, use the binding for a
  one-shot batch step). (2nd instance of this rule after the gtsam batch step below.)
- **Probe layered blockers one at a time, and know when to stop.** gtsam via cppyy
  is the worked example: fixing the boost blocker (add headers) only exposed a
  `GTSAM_USE_TBB` → tbb-headers blocker, which when fixed exposed the Cling **ORC
  static-init wall** (§9) — a *Cling limitation no dependency fixes*. Peel one layer,
  re-probe out-of-process; when the bottom layer is a Cling limitation rather than a
  missing dep, stop and take the Python-binding fallback for that batch step. Don't
  keep adding dependencies against a wall that isn't a dependency problem.
- **Distrust environment shims, prefer the native binary.** A library's console-
  script entry point can be broken in an env while its native binary works (vision:
  the `rerun` console script vs spawning the viewer binary by its executable path).
  When a Python-package CLI shim misbehaves, resolve and invoke the real executable
  directly rather than assuming the library is broken.

### 21. Vendored-source direct-compile (when there's no package)
For a small, well-understood subset of a library that ships no conda package
(DBoW2), clone it + apply a **documented, marker-guarded in-place patch** + compile
with a direct `$CXX` invocation into a `.so` — this beats fighting the library's
CMake/ExternalProject. It generalizes the L2 lowering recipe (`build_l2_node` →
`build_dbow2`): a reproducible build script, artifact gitignored, env-version tagged.

**A second shape: a vendored ROS/MoveIt *plugin* wants its ament install layout, not
a bare `.so` (ik_bench).** The direct-`$CXX`→`.so` recipe does *not* suffice for a
pluginlib plugin, because discovery is via the **ament index**, which only the
package's own `ament_package()` + `pluginlib_export_plugin_description_file` produce
(the `<pkg>` marker, the plugin description XML, the `.so`). So for a plugin,
"vendored build" = **run its own CMake with a plain `cmake` configure/build/install
into a private prefix**, then put that prefix on `AMENT_PREFIX_PATH` — pluginlib then
finds it by lookup name, no different from a packaged one. The two unpackaged IK
solvers (bio_ik, pick_ik) both built first try this way; pick_ik needed only one extra
header-only dep (`range-v3`) added to the env, no source patches.

### 22. Overload mis-resolution: a compilable-but-WRONG overload that crashes
Distinct from the parse/execution faults (§9): with a **thicket of overloads**, cppyy
can pick one that **compiles and runs but is the wrong one**, crashing at runtime
(bus error, no Python traceback). tf: `tf2_ros::Buffer::lookupTransform(target,
source, TimePoint)` resolved into the `rclcpp::Time`+timeout `canTransform` path,
which called `rclcpp::Clock::now()` and bus-errored. The trap is worst when a class
mixes a `using`-imported base form with timeout/clock forms of the same name.
- **Rule:** prefer the **single-signature base class** (here `tf2::BufferCore`, one
  unambiguous `lookupTransform`) over the overload-heavy derived one; or wrap the
  exact call you want in a `cppdef` free function so C++ overload resolution — not
  cppyy's — picks it. Probe a suspicious overloaded call out-of-process; a
  wrong-overload crash gives you nothing to read in Python.

### 23. Compile cache: kill the first-use wrapper JIT persistently (`cppdef_cached`)
The one-time, per-signature call-wrapper JIT (§15) is *relocatable* with `warmup()`
but comes back every process — a PCH can't touch it (that's an AST; this is
codegen). `cppyy_kit.cppdef_cached(code, decls=..., name=...)` **eliminates** it:
compile the C++ glue once into a real `.so` (the direct-compile recipe, factored
into `cppyy_kit._compile`), and on every later run `load_library` it instead of
JIT-generating the wrapper. Measured with bt_kit adopted (t01): first-use register
**~233 ms → ~60 ms**, and freeze + cache compose to **~1.77 s → ~0.43 s** end-to-end
(FREEZE.md §4); pcl_kit's d02 frame-0 **~681 ms → ~88 ms**. Run 1 pays a one-time
`.so` compile (per machine); a kit can *ship warm* by pre-building the `.so` at
package-build time (`cppyy_kit.cache.prebuild`).
- **Declarations are mandatory for the speedup.** Cling emits any function *body*
  it can see (inline or not), ignoring the `.so` copy — so the fast path must give
  Cling **bodiless declarations** (`decls=`) and let the definitions live only in
  the `.so`. Without `decls` the call safely degrades to a plain `cppyy.cppdef`
  (correct, uncached) and says so once. `extern "C"` and free functions / classes
  with out-of-line methods are the clean supported subset.
- **The big win is caching the *crossing*, not just the glue.** The ~0.4 s isn't
  cppyy internals you can intercept — it's the `std::function<Ret(Args)>` thunk +
  the register call wrapper. Build **both in compiled code**: a trampoline whose
  `.so` constructs the `std::function` wrapping the Python callable and does the
  registration, converting the C++ argument to its Python proxy with cppyy's
  public `CPyCppyy::Instance_FromVoidPtr(&obj, "Cpp::Type")` (header under
  `$CONDA_PREFIX/include/pythonX.Y/CPyCppyy/API.h`; link `libcppyy`). Pass the
  Python callable to a `PyObject*` parameter — cppyy hands it across directly.
  `cppdef_cached(..., trampoline=True)` adds the Python + CPyCppyy include and the
  `libcppyy` link. bt's `BT::NodeStatus(BT::TreeNode&)` is the worked example
  (`scripts/cache/validate_cache_bt.py`, the kit-adoption reference).
- **Honest boundary:** this caches the glue/trampolines the *kit* authors. cppyy's
  on-demand template member instantiations from arbitrary user calls
  (`node.getInput[T]` for a new `T`) are not cached — they stay JIT unless routed
  through their own cached helper. Artifacts are env-version-tagged + gitignored
  (same lifecycle as the PCH); a cppyy/compiler/source change is a clean miss, and
  a corrupt/stale `.so` on load is discarded and rebuilt, never wedging a run.
- **Debugging escape hatches (turn it off).** To rule the cache out when a kernel
  misbehaves, bypass it so `cppdef_cached` is a plain in-memory `cppyy.cppdef` (no
  `.so` read/write): per call `cppdef_cached(..., cached=False)` / `@cpp(cached=False)`;
  process-wide `cppyy_kit.disable_caching()` (or `with cppyy_kit.caching_disabled():`);
  or the `CPPYY_KIT_NO_CACHE=1` env var. Nuke artifacts with `cppyy_kit.clear_cache()`.
  The PCH has its own switch (`CPPYY_KIT_NO_AUTOPCH=1`). Full decision tree + artifact
  locations: **FREEZE.md §9, "Debugging: turning the caches off".**

**Kit adoption recipe (copy-paste).** Both bt_kit and pcl_kit follow this shape;
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
`pcl_kit._adopt_glue` (a library template — `VoxelGrid<PointXYZ>` — compiled into
the `.so`).

### 24. Boundary tracer: a typed manifest of every crossing (`cppyy_kit.trace`)
cppyy_kit is the one place Python crosses into C++, so instrumenting *it* — not
Python — yields a small, typed record of what a kit app loaded, compiled and
wrapped, with the C++ signatures, counts and timings. `cppyy_kit.trace.start()` /
`stop()` (or `CPPYY_KIT_TRACE=1` before import) turns it on; the crossing points
(`load_libraries`, `cppdef_cached`, `callback`/`std_function`) record automatically.
Off by default and cheap when off (each crossing asks `trace.span(...)` for a timer
that's a shared no-op until started — no timing syscall, no event).
- **The manifest is the point.** `stop()` returns (and optionally writes) JSON with
  a per-kind summary and an **instantiation manifest**: the distinct C++ signatures
  crossed, sorted by cost — i.e. exactly what a freeze PCH or the compile cache (§23)
  should cover, and the raw material for the `cppyy-accelerate` skill's hotspot
  analysis. `python -m cppyy_kit trace report trace.json` pretty-prints it.
- **Use it to decide what to cache.** Trace a workload once; the top instantiation
  lines (e.g. `std_function` at ~100 ms for `BT::NodeStatus(BT::TreeNode&)`) name the
  crossings worth routing through a cached trampoline (§23) or baking into the PCH.

### 25. `require()` a header-only library — conda-first, fetch only if unpackaged
Some header-only C++ libs aren't in the env. `cppyy_kit.require(name, header, url=,
sha256=)` makes one available, **conda-first**: if `header` already resolves on the
env include path (the conda-forge/robostack package), it registers that dir and does
nothing else — the packaged copy is ABI-matched and offline. Only when the header is
absent *and* a `url`+`sha256` are given does it download once to a gitignored cache,
verify the checksum, unpack (single header / `.zip` / `.tar.gz` with `strip_prefix`),
and register the cache include dir; cached (offline) thereafter.
- **Policy, not convenience:** prefer the conda-forge package for anything on it
  (Eigen, fmt, nlohmann_json). Reach for `url=` only for the unpackaged or an exact
  pinned version — the same discipline as the §21 vendored-source builds, minus the
  compile. `require` fetches *sources*; pair it with `cppdef_cached` (§23) when you
  need a compiled `.so`, not just headers.
- **Integrity + reproducibility:** `sha256` is mandatory for a fetch (a mismatch
  raises, the partial download is removed). Point `$CPPYY_KIT_REQUIRE_DIR` at a
  persistent dir (e.g. `~/.cache`) for a machine-wide header cache.

### 26. `@cpp` — write a C++ kernel in Python, compiled + cached + auto-marshaled
For a small hot kernel you'd otherwise hand-write as a `cppdef` helper plus manual
`uintptr_t` marshaling (§6), `cppyy_kit.cpp` does both. The decorated function's
**docstring is the C++ body** (its Python body never runs) and its **annotations
drive marshaling**; on first call it compiles once into a cached `.so`
(`cppdef_cached`, §23) and loads it thereafter.
```python
@cpp
def sum_sq(data: cpp.arr("float")) -> float:            # numpy -> (float* data, size_t data_size)
    "double s=0; for (std::size_t i=0;i<data_size;++i) s+=data[i]*data[i]; return s;"
sum_sq(np.array([1,2,3], np.float32))                    # 14.0, no manual ctypes/cast
```
- **The marshaling is the §6 pattern, automated.** `int`/`float`/`bool` cross by
  value; a verbatim `"T*"` annotation takes a NumPy array (its `.ctypes.data`) or an
  int address as `uintptr_t` and hands the body the typed pointer (the
  `reinterpret_cast` is injected); `cpp.arr("T")` is the numpy→**pointer+size**
  convenience (body sees `name` and `name_size`). Return `None`→`void`. Only that
  honest subset is marshaled; anything else raises at decoration time.
- **It composes with the cache**, so a `@cpp` kernel is persistent (no first-use JIT
  after the first machine build) — the same guarantee `cppdef_cached` gives. Pass
  `@cpp(include_paths=..., libraries=...)` to call into a real library from the body.
- **`@cpp(nogil=True)` releases the GIL around only the compiled body** (the ergonomic
  form of §27) — plain Python threads calling the kernel run on N cores. **`cached=False`**
  compiles the kernel in-memory and skips the `.so` cache (the §23 debugging escape
  hatch, also `cppyy_kit.disable_caching()` / `CPPYY_KIT_NO_CACHE=1`; see FREEZE.md
  "Debugging: turning the caches off").
- **The honest headline: the win tracks "custom kernel vs library primitive", not
  "C++ vs Python" (webcam).** Reach for a `@cpp`/`cppdef` kernel where you'd otherwise
  write a **per-element Python loop with no vectorized-NumPy/library one-liner** — a
  hand-written NCC patch tracker measured **~12–15×** (4.32 ms vs 66.3 ms/frame at
  640×480). Where the per-frame work is *only* library-provided ops (OpenCV
  ORB/match/RANSAC, where `cv2` is C++ too) the gap collapses to **~1.1–1.2×**
  per-frame orchestration — do **not** expect a win from merely chaining library
  primitives. Robotics code constantly hand-writes the former (trackers, cost
  functions, robust estimators), which is exactly where cppyy_kit earns its keep.
  (For an honest A-vs-B bench, bracket each pipeline with `time.process_time()` deltas:
  `cpu% = 100 * Δcpu/Δwall` is a dependency-free per-pipeline CPU meter when the driver
  is single-threaded and the calls are sequential — no psutil needed.)

### 27. `nogil()` — release the GIL around a blocking C++ call
§13's rule ("cppyy does not release the GIL on a blocking C++ call") has a fix:
`cppyy_kit.nogil(fn)` runs a **C++** nullary callable through a compiled shim that
drops the GIL (`Py_BEGIN_ALLOW_THREADS`) around it, so concurrent Python threads run
during the call. Measured (test_nogil.py): a 500 ms C++ sleep called directly lets a
co-thread advance ~1 tick; through `nogil` it advances **~470** — the co-thread runs
the whole time.
- **The ergonomic front-end: `@cpp(nogil=True)` (§26).** When the C++ you want to run
  GIL-free is a kernel you're writing anyway, skip the `std::function` ceremony — add
  `nogil=True` to `@cpp` and the decorated call releases the GIL around the compiled
  body directly. `@cpp` compiles a small wrapper (in the same cached `.so`) that
  forwards the already-marshaled POD arguments into the kernel inside
  `Py_BEGIN/END_ALLOW_THREADS`, so the GIL is dropped for **only** the C++ body —
  cppyy's argument/result marshaling stays under the lock on either side. The release
  wrapper adds ~0.04 µs/call over `nogil=False` (a trivial `add`; measured), and plain
  Python threads each calling the kernel run on N cores: eight jobs, **7.7× faster**
  than GIL-held on a 16-core box (`examples/parallel_demo`, the front-page snippet).
  Reach for the raw `nogil(fn)` below only for a *pre-existing* C++ callable you did not
  write with `@cpp` (a library's blocking `spin()`/`wait()`).
- **`fn` must be C++, not Python.** A Python callable would re-acquire the GIL to run
  (cppyy takes it to enter Python), defeating the point. Bind args/results in C++ (a
  `cppdef`/`@cpp` nullary wrapper writing its result into a C++ object you read
  after). This is §13's "run the blocking work on a C++ path, not a Python thread",
  made a one-liner.
- **`run_async(fn)`** is the asyncio form: `await`s the blocking C++ work on an
  executor thread *with the GIL released*, so the event loop keeps running.
- **Callback caveat:** if `fn` calls back into Python while the GIL is released, that
  callback must re-take the GIL first — a cppyy Python callback does so automatically;
  hand-written C++ touching `PyObject*` under `nogil` must `PyGILState_Ensure()`.
- **Beyond "other threads run": the loop itself jitters less (jitter_bench).** Running
  the *whole* hot loop (wait + compute) in C++ via `nogil`+`cppdef_cached` also tightens
  its *scheduling determinism*. A 1 kHz control loop held its **~2 µs median wakeup
  latency under load** where the equivalent pure-Python loops rose to ~5 µs, and its p99
  under load was the lowest of the four variants tested: the C++ loop never re-enters the
  interpreter between wake and next sleep, so the scheduler sees one long-running C++
  thread rather than a Python thread cycling the interpreter, and load perturbs it less.
  So for a periodic loop `nogil` is not only "let other threads run" — it is "the loop
  jitters less." (Full jitter matrix + the bigger unprivileged lever: §35.)

### 28. `.pyi` stubs for a kit's mirror surface (IDE/mypy corridor)
A kit assembles its mirror API at runtime, so editors see nothing. `python -m
cppyy_kit stubgen <module> -o <module>.pyi` emits a static `.pyi` for the module's
**public Python surface** — functions, classes (with methods) and scalar constants,
including names re-exported from submodules — giving name + arity completion and a
mypy corridor. Committed pilots: `cppyy_kit/__init__.pyi`, `bt_kit/bt_kit/__init__.pyi`.
- **Honest scope:** it stubs the statically-knowable *kit* API, not the C++ namespace
  a bringup returns (`cppyy.gbl.BT.*` are dynamic cppyy proxies with no static
  signature — a bringup's return is `Any`). Signatures are names + arity with `Any`
  types (always-valid, loose) rather than guessed C++ types; tighten by hand where a
  kit wants richer hints. Regenerate when the surface changes.

### 29. capability / fallback / status — codify detect → fallback → introspect
Kits keep doing the same dance for optional capabilities (a CUDA build of OpenCV, a
working compiler for the compile cache, a frozen PCH): **detect**, **fall back** to a
slower-correct path, and ideally let a user **introspect** why. `cppyy_kit.capability`
makes it uniform:
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
  `set_state("bt_kit.compile_cache", ...)` with the outcome — so `python -m cppyy_kit
  status` shows both the base capability and whether bt_kit actually took the cache
  path (and, if not, why). This is the pattern every kit's CUDA/lifecycle/binding
  probe should follow instead of an ad-hoc `try/except`.

### 30. In-process lifecycle bootstrap: build the node the coupled ctor asks for
Modern ROS 2 cores often take a `rclcpp_lifecycle::LifecycleNode` / a `*ROS` wrapper /
a pluginlib base in their ctor or `configure`, so they *look* like they need the
server. They don't: those objects are **plain classes you construct in-process from
Python** — no lifecycle servers, no manager, no YAML, no action interface. This is the
third instance of the "in-process ROS 2 node/manager" family after moveit_kit's
parameterized `Node` and control_kit's `ControllerManager` (§19), and the cleanest
statement of it (nav2_kit's lifecycle unlock).
- **The key — construct a `LifecycleNode`.** `make_shared["rclcpp_lifecycle::
  LifecycleNode"](name, ns, NodeOptions)`, then walk `configure()` (UNCONFIGURED→
  INACTIVE) and `activate()` (→ACTIVE); `get_clock()`/`get_logger()` are live
  immediately. `lifecycle_node.hpp` **JIT-parses cleanly** (no generate_parameter_
  library wall, like ros2_control and unlike MoveIt's convenience headers). This one
  object fits every lifecycle-coupled ctor in the ecosystem.
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
- **Separate "can I construct it" from "does its runtime path enter a fragile
  transitive dep".** The LifecycleNode key unlocked Smac 2D (`AStarAlgorithm<Node2D>`
  plans from Python) but **not** Hybrid-A\*: its wall is a *non-deterministic
  OMPL-under-Cling segfault* in `precomputeDistanceHeuristic` (~2 of 3 runs), not a
  ctor coupling. `Node2D` is stable precisely because its search never enters OMPL at
  runtime (the header only *parses* the OMPL includes). A coupling wall and a
  runtime-library wall are different failures; one is a ctor you can build, the other
  is a path you can't safely run.
- **Teardown (§14, applied).** These objects own DDS entities (+ a bond timer); their
  destructors must run **before** `rclcpp` shutdown. `register_teardown` a callback that
  drops each one so it runs *before* `shutdown_rclcpp` (LIFO). Verified: nav2_kit's
  14-test suite and all four planner×controller demo combinations exit 0.

The updated authoring heuristic (§20): grep the ctor/`configure` signature. Plain data
(`Costmap2D(w,h,...)`, `NavFn(nx,ny)`) → drive directly. A `LifecycleNode`/`*ROS`/
pluginlib base → still reachable via this in-process bootstrap, **not** a server. The
remaining walls are runtime (missing/unstable transitive libs), not signatures.

### 31. Lower the hot virtual: the ideal cppyy target
A framework whose hot loop repeatedly calls a **user-authored virtual** is the ideal
cppyy target, and the recurring highest-value shape. Three instances now — OMPL's
`StateValidityChecker::isValid` (RRT\* calls it millions of times/solve, §16),
ros2_control's `update` (control_kit), and Crocoddyl's `calc`/`calcDiff` (the DDP
solver calls them per node per iteration plus line-search rollouts) — all share the
same "prototype → lower" arc:
- **Prototype the virtual in Python** (the binding's supported path), then **lower it
  to an inline-C++ subclass in the *same script*** via `cppdef` — JIT-compiled at
  runtime, so the solver calls native C++ in the hot loop with **no build system**.
- cppyy is the *only* tool that offers the *fast* authoring path without the framework's
  usual "write a CMake project linking the library" rebuild. The framework's own
  workflow is "prototype in Python (slow) or ship a C++ model (needs a build)"; cppyy
  fills the missing "fast **and** no build system, one file" cell.
- **Measured (Crocoddyl, wbc):** the inline-C++ custom action model runs at the
  compiled built-in's speed (**0.32 vs 0.34 ms**) and **~21.7×** the Python-derived
  model, converging to a **bit-identical** cost (250.039320, 8 iters — the numeric
  match is the regression gate). ompl_kit's Python validity checker was ~350 ns/override
  call, 1–3 M dispatches/s (§16); the win is invisible for small problems, material when
  the override dominates the loop.
- This is the L2 native-lowering rung (FREEZE.md) made a one-liner: keep the crossing
  out of the hot loop by putting the *whole* per-iteration virtual in C++. Watch for
  versioned pure-virtual creep (§16) and the failed-`cppdef` minefield (§9) when
  authoring the subclass; probe it out-of-process first.

### 32. A library's own Python binding and cppyy coexist in one process
Many robotics libraries ship their own binding (boost::python, pybind, `cv2`) *and* can
be driven by cppyy. Mixing them in one process is fine — and often ideal — with two
rules:
- **Both load the same `.so`; the C++ objects are separate.** The clean division of
  labour is **prototype with the library's own binding, lower the hot path with cppyy,
  in one script** (the Crocoddyl story, §31). But a **cppyy-created C++ object cannot be
  handed to a boost::python API** (two proxy runtimes, wbc-verified) — so the cppyy path
  must build its own containers/solve in C++ (§6), not feed the binding's objects. Don't
  pass objects *between* the two runtimes.
- **Same build only.** Two loaders of *one* build coexist cleanly — `cv2` (the CPU
  `libopencv`) and a cppyy-loaded CPU OpenCV share the same `.so`, no corruption
  (webcam). The hazard is **mixing two builds of the same soname** in one process (a
  CUDA `libopencv` alongside the CPU one corrupts it): a GPU-vs-CPU comparison must be
  single-pipeline or two processes, never a same-process A-vs-B.

### 33. Schema-derived C++ structs — validate at the boundary, compute in C++
*(Design + probe RFC; prototype `cppyy_kit/pydantic_structs.py`. Numbers below measured
on cppyy 3.5.0 / pydantic 2.13.4, 1M `Detection`.)*

You already describe your data with a Pydantic v2 model for edge validation — that
schema *is* a struct layout. `pydantic_structs` emits the equivalent C++ `struct`
(compiled + cached), so the same data lives as a `std::vector<Struct>` instead of a
`list` of model instances: compact, typed, and zero-copy-viewable as NumPy on its
numeric columns. Slogan: **validate at the boundary (Pydantic) → compute compactly
(C++) → re-validate on exit (Pydantic)**; `to_model()` re-runs the validators so the
C++ excursion can't silently violate the model.
- **A struct is a *parse* cost, not a call-wrapper-JIT cost — so cache the kernels, not
  the struct.** A struct is a type *declaration*; cppyy learns its layout by parsing it
  once per process (~7 ms for a small set — the domain of the freeze PCH, §2/L1), and
  there is no function body to compile into a `.so`. What genuinely recurs is (a) the
  `std::vector<Struct>` template first-use JIT (~46 ms) and (b) the **consumer kernels +
  marshaling glue**, which *are* functions with bodies — exactly what `cppdef_cached`
  (§23) persists.
- **Compact storage:** `list[Detection]` (Pydantic instances) **1112 MB** →
  `std::vector<Struct>` **70 MB** (16× smaller); numpy columns 49 MB.
- **Compute — numpy is still the incumbent for flat reductions.** `sum(score)` (pure
  contiguous reduction): numpy **136×** vs Python, the struct loop only 12× (it walks
  the AoS with a 64-B stride). If your hot path is pure columnar numeric reductions,
  **use numpy** (and the zero-copy column view lets you). The **C++ struct kernel wins
  fused/branchy logic** — a `score>0.5` filter+centroid is **7×** vs numpy's 3× (numpy's
  mask+gather allocates intermediates; the C++ loop is one alloc-free pass) — and keeps
  the model's **nested/mixed shape** a flat array cannot represent.
- **"Free" type checks:** consumer kernels compile *against* the struct, so a misused
  field is a Cling compile error that names it (`no member named 'scoree' … did you mean
  'score'?`; `invalid operands ('double' and 'std::string')`). Run that check
  **out-of-process** (`probe_cppdef`) — a failed `cppdef` contaminates the live
  interpreter (§9).
- **Crossing traps:** a `std::string` inside a returned `std::vector<std::string>`
  crosses as **`bytes`** — `to_model()` must `.decode()` string fields (§11). The
  zero-copy numeric column view is **strided/non-contiguous** (stride = `sizeof(Struct)`),
  a read/mutate-in-place convenience, not a free numpy pipeline; contiguous SoA columns
  are just numpy. The view aliases the vector's buffer, so the vector must outlive it and
  any `resize`/`push_back` invalidates it (`column()` pins via `keep_alive`).

Positioning: the "I already maintain Pydantic models — make the hot path compact and
typed without a codegen step" tool (contrast FlatBuffers/protobuf's separate schema +
build), not a wire format and not a numpy replacement.

### 34. Hybrid pipelines: a commodity-ML front end, a cppyy hot path, two envs
A realistic robotics pipeline mixes a Python ML library (its inference *is* a library
primitive — don't wrap it, per §26's honest headline) with a cppyy_kit hot path, and the
two halves can have **incompatible native dependencies**. The retarget capture rig was the
worked example: MediaPipe perception feeds a pinocchio retarget solve, and the two halves
were originally split across two envs because the ROS stack pinned libboost 1.90 while
pinocchio's conda stack pinned 1.86.

> **Dated correction (2026-07-12):** that *specific* clash dissolved — conda-forge rebuilt
> pinocchio 4.x against libboost 1.90, so pinocchio now co-solves with the robostack ROS
> stack in one `solve-group`, and the retarget half runs in a ROS-capable env consuming the
> landmark frames straight off `/tf` (rclcpp_kit's C++ listener). The **two-env pattern
> below remains the general lesson** for any genuinely incompatible pair; it is simply no
> longer forced for this pinocchio+ROS case. Note this is the *solve/ABI* boundary only —
> the Cling **header-parse** wall on `pinocchio::Model` (§9, the 25-type `boost::variant`)
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
  loads cleanly in *both* envs.
- **The first pip dependency in a conda/pixi repo needs discipline.** Put it in a
  dedicated feature env with a `[pypi-dependencies]` section (keep the ROS-free base
  minimal). Two rules that avoid an ABI split: **verify the pip deps' numpy equals the
  conda numpy** (mediapipe brought numpy 2.5.1, matching conda's — no split), and
  **exclude any conda package the pip dep re-provides** (do not compose a conda `opencv`
  with mediapipe's pip `opencv-contrib-python`). Compose with the ROS default via
  `solve-group="default"` so the shared stack stays one solve. Pin ML model bundles by
  URL **and SHA-256** (verify-after-download, refuse a mismatch) — the same supply-chain
  hygiene as §25's `require(..., sha256=)`.
- **The cppyy_kit wins land in the glue, and only where measured.** In this rig they were
  the /tf marshaling (§6 build-once-refill, 265×) and the per-frame retarget glue kernel
  (coord transform + target map + a sequential One-Euro filter in one `cppdef` pass,
  **303.8×**, bit-identical) — both §6/§26, both with numeric-agreement checks. The IK
  *solve* stays a pinocchio-bindings job (§9's `Model` wall); an honest "kit blocked here"
  cell, documented with the exact wall.

### 35. Low-jitter timed loops from Python: timer slack is the first lever
A control/HIL loop *orchestrated from Python* can hit a µs-scale period median on a
**stock (non-PREEMPT_RT) kernel** with only unprivileged tuning — the orchestration
language is not what sets the median.
- **`prctl(PR_SET_TIMERSLACK, 1)` is the big, free lever.** Linux' default timer slack is
  **50 µs** — the kernel may defer any `clock_nanosleep`/`futex`/`poll` wakeup by up to
  that to batch wakeups, and at 1 kHz that slack *is* the median wakeup latency. One
  unprivileged `prctl` call drops the median from **~52 µs → ~2.4 µs (~22×)**; the removed
  ~52 µs was timer slack, not Python overhead. Set it once at loop start.
- **Then `mlockall` + CPU pinning + `clock_nanosleep(TIMER_ABSTIME)`.** With slack tuned,
  a bare-Python loop, a cppyy_kit C++ loop, and a real ros2_control loop all sit at
  **p50 ~2 µs / 1000.0 Hz / <1 % late cycles** idle. `clock_nanosleep` beats
  deadline-corrected `time.sleep` (thinner tail). Driving a *real* ros2_control
  `read→update→write` from Python (cross-inherited PD controller) adds negligible median
  jitter over a bare timer loop.
- **The cppyy_kit angle is the tail under load, not the median.** A `nogil`+`cppdef_cached`
  C++ loop (§27) keeps its ~2 µs median under load where pure-Python loops rise to ~5 µs.
- **The tail is a scheduling problem, not a Python problem.** Idle p99.9 ≈ 2 ms and rare
  multi-hundred-ms spikes on a busy shared machine are CFS preemption on a non-isolated
  core — collapsed by privileged tuning (`SCHED_FIFO` + `preempt=full` +
  `isolcpus`/`nohz_full`/`rcu_nocbs`), and only *bounded* under adversarial load by
  `CONFIG_PREEMPT_RT`; the stock kernel already ships every soft-RT primitive. Verdict:
  soft-real-time (prototyping / HIL / sim / teleop) from Python now; hard-RT is a tuning
  path on the same kernel, and the graduation to a native `update()` (§31) is unchanged.

### 36. Zero-config PCH: eliminate the header parse with nothing to set
The Cling PCH that removes a kit's header-parse cost (FREEZE.md) is normally a manual
build + a launcher. `cppyy_kit.autopch` makes it automatic: **built on first use into
`${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch`, auto-loaded on every later run.**
- **Activation is independent of import order** because it runs from a `.pth` in the
  environment's site-packages, whose line executes at every interpreter start *before
  any user import*. The `.pth` calls a standalone bootstrap (`cppyy_kit._autopch_boot`,
  stdlib-only, never-raising) that binds `CLING_STANDARD_PCH` from the env's manifest.
  cppyy_kit self-installs it on first import (`python -m cppyy_kit.autopch --uninstall`
  to remove). This matters: `import cppyy` early in a program sets `CLING_STANDARD_PCH`
  to cppyy's *own* std PCH, so an in-process `setup()` that runs after that import is too
  late — the earlier design silently recorded the manifest forever and never warmed up
  when cppyy won the import race. The `.pth` runs first, so it always wins. cppyy_kit's
  own import then prints the one `Cling PCH loaded from …` line from a marker the `.pth`
  set (a print at every `python` start would be noise).
- **A kit registers the headers it parses**, once, at bringup:
  `cppyy_kit.register_pch_headers(headers, include_paths=..., force_symbols=None)`. Warm
  run whose PCH already bakes them → cheap no-op; otherwise the set is folded into the
  env manifest and a **detached background build** runs at exit (lockfile-guarded, atomic
  write), so the next run loads it. `force_symbols` is the §1-FREEZE escape hatch for
  internal-linkage statics — applied only on the warm path (the JIT parse defines them
  otherwise); rclcpp needs none.
- **Keys invalidate naturally; the cache self-prunes.** The `.pch` filename hashes the
  env prefix + cppyy versions + the baked header set; a rebuilt env or upgraded cppyy is a
  clean miss (fall back to JIT), never a silent ABI mismatch. After each build the cache
  is trimmed to the newest few artifacts per environment (keeping any a live manifest
  references) so artifacts from many environments do not accumulate. Opt out with
  `CPPYY_KIT_NO_AUTOPCH=1` (or `python -m cppyy_kit.autopch --prune` / `--uninstall`);
  never committed. When debugging, this is the PCH's "off" switch — the compile cache
  (§23) has its own; see **FREEZE.md §9, "Debugging: turning the caches off"** for the
  whole story.
- **Measured (rclcpp):** for `bringup_rclcpp()`, the `rclcpp C++ headers loaded (…)` line
  drops from ~1.9 s to ~0.0 s and the whole call from ~1.9 s to ~0.06 s (~30×) on the warm
  run, with zero user action between the cold and warm runs — including when the program
  imports `cppyy` before `cppyy_kit`. Removes the **parse** only (cppyy's first-use
  call-wrapper JIT is the separate §23 cost).

### 37. Cache the subscription template instantiation (`rclcpp_kit.subscription_cache`)
Creating an rclcpp subscription from Python makes cppyy JIT-instantiate
`rclcpp::create_subscription<MsgT>` on first use, per message type — measured at ~2.8 s
for `sensor_msgs::msg::Image`, and the PCH (§36) does not touch it (that removes the
header *parse*, not template instantiation). This is the §23 compile cache applied to a
template cppyy instantiates on your behalf: a tiny trampoline that calls
`create_subscription<MsgT>` is compiled once into a `.so` per type (the template is
instantiated at compile time), then `load_library`'d thereafter.
- **Never slower than the plain path.** On a cache miss the rclpy-style
  `node.create_subscription(MsgType, topic, cb, qos)` uses the plain template call for
  that run (so it is exactly as fast as before), and the `.so` is compiled in a detached
  background process at interpreter exit; the next run loads it. The trampoline is used
  only when its `.so` exists, and any failure falls back to the plain call — the cache is
  a pure speedup, never a correctness dependency (verified: the pub/sub roundtrip suite
  passes on both the plain and cached paths).
- **Machine-persistent, cwd-independent.** Artifacts live under
  `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/subs/<version-tag>` (not the compile cache's
  default `<cwd>/build`), so a CLI run from any directory reuses them; env-version-tagged
  like the other caches. Opt out with `RCLCPP_KIT_NO_SUB_CACHE=1`.
- **Measured (rclcpp, Image, PCH warm).** Time-to-ready for
  `import rclcpp_kit; bringup_rclcpp(); node.create_subscription(Image, …)` drops from
  ~3.26 s to ~0.56 s once the `.so` is built; the `create_subscription` call itself from
  ~2.9 s to ~0.22 s. The ~0.14 s residual in that call is cppyy's per-signature
  `std::function` thunk (the Python→C++ callable wrapper), the same non-cacheable boundary
  as the §23 residual — generated at the call from Python, not carried in the `.so`.

---

## Today vs L1 ("freeze") — L1 now WORKS

**Today (L0, JIT):** everything above runs by JIT-compiling the library's headers
at bringup — a one-time, idempotent per-process cost (bt ~0.9 s, pcl ~1.3 s).
Correct and fast at steady state; the only downside is startup latency.

**L1 (freeze) — the mechanism is a Cling PCH, not a dictionary.** This is now applied
**automatically** by the zero-config auto-PCH (§36 above); the mechanism below is what
it wraps and remains the manual path for explicit control. The full recipe, artifact
lifecycle, numbers and limitations live in [FREEZE.md](FREEZE.md); the short version:

- The bringup cost is ~89 % header JIT-parse. A `rootcling`/`genreflex` dictionary
  does **not** help — it supplies reflection/autoload metadata, not a parsed AST,
  so Cling still lazily re-parses on first class use (measured ~0.8 s). *(This was
  the prior probe's dead end.)*
- What works is the mechanism **cppyy already uses for its own std headers:** a
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
  call wrappers (`registerSimpleAction`'s `std::function` thunk etc. — ~0.7 s for
  t01, unchanged L0↔L1). A header PCH only kills the *parse*. Cutting the first-use
  JIT is a separate step (L2 native lowering, or caching the instantiations).
- **Generalises:** the same recipe takes `rclcpp/rclcpp.hpp` from ~1.71 s to ~6 ms
  — the PCH-load floor is header-size-independent, so this is not a BT special case.

**What a kit should do now:** make bringup idempotent and staged, and register its
headers once via `cppyy_kit.register_pch_headers(...)` (§36) so the zero-config
auto-PCH removes the header parse automatically on the second run — no per-kit PCH
build or launcher needed. The only manual residue is the occasional `force_symbols`
entry when a freeze surfaces an unresolved internal-linkage static (§1, FREEZE.md);
the explicit `freeze-<kit>-build` path stays available for CI or full control.

---

*Evidence lives in the per-kit reports: `docs/bt_kit/REPORT.md` (capability matrix,
deep-pass verdicts, AOT probe) and `docs/pcl_kit/REPORT.md` (copy accounting,
showcase benchmark). This document is the merged, library-independent layer.*

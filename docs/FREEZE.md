# Freezing a kit — L0 → L1 (and one leaf to L2)

**Status: WORKING.** The bt_kit header parse — ~89 % of bringup, the one real cost
of the JIT approach — is *eliminated* by loading a prebuilt Cling precompiled
header (PCH), and the same 16-test suite passes on the frozen path.

This is the "freeze" rung of the lowering cycle:

| Rung | What it is | bt_kit today |
|---|---|---|
| **L0** | JIT prototype — headers parsed by Cling at bringup | the default kit |
| **L1** | **frozen** — header AST loaded from a prebuilt PCH, no per-run parse | **this doc** |
| **L2** | native C++ emitted for a hot path | one leaf, below (§5) |

The contract is the same tests at every rung: `pixi run -e bt test-bt` (16 tests)
is green on L0 *and* L1, and the L2 leaf is differential-tested against its L0
Python original.

> **Zero-config path.** §1–§4 describe the *manual* freeze: build an artifact, then
> launch scripts through a wrapper that sets `CLING_STANDARD_PCH`. §8 wraps the same
> mechanism so the L1 fast path needs **no configuration at all** — the PCH is built
> on first use into a standard cache dir and auto-loaded on every later run. rclcpp_kit
> uses it; its ~1.7 s header parse disappears on the second run with nothing to set.

---

## 1. The mechanism (why a PCH, not a dictionary)

Bringup cost is dominated by one call: `cppyy.include("behaviortree_cpp/bt_factory.h")`
JIT-parses the header stack (~0.83–0.91 s, measured). A prior probe
(`docs/bt_kit/REPORT.md` §5 Gap 8) showed a **ROOT/genreflex dictionary does not
help** — it supplies reflection/autoload metadata, not a parsed AST, so Cling
still lazily re-parses the header on first class use.

What *does* work is the mechanism **cppyy already uses for its own std headers**: a
**Cling precompiled header**. Cling loads the PCH named by the `CLING_STANDARD_PCH`
environment variable when the interpreter initialises. We build a PCH that bakes
the kit's headers *on top of* cppyy's standard-header set and point
`CLING_STANDARD_PCH` at it. On the frozen path the header AST is materialised from
the PCH at interpreter start; `cppyy.include(...)` becomes a lookup (~6 ms) instead
of a parse.

The build reuses cppyy's shipped machinery: `rootcling -generate-pch` over
`etc/dictpch/allHeaders.h` + `allLinkDefs.h` with the env's `allCppflags.txt`
(exactly what `etc/dictpch/makepch.py` does), plus the kit header and its include
path inserted. The artifact is a normal Cling PCH, ~48 MB.

### The one snag: internal-linkage statics

The AST-only PCH carries a *declaration* for the header's internal-linkage statics
but the JIT never emits their *definition*, and the library's own copy is a
non-exported local symbol. So JIT-compiled glue that ODR-uses one fails to link.
For bt_kit there is exactly one: `BT::UndefinedAnyType`
(`static std::type_index = typeid(nullptr)` in `safe_any.hpp`, used by every
`Any`/`PortInfo`). The fix, applied **only on the frozen path**, emits one strong,
externally-visible definition under its exact mangled name so every JIT module
resolves to it (`rclcppyy/kits/freeze.py::_FORCE_SYMBOLS`). In L0 the live-parsed
header already defines it, so this glue is *not* applied there (a second definition
would clash). If a freeze of another header surfaces more such symbols, add them to
that per-kit table.

---

## 2. Recipe — freeze bt_kit

*This is the explicit/manual path, useful for CI or full control. For everyday use you
run none of it — §8's zero-config auto-PCH builds and loads the PCH for you.*

```bash
pixi install -e bt
pixi run build                 # install the package (bt_kit + freeze module)
pixi run -e bt freeze-bt-build # build the PCH into build/freeze/  (~30–60 s)

# run anything with the frozen PCH active:
pixi run -e bt demo-bt-t01-frozen
pixi run -e bt test-bt-frozen  # the SAME 16 tests, frozen
pixi run -e bt freeze-bench    # L0 vs L1 numbers (§4)
```

To freeze an arbitrary script, wrap it with the launcher:

```bash
RCLCPPYY_FROZEN=1 python scripts/freeze/run_frozen.py your_script.py [args...]
```

### Why a launcher (the import-order rule)

`CLING_STANDARD_PCH` must be set **before the first `import cppyy`** — Cling binds
its PCH at interpreter init, and setting the variable afterwards is ignored
(measured: 911 ms, i.e. still parsing). Because `import rclcppyy` imports cppyy
transitively, you cannot set it from inside a kit. `scripts/freeze/run_frozen.py`
resolves the artifact *without* importing rclcppyy/cppyy, sets the environment, and
`exec`s the target in the same process image, so the target's first cppyy import
already sees the frozen PCH. `bringup_bt()` warns if `RCLCPPYY_FROZEN` is set but no
frozen PCH is active (i.e. the launcher was bypassed) and falls back to JIT.

---

## 3. Artifact lifecycle

* **Location:** `build/freeze/bt_kit.pch.native.<cppstd>.<cppyy-cling-version>`
  (e.g. `…native.17.6.32.8`). `build/` is gitignored — **never commit the PCH or
  the L2 `.so`** (they are large and environment-specific).
* **Env-version-matched:** the filename carries the C++ standard and the
  cppyy-cling version. A PCH is only valid for the Cling it was built with; a
  version bump changes the tag, so a stale artifact is obvious and
  `freeze.artifact_path()` simply won't find one → the launcher runs JIT and prints
  how to rebuild.
* **Rebuild when:** cppyy-cling or behaviortree_cpp changes version, or the kit's
  header set changes. Just rerun `freeze-bt-build`.
* **Not built?** Everything still works unfrozen (JIT) — the freeze is purely a
  startup-latency optimisation, never a correctness dependency.

---

## 4. Numbers (measured, this machine, shared — medians of cold runs)

`pixi run -e bt freeze-bench`. Bringup is a once-per-process cost, so each sample
is a fresh subprocess.

| Bringup stage | L0 JIT | L1 frozen | speedup |
|---|--:|--:|--:|
| `include(bt_factory.h)` — **the parse** | ~890 ms | **~6 ms** | **~140×** |
| `load_library` | ~6 ms | ~5 ms | 1.2× |
| `cppdef(glue)` | ~50 ms | ~78 ms | 0.6× |
| **bringup total** (through cppdef) | **~950 ms** | **~90 ms** | **~10.7×** |
| first factory + register + tick (first-use JIT) | ~690 ms | ~690 ms | 1.0× |
| **end-to-end `t01_first_tree.py`** (process start→exit) | **~1.9 s** | **~1.1 s** | **1.7× (−0.8 s)** |

**What the freeze removes, plainly:** the ~0.83–0.91 s header **parse**, and only
that. What **remains** after freezing:

* `load_library` (~5 ms) and the `cppdef` C++ glue (~78 ms — slightly *higher*
  frozen, because the glue's template instantiations are JIT-emitted fresh rather
  than reused from the live parse);
* **first-use JIT (~0.69 s, unchanged L0↔L1)** — the subject of the section below.

### First-use JIT: attacked, then moved with `warmup()`

The first tree build pays a one-time, per-signature cost as cppyy JIT-compiles a
call wrapper for each C++ signature it crosses. Localised (measured):

| First-use step | cost | what it is |
|---|--:|---|
| `std.function[sig]` (type) | ~3 ms | template lookup — cheap |
| wrap the Python callable → thunk | ~126 ms | cppyy generates the Python↔C++ thunk |
| `registerSimpleAction(name, fn, ports)` | ~299 ms | cppyy generates the *call wrapper* for that C++ method |
| stateful register (3 hook sigs) | ~342 ms | same, for the shim's signatures |
| 2nd registration (same sig) | ~50 ms | wrapper cached; residual per-call codegen |

**Can the cost itself be cut? (probed, timeboxed)** No, not with cppyy 3.5 levers:

* `EXTRA_CLING_ARGS=-O0` vs `-O1` vs default — **identical** (first register ~401 ms,
  tick rate ~1.41 M/s all three). The cost is Clang **front-end** template
  instantiation, not LLVM optimisation, so the opt level can't touch it.
* A **PCH cannot help**: the frozen path pays the *same* ~690 ms (table above), and
  the localization confirms why — the cost is call-wrapper *codegen triggered by the
  Python call*, not anything an AST-only PCH carries. Pre-instantiating the wrapper
  *types* in the PCH would add AST, not the per-call thunk.
* No per-call-wrapper disk cache exists in cppyy 3.5 (its C++-modules cache is for
  header AST). So the cost is **relocatable or eliminable, not reducible**:
  **relocate** it to init with `warmup()`; **eliminate** it for a hot path by
  lowering to **L2** native (`registerFromPlugin`, §5 — no cppyy in the tick path).

**Moved with `bt_kit.warmup()`** — it exercises every wrapper signature on a
throwaway factory during init (see COMMON_PATTERNS §15). Redistribution (t01-shape
workload, this machine):

| | bringup | warmup (init) | time-to-first-tick | end-to-end |
|---|--:|--:|--:|--:|
| L0, no warmup | ~920 ms | — | **~678 ms** | ~1.80 s |
| L0, warmup | ~905 ms | ~930 ms | **~98 ms** | ~2.14 s |
| **L1 frozen, no warmup** | **~85 ms** | — | **~667 ms** | ~1.01 s |
| **L1 frozen + warmup** | **~85 ms** | ~920 ms | **~94 ms** | ~1.36 s |

The first live tick drops **678 → 98 ms** — the stall moves into a predictable init
phase, which is the point (no surprise halt mid-run). End-to-end rises modestly for
t01 specifically because `warmup()` also warms the *stateful* path (~340 ms) that
t01 doesn't use; for a tree that uses all node kinds the totals converge. The win
is **predictability**, not throughput.

**Cold start, best case = freeze + the compile cache — both automatic now.** The
auto-PCH (§8) removes the ~0.9 s header parse with nothing to set, and the compile
cache (§4, "The compile cache", below) eliminates the first-use wrapper JIT
*persistently*. `warmup()` only *relocates* that JIT to init, so it is no longer the
answer — it stays useful only as a fallback when no compiler/CPyCppyy toolchain is
present. Measured freeze+cache cold start: ~1.77 s → ~0.43 s (below), versus L0's
~920 ms bringup and an unpredictable ~680 ms stall on the first live tick.

### The mechanism generalises (second data point)

Same recipe applied to `rclcpp/rclcpp.hpp` (the rclcpp bringup's dominant cost):

| | L0 JIT | L1 frozen |
|---|--:|--:|
| `include("rclcpp/rclcpp.hpp")` | **~1.71 s** | **~6 ms** (~290×) |

Both libraries collapse to the same ~6 ms PCH-load floor regardless of header
size — evidence the freeze is library-independent, not a BT.CPP special case. (The
rclcpp measurement is parse-elimination only; a full frozen rclcpp bringup would
need its own force-symbol pass and is out of scope here.)

### The compile cache: eliminate the first-use JIT, don't just relocate it

The subsection above says the ~0.7 s first-use call-wrapper JIT is *relocatable*
(warmup) but not *reducible* — true for cppyy's own levers. But it **is**
eliminable, persistently, by not asking cppyy to generate the wrapper at all:
compile the crossing **once** into a real `.so` and `load_library` it thereafter.
This is `cppyy_kit.cppdef_cached` (see COMMON_PATTERNS §23).

The wrapper JIT is Clang front-end codegen — call it at a compiler once, cache the
`.so`, and every later run pays a ~ms symbol call. Two things are cacheable:

* **kit glue** (`makePorts`, the stateful shim, …) — definitions we control,
  split into a bodiless-declarations header (cheap to `cppdef` on a hit) and the
  `.so` (the definitions). Cling emits any body it can *see*, so the fast path
  must give it only declarations.
* **the boundary crossing itself** — the ~0.4 s isn't cppyy's internal codegen
  (that we can't intercept), it's the `std::function<NodeStatus(TreeNode&)>` thunk
  + the `registerSimpleAction` call wrapper. Build **both in compiled code**: a
  trampoline `.so` that constructs the `std::function` wrapping the Python callable
  and does the registration, converting the C++ `TreeNode&` to the Python node
  proxy with cppyy's public `CPyCppyy::Instance_FromVoidPtr`. All the heavy
  instantiation then happens at `.so` build time.

The isolated crossing shows the ceiling: the bare `std::function` thunk + a single
`registerSimpleAction` fall from ~414 ms (JIT) to ~16 ms (cached load + one call)
— the whole first-use JIT gone.

**bt_kit adopted end-to-end (t01: 4 leaves in a Sequence, cold subprocesses, this
machine, `pixi run -e bt bench-cache-bt[-frozen]`):**

| config | first register (first-use) | first tick | end-to-end wall |
|---|--:|--:|--:|
| L0 JIT baseline | ~233 ms | ~8 ms | ~1770 ms |
| L0 + cache (run ≥2) | ~60 ms | ~5 ms | ~1200 ms |
| frozen JIT baseline | ~278 ms | ~9 ms | ~970 ms |
| **frozen + cache (run ≥2)** | **~62 ms** | **~5 ms** | **~425 ms** |
| cached run 1 (miss) | — | — | +~2 s one-time `.so` compile |

So freeze + cache compose: the PCH removes the ~0.89 s **parse**, the cache removes
the bulk of the first-use **wrapper JIT** — best cold start **~1.77 s → ~0.43 s
(~4.1×)**, first-use register **~233 → ~60 ms persistently** (not just moved into a
warmup window; `bt_kit.warmup()` becomes a no-op on the cached path). Run 1 pays a
one-time ~2 s to compile the `.so`; a kit can skip even that by *shipping warm* —
building the `.so` at package-build time (`cppyy_kit.cache.prebuild`) so the
artifact is present on first run.

**The residual ~60 ms** is honest and expected: the cache kills the `std::function`
thunk and the `registerSimpleAction`/`registerStateful` wrapper (the big costs, all
compiled into the `.so`), but cppyy still JIT-generates a call wrapper the first time
Python calls *our* trampoline entry points (`register_py_action`, `makePorts`) —
that codegen is cppyy-internal, not interceptable at this layer. It is a smaller,
simpler-signature wrapper (~60 ms vs ~233 ms), and it is the same cost whether or
not the `.so` is cached.

**Second data point (pcl_kit).** The same mechanism caches PCL's heavy template
first-use: `pcl_kit.voxel_downsample` compiles a `pcl::VoxelGrid<PointXYZ>` into the
kit's `.so`, taking the filter's first-use ~594 ms → ~5 ms and the d02 showcase
frame-0 (`from_msg → voxel → to_msg`) **~681 ms → ~88 ms (~7.7×)** — evidence the
cache is library-independent, like the PCH. (Here the win is instantiating a
*library* template in compiled code, the pcl analogue of bt's callback trampoline.)

**Honest boundary.** This caches the glue/trampolines the kit *authors*. cppyy's
on-demand template instantiations triggered by arbitrary user calls (e.g.
`node.getInput[T](key)` for a new `T`), and the call wrappers cppyy makes to reach
the kit's entry points, are not cached by this. Artifacts are env-version-tagged and
gitignored, same lifecycle as the PCH (§3): a cppyy/compiler/source change is a
clean cache miss, never a silent ABI mismatch. When the compiler/CPyCppyy toolchain
is unavailable the kit falls back to the JIT registration path (a one-time notice),
so the cache is a pure optimisation, never a correctness dependency.

---

## 5. L2 — one leaf lowered to native C++

t01's `ApproachObject` Python leaf, emitted as a native `BT::SyncActionNode` in a
compiled plugin `.so` and registered **JIT-free** via
`factory.registerFromPlugin(...)` (the engine `dlopen`s it; no cppyy, no Python in
the tick path).

```bash
pixi run -e bt freeze-l2-build   # compile scripts/freeze/l2_approach_object.cpp -> .so
pixi run -e bt freeze-l2-diff    # differential test vs the L0 Python leaf
```

Differential result (same tree XML, same node ID):

* **Correctness:** identical stdout (`ApproachObject: approach_object`) and status
  (SUCCESS) — the test is the contract across the rung.
* **Tick rate** (single-leaf tree, SUCCESS/tick, no I/O): L0 Python leaf ~0.55
  µs/tick vs **L2 native ~0.20 µs/tick — ~2.7× faster**, i.e. the Python↔C++
  boundary cost per leaf is removed.

L2 here is hand-written; the point proven is the *rung* — a leaf authored/prototyped
in Python (L0) has a mechanical native equivalent (L2) that passes the same test
and runs at engine speed. Registration still crosses cppyy once
(`registerFromPlugin`), but the leaf executes as native code every tick.

---

## 6. Files

| File | Role |
|---|---|
| `cppyy_kit/freeze.py` | artifact path/version tag, frozen-path detection, force-symbol glue |
| `scripts/freeze/build_bt_pch.py` | build the frozen PCH (rootcling `-generate-pch`) |
| `scripts/freeze/run_frozen.py` | launcher: set `CLING_STANDARD_PCH` before cppyy, exec target |
| `scripts/freeze/bench_freeze.py` | L0-vs-L1 numbers |
| `scripts/freeze/l2_approach_object.cpp` / `build_l2_node.py` / `l2_diff.py` | L2 leaf + build + differential test |
| `bt_kit/tests/test_bt_freeze.py`, `bt_kit/tests/_freeze_helper.py` | frozen-path tests (parse eliminated + correct) |
| `bt_kit/bt_kit/__init__.py` | `bringup_bt()` applies force-symbols when frozen; `bt_kit.frozen()` |

## 7. Limitations

* The PCH is a startup-latency optimisation for the **parse only**; the first-use
  JIT of cppyy call wrappers (~0.7 s for t01) is untouched by it — it is *moved*
  off the first live call by `warmup()`, or *eliminated* persistently by the
  compile cache (§4, "The compile cache"): freeze + cache compose into the
  best-case cold start (~1.77 s → ~0.43 s), leaving a small ~60 ms residual
  (cppyy's call wrappers to the kit's own trampoline entry points).
* Artifacts are Cling-version-specific and must be rebuilt (never committed) on any
  cppyy-cling / library version change.
* Freezing a new header may surface further internal-linkage symbols to force
  (§1); the failure mode is a clear "unresolved while linking" error naming the
  symbol.
* The **manual** launcher must run before any cppyy import (import-order rule, §2);
  the zero-config auto-PCH (§8) removes this constraint by activating from a startup
  `.pth` before any user import.

---

## 8. Zero-config auto-PCH (`cppyy_kit.autopch`)

§1–§4 prove the mechanism but ask the user to build an artifact and launch through a
wrapper. `cppyy_kit.autopch` removes both steps: the PCH is created on first use into
a standard cache dir and auto-loaded on every later run, with a clear line printed for
each event. Nothing to set, no launcher, no pixi task.

### How it engages — a startup `.pth`, so import order does not matter

Cling binds its PCH when the interpreter first imports cppyy, so `CLING_STANDARD_PCH`
must be set before that. Rather than depend on a program importing `cppyy_kit` before
`cppyy` (which many programs do not — `import cppyy` early in a module wins the race),
activation runs from a `.pth` file installed in the environment's site-packages. A
`.pth` line executes at every interpreter start, *before any user import*, so the PCH
binds regardless of import order.

* **The `.pth`** runs `cppyy_kit._autopch_boot.activate()` (installed alongside it as a
  standalone, stdlib-only module). `activate()` reads this environment's manifest, and
  if a matching PCH exists, points `CLING_STANDARD_PCH` at it and sets a marker. It is
  silent, costs a few milliseconds (it imports `cppyy_backend` only, to read the cppyy
  version for the cache key — not cppyy itself), respects `CPPYY_KIT_NO_AUTOPCH=1` and
  any already-set `CLING_STANDARD_PCH`, and never raises (a broken bootstrap would
  otherwise print on every `python` start).
* **`cppyy_kit` self-installs the `.pth`** on first import (a one-time notice), and
  refreshes it if out of date. `python -m cppyy_kit.autopch --uninstall` removes it;
  `--status` shows install + cache state.
* **`cppyy_kit`'s own import** (`autopch.setup()`) reads the marker and prints the one
  user-facing line, `cppyy_kit: Cling PCH loaded from <path>` (a print from the `.pth`
  on every `python` start would be noise). Before the `.pth` exists — the very first
  run — `setup()` still activates from the manifest if cppyy is not yet loaded, so even
  that run can be warm.

A kit declares the headers it parses via the hook

```python
cppyy_kit.register_pch_headers(headers, include_paths=..., force_symbols=None)
```

called at bringup around its `cppyy.include(...)`. On a warm run whose active PCH
already bakes those headers this is a cheap no-op; otherwise the header set is folded
into the environment manifest and a **detached background build** is kicked off at
interpreter exit (guarded by a lockfile, written atomically), so the *next* run is
warm. rclcpp_kit's `bringup_rclcpp()` registers `rclcpp/rclcpp.hpp` +
`rcl_interfaces/msg/parameter_event.hpp` with every ament include dir; no
force-symbols are needed for rclcpp (verified: the full `test-rclcpp` suite passes with
the PCH active). The kit modules import `cppyy_kit` before `cppyy` as a secondary
safety net, so a kit program is warm on its second run even in an environment where the
`.pth` could not be installed (e.g. a read-only site-packages).

### Cache layout — `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch/`

| File | Role |
|---|---|
| `<env-tag>.manifest.json` | the accumulated baked-header set, include paths, and the current header-set's `pch_key` for this env; `<env-tag>` hashes the env prefix and the cppyy/backend versions |
| `<pch-key>.pch` | the artifact; `<pch-key>` hashes the same env material **and** the header set, so any change is a clean miss (never a silent ABI mismatch) |
| `<pch-key>.pch.json` | metadata (env tag, headers, version) used to group artifacts for pruning |
| `<pch-key>.pch.log` | the background build's output (and the prune summary), for diagnosis |
| `<pch-key>.pch.lock` | held while a build is in flight (prevents double-builds) |

A rebuilt env or an upgraded cppyy changes the tag/key, so a stale artifact is simply
not found and the run falls back to JIT — the same lifecycle as the manual PCH (§3),
and nothing is ever committed. **Pruning:** after each successful build the cache is
trimmed to the newest few PCHs per environment (plus any still referenced by a live
manifest), and orphaned sidecars, stale locks, and dead-environment manifests are
swept — so accumulated artifacts from many environments do not pile up. Prune manually
with `python -m cppyy_kit.autopch --prune`.

### Measured (rclcpp bringup, this machine)

Each row is a fresh process; the "header parse" is the `rclcpp C++ headers loaded (…)`
line, bringup is the whole `bringup_rclcpp()` call.

| Run | header parse | bringup total | notes |
|---|--:|--:|---|
| cold (auto-PCH disabled) | ~1.9 s | ~1.91 s | baseline JIT |
| first run (empty cache) | ~1.9 s | ~1.92 s | JIT + `building …` printed; build scheduled |
| **warm run (PCH loaded)** | **~0.0 s** | **~0.06 s** | `Cling PCH loaded from …` printed |

The header parse is eliminated (~1.9 s → ~0 s) and bringup drops **~30×** on the warm
run, with no user action between the two. As with the manual freeze, this removes the
**parse** only; cppyy's first-use call-wrapper JIT is a separate cost (see §4, the
compile cache).

### Files

| File | Role |
|---|---|
| `cppyy_kit/_autopch_boot.py` | standalone, stdlib-only bootstrap; `activate()` runs from the `.pth` and is the single source of the cache-path/key logic (shared with `autopch`) |
| `cppyy_kit/autopch.py` | `setup()`, `register_pch_headers()`, `.pth` self-install/uninstall, `generate_pch()`, at-exit scheduler, `prune()`, the `python -m` CLI |
| `cppyy_kit/autopch_build.py` | detached worker that builds a PCH from a manifest, prunes, and releases the lock |
| `cppyy_kit/tests/test_autopch.py` | hermetic tests (keys/invalidation, override, `.pth` install/uninstall/opt-out/crash-proofing, manifest union, scheduling, pruning, cross-process pickup) + an opt-in real-build test |

## 9. Debugging: turning the caches off

cppyy_kit keeps **two independent caches**, and both are pure optimisations you can
switch off when a run misbehaves and you want to rule caching out. They cover different
costs and have different switches:

| Cache | What it removes | Artifacts | Turn it off with |
|---|---|---|---|
| **auto-PCH** (§8) | the header *parse* at bringup | `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch/*.pch` | `CPPYY_KIT_NO_AUTOPCH=1` (env, before launch) |
| **compile cache** (§4, `cppdef_cached`) | cppyy's first-use call-wrapper *JIT* (kernel `.so`s, incl. `@cpp`) | `$CPPYY_KIT_CACHE_DIR` or `<cwd>/build/cppyy_kit_cache/` | `CPPYY_KIT_NO_CACHE=1` (env) · `cppyy_kit.disable_caching()` (runtime) · `cached=False` (per call) |

Both are content-addressed and self-invalidating, so a *stale* artifact is normally a
clean miss (rebuild), never a silent wrong answer. These switches are for when you
suspect the cache anyway — a miscompiled kernel, a debugger that needs source, an
edit that isn't taking.

### Decision tree

* **Bringup is slow, or a header change isn't taking, or a symbol resolves wrong at
  parse time → suspect the PCH.** Launch with `CPPYY_KIT_NO_AUTOPCH=1` to run entirely
  on the JIT path (the `.pth` and `setup()` both honour it — no PCH is bound or built).
  If the JIT path is healthy, the baked PCH is stale: rebuild it by pruning
  (`python -m cppyy_kit.autopch --prune`) or remove the startup hook entirely
  (`python -m cppyy_kit.autopch --uninstall`). `--status` shows what is installed and
  how many PCHs are cached. Because the PCH binds at interpreter start (via the `.pth`),
  it can only be disabled by the **env var / CLI** — there is no runtime toggle (by the
  time Python code runs, Cling has already bound it).

* **A kernel gives wrong results, or a `@cpp`/`cppdef_cached` edit isn't taking, or you
  need to step into the source → suspect a stale kernel `.so`.** Three bypasses, all
  making `cppdef_cached` behave exactly like a plain in-memory `cppyy.cppdef(code)` (no
  `.so` read, no `.so` write):
  * **Per call:** `@cpp(cached=False)` or `cppyy_kit.cppdef_cached(..., cached=False)`
    — narrow, leaves everything else cached.
  * **Runtime, process-wide:** `cppyy_kit.disable_caching()` (undo with
    `enable_caching()`), or the scoped `with cppyy_kit.caching_disabled(): ...`.
  * **Whole process, before import:** set `CPPYY_KIT_NO_CACHE=1` in the environment.

  If the run is then correct, the cached `.so` was stale — nuke it (below) so the next
  cached run rebuilds it clean.

### Where the artifacts live, and how to nuke them safely

* **Compile cache.** `$CPPYY_KIT_CACHE_DIR` if set, else `<cwd>/build/cppyy_kit_cache/`,
  under a version-tagged subdir. Everything there is regenerable and gitignored.
  `cppyy_kit.clear_cache()` deletes every artifact in the active (version-tagged) dir
  and returns the count; `cppyy_kit.cache_info()` lists what's there; `cache_dir()`
  prints the path. Deleting the directory by hand is equally safe — a missing `.so` is
  just a miss. (Already-loaded `.so`s stay mapped in the running process; the switches
  above only affect *later* calls, so bounce the process to fully drop them.)
* **auto-PCH.** `${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch/`. `python -m cppyy_kit.autopch
  --prune` trims to the newest per environment (keeping any a live manifest references);
  deleting the dir is safe (the next run falls back to JIT and reschedules a build).

Both caches key on the cppyy/compiler versions and the source, so upgrading cppyy or
editing the C++ is *already* a clean miss — reach for these switches only to force the
issue while debugging.

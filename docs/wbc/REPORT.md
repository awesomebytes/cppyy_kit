# wbc_kit spike — Whole-Body Control frameworks via cppyy (M6e)

**Date:** 2026-07-12 · **Env:** pixi `wbc` (standalone, conda-forge only),
`pinocchio 4.0.0`, `crocoddyl 3.2.1`, `tsid 1.x`, `casadi 3.7.2`,
`example-robot-data 5.0.0`, `libboost 1.90`, `cppyy 3.5.0`, Python 3.12.13, linux-64.

**Question (from PLAN.md M6e):** tsid / crocoddyl / pinocchio are all on conda-forge
**with** Python bindings (verified). So the cppyy win — if there is one — must be
**sharper than "bindings exist"**. Where, for whole-body control, does cppyy do
something the bindings genuinely cannot? "Bindings are fine, no kit needed" is an
acceptable honest answer.

**Verdict: GO, narrowly and specifically — Crocoddyl.** The sharp, measurable,
demo-worthy win is **authoring a custom Crocoddyl action model in C++ inline and
JIT-compiling it at runtime with no build system**, so the DDP solver calls its
`calc`/`calcDiff` natively. On the canonical unicycle problem this runs at the exact
speed of Crocoddyl's compiled built-in model and **~21x faster** than the
Python-derived model that Crocoddyl's bindings support — converging to a
bit-identical cost. This is the ompl_kit "lower the hot path to C++" story applied
to optimal control, and it is a genuinely new capability: the bindings let you
prototype a custom model in Python (slow) or ship one in C++ (needs a CMake build);
cppyy fills the missing "fast **and** no build system, in the same script" cell. A
thin `wbc_kit` (bringup + the crash-safe custom-model compile + the one mandatory
Crocoddyl-3.2 boilerplate) is justified. pinocchio's templated-scalar angle and
tsid's custom-task angle were evaluated and are **weaker** (see the table).

---

## 1. Decision table — where does cppyy actually win for WBC?

| Framework | Binding today | Failure-mode the binding leaves | cppyy angle | Effort | Demo value | Evidence |
|---|---|---|---|:--:|:--:|---|
| **Crocoddyl 3.2** | boost::python; **can** subclass `ActionModelAbstract` in Python | Python-authored models are **slow** in the DDP hot loop; the fast path (C++ model) needs a CMake project + rebuild | **Author the custom C++ action model inline (`cppdef`), JIT'd at runtime — native speed, no build system.** Prototype in Python, *lower* to inline C++ in the same script | **Low–Med** | **HIGH** | S2/S3: WORKS. 21.7x vs Python model, == built-in speed, cost bit-identical; bringup clean (~0.4 s JIT, no boost/ORC wall) |
| **pinocchio 4.0** | double + **casadi** scalar (`pinocchio.casadi`) shipped; cppad **not** built | Non-casadi scalar surfaces (`ModelTpl<Scalar>` for a scalar the binding never built) | Instantiate `ModelTpl<Scalar>` on demand (the pcl `PointCloud<T>` pattern) | Med | Low–Med | S4: **BLOCKED in this env** — pinocchio's 25-type `JointModel` `boost::variant` exceeds **boost 1.90**'s template-arity when re-instantiated for a new scalar (`ModelTpl<float>` and `.cast<float>()` both fail to compile). casadi (the main autodiff scalar) already shipped |
| **tsid 1.x** | boost::python; bindings expose **only concrete task types** (no subclassable `TaskBase`/`TaskMotion` trampoline) | Authoring a **new task type** from Python isn't exposed | Cross-inheritance: derive `TaskMotion` and override `compute` (plain virtuals, not `final`) | Med | Med | S5: expected WORKS (same pattern as ompl/control), **not probed** — it is the *same* cross-inheritance capability those kits already prove, so lower novelty. Bringup shares Crocoddyl's clean pinocchio+boost stack |
| **OCS2** (C++-only) | none | — | Would be a pure cppyy target | — | — | S6: **not on conda-forge / robostack**; source build is large (multi-package MPC framework) -> out of scope, documented |
| **mc_rtc** (C++-only) | own Python bindings, own build | — | — | — | — | S6: **not on conda-forge / robostack** -> out of scope |
| **proxsuite / QP** | conda-forge package **with** bindings | — | none | — | — | S6: bindings are complete; no cppyy win. Honest "no kit needed" |

**Chosen probe target: Crocoddyl** — the only candidate whose cppyy win is
simultaneously (a) sharper than "bindings exist", (b) measurable end-to-end, and
(c) demo-worthy in a new domain (optimal control) rather than a re-run of the
cross-inheritance pattern ompl_kit/control_kit already own.

---

## 2. Probe — Crocoddyl capability matrix

Each probed in a fresh subprocess (S20: risky includes/`cppdef` out-of-process first)
from the `wbc` env against the installed Crocoddyl 3.2.1.

| # | Capability | Result | Evidence |
|---|---|:--:|---|
| 1 | **Bringup + JIT**: `pinocchio/fwd.hpp` + Crocoddyl core/action/state/solver headers, load `libpinocchio_default.so` + `libcrocoddyl.so` | **WORKS** | Header JIT **~420 ms** (pinocchio/fwd 224 + crocoddyl core 155 + solvers 44); lib load ~5 ms. **No boost/ORC wall** despite the eigen+boost+pinocchio stack (S20's warning did not bite here). Comparable to ompl (~538 ms) |
| 2 | **Built-in model via cppyy**: construct `crocoddyl::ActionModelUnicycle`, run `calc` | **WORKS** | `xnext=[1.05, 0, 0.01]`, `cost=50.13` for `x=[1,0,0], u=[0.5,0.1]` — exact |
| 3 | **Custom C++ action model (cross-inheritance in C++)**: `cppdef` a subclass of `crocoddyl::ActionModelAbstract`, drive a real FDDP solve | **WORKS** | 100-node unicycle, FDDP -> `cost=250.039320`, 8 iters, converged. Solve driven entirely in C++ (Pattern 6 containers). See S3 |
| 4 | **Numeric verification** vs the binding's built-in model | **WORKS** | The inline-C++ model's converged cost is **bit-identical** to Crocoddyl's compiled `ActionModelUnicycle` and to a Python-derived model (all three: `250.039320`, 8 iters) |
| 5 | **Benchmark** (the honest number) | **WORKS** | S3 — 21.7x vs Python model; == built-in C++ speed |

**One sharp edge (fixed in the kit):** authoring a custom model is a
**failed-`cppdef`-crash** minefield (S20 Pattern 9). Two forced dead-ends before it
compiled: (a) `calc`/`calcDiff` signatures must match the base's
`const Eigen::Ref<const VectorXs>&` exactly; (b) **Crocoddyl 3.2's `CROCODDYL_BASE_CAST`
macro adds two pure-virtual clone methods** (`cloneAsDouble`/`cloneAsFloat`) that a
subclass **must** implement or it stays abstract. Each mistake crashed Cling during
transaction revert (no Python traceback). `wbc_kit.safe_cppdef` probes the model
out-of-process first and raises a clean `CppyyKitError`; `wbc_kit.ACTION_MODEL_CLONES`
is the mandatory boilerplate.

---

## 3. Bench — custom action model, three ways (the honest number)

Same unicycle optimal-control problem (T=100 nodes, FDDP, x0=[-1,-1,1]), authored
and solved three ways. **Shared machine during measurement — provisional,
directional not exact** (best of 7 after warm-up).

| model authoring path | cost | iters | solve | vs Python-model |
|---|--:|--:|--:|--:|
| **(A) Python-derived** (subclass `ActionModelAbstract` in Python — the binding's prototype path) | 250.039320 | 8 | **6.84 ms** | 1.0x |
| **(ref) built-in C++** (`crocoddyl::ActionModelUnicycle`, compiled in the binding) | 250.039320 | 8 | **0.34 ms** | 20.2x |
| **(B) cppyy inline C++** (custom model `cppdef`'d at runtime, no build system) | 250.039320 | 8 | **0.32 ms** | **21.7x** |

**Reading these honestly:**
- **All three converge to a bit-identical cost** — the inline-C++ model is a faithful
  *lowering* of the Python prototype, not an approximation. This is the tests-as-
  contract discipline: the numeric match is the regression gate (`test_wbc_kit.py`).
- **The inline-C++ model runs at the compiled built-in's speed** (0.32 vs 0.34 ms) —
  cppyy's JIT'd C++ is *native* C++; there is no Python in the DDP hot loop.
- **~21x over the Python-derived model.** The DDP solver calls `calc`/`calcDiff` per
  node per iteration (plus line-search rollouts) — thousands of crossings; the Python
  model pays the boundary + NumPy-allocation cost on every one. This is the crocoddyl
  analogue of ompl_kit's "Python validity checker in the hot loop" figure, in a domain
  (trajectory optimization) where the callback truly dominates.
- **The win is "fast *and* no build system, same script."** Crocoddyl's own workflow
  is "prototype in Python, rewrite the hot model in C++"; the rewrite normally means a
  CMake project linking libcrocoddyl. cppyy makes the lowered C++ model a `cppdef`
  string in the same file — the exact ROSCon storyline (prototype -> lower -> benchmark,
  code stays ~the same).

Run it: `pixi run -e wbc demo-wbc-lower` (and `pixi run -e wbc test-wbc`).

---

## 4. pinocchio templated-scalar — BLOCKED in this env (the plan's named gap)

The plan flagged `pinocchio::ModelTpl<Scalar>` for a non-double `Scalar` (autodiff)
as the pinocchio angle. Findings:
- **casadi is already shipped.** `pinocchio.casadi` (cpin) imports; the conda-forge
  feedstock builds `WITH_CASADI`. So the main autodiff scalar is a binding feature,
  not a cppyy gap.
- **Non-casadi scalars are env-blocked.** `ModelTpl<float>` and the
  build-as-double-then-`.cast<float>()` path both **fail to compile** — not a Cling
  quirk (`g++` reproduces it): pinocchio's `JointModelVariant` is a **25-type
  `boost::variant`**, and re-instantiating it for a new scalar hits **boost 1.90**'s
  `make_variant_list` template-arity limit (`wrong number of template arguments (25,
  should be at least 0)`). The shipped double/casadi libraries sidestep this by being
  *precompiled*; JIT-instantiating a fresh scalar from headers does not.
- Additionally, pinocchio's `buildModels::` sample builders hardcode `double` inertias,
  so the ergonomic "any scalar on demand" (the pcl `PointCloud<T>` pattern) does not
  transfer cleanly even setting boost aside.

**So the plan's named gap is real in principle but env-blocked here.** It might be
pried open with boost-preprocessor arity defines (a S20 "peel one layer" exercise),
but that is a dependency-config fight, not a clean cppyy win — and casadi already
covers the motivating use case. Honest verdict: **not the probe target.**

---

## 5. tsid custom tasks — genuine but not novel

tsid's boost::python bindings expose **only concrete task classes**
(`TaskSE3Equality`, `TaskComEquality`, `TaskJointPosture`, ...) — there is no exposed
subclassable `TaskBase`/`TaskMotion` with a virtual trampoline, so you **cannot
author a new task type from Python** through the binding. `TaskMotion`'s virtuals
(`compute`, `getConstraint`, ...) are plain (not `final`), so a Python or C++ subclass
via cppyy cross-inheritance would work — this is a real capability the binding lacks.
**But it is the *same* cross-inheritance pattern ompl_kit and control_kit already
prove** (Python derives a C++ virtual base; S16). Lower novelty, medium demo value,
and it shares Crocoddyl's clean pinocchio+boost bringup. Documented as a viable
follow-on, not the headline.

---

## 6. C++-only candidates & environment findings

- **OCS2** — not on conda-forge or robostack; a large multi-package MPC framework
  (source build with catkin/ROS deps). A pure cppyy target in principle, but the
  source-build size puts it out of scope for this spike. Documented.
- **mc_rtc** — not on conda-forge/robostack; ships its own Python bindings + build.
  Out of scope.
- **proxsuite** and the QP-solver ecosystem — on conda-forge **with** complete
  bindings. No cppyy win; honest "no kit needed."

### Environment / lock changes (flag for the lead)
- **Added `[feature.wbc]` + `wbc` env to pixi.toml — STANDALONE (`no-default-feature`),
  NOT `solve-group="default"`.** The plan suggested solve-group default; it is
  **infeasible** and I switched to standalone on evidence:
  - conda-forge pinocchio/crocoddyl/tsid can only co-resolve with the robostack-jazzy
    ROS stack if they share a boost, but the shared solve-group is over-constrained by
    the ROS stack — `pixi install -e wbc` under solve-group default **fails**
    (`libboost 1.86 ... conflicts with the versions reported above`; some pinocchio
    builds also demanded python 3.9). pinocchio/crocoddyl/tsid carry **no ROS
    dependency**, so a standalone conda-forge env is the correct home; it resolves
    cleanly (boost 1.90, py3.12) and **leaves the shared ROS lock untouched**.
  - The standalone `wbc` feature re-declares its own `python`/`cppyy`/`compilers`/
    `numpy`/`pytest`, since it does not inherit the default.
- **`pixi.lock` re-locked** to add the `wbc` env's solve (a new standalone env; the
  default and existing kit envs are unchanged — the standalone choice was made
  specifically to avoid perturbing the shared lock). **Re-lock flagged as requested.**
- Registered `wbc_kit` in the `lint` and `test` pixi tasks and on the default
  `PYTHONPATH`; the new tests **auto-skip** outside the `wbc` env (verified: `6 skipped`
  in the default env), so the default suite is untouched.

---

## 7. Generic-lesson candidates for COMMON_PATTERNS

Noted for the lead (COMMON_PATTERNS.md is the lead's to edit):

- **Cross-binding-runtime co-existence (NEW nuance for S16/S20).** A library can ship
  **boost::python** bindings *and* be driven by cppyy **in the same process** — both
  load the same `libcrocoddyl.so`, the C++ objects are separate. The clean division of
  labour: **prototype with the library's own binding, lower the hot path with cppyy**,
  in one script. But a **cppyy-created C++ object cannot be handed to a boost::python
  API** (two proxy runtimes) — so the cppyy path must build its own containers/solve in
  C++ (Pattern 6), not feed the binding's objects. Worth a sentence: "mixing a library's
  own binding with cppyy is fine and often ideal; just don't pass objects *between* the
  two runtimes."

- **Versioned pure-virtual creep across the inheritance boundary (sharpens S16).**
  A library minor version can **add pure virtuals to a base you subclass** and silently
  make your override abstract — Crocoddyl 3.2's `CROCODDYL_BASE_CAST` added
  `cloneAsDouble`/`cloneAsFloat` to `ActionModelAbstract`. In C++ it is a compile error;
  **in cppyy it is a failed-`cppdef` crash** (S9/S20 Pattern 9). Rule: when
  cross-inheriting a C++ base, `nm`/grep the base for **every** pure virtual (including
  macro-injected ones) before authoring; probe the subclass `cppdef` out-of-process. A
  kit should ship the mandatory boilerplate as a constant (wbc_kit.ACTION_MODEL_CLONES).

- **`boost::variant` template-arity is an env-version JIT wall (new S20 sub-case).**
  A big `boost::variant` (pinocchio's 25-type `JointModel`) that a **precompiled** `.so`
  carries fine can be **un-JIT-able from headers** under a newer boost whose
  preprocessed arity limit it exceeds — parse fails, not execution (distinct from the
  ORC static-init wall). Suspect it when a template *class* re-instantiation for a new
  parameter fails but the shipped specialization works. Reproduce with `g++` to
  distinguish from a Cling quirk.

- **"Slow Python subclass, fast C++ subclass, no build system" is a recurring shape.**
  Third instance of the *lowering* pattern after ompl_kit (validity checker) and
  control_kit (controller): a framework whose hot loop calls a user-authored virtual
  (OMPL validity, ros2_control update, Crocoddyl `calc`/`calcDiff`) is the ideal cppyy
  target — cppyy uniquely offers the *fast* authoring path (inline C++) without the
  framework's usual build step. Candidate for a named pattern: "lower the hot virtual."

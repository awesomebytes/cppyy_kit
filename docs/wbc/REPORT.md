# wbc_kit study: Whole-Body Control frameworks via cppyy

**Date:** 2026-07-12 · **Env:** pixi `wbc` (standalone, conda-forge only),
`pinocchio 4.0.0`, `crocoddyl 3.2.1`, `tsid 1.x`, `casadi 3.7.2`,
`example-robot-data 5.0.0`, `libboost 1.90`, `cppyy 3.5.0`, Python 3.12.13, linux-64.

**Question:** tsid, crocoddyl, and pinocchio are all on conda-forge with Python
bindings. This study looked for work that cppyy can do that these bindings cannot.
If the bindings cover a framework's use cases, then no cppyy kit is needed.

**Result: Crocoddyl is the best target.** The measured benefit is that users can
author a custom Crocoddyl action model in inline C++ and JIT-compile it at runtime
without a build system. The DDP solver then calls its `calc`/`calcDiff` natively.
On the canonical unicycle problem, this runs at the speed of Crocoddyl's compiled
built-in model and 21.7 times faster than the Python-derived model. All three
models reach the cost 250.039320 in 8 iterations. Crocoddyl's bindings support Python models and
compiled C++ models. cppyy lets users write and compile the C++ model in the same
script. A small `wbc_kit` is useful for bringup, safe model compilation, and required
Crocoddyl 3.2 boilerplate. pinocchio's templated-scalar feature and tsid's custom-task
feature were also evaluated (see the table).

---

## 1. Framework comparison

| Framework | Binding today | Failure-mode the binding leaves | cppyy angle | Effort | Demo value | Evidence |
|---|---|---|---|:--:|:--:|---|
| **Crocoddyl 3.2** | boost::python; supports Python subclasses of `ActionModelAbstract` | Python models are slow in the DDP loop. C++ models usually require a CMake project and rebuild. | Define a custom action model in inline C++ with `cppdef`. | Low to medium | High | S2/S3: 21.7x faster than the Python model, same cost as built-in C++; header JIT about 0.4 s |
| **pinocchio 4.0** | double + **casadi** scalar (`pinocchio.casadi`) shipped; cppad **not** built | Non-casadi scalar surfaces (`ModelTpl<Scalar>` for a scalar the binding never built) | Instantiate `ModelTpl<Scalar>` on demand (the pcl `PointCloud<T>` pattern) | Med | Low–Med | S4: **BLOCKED in this env**, pinocchio's 25-type `JointModel` `boost::variant` exceeds **boost 1.90**'s template-arity when re-instantiated for a new scalar (`ModelTpl<float>` and `.cast<float>()` both fail to compile). casadi (the main autodiff scalar) already shipped |
| **tsid 1.x** | boost::python; bindings expose concrete task types only | Python bindings do not expose a subclassable `TaskBase` or `TaskMotion` trampoline. | Derive `TaskMotion` through cppyy and override `compute`. | Medium | Medium | S5: not tested; uses the cross-inheritance pattern used in ompl_kit and control_kit. Bringup shares Crocoddyl's pinocchio and boost packages. |
| **OCS2** (C++-only) | none | none | Would be a pure cppyy target | N/A | N/A | S6: **not on conda-forge / robostack**; source build is large (multi-package MPC framework), so it is out of scope |
| **mc_rtc** (C++-only) | own Python bindings, own build | none | none | N/A | N/A | S6: **not on conda-forge/robostack**, so it is out of scope |
| **proxsuite / QP** | conda-forge package **with** bindings | none | none | N/A | N/A | S6: bindings are complete; no cppyy benefit, so no kit is needed |

**Probe target: Crocoddyl.** It is the only candidate with a measurable end-to-end
cppyy benefit in optimal control. The other kits already cover cross-inheritance.

---

## 2. Crocoddyl capability results

Each probe ran in a fresh subprocess in the `wbc` environment, against Crocoddyl 3.2.1.

| # | Capability | Result | Evidence |
|---|---|:--:|---|
| 1 | Bringup and JIT: include pinocchio and Crocoddyl headers; load both shared libraries | Yes | Header JIT took about **420 ms**: pinocchio/fwd 224 ms, Crocoddyl core 155 ms, solvers 44 ms. Library loading took about 5 ms. Comparable to OMPL at about 538 ms. |
| 2 | Construct `crocoddyl::ActionModelUnicycle` through cppyy and call `calc` | Yes | `xnext=[1.05, 0, 0.01]`, `cost=50.13` for `x=[1,0,0], u=[0.5,0.1]`. |
| 3 | Define a C++ subclass of `ActionModelAbstract` with `cppdef` and run FDDP | Yes | The 100-node unicycle solve converged to `cost=250.039320` in 8 iterations. The solve ran in C++ using Pattern 6 containers. See S3. |
| 4 | Compare against the built-in model | Yes | The inline C++ model, built-in model, and Python model all reached `250.039320` in 8 iterations. Costs were bit-identical. |
| 5 | Benchmark inline C++ against the Python model | Yes | S3 measured a 21.7x speedup over the Python model and similar speed to the built-in C++ model. |

**Failure handling:** an error during `cppdef` can crash the process (S20 Pattern 9).
Two compile errors were found while developing the model. First, `calc` and `calcDiff`
signatures must match the base class's `const Eigen::Ref<const VectorXs>&`. Second,
**Crocoddyl 3.2's `CROCODDYL_BASE_CAST`
macro adds two pure-virtual clone methods** (`cloneAsDouble`/`cloneAsFloat`) that a
subclass **must** implement or it stays abstract. Each mistake crashed Cling during
transaction revert (no Python traceback). `wbc_kit.safe_cppdef` probes the model
out-of-process first and raises `CppyyKitError`; `wbc_kit.ACTION_MODEL_CLONES`
contains the required clone methods.

---

## 3. Action model benchmark

Same unicycle optimal-control problem (T=100 nodes, FDDP, x0=[-1,-1,1]), authored
and solved three ways. The machine was shared during measurement, so results are
provisional (best of 7 after warm-up).

| model authoring path | cost | iters | solve | vs Python-model |
|---|--:|--:|--:|--:|
| **(A) Python-derived** (subclass `ActionModelAbstract` in Python, the binding's prototype path) | 250.039320 | 8 | **6.84 ms** | 1.0x |
| **(ref) built-in C++** (`crocoddyl::ActionModelUnicycle`, compiled in the binding) | 250.039320 | 8 | **0.34 ms** | 20.2x |
| **(B) cppyy inline C++** (custom model `cppdef`'d at runtime, no build system) | 250.039320 | 8 | **0.32 ms** | **21.7x** |

**Results:**
- **All three converge to a bit-identical cost.** The inline-C++ model implements the
  same math as the Python prototype. The numeric match is checked in `test_wbc_kit.py`.
- **The inline-C++ model runs at the compiled built-in's speed** (0.32 vs 0.34 ms).
  There is no Python in the DDP hot loop.
- **~21x over the Python-derived model.** The DDP solver calls `calc`/`calcDiff` per
  node per iteration (plus line-search rollouts), causing thousands of calls. The Python
  model pays the Python boundary and NumPy allocation cost on each call. This is similar
  to the OMPL validity-checker benchmark, in trajectory optimization.
- **The benefit is C++ without a separate build system.** Crocoddyl's own workflow
  is "prototype in Python, rewrite the hot model in C++"; the rewrite normally means a
  CMake project linking libcrocoddyl. cppyy makes the lowered C++ model a `cppdef`
  string in the same file. The workflow is prototype, translate to C++, then benchmark.

Run it: `pixi run -e wbc demo-wbc-lower` (and `pixi run -e wbc test-wbc`).

---

## 4. pinocchio templated scalar: blocked in this environment

The plan flagged `pinocchio::ModelTpl<Scalar>` for a non-double `Scalar` (autodiff)
as the pinocchio angle. Findings:
- **casadi is already shipped.** `pinocchio.casadi` (cpin) imports; the conda-forge
  feedstock builds `WITH_CASADI`. So the main autodiff scalar is a binding feature,
  not a cppyy gap.
- **Non-casadi scalars are env-blocked.** `ModelTpl<float>` and the
  build-as-double-then-`.cast<float>()` path both **fail to compile**, not a Cling
  quirk (`g++` reproduces it): pinocchio's `JointModelVariant` is a **25-type
  `boost::variant`**, and re-instantiating it for a new scalar exceeds **boost 1.90**'s
  `make_variant_list` template-arity limit (`wrong number of template arguments (25,
  should be at least 0)`). The shipped double/casadi libraries sidestep this by being
  *precompiled*; JIT-instantiating a fresh scalar from headers does not.
- Additionally, pinocchio's `buildModels::` sample builders hardcode `double` inertias,
  so the ergonomic "any scalar on demand" (the pcl `PointCloud<T>` pattern) does not
  transfer cleanly even setting boost aside.

The plan's proposed gap exists in principle, but this environment blocks it. Boost
preprocessor arity defines might resolve the issue (an S20 "peel one layer" exercise),
but that would require dependency configuration. Casadi already covers the main use
case, so pinocchio is not the probe target.

---

## 5. tsid custom tasks

tsid's boost::python bindings expose concrete task classes such as
`TaskSE3Equality`, `TaskComEquality`, and `TaskJointPosture`. They do not expose a
subclassable `TaskBase` or `TaskMotion` trampoline, so Python cannot define a new task
type through the bindings. `TaskMotion`'s virtual methods, including `compute` and
`getConstraint`, are not `final`. A cppyy subclass can override them.
This is the same cross-inheritance pattern used in ompl_kit and control_kit (Python
derives a C++ virtual base; S16). It has lower novelty and medium demo value. It also
uses the same pinocchio and boost setup as Crocoddyl. It is a possible follow-on.

---

## 6. C++-only candidates and environment

- OCS2 is not available on conda-forge or robostack. It is a multi-package MPC
  framework with ROS and catkin build dependencies, so building it from source is
  outside this study.
- mc_rtc is not available on conda-forge or robostack. It has its own Python bindings
  and build system.
- proxsuite and the QP solver packages have complete conda-forge bindings. No cppyy
  package is needed for them.

### Environment and lock changes

- Added the standalone `[feature.wbc]` feature and `wbc` environment in `pixi.toml`.
  The default solve group failed: pinocchio, Crocoddyl, and tsid required Boost 1.86,
  which conflicted with the robostack Jazzy ROS stack. Some pinocchio packages also
  required Python 3.9. These packages do not depend on ROS, so the standalone
  conda-forge environment uses Boost 1.90 and Python 3.12.
- **Correction (2026-07-12):** conda-forge rebuilt pinocchio 4.x against Boost 1.90.
  pinocchio and example-robot-data now resolve with the robostack ROS stack in the
  default solve group. The `retarget-ros` environment uses this setup. Crocoddyl and
  tsid still require the older Boost version here, so `wbc` remains standalone. The
  Cling template-arity error for `pinocchio::Model` remains with Boost 1.90; use
  pinocchio's bindings for rigid-body computations.
- The standalone `wbc` feature declares its own Python, cppyy, compilers, NumPy, and
  pytest dependencies. Its lock entries were added without changing existing
  environments. `wbc_kit` was added to the lint and test tasks and default `PYTHONPATH`.
  Its tests skip when Crocoddyl is absent; six tests skipped in the default environment.

---

## 7. Notes for COMMON_PATTERNS

These notes are for the maintainer of COMMON_PATTERNS.md.

- A library's boost::python binding and cppyy can load the same shared library in one
  process. Their C++ objects use different proxy runtimes and cannot be passed between
  them. Use the library binding to prototype, then build cppyy containers and the solve
  in C++ (Pattern 6).
- A library update can add pure virtual methods to a base class. Crocoddyl 3.2 added
  `cloneAsDouble` and `cloneAsFloat` to `ActionModelAbstract` through
  `CROCODDYL_BASE_CAST`. A subclass that omits them remains abstract. A failed cppyy
  `cppdef` can crash Cling (S9/S20 Pattern 9). Check for all pure virtual methods,
  including macro-generated methods, and test risky subclasses in a subprocess.
- Pinocchio's 25-type `JointModel` `boost::variant` is available in the precompiled
  library but exceeds Boost 1.90's `make_variant_list` template arity when Cling tries
  to instantiate it from headers for a new scalar. This is a compile-time error, not
  an execution error. Reproduce it with `g++` to distinguish it from a Cling issue.
- A framework may call user-defined virtual methods many times in a hot loop. Moving
  such a method from a Python subclass to inline C++ can avoid per-call Python overhead
  without a separate build system. Examples here are OMPL validity checks,
  ros2_control updates, and Crocoddyl `calc` and `calcDiff`.

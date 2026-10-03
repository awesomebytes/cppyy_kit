# Architecture decision: cppyy_kit and the kit suite

This is a maintainer decision record, not setup guidance. For installation or
development steps, see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/).
Section 4 records the approved direction; Sections 1–3 preserve the earlier
proposal evaluation and layout.

**Status: approved direction (2026-07-11).** Section 4 records two changes to
the original proposal. The ROS core is packaged as **`rclcpp_kit`**. The
standalone **`rclcppyy`** product depends on that kit. The kit suite moves to a
separate repository. Sections 1-3 contain the proposal evaluation. Section 4
records the approved layout.

---

## 1. Design proposal evaluation

### 1.1 Proposed features to adopt

| Idea (doc §) | Decision | Evidence and implementation |
|---|---|---|
| **Content-hash compile cache** (§5.7) | **Adopt** | The PCH reduced header parsing from 890 ms to 6 ms. It did not reduce the per-signature wrapper JIT, which took about 0.69 s in both L0 and L1 and did not improve with compiler flags. Hash each C++ source, compile it to a `.so`, and load that artifact on later calls. Apply the existing L2 direct-compile recipe to each `cppdef`. The direct-compile recipe (§21) and artifact tagging in FREEZE.md provide starting points. |
| **`require()` header-only fetcher** (§5.5) | **Adopt, conda-first** | Extend the vendored-source pattern from §21, which DBoW2 currently handles with a custom script. Prefer conda or Pixi packages. Use `require()` for un-packaged header-only libraries or when a specific version is needed. Cache files and verify checksums, as the dataset tools do. |
| **`@cpp` decorator** (§5.6) | **Adopt** | Combine the signature inference used by `callback()` with the marshaling patterns in §6. The decorated function body is not executed by Python. Its annotations determine argument conversion. Cache each `@cpp` block as a separate `.so`. |
| **`.pyi` stub generation** (§5.1) | **Adopt** | `scripts/create_stubs.py` was an unfinished item on the rclcppyy roadmap. Add stub generation to cppyy_kit so editors and static analysis tools can read dynamic APIs. |
| **async/nogil helpers** (§2.2) | **Adopt with correction** | The proposal says cppyy releases the GIL automatically. Measurements in control_kit show that cppyy holds the GIL during blocking C++ calls (COMMON_PATTERNS §13). Add an explicit `nogil()` wrapper and an asyncio `run_in_executor` integration for blocking calls. |
| **Layered packaging: base + kit packages** (§5.1/§5.8) | **Adopt** | cppyy_kit has no ROS dependency, and six kits already depend on it. See §3. |
| **Fallback contract + `status()` introspection** (§5.2) | **Adopt** | Kits currently handle capability checks and fallback behavior separately. Add a shared probe, fallback, and status interface to cppyy_kit. |

### 1.2 Proposed features not included

| Idea | Decision | Reason |
|---|---|---|
| **`kits.fast`: monkey-patching builtins/stdlib** (§5.2) | **Skip** | This project moves work on messages, point clouds, and images that already use C++. It does not target Python lists with millions of elements. Global patches also add CPython-version constraints, can change floating-point results, and can conflict with other packages. |
| **Lambda transpilation** (§5.3) | **Skip** | The proposal does not define which Python expressions can be translated from AST to C++. Use tested L0-to-L2 lowering instead. Add small C++ functor helpers only when a kit requires them. |
| **"GIL bypass is automatic"** (§2.1) | **Reject the claim** | Measured false for cppyy (control_kit). Keep the corrected record; build explicit nogil instead. |
| **`kits.parallel` as a main feature** (§5.3) | **Defer** | Parallel STL is a general Python capability rather than a robotics-specific one. Consider it after the base package is available to the community. A separate `concurrent` module for lock-free SPSC queues may help robotics pipelines; consider it after `require()` is available. |
| **Freeze CLI that removes the Cling dependency** (§5.7) | **Do not promise** | Full ahead-of-time compilation of arbitrary reflection surfaces is outside cppyy's current support. The measured options use PCH with Cling, JIT-compiled wrappers, or L2 code that removes cppyy from selected call paths. |

---

## 2. Existing implementation constraints

The 21 existing patterns document constraints relevant to this design: GIL
behavior (§13), SIGSEGVs involving containers and failed `cppdef`/`include` calls
(§9), object lifetime (§3), ownership (§5, §16), limitations on `final` virtual
methods and `generate_parameter_library`, value conversion (§11), and the
per-call cost at the Python/C++ boundary. Engine performance does not imply
faster tree ticks. COMMON_PATTERNS.md and cppyy_kit provide these implementations
and patterns. The proposal adds packaging, caching, and API usability work.

---

## 3. Re-architecture plan

### Target layout (monorepo, separate pixi/conda packages)

```
repo root
├── cppyy_kit/              # Package 1: ROS-FREE base (the doc's "base")
│   ├── (current rclcppyy/kits/cppyy_kit.py content)
│   ├── cache.py            # NEW: content-hash cppdef→.so compile cache
│   ├── require.py          # NEW: header-only fetcher (conda-first policy)
│   ├── cpp.py              # NEW: @cpp decorator (annotation marshaling)
│   ├── nogil.py            # NEW: GIL-release shim + asyncio integration
│   ├── stubs.py            # REVIVED: .pyi generation
│   ├── freeze/             # generalized PCH + direct-compile + vendored-source tooling
│   └── capability.py       # NEW: probe/fallback/status pattern, codified
├── rclcppyy/               # Package 2: ROS core (depends on cppyy_kit)
│   └── (bringup, monkeypatching, messages, serialization, rosbag, TF)
├── kits/                   # Packages 3..n (each depends on cppyy_kit; ROS kits also on rclcppyy)
│   ├── bt_kit/  pcl_kit/  ompl_kit/  nav2_kit/  moveit_kit/  control_kit/
│   └── cv_kit/  dbow_kit/           # vision pair
├── recipe/                 # one rattler-build recipe per package
└── docs/, scripts/, test/  # as today
```

- **Same repo** (monorepo), **separate conda packages** on the existing
  prefix.dev channel: `cppyy-kit` (no ROS deps: conda-forge candidate later),
  `ros-jazzy-rclcppyy` (as today), `ros-jazzy-<kit>-kit` each declaring its own
  C++ deps (behaviortree-cpp, pcl, ompl, nav2-*, moveit, ros2-control…).
- The kit Python modules do not require ament. Only rclcppyy uses ament_cmake
  for the ROS index. Recipes may also build or include optional C++ sources.
- **Import compatibility**: `rclcppyy.kits.X` shims re-exporting the new
  top-level modules for one release cycle, with a deprecation note.
- **Versioning**: lockstep initially (one tag releases the whole suite via a
  release.yml matrix over `recipe/*/recipe.yaml`); split later only if needed.

### Phases

- **Phase A: Extract & re-plumb (mechanical, no new features).**
  Move cppyy_kit to top-level; kits import it; compat shims; multi-recipe
  release matrix; CI path-filtered test jobs. Gate: every existing suite green,
  every artifact builds, fresh-env install of each package proven (the Phase-4
  playbook per package).
- **Phase B: Add base features in priority order.**
  1. Compile cache to reuse wrapper builds across runs. Compare with warmup.
  2. `require()` + port the DBoW2 build onto it.
  3. `@cpp` decorator (unify with callback() inference).
  4. `nogil()` + asyncio helper (with a measured GIL-release proof).
  5. Stubs revival. 6. capability/status codification.
  For each feature, run the required checks and update COMMON_PATTERNS.
- **Phase C: Publish the suite.** Tag → matrix build → prefix.dev; README
  install matrix; per-kit WHY docs become per-package READMEs.
- **Phase D: Optional follow-up work.** `concurrent` module; conda-forge submission
  of `cppyy-kit`; community-kit contribution guide (COMMON_PATTERNS as the
  authoring manual).

### Risks

- **Import changes**: Phase A updates every import path. Complete these changes
  in one pass and run the full test matrix before starting feature work.
- **ROS-free package**: add a CI job that tests cppyy-kit in an environment
  without ROS dependencies.
- **Package name**: publish `cppyy-kit` to prefix.dev first, then consider
  submitting it to conda-forge when it is stable.

---

## 4. Approved direction (2026-07-11): naming + two-repo reorganization

### 4.1 Project name: **`cppyy_kit`** (recommended; availability verified)

The project, repository, and base package share one name. The 2026-07-11
availability check found no PyPI package or GitHub repository named
`cppyy-kit`/`cppyy_kit`. conda-forge had no package with that name. It also had
no package named `rclcpp-kit`.

- **One project and package name**: the base and domain kits share the
  `cppyy_kit` name.
- **Searchability**: the name associates the suite with cppyy.
- **Scope**: the base has no ROS dependency. The domain kits target robotics.

Other names considered were `kitforge` and `kitbash`. `kitforge` was available
on PyPI but had only small GitHub repositories. `kitbash` was already used on
PyPI and is close to the KitBash3D brand.

### 4.2 The two-repo model

**Repository 1: `github.com/awesomebytes/cppyy_kit`** (the suite):

```
cppyy_kit/    # ROS-free base: primitives, cache, require, @cpp, nogil, stubs,
              # capability reporting, freeze and vendored-source tools
rclcpp_kit/   # rclcpp interfaces: bringup, message resolution and conversion,
              # serialization, rosbag2_cpp, tf, executor/node helpers, rclcpp PCH
bt_kit/ pcl_kit/ ompl_kit/ nav2_kit/ moveit_kit/ control_kit/ cv_kit/ dbow_kit/
docs/         # COMMON_PATTERNS, FREEZE, per-kit trios, vision tutorial
scripts/      # freeze, datasets, kit demos & benches
recipe/<pkg>/ # one rattler-build recipe per package; release matrix on tag
```

Conda packages: `cppyy-kit` (distro-free; conda-forge candidate when stable),
`ros-jazzy-rclcpp-kit`, and distro-scoped `ros-jazzy-<name>-kit` for every kit
that imports ROS bits (nav2, moveit, control, cv, pcl: their ROS bridges pull
sensor_msgs/pcl_conversions). bt/ompl/dbow kits need only `cppyy-kit` at import
time (their ROS demos declare extras). All on the existing prefix.dev
`awesomebytes` channel; lockstep versioning from a single tag initially.

**Repository 2: `rclcppyy`** (the product):

- **Keeps**: `enable_cpp_acceleration()`, the
  monkeypatching layer (`monkey.py`, `monkeypatch_messages.py`,
  `RclcppyyNode`), rclpy-parity benchmarks + tutorial demos, the release
  pipeline (`ros-jazzy-rclcppyy`, now with a run-dep on `ros-jazzy-rclcpp-kit`).
- **Sheds** (moves to `rclcpp_kit`): `bringup_rclcpp.py`, `serialization.py`,
  `rosbag2_cpp.py` + compat, `tf.py`, the shared converter. For one release
  cycle of deprecation re-export shims (`rclcppyy.bringup_rclcpp` →
  `rclcpp_kit` + warning). Version bump to 0.2.0 marks the split.
- Verify the split by running rclcppyy's benchmark and test suite before and
  after. Results must match.

Dependency graph:
`cppyy-kit` ← `rclcpp-kit` ← { `rclcppyy`, nav2/moveit/control/cv/pcl kits };
`cppyy-kit` ← { bt/ompl/dbow kits }.

### 4.3 Migration phases (supersedes §3's Phase A)

- **Bootstrap the new repo:** create the `cppyy_kit` repository; migrate kits,
  documentation, freeze tools, and dataset tools with history using
  `git filter-repo` path filters;
  replicate the proven pixi workspace, CI, and multi-recipe release plumbing;
  all suites green in the new home before anything is deleted here.
- **Carve `rclcpp_kit`:** move the ROS-core capability layer out of
  rclcppyy into the new repo's `rclcpp_kit` package; its tests move with it;
  then add Phase B features there. Start with the compile cache to reuse
  wrapper builds across runs and packages.
- **Slim rclcppyy:** replace moved internals with `rclcpp_kit` imports +
  deprecation shims; 0.2.0; recipes updated (rclcppyy depends on rclcpp-kit);
  release both repos; parity benchmarks green.
- **Publish + outward:** suite on prefix.dev; README cross-links both
  ways; `cppyy-kit` → conda-forge submission when stable; community-kit
  authoring guide (COMMON_PATTERNS as the manual).

**Resolved decisions:** (a) `cppyy_kit` confirmed as the project name;
(b) distro-scoping convention `ros-jazzy-<kit>-kit` adopted for resolver hygiene
alongside robostack; (c) the roscon archive stays in rclcppyy (product history,
not suite material).

### 4.4 Kit package contents (correction, 2026-07-11)

The kits use Python packages without ament or colcon. Some also include C++
sources, so the package may build or ship native code. A kit contains:

```
<name>_kit/
├── <name>_kit/       # Python package: the library API and helper functions
├── cpp/              # optional C++ sources: bridge shims, L2-lowered nodes,
│                     # vendored-source build scripts, PCH/freeze recipes
├── SKILL.md          # API usage and examples for coding agents
├── WHY.md            # reasons to use the kit and available functions
├── REPORT.md         # measurements, tests, and known limits
├── demos/  tests/
└── recipe/           # rattler-build recipe; may compile cpp/ shims while
                      # building the package to populate the compile cache
```

Recipes can compile a kit's C++ glue during package builds. This populates the
compile cache before installation. Each kit's `SKILL.md` documents its API and
usage patterns for coding agents. COMMON_PATTERNS.md documents patterns shared
across kits.

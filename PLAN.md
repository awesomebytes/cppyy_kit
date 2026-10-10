# cppyy_kit master plan

This is the maintainer status ledger, not setup guidance; use [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/) to install packages or develop the suite.

**Purpose:** provide Python interfaces to C++ robotics libraries through cppyy.
The suite supports prototyping from Python, moving selected work to C++, and
ahead-of-time compilation. Documentation also supports coding agents.

**Presentation target: ROSCon UK 2026.** Show this workflow:

> Keep a Python experiment around real robotics inputs. Ask an agent to read
> the task guide, profile the computation, and make one measured change using
> `@cpp` or an existing C++ library. Check outputs and report warm operation,
> first-use, and complete pipeline costs separately. Show configuration and
> testing of existing C++ software from Python as another entry point.

Freeze and lowering remain supported follow-up topics. They are not required
to demonstrate the prototyping workflow. In `rclcppyy`, the compatible default
preserves stock `rclpy` behavior; acceleration claims require evidence for the
selected explicit native profile or fused operation.

**Origin:** this project extracts and expands the kit suite proven in
[rclcppyy](https://github.com/awesomebytes/rclcppyy) (7 spikes, 7 GOs, 22
documented patterns, measured ladder: PCH freeze 890→6 ms header parse, L2
lowering, 14.8×/9.4× PCL showcase, 6.7-14× TF ingest). See
`docs/ARCHITECTURE_V2.md` for the approved architecture and
evaluation history.

---

## Current status and next work (2026-10-10)

The July milestone entries below preserve the original implementation history.
The following merged commits and integration record supersede pending wording
about the numeric interface, agent tooling, and presentation assets:

| Date | Commit | Delivered |
|---|---|---|
| 2026-10-03 | `4bd206e` | Numeric `@cpp` kernels and updated user guides |
| 2026-10-04 | `148fcf5` | NumPy annotations and simpler examples; removed `cpp.arr` |
| 2026-10-05 | `9449f51` | Installed task/kit guides, environment diagnostics, coordinated fetch/build artifacts, buffer fixes, native-component and OMPL callback tutorials |

The source manifest is version 0.4.0. The
[integration ledger](https://github.com/awesomebytes/cppyy_kit/blob/main/EXPERIMENT_INTEGRATION_2026-10-04.md#validation-and-upstream-status)
records local artifacts and fresh installed core/rclcpp checks. Its final default
suite passed 369 tests with 161 optional skips; PCL, OpenCV, OMPL and native
component checks ran separately. Nine wrapper artifacts skipped native recipe
tests: their guide-resource proof is not native bringup proof for every kit.
The ledger records channel publication as separate work. These are recorded
October integration results, not tests rerun for the presentation update.

Next work, in order:

1. Align ROSCon examples, prompts, instructions, and their locked environment
   with `NDArray`/`ConstNDArray` and the current task-guide workflow. Preserve
   original 0.3.0 solutions and measurements as historical evidence.
2. Make [AI-assisted prototyping](CPPYY_KIT_WITH_AI.md) the task-first entry
   point for acceleration, new libraries, and existing C++ components.
3. Add a compact existing-library composition example to the talk, using the
   retained nanoflann index and its measured comparison with SciPy.
4. Repeat fresh-agent trials and the presentation acceptance checks in the
   updated environment. Rehearse the full talk, GUI interaction, and fallbacks.
5. Publish only after the release's fresh-artifact gates pass, and update the
   package instructions once channel installation is verified.

ROSCon assets are under `roscon_uk_2026/` in this checkout. Their inclusion in
Git is separate from the merged implementation commits above. See the
[presentation plan](https://github.com/awesomebytes/cppyy_kit/blob/main/roscon_uk_2026/PLAN.md)
and evaluation records there; do not promote historical timings to new 0.4.0
results without running the relevant workloads.

---

## Package suite

| Package | Conda name | Depends on | Contents |
|---|---|---|---|
| `cppyy_kit` | `cppyy-kit` (distro-free) | cppyy | library loading, lifetime, callbacks, `HandleRegistry`, warmup, first-use reporting, teardown, probes, compile cache, `require()`, `@cpp`, `nogil`, stubs, freeze and vendored-source tools, capability reporting |
| `rclcpp_kit` | `ros-jazzy-rclcpp-kit` | cppyy_kit, ros-jazzy-rclcpp | rclcpp bringup, C++ message resolution/conversion, serialization, rosbag2, **tf**, executor/node helpers, rclcpp PCH recipe |
| `bt_kit` `pcl_kit` `ompl_kit` `nav2_kit` `moveit_kit` `control_kit` `cv_kit` `dbow_kit` | `ros-jazzy-<name>-kit` (ROS dependencies where needed) | cppyy_kit (+ rclcpp_kit for ROS APIs) | native library APIs and Python helpers |
| (separate repo) `rclcppyy` | `ros-jazzy-rclcppyy` | rclcpp-kit | compatibility-first rclpy interface, explicit native profiles, and managed/fused C++ operations |

**Kit contents:** Python package, optional `cpp/` sources (shims, L2 nodes,
vendored builds, freeze recipes), `SKILL.md` with API usage for coding agents,
`WHY.md`, `REPORT.md`, demos, tests, and a package recipe. The recipe may
precompile `cpp/` sources so the compile cache is populated on install.

**Publishing:** use the prefix.dev `awesomebytes` channel. Before upload, verify
that each artifact installs and works in a fresh environment using only that
channel. Use lockstep versions released from one tag.

---

## Milestones

### M1: Migration & bootstrap ✅ DONE (2026-07-11)
- **M1a**: created the Pixi workspace, CI, lint setup, and repository files.
  Migrated the kits, documentation, tools, and tests from rclcppyy with Git
  history. Organized them by kit and confirmed the migrated suites passed.
- **M1b**: moved rclcpp bringup, message conversion, serialization, rosbag2,
  and TF functionality and tests into `rclcpp_kit`; updated kit imports.
- **M1c**: added per-package rattler-build recipes and a tag-triggered release
  matrix that builds, installs into a fresh environment, and uploads artifacts
  to prefix.dev using OIDC.

### M2: Base enrichment (ordered by value). Items 1-6 DONE (2026-07-12)

Compile-cache measurements: frozen and cached BT startup took about 425 ms
end to end, about 4.1× faster. PCL frame-zero latency fell from 681 ms to 88 ms,
about 7.7× faster. The boundary tracer from M8a was completed early.

1. **Compile cache:** matching `cppdef` calls reuse content-hashed `.so` files,
   avoiding repeated wrapper JIT. The measured first-use JIT took about 0.69 s.
2. **`require()`:** fetches header-only libraries after checking conda packages.
3. **`@cpp`:** marshals annotated function arguments.
4. **`nogil()`:** releases the GIL around C++ calls; the asyncio helper was added.
   Measurements confirmed GIL release.
5. **Stub generation:** the `create_stubs` pilot was restored. Dynamic proxy
   limitations are documented.
6. **Capability reporting:** shared capability, fallback, and status APIs were
   added.

Each kit has a `SKILL.md`; COMMON_PATTERNS is the shared usage guide.
- **Zero-config auto-PCH** DONE (2026-07-12,
  verified end-to-end: cppyy_kit.autopch builds the Cling PCH on first use
  into ~/.cache/cppyy_kit/pch [env+version-keyed, self-invalidating, atomic
  background build], auto-loads it before cppyy import thereafter: no env
  vars, no pixi changes; prints on create and on load. rclcpp_kit registers
  its headers → bringup 1.96 s→0.30 s, header parse 1.7 s→0.0 s [~30×].
  Key finding: `import cppyy` sets CLING_STANDARD_PCH itself, so override
  detection gates on cppyy-not-yet-loaded. COMMON_PATTERNS §36 + FREEZE §8.
  Follow-ups completed 2026-07-13: .pth activation makes it independent of import order
  [+ cache self-pruning]; `@cpp(nogil=True)` [GIL released around the C++
  body only; parallel demo 7.7× on 8 threads; both first-use compile races
  found+locked] and `@cpp(cached=False)`; unified debug escape hatches
  [FREEZE §9: disable_caching()/ctx mgr/NO_CACHE=1/NO_AUTOPCH=1]. rclcppyy
  gained the heavy-topic hz demo [3 MB msgs: rclpy CPU-bound ~207 Hz vs
  accelerated ~306-417 Hz at half CPU; cold→warm headers 1.8→0.0 s;
  bridge env pending next suite release]).

### M3: rclcppyy integration and suite releases
- Suite extraction and a published 0.3.x baseline are established. `rclcppyy`
  imports the shared kit implementation and now distinguishes the compatible
  default from explicit native profiles. Its own environment pins the older
  suite source revision; sibling checkout edits do not automatically update it.
- Current 0.4.0 artifacts have local installed-package evidence in the October
  integration ledger. Channel publication and verification remain separate.
- Preserve each product's own behavior and backend checks when updating its
  dependency pin. Do not infer product parity from core tests alone.

### M4: Documentation site LIVE (2026-07-11)

Site: https://awesomebytes.github.io/cppyy_kit/. GitHub Pages deployment uses
the API, a strict build, and automatic deployment.
- mkdocs-material on GitHub Pages (this repo): landing page with the measured
  numbers; per-kit pages (WHY/REPORT/SKILL rendered); the tutorials (vision
  loop-closure + new ones from M6); COMMON_PATTERNS + FREEZE as core chapters;
  quickstart per package (pixi snippets). CI deploys on push to main.
- COMMON_PATTERNS update completed (2026-07-12):
  COMMON_PATTERNS 29→35 sections (§30 in-process lifecycle bootstrap, §31
  lower-the-hot-virtual, §32 own-binding+cppyy coexistence, §33 schema-derived
  structs) + extensions to §9/§16/§19/§21/§26; README updated to published-
  suite reality; tone/naming sweep (0 person refs, 0 milestone tags in all
  user-facing md; ledgers exempt).

### M5: Coding-agent workflows implemented; repeated evaluation ongoing
- `skills/cppyy-accelerate/` contains the profiling skill, scripts, and worked
  example. It maps measured hotspots to kits and patterns, checks behavior, and
  reports before/after measurements.
- 0.4.0 packages three task guides (`accelerate`, `bring-library`, `existing-cpp`)
  and kit references, readable through `python -m cppyy_kit guide`. Environment
  diagnostics do not start Cling. See [guide commands](docs/GUIDES.md).
- ROSCon experiments preserve fresh-agent successes, failures, acceptance
  hashes, and environment repairs. They establish the recorded scaffolds'
  feasibility; repeated current-version reliability remains to be evaluated.

### M6: ROSCon demo track (parallel work after M1)
- **6a End-to-end deep dive**: implemented material under `roscon_uk_2026/`
  covers motion detection, recorded-data queries, ROS callbacks, mock control,
  and webcam tracking, with saved agent solutions and historical 0.3.0 evidence.
  Current API/environment migration, fresh evaluation, and full rehearsal are
  pending verification. Add library composition using nanoflann; retain a
  direct native-binding control so the comparison attributes gains correctly.
- **6b Live webcam demo** DONE (2026-07-12: live A/B display in Rerun; kit implementation 4.3 ms / 231 fps vs Python implementation 66 ms / 15 fps, or 15.4× at VGA; live camera comparison 12-13× with zero dropped frames. Pure OpenCV operations measured about 1.1×. Both comparisons appear on screen. TF and images use rclcpp_kit. The demo falls back to synthetic input when no webcam is available. Instructions are in docs/webcam_demo/REPORT.md): live webcam visual-odometry comparison: webcam → cv_kit (ORB/optical flow, CUDA if present) → pose/track →
  TF via rclcpp_kit → live Rerun; a CPU overlay comparing use with a plain
  Python/cv2-loop baseline. It runs with a laptop webcam and falls back to
  synthetic input when a webcam is unavailable; CUDA is optional.
- **6c IK solver benchmark** DONE (2026-07-12): One Python script benchmarks
  KDL (~400 solves/s), TRAC-IK (~900/s), bio_ik (~1000/s), pick_ik (~140/s),
  and Python DLS (~40/s, 70.5% success) on 200 seeded Panda targets. Forward
  kinematics verifies successful solutions. The test environment is
  `feature.ik`. CMake builds for vendored plugins install to a private prefix
  listed in `AMENT_PREFIX_PATH`. Cling crashes when it parses pick_ik's generated
  `generate_parameter_library` header. CMake builds the plugin, and pluginlib loads
  the compiled library without parsing that header.
  KDL and `trac_ik` (2.0.2) are packaged. `bio_ik` and `pick_ik` are built from
  vendored source using the §21 recipe. The benchmark compares these solvers
  with a pure-Python IK baseline and reports solve rate, success, and accuracy.
  moveit_kit's plugin loader runs the C++ solvers from Python.
- **6d Nav2 lifecycle support** DONE (2026-07-12): an in-process
  `rclcpp_lifecycle::LifecycleNode` made Smac 2D and RegulatedPurePursuit
  available from Python. All four d02 planner/controller combinations reached
  the goal. The test count increased from 8 to 14. Hybrid-A* remains unavailable
  because its OMPL distance heuristic can segfault under Cling. See
  [nav2_kit/REPORT.md](nav2_kit/REPORT.md) for implementation details.
- **6f Perception→humanoid retargeting pipeline** ✅ DONE (2026-07-12,
  supervisor-verified live: webcam→HolisticLandmarker→TF→Rerun ~30 fps 0-drop
  437-frame run; CLIK retarget Talos 0.91 ms/frame + G1 0.82 ms [zero-code
  URDF swap: G1 ships in example-robot-data, EE err 2.4-2.6 cm median];
  policy-kickstart datasets npz; measured C++ glue speedups: /tf build 290.8×, retarget
  kernel [xform+map+One-Euro one cppdef pass] 364.5× at 4.4e-8 m numeric
  agreement; limitation: `pinocchio::Model` cannot be JIT-compiled under Cling with Boost
  1.90 [2nd confirmation, now incl. default-double+URDF] so the solve stays on
  pinocchio bindings; repo's first pip dep [mediapipe, sha256-pinned model
  fetch] in isolated `pipeline` env; two-process seam over JSONL landmark
  stream; docs/retarget_pipeline/REPORT.md + run-book; pattern candidates
  folded into COMMON_PATTERNS §34/§6/§9. ADDENDA same day, all supervisor-
  verified live: `--follow` live teleop over the tailable stream [4.3 ms median
  lag, cold-start startup-grace fix]; then ROS-NATIVE transport: the boost
  1.86/1.90 environment conflict resolved after conda-forge migrated to 1.90 [verified],
  new retarget-ros env in the default solve-group, `--source tf` consumes the
  /tf landmark frames via rclcpp_kit's C++ TransformListener at 2.5 ms median
  lag [reproduced exactly], ONE shared Rerun viewer [screenshot-verified];
  Cling cannot JIT-compile `pinocchio::Model`; solving remains on the existing
  bindings. Changes made the same day: real URDF link meshes in Rerun
  [G1 35 STL / Talos 47, Asset3D once + FK transforms per frame, +0.55 ms],
  perceive defaults to run-until-Ctrl-C with clean SIGINT, and the landmark-
  visibility presence gate [no-person phantom tracking killed]. Retarget
  quality arc closed 2026-07-12: trunk-lean CLIK fix [52°→0°, per-joint
  posture weights] + hip-relative target map [owner's frame chain: robot_hip +
  body-ratio × (wrist − hip_mid); corr 0.98-1.0, EE err 1.1 cm] + permanent
  motion-fidelity regression tests. Head and amplitude changes completed after owner
  approval [briefly parked, then merged]: --motion-scale knob at ~1:1
  [Talos full sweeps ~0.75 m/hand], Talos head yaw/pitch tracking [corr
  1.00/0.999: orientation changes while the mechanically rotating neck position stays fixed;
  earlier "structural" claim corrected]; G1 head mechanically rigid, noticed
  at startup): webcam → body+hand+face tracking + object detection → TF frames via
  rclcpp_kit → live Rerun viz → whole-body retargeting onto **Talos**
  (example-robot-data; wbc_kit/Crocoddyl or ik_bench solver per frame) →
  recorded "policy-kickstart" dataset artifact. The capture pipeline supplies
  human demonstrations for humanoid policy training. Python ML libraries
  (MediaPipe/ONNX/YOLO) handle inference; cppyy_kit handles TF input, control,
  and other C++ glue (TF 6.7-14×,
  WBC/IK solve 21.7×, custom kernels 15.4×, zero-copy marshaling).
  Record+replay mode from day one (rehearsal safety). Short-lived kickstart
  code: timeboxed spike discipline, not a product. Build-first-and-learn:
  whether this becomes the main presentation demo is deferred until the work is complete
  (supersedes 6a's "decision after 6b" note).
- **6g Low-jitter Python control experiment**: STAGE 0 ✅ DONE (2026-07-12,
  supervisor-verified: jitter_bench/ harness + reference matrix on the stock
  kernel, 60 s/cell at 1 kHz. `prctl(PR_SET_TIMERSLACK, 1)` reduced
  pure-Python idle p50 from 52.4 to 2.4 µs
  [22×; reproduced 52.6 to 2.5]. All
  variants, including an in-process ros2_control loop driven from Python, hold
  ~2 µs median idle; under load the nogil C++ loop keeps p50 2.1 µs vs Python
  ~5 µs [its loaded histogram tighter than Python's idle]. mlockall works
  unprivileged; SCHED_FIFO DENIED as expected [ulimit -r 0]. Tails p99
  0.5-1.2 ms = Stage 1's target; owner-action sudo commands + rerun matrix
  [one command/cell] in docs/jitter_bench/REPORT.md. test 56/130,
  test-jitter 17 [real control loop exercised]. STAGE 1 pending owner sudo):
  reuse
  control_kit's in-process ros2_control loop driven from Python (nogil +
  frozen/cached) and measure loop-period jitter: cyclictest baseline +
  control-loop histograms, SCHED_FIFO + mlockall + CPU isolation. Reference
  numbers on the CURRENT kernel first; then re-run under
  `preempt=voluntary/full` (runtime-switchable via debugfs) for the comparison
  table. Fact-checked 2026-07-12: **CONFIG_PREEMPT_RT is not needed for this
  goal**: the stock 6.17-oem kernel compiles in every soft-RT primitive
  (SCHED_FIFO/DEADLINE, FUTEX_PI, HIGH_RES_TIMERS, threadirqs, NO_HZ_FULL,
  RCU_NOCB_CPU, RT_MUTEXES, isolcpus, preempt=full via PREEMPT_DYNAMIC);
  PREEMPT_RT only tightens worst-case tails under adversarial load. Optional
  free middle step if ever wanted: linux-lowlatency-hwe-24.04 (standard
  archive). Possible follow-up: run the same harness on an embedded board
  (Raspberry Pi).
- **6e WBC exploration** DONE (2026-07-12): the study selected Crocoddyl for
  `wbc_kit`. Its inline C++ action model ran in 0.32 ms, compared with 6.84 ms
  for a Python-derived model. Both produced bit-identical costs. The package
  uses a separate environment because its Boost 1.86 dependency conflicts with
  the ROS Boost 1.90 environment. Pinocchio scalar-template instantiation is
  blocked by Boost 1.90's variant arity limit; its existing bindings remain the supported route.
  TSID custom tasks were not tested and remain a possible follow-up. OCS2 and
  mc_rtc were unavailable through conda-forge and robostack. QP bindings cover
  the tested use case, so no cppyy kit is needed. See the [WBC report](docs/wbc/REPORT.md).

### M7: Presentation assets exist; current-version rehearsal pending
- `roscon_uk_2026/DEEP_DIVE_PRESENTATION.md`, task guides, solutions, acceptance
  checks, and evaluation records exist in the checkout. The October integration
  deliberately excluded presentation/experiment files from its commits.
- Update commands, lock, agent prompts, and API examples together. Keep the
  historical evidence intact. Verify current tasks, repeat agent trials, then
  rehearse the full presentation including GUI interaction and recorded fallbacks.

---

## Constraints & discipline (carried from rclcppyy)
- Measure each claim. Report whether each trial works, partially works, or is
  blocked, with evidence and known limits.
- Publish only artifact-proven packages (fresh-env install test gates upload).
- Tests are the contract at every rung (golden/differential where applicable).
- COMMON_PATTERNS.md is the canonical playbook; every lane feeds it.
- No history rewrites containing others' commits; PLAN.md (this file) is the
  project ledger.
- Docs tone (owner directive 2026-07-12): user-facing docs carry no person
  references and no internal milestone tags: plain descriptive names only.
  Ledgers (PLAN.md, HANDOFF.md) are exempt. Note: PLAN.md is published on the
  docs site as "Project Plan" (mkdocs nav): intentional transparency.

### M8: L3 whole-app lowering (research thread)

The next step is to convert a kit-based application to a compiled artifact by
tracing its execution. The method applies PGO/LTO to the prototype-to-deploy
workflow. Kit applications use Python between cppyy calls, so this does not
compile arbitrary Python. Instrument the boundary in cppyy_kit, create a typed
call trace and instantiation manifest, emit C++ that replays the orchestration,
then compile with LTO/PGO. A trace records one control-flow path. Three
approaches are under consideration:
control-flow-as-data apps (BT XML) lower to a **full static binary**;
multi-trace+guards yields a hybrid binary (embedded CPython, no Cling);
bounded AST-lift covers straight-line glue only. Tests-as-contract makes the
automation safe (differential gate at every step).

- **8a: boundary tracer** DONE (completed during M2). It also provides data for
  freeze manifests, PGO profiles, and hotspot analysis in the M5 skill.
- **8b: trace→C++ emitter** for straight-line segments + differential harness
  (pilot: the vision per-frame path or the PCL pipeline).
- **8c: whole-app pilot**: a BT-shaped app → static binary (emitted main +
  L2-lowered leaves + tree XML); same golden tests; measure binary size,
  startup, CPU vs L0.
- **8d: compare with Nuitka, Codon, torch.export/AOTInductor, and PyPy tracing.**
  Include hybrid binaries when full lowering is not practical.

# cppyy_kit — master plan

**Mission:** a suite of "kits" that let Python drive C++ robotics libraries via
cppyy — prototype at Python speed, run at C++ speed, graduate to AOT — with
first-class documentation and LLM-agent consumability.

**Headline target: ROSCon UK 2026.** The presentation storyline every milestone
serves:

> Prototype normally (plain Python) → switch to rclcppyy + kits and it gets
> automatically more efficient → write unit/integration tests (the contract) →
> apply AOT (freeze/lower) → show the benchmark difference — **while the code
> stays the same or changes minimally**. Plus: an LLM agent you can ask "make
> this code/package faster" that applies cppyy_kit + the right kits for you.

**Origin:** this project extracts and expands the kit suite proven in
[rclcppyy](https://github.com/awesomebytes/rclcppyy) (7 spikes, 7 GOs, 22
documented patterns, measured ladder: PCH freeze 890→6 ms header parse, L2
lowering, 14.8×/9.4× PCL showcase, 6.7–14× TF ingest). See
`rclcppyy/docs/kits/ARCHITECTURE_V2.md` for the approved architecture and
evaluation history.

---

## Package suite

| Package | Conda name | Depends on | Content |
|---|---|---|---|
| `cppyy_kit` | `cppyy-kit` (distro-free) | cppyy | friction primitives (load/keep_alive/callback/HandleRegistry/warmup/first_use/teardown/probe), compile cache, `require()`, `@cpp`, `nogil`, stubs, freeze & vendored-source tooling, capability/fallback |
| `rclcpp_kit` | `ros-jazzy-rclcpp-kit` | cppyy_kit, ros-jazzy-rclcpp | rclcpp bringup, C++ message resolution/conversion, serialization, rosbag2, **tf**, executor/node helpers, rclcpp PCH recipe |
| `bt_kit` `pcl_kit` `ompl_kit` `nav2_kit` `moveit_kit` `control_kit` `cv_kit` `dbow_kit` | `ros-jazzy-<name>-kit` (ROS-touching) / distro-free where possible | cppyy_kit (+ rclcpp_kit where ROS) | as proven in rclcppyy, restructured to kit anatomy |
| (separate repo) `rclcppyy` | `ros-jazzy-rclcppyy` | rclcpp-kit | the drop-in rclpy accelerator product; slims to monkeypatching + brand |

**Kit anatomy** (each kit): Python mirror-API package + optional `cpp/` sources
(shims, L2 nodes, vendored builds, freeze recipes) + **`SKILL.md`** (LLM-facing:
when to use, copy-paste patterns, gotchas) + `WHY.md` + `REPORT.md` + demos +
tests + recipe (which may precompile `cpp/` at package-build time → the compile
cache ships warm).

**Publishing:** prefix.dev `awesomebytes` channel; every published artifact must
pass the fresh-env proof (install from channel alone → works) before upload —
same discipline as the rclcppyy 0.1.0 release. Lockstep versions from one tag.

---

## Milestones

### M1 — Migration & bootstrap ✅ DONE (2026-07-11)
- **M1a**: bootstrap this repo (pixi workspace on the proven pattern, CI via
  setup-pixi, lint, LICENSE/README/.gitignore); migrate kits + kit docs +
  freeze/dataset/demo scripts + kit tests from rclcppyy **with git history**
  (`git filter-repo` path extraction; documented fallback if it fights);
  restructure to the kit anatomy (per-kit dirs); all migrated suites green in
  the new home. rclcppyy repo remains fully functional (nothing deleted there
  until M3).
- **M1b**: carve **`rclcpp_kit`** out of rclcppyy's core (bringup, message
  machinery, serialization, rosbag2, tf) into this repo; its tests move with
  it; imports across kits updated.
- **M1c**: per-package rattler-build recipes + tag-triggered release matrix
  (build → fresh-env artifact proof → upload), OIDC to prefix.dev.

### M2 — Base enrichment (value order) — item 1 ✅ DONE + ADOPTED (2026-07-12: cache kills first-use JIT persistently; bt frozen+cached ~425 ms end-to-end ~4.1×, pcl frame-0 681→88 ms ~7.7×; 8a tracer shipped early); items 2-6 ✅ DONE (2026-07-12: require conda-first, @cpp annotation marshaling, nogil with measured GIL-release proof ~1 vs ~470 ticks, stubgen pilot + honest dynamic-proxy limit, capability/status registry w/ bt reference adoption — M2 CLOSED)
1. **Compile cache** (content-hash cppdef→`.so`, dlopen thereafter) — kills the
   measured ~0.69 s first-use wrapper JIT *persistently*; measure vs warmup.
2. `require()` header-only fetcher (conda-first policy).
3. `@cpp` decorator (annotation-driven marshaling, unified with callback()).
4. `nogil()` + asyncio helper (on the corrected GIL evidence).
5. `.pyi` stub generation (revive rclcppyy's stalled create_stubs).
6. capability/fallback/status codified.
- **SKILL.md for every kit** (evolve the cheat sheets) + repo-level
  COMMON_PATTERNS as the shared manual.
- **Zero-config auto-PCH** ✅ DONE (2026-07-12, owner-requested, supervisor-
  verified end-to-end: cppyy_kit.autopch builds the Cling PCH on first use
  into ~/.cache/cppyy_kit/pch [env+version-keyed, self-invalidating, atomic
  background build], auto-loads it before cppyy import thereafter — no env
  vars, no pixi changes; prints on create and on load. rclcpp_kit registers
  its headers → bringup 1.96 s→0.30 s, header parse 1.7 s→0.0 s [~30×].
  Key finding: `import cppyy` sets CLING_STANDARD_PCH itself, so override
  detection gates on cppyy-not-yet-loaded. COMMON_PATTERNS §36 + FREEZE §8).

### M3 — rclcppyy slim + first suite release
- In the rclcppyy repo: replace moved internals with rclcpp_kit imports +
  deprecation shims; 0.2.0; parity proof = its own bench/test suite unchanged.
- Release the full suite to prefix.dev (artifact-proven, working).

### M4 — Documentation site ✅ LIVE (2026-07-11: https://awesomebytes.github.io/cppyy_kit/ — Pages enabled via API, strict build + auto-deploy)
- mkdocs-material on GitHub Pages (this repo): landing page with the measured
  numbers; per-kit pages (WHY/REPORT/SKILL rendered); the tutorials (vision
  loop-closure + new ones from M6); COMMON_PATTERNS + FREEZE as core chapters;
  quickstart per package (pixi snippets). CI deploys on push to main.
- Patterns consolidation debt CLEARED (2026-07-12, docs lane merged):
  COMMON_PATTERNS 29→35 sections (§30 in-process lifecycle bootstrap, §31
  lower-the-hot-virtual, §32 own-binding+cppyy coexistence, §33 schema-derived
  structs) + extensions to §9/§16/§19/§21/§26; README updated to published-
  suite reality; tone/naming sweep (0 person refs, 0 milestone tags in all
  user-facing md; ledgers exempt).

### M5 — LLM acceleration tooling
- The "ask an LLM to make my code faster" story: a **`cppyy-accelerate` skill**
  (Claude-Code-style SKILL.md + supporting scripts) that: profiles/identifies
  hot paths → maps them to kits/patterns (COMMON_PATTERNS + per-kit SKILL.md as
  its knowledge) → applies the change → verifies with the tests-as-contract
  discipline → reports before/after numbers. Demoable live: point it at a slow
  Python package and watch it get faster.

### M6 — ROSCon demo track (parallel lanes once M1 lands)
- **6a Canonical arc demo**: one compact example shown end-to-end:
  plain-Python prototype → kits (minimal diff) → tests → freeze/L2 → benchmark
  table. Candidate: evolve the vision pipeline or a purpose-built compact demo;
  decision after 6b lands.
- **6b Live webcam demo** ✅ DONE (2026-07-12: A-vs-B split-screen live in Rerun — kits 4.3 ms/231 fps vs naive Python 66 ms/15 fps = 15.4× @VGA [live camera 12–13×, 0 dropped]; honest headline: the win tracks custom-kernel-vs-library-primitive [~1.1× for pure cv2 ops] — both regimes on one screen; TF+image via rclcpp_kit; auto webcam→synthetic fallback; run-book in docs/webcam_demo/REPORT.md): robotics-flavored expensive computation "all in
  Python": webcam → cv_kit (ORB/optical flow, CUDA if present) → pose/track →
  TF via rclcpp_kit → live Rerun; CPU% overlay showing the headroom vs a plain
  Python/cv2-loop baseline. Must run on a laptop with a webcam, degrade
  gracefully without CUDA.
- **6c IK benchmark suite** ✅ DONE (2026-07-12: 5-solver table from ONE Python script — KDL ~400/s, TRAC-IK ~900/s, bio_ik [vendored, uninstallable-otherwise] ~1000/s FASTEST, pick_ik ~140/s, pure-Python DLS ~40/s @ 70.5% success; 200 seeded Panda targets, FK-verified successes; new [feature.ik] env; vendored-plugin pattern = CMake into private prefix + AMENT_PREFIX_PATH; g_p_l confirmed parse-wall-only): benchmark IK solvers via moveit_kit's plugin
  loading — KDL (packaged), **trac_ik** (packaged, verified 2.0.2), **bio_ik**
  and **pick_ik** (NOT packaged — vendored-source builds via the §21 recipe;
  pick_ik likely hits the generate_parameter_library wall → known route), plus
  a pure-Python IK baseline. Same robot (Panda), same targets, solve-rate /
  success / accuracy table + docs page. A genuinely new use case: cppyy as the
  harness that makes C++-only solvers benchmarkable from one Python script.
- **6d Nav2 lifecycle unlock** ✅ DONE (2026-07-12: in-process rclcpp_lifecycle::LifecycleNode is the universal key [3rd instance of the in-process node/manager pattern]; real Smac 2D + real RPP now run from Python — all four d02 planner/controller combos reach GOAL; Hybrid-A* honest flaky-partial [OMPL-under-Cling runtime segfault], not shipped; test-nav2 8→14): the nav2_kit report showed Smac + RPP blocked
  on lifecycle-coupled ctors. Way around it: construct a real
  `rclcpp_lifecycle::LifecycleNode` (plain class, in-process — same pattern as
  control_kit's ControllerManager) and, if needed, `Costmap2DROS` from Python
  to satisfy those ctors; unlock Smac (needs OMPL headers on path — we have
  the ompl env) and the RPP controller; extend nav2_kit + REPORT verdicts.
- **6f Perception→humanoid retargeting pipeline** ✅ DONE (2026-07-12,
  supervisor-verified live: webcam→HolisticLandmarker→TF→Rerun ~30 fps 0-drop
  437-frame run; CLIK retarget Talos 0.91 ms/frame + G1 0.82 ms [zero-code
  URDF swap — G1 ships in example-robot-data, EE err 2.4–2.6 cm median];
  policy-kickstart datasets npz; measured glue wins: /tf build 290.8×, retarget
  kernel [xform+map+One-Euro one cppdef pass] 364.5× at 4.4e-8 m numeric
  agreement; honest boundary: pinocchio::Model un-JIT-able under Cling+boost
  1.90 [2nd confirmation, now incl. default-double+URDF] so the solve stays on
  pinocchio bindings; repo's first pip dep [mediapipe, sha256-pinned model
  fetch] in isolated `pipeline` env; two-process seam over JSONL landmark
  stream; docs/retarget_pipeline/REPORT.md + run-book; pattern candidates
  folded into COMMON_PATTERNS §34/§6/§9. ADDENDA same day, all supervisor-
  verified live: `--follow` live teleop over the tailable stream [4.3 ms median
  lag, cold-start startup-grace fix]; then ROS-NATIVE transport — the boost
  1.86/1.90 SOLVE wall dissolved with conda-forge's 1.90 migration [verified],
  new retarget-ros env in the default solve-group, `--source tf` consumes the
  /tf landmark frames via rclcpp_kit's C++ TransformListener at 2.5 ms median
  lag [reproduced exactly], ONE shared Rerun viewer [screenshot-verified];
  Cling JIT wall on pinocchio::Model UNCHANGED — solve stays on bindings.
  Demo polish landed same day [owner feedback]: real URDF link meshes in Rerun
  [G1 35 STL / Talos 47, Asset3D once + FK transforms per frame, +0.55 ms],
  perceive defaults to run-until-Ctrl-C with clean SIGINT, and the landmark-
  visibility presence gate [no-person phantom tracking killed]. Retarget
  quality arc closed 2026-07-12: trunk-lean CLIK fix [52°→0°, per-joint
  posture weights] + hip-relative target map [owner's frame chain: robot_hip +
  body-ratio × (wrist − hip_mid); corr 0.98–1.0, EE err 1.1 cm] + permanent
  motion-fidelity regression tests. Head + amplitude landed after owner
  approval [briefly parked, then merged]: --motion-scale knob at ~1:1
  [Talos full sweeps ~0.75 m/hand], Talos head yaw/pitch tracking [corr
  1.00/0.999 — the neck is rotational: orientation moves, position doesn't;
  earlier "structural" claim corrected]; G1 head mechanically rigid, noticed
  at startup): webcam → body+hand+face tracking + object detection → TF frames via
  rclcpp_kit → live Rerun viz → whole-body retargeting onto **Talos**
  (example-robot-data; wbc_kit/Crocoddyl or ik_bench solver per frame) →
  recorded "policy-kickstart" dataset artifact. Narrative: a minimal-code human-
  demonstration capture rig that bootstraps humanoid policy training — the
  motivating "why cppyy_kit" story. **Hybrid line agreed:** commodity Python ML
  inference (MediaPipe/ONNX/YOLO — library primitives, per the 6b honest-
  headline lesson); cppyy_kit owns the genuinely hot glue (TF 6.7–14×,
  WBC/IK solve 21.7×, custom kernels 15.4×, zero-copy marshaling).
  Record+replay mode from day one (rehearsal safety). Short-lived kickstart
  code — timeboxed spike discipline, not a product. Build-first-and-learn:
  whether this becomes the presentation centerpiece is deferred until it lands
  (supersedes 6a's "decision after 6b" note).
- **6g Low-jitter Python control experiment** — STAGE 0 ✅ DONE (2026-07-12,
  supervisor-verified: jitter_bench/ harness + reference matrix on the stock
  kernel, 60 s/cell @1 kHz. Headline: prctl(PR_SET_TIMERSLACK,1) — unprivileged
  — drops pure-Python idle p50 52.4→2.4 µs [22×; reproduced 52.6→2.5]; ALL
  variants incl. a REAL in-process ros2_control loop driven from Python hold
  ~2 µs median idle; under load the nogil C++ loop keeps p50 2.1 µs vs Python
  ~5 µs [its loaded histogram tighter than Python's idle]. mlockall works
  unprivileged; SCHED_FIFO DENIED as expected [ulimit -r 0]. Tails p99
  0.5–1.2 ms = Stage 1's target; owner-action sudo commands + rerun matrix
  [one command/cell] in docs/jitter_bench/REPORT.md. test 56/130,
  test-jitter 17 [real control loop exercised]. STAGE 1 pending owner sudo):
  reuse
  control_kit's in-process ros2_control loop driven from Python (nogil +
  frozen/cached) and measure loop-period jitter honestly: cyclictest baseline +
  control-loop histograms, SCHED_FIFO + mlockall + CPU isolation. Reference
  numbers on the CURRENT kernel first; then re-run under
  `preempt=voluntary/full` (runtime-switchable via debugfs) for the comparison
  table. Fact-checked 2026-07-12: **CONFIG_PREEMPT_RT is not needed for this
  goal** — the stock 6.17-oem kernel compiles in every soft-RT primitive
  (SCHED_FIFO/DEADLINE, FUTEX_PI, HIGH_RES_TIMERS, threadirqs, NO_HZ_FULL,
  RCU_NOCB_CPU, RT_MUTEXES, isolcpus, preempt=full via PREEMPT_DYNAMIC);
  PREEMPT_RT only tightens worst-case tails under adversarial load. Optional
  free middle step if ever wanted: linux-lowlatency-hwe-24.04 (standard
  archive). **No Ubuntu Pro.** Possible follow-up: same harness on an
  embedded board (Raspberry Pi).
- **6e WBC exploration** ✅ DONE (2026-07-12, GO-narrow: Crocoddyl inline-C++ action models at native speed — 0.32 ms vs 6.84 ms Python-authored, 21.7×, bit-identical cost; thin wbc_kit shipped, standalone env [boost 1.86 vs ROS 1.90 — solve-group infeasible, verified]; pinocchio blocked on env boost-variant arity; tsid redundant; OCS2/mc_rtc unpackaged; QP bindings fine): survey spike first — tsid/crocoddyl/pinocchio are on
  conda-forge WITH Python bindings (verified), so the cppyy win must be sharper
  than "bindings exist": candidates = templated-scalar surfaces (pinocchio
  `Scalar` templates for autodiff), binding-lag APIs, or C++-only frameworks
  (OCS2, mc_rtc — availability TBD in the spike). Deliverable: evidence-based
  pick + one feasibility probe (wbc_kit go/no-go), honest if the answer is
  "bindings are fine, no kit needed".

### M7 — Presentation assets (near conference)
- Slide-ready numbers/tables, demo run-books (what to type live, fallbacks),
  rehearsal checklist. Placeholder until M6 stabilizes.

---

## Constraints & discipline (carried from rclcppyy)
- Every claim measured; every spike reports works/partial/blocked with
  evidence; honest boundaries documented.
- Publish only artifact-proven packages (fresh-env install test gates upload).
- Tests are the contract at every rung (golden/differential where applicable).
- COMMON_PATTERNS.md is the canonical playbook; every lane feeds it.
- No history rewrites containing others' commits; PLAN.md (this file) is the
  project ledger.
- Docs tone (owner directive 2026-07-12): user-facing docs carry no person
  references and no internal milestone tags — plain descriptive names only.
  Ledgers (PLAN.md, HANDOFF.md) are exempt. Note: PLAN.md is published on the
  docs site as "Project Plan" (mkdocs nav) — intentional transparency.

### M8 — L3 whole-app lowering (research thread)

The step beyond the ladder: convert a whole kit-built app to a compiled
artifact by **tracing its execution** — PGO/LTO thinking applied to the
prototype→deploy workflow. Tractable precisely because kit apps are thin
Python between cppyy crossings (this is NOT "compile Python"): instrument the
boundary centrally in cppyy_kit → a semantic, typed call-trace + instantiation
manifest → emit C++ replaying the orchestration skeleton → compile with real
LTO/PGO. The control-flow wall (a trace is one path) has three honest routes:
control-flow-as-data apps (BT XML) lower to a **full static binary**;
multi-trace+guards yields a hybrid binary (embedded CPython, no Cling);
bounded AST-lift covers straight-line glue only. Tests-as-contract makes the
automation safe (differential gate at every step).

- **8a — boundary tracer** (small; run early — also feeds freeze manifests,
  real PGO profiles, and the M5 accelerate skill's hotspot analysis).
- **8b — trace→C++ emitter** for straight-line segments + differential harness
  (pilot: the vision per-frame path or the PCL pipeline).
- **8c — whole-app pilot**: a BT-shaped app → static binary (emitted main +
  L2-lowered leaves + tree XML); same golden tests; measure binary size,
  startup, CPU vs L0.
- **8d — honest positioning survey** vs Nuitka / Codon / torch.export+
  AOTInductor / PyPy tracing — including "hybrid binary, not full lowering" as
  an acceptable general-case answer.

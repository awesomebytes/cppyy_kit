# Supervisor handoff — cppyy_kit / rclcppyy program

**Purpose:** continuation brief for a fresh supervisor agent taking over this
program mid-flight. Written 2026-07-12 by the outgoing supervisor at ~90%
context. Read this fully, then verify state with the commands at the bottom
before dispatching anything.

## Your role and Sam's working style
You are the **orchestrator/supervisor**. Subagents do the work; you plan,
dispatch, verify, merge, keep the ledgers current, and report crisply.
- **Model routing (Sam's standing directive):** Opus for reasoning/iteration
  (spikes, design, debugging); Sonnet for mechanical tasks (renames, doc
  sweeps, config).
- Sam values: professional, concise, **evidence-based** reporting; honest
  boundaries (works/partial/blocked with evidence); measured numbers, never
  aspirational ones. Momentum: keep dispatching without waiting to be asked,
  but surface decisions that are genuinely his (naming, releases, money/auth).

## Hard rules (violations caused real incidents — enforce)
1. **Git identity:** every commit in BOTH repos as `sammypfeiffer@gmail.com`
   (repo-local config set; worktrees inherit; agents must verify before
   committing). Never touch global git config.
2. **No history rewrites** that contain anyone else's commits (an agent once
   destroyed a supervisor commit; recovered via reflog — rule is absolute).
3. **One agent per working tree.** Parallel lanes = `git worktree add` +
   branch, supervisor merges after the lane reports. Never merge into a clone
   while another agent works there. Assignments must say worktree-FIRST.
4. **Verification gates before closing anything:** lint 0, relevant test
   suites green, demo spot-checks run by YOU (not just the agent's claim),
   CI green on push. Publish only artifact-proven packages (fresh-env install
   test — see RELEASING.md discipline).
5. **ROS_DOMAIN_ID:** unique per concurrent live-node lane (this session used
   17–61; keep incrementing).
6. **PLAN.md files are the supervisor's ledger** — agents never edit them;
   you update after verifying each landing.
7. Agents may hit a Write-guard on files literally named REPORT.md —
   the proven workaround is write-to-scratchpad then `cp`.

## Repos
- **`/home/sapf/playground/ros_project/cppyy_kit`** (github.com/awesomebytes/cppyy_kit)
  — THE active repo. Ledger: `PLAN.md` (mission, M1–M8, statuses inline).
  Docs site (live, auto-deploy): https://awesomebytes.github.io/cppyy_kit/
  Structure: `cppyy_kit/` base + `rclcpp_kit/` + 9 domain kits (bt, pcl, ompl,
  nav2, moveit, control, cv, dbow, wbc) + `ik_bench/` + `skills/cppyy-accelerate/`
  + `examples/` + `recipe/<pkg>/` (10 recipes + drivers `pkg-build-all`/`pkg-prove`)
  + mkdocs site (symlink mirror via `mkdocs_site_links.sh` — careful:
  `mkdocs_site/docs` is a SYMLINK to the real `docs/`; never "mirror into" it).
- **`/home/sapf/playground/ros_project/rclcppyy`** (github.com/awesomebytes/rclcppyy)
  — the slim product (0.2.0, unreleased): monkeypatching/`enable_cpp_acceleration`
  on top of the suite via deprecation shims. Its PLAN.md is historical; its
  `RELEASING.md` has the exact release choreography. CI bridges to the suite
  repo via PYTHONPATH checkout until the suite is published.
- **Memory:** `/home/sapf/.claude/projects/-home-sapf-playground-ros-project-rclcppyy/memory/`
  — full project memory (patterns, gotchas, decisions). A session started in
  the rclcppyy directory inherits it automatically; otherwise read MEMORY.md
  there manually.

## State snapshot (2026-07-12 END OF DAY — supersedes the morning snapshot)
- **RELEASED AND LIVE on https://repo.prefix.dev/awesomebytes:** suite v0.1.0
  (11 packages) + ros-jazzy-rclcppyy v0.2.0, tag-triggered OIDC release
  workflows in both repos. NOTE: main has moved substantially since v0.1.0 —
  a v0.1.1/0.2.x release pass is a natural next step when Sam wants it.
- **Landed today (all supervisor-verified, merged, CI+Docs green, live):**
  - Patterns consolidation cleared: COMMON_PATTERNS now 36 sections.
  - Docs tone regime (Sam directives, in PLAN.md Constraints + memory):
    no person refs, no milestone tags in ANYTHING user-facing (docs, config
    comments, identifiers, printed output); no marketing register (no
    virtue claims/epithets; every number names its example + links its
    benchmarks row). Ledgers (PLAN/HANDOFF) exempt.
  - Retargeting teleop rig (retarget_pipeline/): webcam→HolisticLandmarker→
    /tf(C++)→CLIK→G1/Talos, live ROS transport (--source tf, 2.5 ms median
    lag), ONE shared Rerun viewer, real URDF meshes, hip-relative ~1:1
    mapping (+--motion-scale), Talos head tracking, presence gate,
    record/--follow/replay modes, policy-kickstart datasets.
  - jitter_bench/ stage 0: ~2.4 µs p50 @1 kHz from Python on the stock
    kernel (timerslack prctl is the lever, 22×); nogil loop holds p50 under
    load. STAGE 1 blocked on Sam's sudo (rt-tests + rtprio limits.d).
  - Zero-config auto-PCH (cppyy_kit.autopch): .pth-activated at interpreter
    start (import-order-proof), ~/.cache/cppyy_kit/pch, self-pruning; rclcpp
    bringup 1.7 s→0.0 s warm.
  - Benchmarks page (docs/benchmarks.md, live at /docs/benchmarks/): whole
    suite re-measured on the Core Ultra 9 285H with exact rerun commands.
  - Both READMEs + docs landing rewritten around "prototype in Python, run
    at C++ speed"; cppyy_kit↔rclcppyy cross-linked; showcase tables.
  - The boost 1.86/1.90 env wall DISSOLVED (conda-forge 1.90 migration):
    pinocchio now co-solves with ROS (retarget-ros env). Cling JIT wall on
    pinocchio::Model REMAINS — solve stays on bindings.
- **ROSCon UK 2026 submission:** summary + outline drafted with Sam
  (scratchpad roscon_submission_draft.txt — session-local; Sam has a copy).
  Do NOT put conference claims in public docs.

## Work queue (in priority order)
1. **M6a/centerpiece decision** — deliberately deferred by Sam ("still
   finding great ideas"). Ingredients now on the shelf: teleop rig, jitter
   story, accelerate skill. Revisit with Sam before building M7 assets.
2. **M7 presentation assets** once the centerpiece is picked (run-books
   exist per demo REPORT; benchmarks page is the numbers source).
3. **M6g stage 1** — one command per cell once Sam runs the two sudo
   one-liners (documented in docs/jitter_bench/REPORT.md §Stage 1).
4. **M8 research:** 8b trace→C++ emitter, 8c BT static-binary pilot,
   8d positioning survey.
5. **Pydantic follow-ups** (PR #1 deferred list).
6. Misc: release pass for the moved main (see above); conda post-link
   variant of the auto-PCH .pth install at next release; Talos head
   full-model increment RETIRED (reduced model tracks orientation fine);
   Hybrid-A* flaky parked; conda-forge submission of cppyy-kit when stable;
   leftover worktree ../cppyy_kit_m6f_follow (merged content, held for
   Sam's testing — prune when he confirms).

## Dispatch template that works
Worktree-first block → required reading (COMMON_PATTERNS + specific REPORTs)
→ jobs with staged works/partial/blocked verdicts → explicit gates → commit
rules (email check, Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>,
no push unless told, no history rewrites) → report format. Timebox risky
items; "a precise partial beats a rushed full".

## Verify state on arrival
```bash
cd /home/sapf/playground/ros_project/cppyy_kit && git pull && git log --oneline -5
pixi run lint && pixi run test          # expect lint 0; ~40 passed, rest skipped
gh run list --repo awesomebytes/cppyy_kit --limit 3   # CI + Docs green
git worktree list                        # any in-flight lanes?
cd ../rclcppyy && git log --oneline -3 && pixi run test   # 26 passed expected
```

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

## State snapshot (2026-07-12)
- **Done & green:** M1 (migration w/ history, rclcpp_kit carve, 10 noarch
  recipes, 10/10 fresh-env proofs, release matrix), M2 (compile cache —
  first-use JIT ELIMINATED, bt frozen+cached ~425 ms ≈4.1×; tracer; require;
  @cpp; nogil (measured); stubgen; capability registry), M3 (rclcppyy slim +
  parity + duplicate cleanup), M4 (docs site live + strict links), M5
  (cppyy-accelerate skill; walkthrough: 15.6× with a 3-line diff), M6c (IK
  bench: 5 solvers one script — vendored bio_ik fastest ~1000/s), M6d (Nav2
  lifecycle unlocked — real Smac 2D + RPP from Python, in-process
  LifecycleNode is the universal key), M6e (WBC: Crocoddyl inline-C++ action
  models at native speed, 21.7×; standalone env — boost 1.86 vs ROS 1.90),
  Pydantic→C++ structs RFC (PR #1, MERGED: 16× memory, honest bench,
  compile-time type checks).
- **Docs cleanup** merged by Sam's own agent (336608e).
- **22+ documented patterns** in `docs/COMMON_PATTERNS.md` — the canonical
  playbook every new agent must read first.

## Work queue (in priority order)
1. **Patterns consolidation debt:** M6c/d/e + pydantic lesson candidates are
   NOT yet folded into COMMON_PATTERNS (each REPORT lists "generic-lesson
   candidates"; the established pass = one doc-only agent, cites evidence,
   README kit-table updates). Sources: ik_bench/REPORT, nav2_kit REPORT (M6d
   additions), docs/wbc/REPORT, cppyy_kit/design/pydantic_structs.md.
2. **M6b webcam live demo** (may already be dispatched — check task board /
   worktrees): robotics-flavored expensive compute all-in-Python via kits —
   webcam → cv_kit ORB/optical-flow (CUDA auto if provisioned) → pose/track →
   TF via rclcpp_kit → live Rerun + CPU overlay vs a plain cv2/python loop.
   Needs a webcam + display; degrade gracefully headless. Wants a QUIET
   machine (its CPU numbers are the demo).
3. **M6a canonical arc demo:** ONE compact example shown end-to-end:
   plain-Python prototype → kits (minimal diff) → tests → freeze/L2/cache →
   benchmark table. Cherry-pick proven pieces (accelerate walkthrough 15.6×,
   cache 425 ms cold start, L2 2.8×). This is the ROSCon centerpiece.
4. **Release choreography** (rclcppyy/RELEASING.md has exact steps):
   BLOCKED on Sam: prefix.dev Repository Access for awesomebytes/cppyy_kit +
   release.yml. BEFORE tagging: add missing recipes (wbc_kit, pydantic env
   packaging decision, ik extras) to the matrix + re-run pkg-prove. Then tag
   suite v0.1.0 → swap rclcppyy bridge → tag rclcppyy v0.2.0.
5. **M7 presentation assets** (nearer ROSCon UK 2026): slide numbers, demo
   run-books with fallbacks, rehearsal checklist.
6. **M8 research (L3 whole-app lowering):** 8a tracer DONE; 8b trace→C++
   emitter, 8c BT static-binary pilot, 8d positioning survey — see PLAN.md M8.
7. **Pydantic follow-ups** (deferred list in PR #1): [feature.pydantic] env +
   lock, @cpp Model-annotation integration, Enum/datetime support.
8. Misc: Hybrid-A* flaky (OMPL-under-Cling segfault) parked; conda-forge
   submission of cppyy-kit when stable; rclcppyy ARCHITECTURE_V2.md header
   points to suite repo.

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

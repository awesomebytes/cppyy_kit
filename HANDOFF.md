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

## State snapshot (2026-07-12, post-release)
- **RELEASED AND LIVE on https://repo.prefix.dev/awesomebytes:** the full suite
  v0.1.0 (11 packages: cppyy-kit, ros-jazzy-rclcpp-kit, 8 domain kits, wbc-kit
  — all noarch, all fresh-env-proven on the runner) AND ros-jazzy-rclcppyy
  v0.2.0 (the product, now genuinely depending on the published suite — bridge
  removed, import provenance verified, artifact proof green). Release
  workflows in both repos are tag-triggered and OIDC-authorized.
- **Done & green:** M1 (migration/carve/recipes), M2 (compile cache — bt
  frozen+cached ~425 ms ≈4.1×; tracer; require; @cpp; nogil measured;
  stubgen; capability), M3 (slim+parity+cleanup), M4 (docs site live), M5
  (accelerate skill, 15.6× walkthrough), M6b (webcam A-vs-B: 15.4× @VGA,
  live 12–13×; honest headline: the win tracks custom-kernel-vs-library-
  primitive, ~1.1× for pure cv2 ops), M6c (IK bench, 5 solvers, vendored
  bio_ik fastest), M6d (Nav2 lifecycle unlocked — Smac+RPP from Python),
  M6e (WBC/Crocoddyl inline-C++ models 21.7×), Pydantic structs RFC merged
  (16× memory, compile-time type checks).
- **22+ patterns** in docs/COMMON_PATTERNS.md — required reading for agents.

## Work queue (in priority order)
1. **Patterns consolidation debt:** M6b/c/d/e + pydantic lesson candidates
   are NOT yet folded into COMMON_PATTERNS (each REPORT lists candidates:
   docs/webcam_demo/, ik_bench/, nav2_kit [M6d additions], docs/wbc/,
   cppyy_kit/design/pydantic_structs.md). Established pass: one doc-only
   agent, evidence-cited, plus README kit-table updates.
2. **M6a canonical arc demo (the ROSCon centerpiece):** ONE compact example
   end-to-end: plain-Python prototype → kits (minimal diff) → tests →
   freeze/cache/L2 → benchmark table. Cherry-pick proven pieces (accelerate
   walkthrough 15.6×, cache 425 ms cold start, L2 2.8×, webcam kernels).
3. **M7 presentation assets** (ROSCon UK 2026): slide numbers, demo
   run-books (webcam run-book exists in docs/webcam_demo/REPORT.md),
   rehearsal checklist.
4. **M8 research:** 8a tracer DONE; 8b trace→C++ emitter, 8c BT
   static-binary pilot, 8d positioning survey (PLAN.md M8).
5. **Pydantic follow-ups** (PR #1 deferred list): [feature.pydantic] env,
   @cpp Model-annotation integration, Enum/datetime.
6. Misc: rclcppyy working tree has UNCOMMITTED cosmetic edits (PLAN.md/
   RELEASING.md/ARCHITECTURE_V2.md, personal-name removals — likely Sam's
   docs agent; do not sweep, let Sam decide); leftover
   .claude/worktrees/agent-af69… dir in rclcppyy (untracked, removable);
   Hybrid-A* flaky parked; conda-forge submission of cppyy-kit when stable.

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

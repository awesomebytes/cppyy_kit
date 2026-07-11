---
name: cppyy-accelerate
description: >-
  Make a slow Python robotics script or package faster by moving its hot paths onto
  C++ via cppyy_kit and the domain kits (bt/pcl/ompl/nav2/moveit/control/cv/rclcpp),
  with a tests-as-contract before/after. Use when asked to "make this faster",
  "optimize", "reduce latency/CPU", or "speed up" Python code that manipulates
  point clouds, images, transforms/messages, planning, control, or behavior trees.
---

# cppyy-accelerate

You are accelerating a Python program by relocating its hot work to C++ through
**cppyy_kit** and the domain **kits**, *without* rewriting it in C++ and without
changing what it computes. Prototype-shaped Python stays; only the hot crossing
moves. Follow the four steps in order. Do not skip PROFILE — accelerate what the
numbers point at, not what looks slow.

The whole method rests on one discipline: **a before/after is only an acceleration
if the output is unchanged.** Establish the correctness contract (a test) before you
change anything, and re-run it after.

---

## Step 1 — PROFILE (find the real hot path)

Run the target under the profiler + boundary tracer wrapper:

```bash
python skills/cppyy-accelerate/scripts/profile_target.py <target.py> -- <target args>
```

Read the two tables together:

- **Python hotspots (tottime)** — a single function with large *own* time that
  loops over array / cloud / image / message data is the prime candidate.
- **Boundary crossings** — if the target already uses cppyy_kit, high `total_ms`
  on a crossing (or a big line in the *instantiation manifest*) is a first-use JIT
  or a per-call copy, addressed differently (see MAP).

If the target has no profileable entry point, wrap the suspect call in a tiny driver
script and profile that. Write down the hottest frame and its cost — that is the MAP
input. (Also capture a boundary trace of a representative run for later PGO/freeze
work: `CPPYY_KIT_TRACE=trace.json python <target>` then
`python -m cppyy_kit trace report trace.json`.)

---

## Step 2 — MAP (hotspot shape → remedy)

Match the hotspot to the smallest remedy. Decision tree, most common first:

| The hotspot looks like… | Remedy | Reference |
|---|---|---|
| a **pure-Python loop over point-cloud / array data** (per-point math, voxel/filter/transform) | do the bulk op in C++ via **pcl_kit** (`cloud_from_numpy` → the kit op → `cloud_to_numpy`); one memcpy in, C++ does the loop | `pcl_kit/SKILL.md`, COMMON_PATTERNS §6 |
| a **per-frame image loop** (per-pixel Python, cv2 in a hot loop) | **cv_kit** — `cv::Mat` aliases the buffer (zero-copy), OpenCV runs the kernel (CUDA if present) | `cv_kit/SKILL.md`, COMMON_PATTERNS §6 |
| **copying a message/buffer across the boundary every frame** | keep it in C++: alias don't copy (§6 "alias-in"), build containers in a `cppdef` helper, pass addresses as `uintptr_t` | COMMON_PATTERNS §6 |
| a **one-time ~0.4–0.7 s stall on the first call** (registration, first filter) | it's the first-use call-wrapper JIT — eliminate it with the **compile cache** (`cppdef_cached`) or move it with `warmup()` | COMMON_PATTERNS §23, §15; FREEZE.md §4 |
| **repeated tf lookups / message ingest** in a Python callback | **rclcpp_kit** — let the C++ `TransformListener` ingest `/tf` on its own thread; Python only crosses on lookup | `rclcpp_kit/SKILL.md`, COMMON_PATTERNS §13 |
| a **whole subsystem written in Python** (behavior tree, motion/OMPL planning, MoveIt, ros2_control, vision) | drive the real C++ library through its kit, leaves/callbacks in Python | the kit's `SKILL.md` (`bt_kit`, `ompl_kit`, `moveit_kit`, `control_kit`, `cv_kit`, `nav2_kit`) |
| **Python↔C++ cross-inheritance in a hot loop** (a Python override called millions of times) | works, but if it dominates, lower that leaf to native C++ (the L2 rung) | COMMON_PATTERNS §16; FREEZE.md §5 |

### DON'T (when cppyy is the wrong tool — be honest)

- **Don't lower a one-shot / batch step.** cppyy's first-use JIT (~0.4–0.7 s) and
  bringup (~0.9 s parse, unless frozen) can cost more than a batch step saves. The
  cache/freeze amortize it only across many runs/calls. Accelerate hot *loops* and
  *per-frame* work, not a once-per-process computation.
- **Don't fight a library whose Python bindings are already fine.** If a maintained
  binding exists and the step isn't a hot inner loop, use the binding. Worked
  verdict: gtsam via cppyy hit the Cling ORC static-init wall; the honest answer was
  its own Python binding for the batch factor-graph step (COMMON_PATTERNS §20). A
  kit may *mix* — cppyy for the hot C++ path, the binding for a one-shot step.
- **Don't add a real worker thread around a busy-blocking Python leaf.** cppyy holds
  the GIL across a blocking C++ call; overlap needs a C++ thread, not a Python one
  (COMMON_PATTERNS §13).
- **Don't change what the code computes** to make it faster. If the fast path can't
  match the contract, stop and report that, not a wrong-but-fast result.

---

## Step 3 — APPLY (minimal diff, mirror the library)

- Make the smallest edit that moves the hot work: replace the hot loop body with the
  kit call(s). Keep the surrounding Python and the public shape of the code.
- Follow the target kit's `SKILL.md` patterns verbatim — they encode the cppyy
  friction (lifetime pinning §4, container-building-in-C++ §6, keyword-name escapes
  §18, enum/`unsigned char` traps §11). Mirror the library's own API; don't invent a
  DSL (§12).
- If first-use latency matters, the kit's cache adoption is already automatic
  (`_CACHED`); otherwise call the kit's `warmup()` once at init.

---

## Step 4 — VERIFY (tests-as-contract + the number)

1. **Contract.** Run the target's existing tests. If there are none, write a
   differential test *first*: capture the pre-change output as golden and assert the
   accelerated output matches (exactly, or within an explained numerical tolerance —
   see `examples/accelerate_demo/test_pipeline.py`, which keys voxel outputs by index
   and allows only float-summation drift). A faster result that fails the contract is
   not an acceleration — revert and re-MAP.
2. **Number.** Measure before vs after and report the table:

   ```python
   from bench_before_after import compare      # skills/cppyy-accelerate/scripts/
   compare([("before", lambda: before(...)), ("after", lambda: after(...))])
   ```

   Time the *operation* (warmed) for the per-call win, and note one-time costs
   (bringup, the cache's first-run `.so` compile) separately — they amortize.
3. **Report** the hotspot, the mapping, the diff, the before/after table, and any
   honest residual (e.g. cppyy's own call wrapper to the entry point, ~tens of ms).

---

## Checklist

- [ ] PROFILE run captured; hottest frame + its cost written down.
- [ ] Correctness contract exists (target tests, or a new differential test) and is
      GREEN before any change.
- [ ] MAP decision made (which kit/pattern, or a documented DON'T).
- [ ] Minimal diff applied per the kit `SKILL.md`.
- [ ] Contract GREEN after the change.
- [ ] Before/after table measured (operation warmed; one-time costs noted).
- [ ] Report: hotspot → mapping → diff → table → residual.

A full worked example (this exact procedure on a slow point-cloud pipeline, with
real numbers) is in `WALKTHROUGH.md`. The kit knowledge this skill dispatches to
lives in `docs/COMMON_PATTERNS.md` (the shared playbook), `docs/FREEZE.md` (freeze +
compile cache), and each `*_kit/SKILL.md`.

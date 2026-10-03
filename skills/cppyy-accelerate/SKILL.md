---
name: cppyy-accelerate
description: >-
  Speed up Python robotics code by moving measured hot paths to C++ with cppyy_kit
  and its domain kits (bt/pcl/ompl/nav2/moveit/control/cv/rclcpp). Check output with
  tests or a comparison, and report before/after timing. Use when asked to "make this faster",
  "optimize", "reduce latency/CPU", or "speed up" Python code that manipulates
  point clouds, images, transforms/messages, planning, control, or behavior trees.
---

# cppyy-accelerate

Speed up Python programs by moving selected work to C++ through **cppyy_kit** and
the domain **kits**. Keep the Python code and preserve its results. Follow these
four steps in order. Use the profile to choose what to move.

The output must stay the same. Run an existing test before and after the change. If
there is no test, add a comparison test before changing the code.

---

## Step 1: Profile the program

Run the target with the profiler and boundary tracer:

```bash
python skills/cppyy-accelerate/scripts/profile_target.py <target.py> -- <target args>
```

Read the two tables together:

- **Python hotspots (tottime):** a function with high *own* time that
  loops over array, cloud, image, or message data is a likely target.
- **Boundary crossings:** if the target already uses cppyy_kit, high `total_ms`
  on a crossing (or a large entry in the *instantiation manifest*) may indicate
  first-use JIT compilation or a per-call copy. These need different remedies.

If the target has no profileable entry point, wrap the suspect call in a tiny driver
script and profile that. Record the hottest frame and its cost for the next step.
Capture a boundary trace of a representative run for later PGO/freeze work:

```bash
CPPYY_KIT_TRACE=trace.json python <target>
python -m cppyy_kit trace report trace.json
```

---

## Step 2: Choose a remedy

Match the hotspot to the smallest suitable remedy. The table lists common cases:

| The hotspot looks like… | Remedy | Reference |
|---|---|---|
| a **pure-Python loop over point-cloud / array data** (per-point math, voxel/filter/transform) | do the bulk op in C++ via **pcl_kit** (`cloud_from_numpy` → the kit op → `cloud_to_numpy`); one memcpy in, C++ does the loop | `pcl_kit/SKILL.md`, COMMON_PATTERNS §6 |
| a **per-frame image loop** (per-pixel Python, cv2 in a hot loop) | **cv_kit**. `cv::Mat` can alias the buffer, and OpenCV runs the operation (CUDA if present) | `cv_kit/SKILL.md`, COMMON_PATTERNS §6 |
| **copying a message/buffer across the boundary every frame** | keep it in C++: alias don't copy (§6 "alias-in"), build containers in a `cppdef` helper, pass addresses as `uintptr_t` | COMMON_PATTERNS §6 |
| a **one-time ~0.4–0.7 s stall on the first call** (registration, first filter) | The call-wrapper JIT runs on first use. Avoid repeated compilation with the **compile cache** (`cppdef_cached`) or run it earlier with `warmup()` | COMMON_PATTERNS §23, §15; FREEZE.md §4 |
| **repeated tf lookups / message ingest** in a Python callback | **rclcpp_kit**. Let the C++ `TransformListener` ingest `/tf` on its own thread. Python crosses the boundary only for lookups | `rclcpp_kit/SKILL.md`, COMMON_PATTERNS §13 |
| a **whole subsystem written in Python** (behavior tree, motion/OMPL planning, MoveIt, ros2_control, vision) | Use the kit to call the C++ library. Keep leaves and callbacks in Python | the kit's `SKILL.md` (`bt_kit`, `ompl_kit`, `moveit_kit`, `control_kit`, `cv_kit`, `nav2_kit`) |
| **Python↔C++ cross-inheritance in a hot loop** (a Python override called millions of times) | If the override dominates, move that operation to native C++ (the L2 rung) | COMMON_PATTERNS §16; FREEZE.md §5 |

### When not to use cppyy

- **Don't lower a one-shot / batch step.** cppyy's first-use JIT (~0.4–0.7 s) and
  bringup (~0.9 s parse, unless frozen) can cost more than a batch step saves. The
  cache/freeze amortize it only across many runs/calls. Accelerate hot *loops* and
  *per-frame* work, not a once-per-process computation.
- **Use maintained Python bindings for non-hot code.** For example, cppyy could not
  handle gtsam's Cling ORC static initialization, so the batch factor-graph step
  uses gtsam's Python binding (COMMON_PATTERNS §20). A kit can use cppyy for the
  hot C++ path and the Python binding for one-shot work.
- **Don't add a real worker thread around a busy-blocking Python leaf.** cppyy holds
  the GIL across a blocking C++ call; overlap needs a C++ thread, not a Python one
  (COMMON_PATTERNS §13).
- **Keep the same computation.** If the faster path does not match the test, stop
  and report the difference.

---

## Step 3: Apply the change

- Make the smallest edit that moves the hot work: replace the hot loop body with the
  kit call(s). Keep the surrounding Python and the public shape of the code.
- Follow the target kit's `SKILL.md` patterns. They cover cppyy
  friction (lifetime pinning §4, container-building-in-C++ §6, keyword-name escapes
  §18, enum/`unsigned char` traps §11). Mirror the library's own API; don't invent a
  DSL (§12).
- If first-use latency matters, the kit's cache adoption is already automatic
  (`_CACHED`); otherwise call the kit's `warmup()` once at init.

---

## Step 4: Verify the result and measure it

1. **Check output.** Run the target's existing tests. If there are none, first write
   a comparison test that records the original output and checks the new output.
   Require an exact match or state the allowed numerical tolerance. For an example,
   see `examples/accelerate_demo/test_pipeline.py`, which compares occupied voxel
   indices and allows small float-summation differences. If the output does not
   match, revert the change and choose another remedy.
2. **Measure speed.** Compare before and after, then report the table:

   ```python
   from bench_before_after import compare      # skills/cppyy-accelerate/scripts/
   compare([("before", lambda: before(...)), ("after", lambda: after(...))])
   ```

   Time the *operation* after warmup. Report one-time costs such as startup and the
   cache's first `.so` compile separately. These costs are shared across later calls.
3. **Report** the hotspot, the chosen remedy, the code change, the before/after
   table, and any remaining cost, such as cppyy call wrappers.

---

## Checklist

- [ ] Profile captured; hottest frame and its cost recorded.
- [ ] Correctness contract exists (target tests, or a new differential test) and is
      passing before any change.
- [ ] Selected a kit and pattern, or documented why cppyy is not suitable.
- [ ] Minimal diff applied per the kit `SKILL.md`.
- [ ] Contract tests pass after the change.
- [ ] Before/after table measured (operation warmed; one-time costs noted).
- [ ] Report includes the hotspot, remedy, code change, timing table, and remaining cost.

A worked example on a slow point-cloud pipeline is in `WALKTHROUGH.md`. The kit
guidance this skill uses is in `docs/COMMON_PATTERNS.md`, `docs/FREEZE.md`, and
each `*_kit/SKILL.md`.

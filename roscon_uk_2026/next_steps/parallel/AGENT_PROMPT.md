# Exact agent prompt

```text
Work in this existing repository checkout. Do not create worktrees or checkouts.
Read AGENTS.md and roscon_uk_2026/next_steps/parallel/GUIDE.md explicitly.
Complete only the TODO operations in
roscon_uk_2026/next_steps/parallel/skeleton/native.cpp.
Do not read the saved solution native.cpp while solving this exercise.
Do not change model.py, native.h, processing.py, build.py, the manifest, or tests.

Detection records have validated finite x/y/z positions in metres, confidence
in [0,1], and a frame index smaller than frames. Select a row when confidence
is at least threshold and ((x*x + y*y) + z*z) is at most radius2. Return exact
byte flags and per-frame uint64 counts. No floating reduction is required.

Implement the serial predicate, a bounded oneTBB parallel_for that invokes the
provided range operation on disjoint rows, and the xsimd predicate mask. Retain
the provided scalar tail, serial histogram, input lifetime rules, instrumentation,
task arena, and global_control. Native tasks must not touch Python objects.
Consult the primary library links in GUIDE.md when needed. Do not combine TBB
and xsimd. Keep compiler vectorization enabled in the serial column baseline.

From roscon_uk_2026/next_steps/parallel run:
DETECTION_SOURCE=skeleton/native.cpp pixi run check
The unchanged checks must pass. Record the command, test-file SHA256, elapsed
time, failures, and any manual corrections. Do not install skills or change
agent settings. Report implementation results separately from performance.
```

The initial skeleton compiles but raises `std::logic_error` for unimplemented
scalar and oneTBB operations. [evidence/skeleton_failure.txt](evidence/skeleton_failure.txt)
preserves an initial failed check. This is a skeleton smoke check, not a
fresh-agent evaluation. The coordinator owns that evaluation.

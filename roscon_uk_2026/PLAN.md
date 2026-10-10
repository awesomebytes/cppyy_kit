# ROSCon UK 2026: agent-driven cppyy_kit deep dive

## Objective

Show an agent implementing robotics logic, checking its behavior, and moving
measured Python work into C++ with cppyy_kit. Use GPT-6 Luna with high reasoning
for the agent evaluations. Preserve each prompt, generated implementation,
acceptance result, and timing. One-shot means one user request without human
repair; agent self-correction is allowed and counted.

## Presentation milestones

| Order | Task | Agent entry point | Acceptance evidence |
|---|---|---|---|
| 1 | Accelerate Python motion-event detection | Implement the missing logic without cppyy guidance; use a fresh session to accelerate it | Identical event masks and intervals; warmed and first-call times |
| 2 | Accelerate MCAP analysis | Inspect humanoid recordings; implement and accelerate a pose-based query | Actual schemas and decoded samples; matching query results; I/O separated from compute |
| 2a | Compose a new native library with a selection rule | Use the prepared nanoflann recipe and retained index | Independent nearest-neighbor oracle; ownership and tie checks; build and query costs separated |
| 3 | Accelerate an rclpy node during MCAP playback | Use the same pose computation in an existing subscription | Topic/QoS contract retained; output parity; processing latency and delivered counts |
| 4 | Write a Python ROS controller at a high rate | Read control_kit guidance; complete a controller against mock hardware | Tracking error, achieved rate, update timing, missed cycles |
| 5 | Build an interactive webcam example | Complete a custom tracker; accelerate its measured hot path | Matching synthetic tests; responsiveness and processing time on a live camera |

## Work packages

1. Build small skeletons, deterministic inputs, independent acceptance checks,
   and exact prompts. Boilerplate supplies loading, entry points, and display;
   agents write the task logic. Do not require an intentionally slow baseline.
2. Supply explicit, read-only agent guidance. Users run a command to locate or
   print the guide. Do not install or modify skills, agent configuration, or
   user instructions automatically. Keep the rehearsal guide focused on its
   exact task; use the core package command for task and kit references.
3. Correct conflicting documentation and stale paths encountered by these tasks.
   Prioritize the generic @cpp route, cached calls, buffer contracts, and
   per-kit environment/setup guidance.
4. Select and pin one public humanoid manipulation MCAP. Record the source,
   revision, license, SHA-256, schemas, size, and actual topics. Keep downloaded
   data outside Git. Prefer Cartesian end-effector observations for the first
   analysis. Joint states plus URDF/FK are an extension with explicit joint
   mapping, base frame, and target link.
5. Add helpers for inspection, output comparison, benchmarking, and replay.
   Separate parsing/decompression/inference from custom numerical work.
   Probe a previously unwrapped C++ library and demonstrate a useful operation
   without creating a full kit first. Use the evaluated nanoflann recipe as
   a compact prepared segment after milestone 2.
6. Evaluate Luna high in fresh sessions and save measured results. Start with
   milestone 1. Repeat successful prompts and introduce changed requirements
   before claiming live-demo reliability. Extend evaluations to milestones 2–5.
7. Maintain DEEP_DIVE_PRESENTATION.md with exact prompts, linked source,
   inline code excerpts, rehearsal commands, measured results, and remaining
   gaps. Freeze a rehearsal version before the talk.

## Evaluation rules

- Use actual agent runs, not a hand-written solution labeled as agent output.
- Do not give the baseline session accelerated solutions or cppyy instructions.
- Keep generated work inside this folder in the existing checkout. No worktrees
  or new repository checkouts.
- Evaluate correctness independently of the agent's final message.
- Count failed attempts and human corrections. A corrected trial is a separate
  trial; do not overwrite its original evidence.
- Report model configuration, environment, input size, cache conditions,
  elapsed agent time, command counts, and program timings separately.
- A synthetic input check does not establish real-recording, middleware,
  controller, or webcam behavior. Mark each validation level explicitly.
- Program speedup is measured, never required by assertion on a shared host.

## First completion gate

A baseline agent implements the Python computation. A fresh Luna-high agent
reads the explicit guide and accelerates it. Independent checks pass and the
before/after operation timings are recorded. Dataset research and the later
milestones proceed without changing this first gate.

## Current state

Selected C++ capability experiments and the reverse direction, using Python to
improve existing C++ workflows, are recorded separately in
[CPP_PYTHON_NEXT_STEPS.md](CPP_PYTHON_NEXT_STEPS.md).

The first gate passed in the 3 October 2026 cppyy-kit 0.3.0 evaluation.
There are skeletons, exact prompts and archived agent solutions for all five
milestones. Initial Luna-high runs and independent checks are recorded in
[EVALUATION.md](EVALUATION.md). That environment passed real MCAP analysis,
ROS playback, controller-manager execution with mock hardware and headless
webcam tracking. These results remain historical evidence, separate from the
migrated current-source rehearsal.

Published 0.4.1 includes task and kit guide resources, the explicit discovery
command and environment diagnostics. Public package bytes and a standalone
NumPy/BehaviorTree.CPP project are verified in the
[release record](https://github.com/awesomebytes/cppyy_kit/blob/main/RELEASE_0.4.1_2026-10-10.md).
Diagnostics locate the compiler, runtime and development
headers; native initialization, binary compatibility and DDS communication
still require execution checks.

The [nanoflann experiment](next_steps/nanoflann/RESULTS.md) passed independent
neighbor, tie, distance and storage-lifetime checks. A fresh Sol-high agent
completed its supplied skeleton without human logic repairs in 513 seconds.
The measured benefit was native composition of tree traversal, metadata
selection and centroid calculation on synthetic data. It is not a full kit,
a recorded-data proof or repeated agent-reliability evidence.

The [talk script](DEEP_DIVE_PRESENTATION.md) now includes a prepared four-minute
native-library segment within a 45-minute budget. Its numerical example uses
`ConstNDArray[np.float64]`; mutable output annotations use `NDArray[T]`.
The rehearsal uses the current checkout through explicit Pixi activation.
Keep original generated source hashes and 0.3.0 measurements archived; record
new correctness checks and timings under the migrated environment identity.

The [AI entry point](../CPPYY_KIT_WITH_AI.md) and
[native-component tutorial](../docs/tutorials/native_component.md) support the
follow-up direction: Python configuration, testing and experimentation around
existing C++ software.

The [current source rehearsal](CURRENT_REHEARSAL.md) passed six migration
checks, 35 acceptance checks, recorded-data parity for 2,735 observations,
actual ROS playback, mock controller execution and 64 independent nanoflann
comparisons. Publication is complete. These checks remain distinct from fresh
agent evaluation and the physical-camera/full talk rehearsal.

The next work, in order:

1. Run environment diagnostics and verify Pixi activation, compiler selection,
   writable native cache and kit imports on the talk host. Check DDS with actual
   playback. Preserve failure evidence rather than assuming diagnostics prove
   the runtime works.
2. Evaluate the corrected task-first guidance with fresh sessions. Repeat each
   prompt three times and run changed-requirement cases. Treat speed
   and correctness as separate criteria. Keep unsuccessful trials.
3. Rehearse webcam mouse interaction and the whole presentation on the talk host.
   Prepare saved solutions and recorded runs for time-limited live attempts.
4. Rehearse the prepared nanoflann segment with its separate environment. Apply
   the index to verified recorded positions only as a later extension. Add
   calibrated FK and recorded-image analysis when their input/frame contracts
   are verified. Keep the five original milestone evaluations identifiable.

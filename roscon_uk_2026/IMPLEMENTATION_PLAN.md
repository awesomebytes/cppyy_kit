# Implementation plan for the selected Python and C++ experiments

Started on 4 October 2026. This implements the selected work in
[CPP_PYTHON_NEXT_STEPS.md](CPP_PYTHON_NEXT_STEPS.md). Work is performed in this
checkout. Each implementation agent uses GPT-6.1 Sol with high reasoning.

The twelve scoped experiments are delivered. The coordinator reran their checks,
reviewed shared integration, proved installed base-guide delivery, and evaluated
four fresh Sol-high completions. See [next_steps/README.md](next_steps/README.md)
and [next_steps/EVALUATION.md](next_steps/EVALUATION.md). Remaining broader scope
is recorded separately in the next-steps document.

## Subsequent upstream integration

Status reviewed on 10 October 2026 against `9449f51`. The experiment phase above
is complete. The [integration report](../EXPERIMENT_INTEGRATION_2026-10-04.md)
records the selected reusable changes, their native checks and local 0.4.0 Conda
artifact proofs. Base and per-kit guide delivery and environment diagnostics
are implemented. Channel publication remains separate. The twelve experiment
scopes below describe the original implementation assignments, not pending work.

Presentation work now follows [PLAN.md](PLAN.md): align the rehearsal with the
current API, shorten the new-library segment, repeat fresh-agent checks and
rehearse the full sequence. Preserve the dated experiment evidence during that
migration; its timings and agent completions do not measure the new workflow.

## Priorities and agent scopes

| Priority | Agent | Owned directory under next_steps/ | Plan and acceptance |
|---|---|---|---|
| 1 | reverse_core | reverse_core | Build a compiled pose-filter fixture with headers and a C++ driver. Validate configuration with Pydantic and adapt it explicitly. Provide reproducible synthetic episodes, reset, batch processing, export, and native-driver parity. State that this is a demonstration library, not an external production estimator. |
| 1 | generated_tests | generated_tests | Test the shared native filter with Hypothesis. Check reset, chunk boundaries, timestamps, empty inputs, and invalid data. Demonstrate detection of a deliberately faulty fixture and save a reduced reproducer. |
| 2 | tuning | tuning | Tune the shared native filter with Optuna. Compare defaults and an equal-budget simple search. Save trial history, independent held-out results, and a configuration reproduced by the C++ driver. |
| 2 | typed_mcap | typed_mcap | Generate typed ROS PoseStamped/Image MCAP fixtures. Extract selected fields natively with schema checks. Compare Python decoding, investigate a public typed recording, and expose timestamped arrays for reports. |
| 2 | reports | reports | Rank native-filter diagnostic/error windows and join selected images and metadata. Preserve clock rules and unmatched records. Generate a reproducible report with provenance and independent alignment checks. |
| 2 | nanoflann | nanoflann | Import a new pinned library without a kit. Retain an exact nearest-neighbor index, batch queries, and custom selection. Check distances and tie handling against brute force. |
| 3 | buffers | buffers | Demonstrate NumPy/Eigen native views, explicit shape/stride/copy rules, retained ownership, and CPU DLPack where dependencies permit. Check sharing and lifetime rather than assuming zero-copy. |
| 3 | workers | workers | Implement a bounded native worker with lifecycle and overload rules. Check ordering, errors, drain/stop, Python supervisor progress, and repeated teardown. Start with ordinary native threads. |
| 3 | extensions | extensions | Use an existing kit extension point for Python mock behavior inside a native engine. Check dispatch, lifetime, errors, and callback cost. Do not add an interface only to demonstrate inheritance. |
| 4 | parallel | parallel | Evaluate oneTBB and xsimd independently on validated Pydantic detection records. Compare serial/compiler-vectorized baselines, worker counts, conversions, and bounded thread use. |
| 4 | ceres_manif | ceres_manif | Probe toolchain compatibility first. Implement observable rigid-transform calibration if feasible. Compare derivatives, robust objectives, degeneracy, and an equivalent SciPy solve. Preserve concrete failures. |
| 4 | ruckig | ruckig | Compose persistent Ruckig trajectory generation and tracking in a native step. Compare its Python binding, test changing/invalid targets, and integrate mock control where practical. |

## Shared working rules

- Agents own separate directories and do not change another agent's files.
  The coordinator owns this plan, the overview, and integrated evaluation.
- Reuse the shared reverse_core API for tests, tuning, and reports. Agree on its
  contract before connecting consumers. Generated test inputs have independent
  expected results; tuning never uses held-out episodes to select settings.
- Use Pixi. Each independent directory may own a small manifest and lock if
  extra dependencies are needed. Do not mutate shared environments with pip.
  Clearly label checkout-only commands and local-package imports.
- Implement useful examples before general library APIs. Keep changes to the
  core package limited to demonstrated defects or required reusable behavior.
  Send the coordinator any proposed core change before editing shared files.
- Each scope delivers runnable code, a short task guide, an exact agent prompt,
  meaningful correctness checks, and a RESULTS.md with actual commands and results.
  Record dependency versions and setup/compile/conversion/runtime costs separately.
- Pin or checksum downloaded sources. Keep generated binaries and large datasets
  in ignored build/data directories. Never create a worktree or another checkout.
- Run uncertain template/library probes in subprocesses. An incompatible dependency
  must produce a reproducible feasibility result, not an unsupported success claim.
- Guides are explicitly read by the user or agent. Do not install skills or change
  agent settings automatically. Follow the skill-creator instructions if authoring
  a SKILL.md; ordinary task guides do not need to be skills.

## Integration and review

1. Review the configuration/filter contract before dependent agents finalize code.
2. Run each scope's correctness checks and inspect its evidence and limitations.
3. Exercise the shared flow: validated settings, generated tests, tuning, native
   reproduction, and dataset report. Resolve interface and environment failures.
4. Provide explicit guide discovery and a single overview of runnable commands.
5. Evaluate representative missing-logic skeletons in fresh agent sessions when
   the environment and acceptance checks are ready. Preserve unchanged check hashes,
   elapsed time, exact prompts, failures, and any manual repairs. Keep implementation
   checks distinct from fresh-agent evaluations and installed-package proofs.
6. Update the next-steps document with implemented, tested, and feasibility-only
   status. Report measured results without turning local timing into a guarantee.

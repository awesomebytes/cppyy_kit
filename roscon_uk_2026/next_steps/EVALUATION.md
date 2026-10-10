# Next-steps implementation and agent evaluation

Recorded on 4 October 2026. Twelve GPT-6.1 Sol high implementation agents worked
in separate directories in the existing checkout. The coordinator integrated
their results and added packaged guide discovery. No worktrees, extra repository
checkouts, or skill installations were created.

Status note, 10 October 2026: this page preserves the 4 October experiment
checks and agent trials. The later [integration report](../../EXPERIMENT_INTEGRATION_2026-10-04.md)
records reusable changes at `9449f51`, including per-kit guides and local 0.4.0
Conda artifact proofs. Those checks are additional evidence, not replacements
for the measurements or source hashes in this record.

## Independent implementation checks

The coordinator ran these commands from the repository root. Each directory's
RESULTS.md records additional checks, dependency versions, timings, and limits.

| Operation | Command | Result |
|---|---|---|
| Connected configuration, generated tests, tuning and reports | `pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen check-reverse` | 65 passed |
| Typed MCAP extraction | `pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen python -m pytest roscon_uk_2026/next_steps/typed_mcap/test_extract.py -q` | 9 passed |
| NumPy/Eigen/CPU DLPack | `pixi run --manifest-path roscon_uk_2026/next_steps/buffers/pixi.toml --frozen check` | 31 passed |
| Native workers | `pixi run --frozen python -m roscon_uk_2026.next_steps.workers.check` | 6 passed |
| Existing OMPL extension | `pixi run --frozen -e ompl python -m pytest roscon_uk_2026/next_steps/extensions/test_extensions.py -q` | 5 passed |
| Pydantic/oneTBB/xsimd | `pixi run --manifest-path roscon_uk_2026/next_steps/parallel/pixi.toml --frozen check` | 29 passed |
| Ruckig | `pixi run --manifest-path roscon_uk_2026/next_steps/ruckig/pixi.toml --frozen check` | 40 passed |
| nanoflann | `pixi run --manifest-path roscon_uk_2026/next_steps/nanoflann/pixi.toml --frozen check` | 64 independent oracle comparisons plus input/lifetime checks passed |
| Ceres/manif compiled adapter | `pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml --frozen check` | Recovery, derivatives, SciPy objective, degeneracy and solver-failure checks passed |
| Ceres/manif direct JIT | `pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml --frozen check-jit` | Same checks passed |
| Existing core package suite, including new guide checks | `pixi run --frozen python -m pytest cppyy_kit/tests -q` | 194 passed, 49 skipped |

The 49 core skips reflect unavailable optional capabilities in the default
environment. They are not evidence that those skipped operations work.

The report commands in [README.md](README.md) were exercised on the generated
typed fixture. The result has five ranked windows and five matched images,
with portable HTML, trace JSON, window JSON, and configuration/input provenance.
The inputs are synthetic. The separate public image slice proves extraction
parity, not native-estimator ground truth or faster image processing.

## Fresh completion sessions

[run_next_steps_eval.py](../scripts/run_next_steps_eval.py) runs isolated
ephemeral Codex sessions using GPT-6.1 Sol with high reasoning. Each starts with
an unfinished candidate, reads explicit guides, and uses checkout scaffolding.
Saved missing-logic implementations are excluded by instruction. The coordinator
reviewed command transcripts for solution reads and inspected the generated code.
This is instruction-based exclusion, not a filesystem isolation guarantee.

| Task | Elapsed seconds | Failed commands inside session | Result and evidence |
|---|---:|---:|---|
| Validate, adapt and export native configuration | 34.6 | 0 | [Passed](../next_steps_evaluation/config_sol_high_01/report.json) |
| Write Eigen transform and filtering kernels | 134.6 | 0 | [Passed](../next_steps_evaluation/buffers_sol_high_01/report.json) |
| Native replay and Optuna ask-and-tell | 242.3 | 0 | [Passed](../next_steps_evaluation/tuning_sol_high_01/report.json) |
| Bring nanoflann in and implement retained filtered index | 513.4 | 4 | [Passed](../next_steps_evaluation/nanoflann_sol_high_01/report.json) |

All four initial skeletons failed acceptance. All completed candidates passed
unchanged acceptance, exited successfully, and needed zero human logic repairs.
Each evidence directory retains the exact prompt, candidate, acceptance command,
initial failure, final output, check/candidate hashes, usage, and compressed event
transcript. The nanoflann directory also preserves generated native sources.
Its failed commands include three unsuccessful header/compiler searches and one
initial pointer-type mismatch corrected within the session.

These are four single sessions, not a statistical estimate of success rate.
They permit independent checks and documented helper imports. The configuration
exercise's relative import was adapted to an absolute import during preparation;
no missing logic was supplied. The tuning exercise evaluates three functions,
not agent delivery of a complete new tuning application.

The full new-library task took 8.6 minutes on this run. It is not yet suitable
for a short live segment. Prepare the ownership/build scaffold and leave only
the interesting native query logic missing before evaluating that presentation
variant. Luna high completion has not been evaluated for these new experiments.

## Installed guide delivery on 4 October

At this experiment stage, `python -m cppyy_kit guide` listed three packaged topics: `accelerate`,
`bring-library`, and `existing-cpp`. A topic command prints its Markdown.
The shared package recipe includes these resources. No skills or agent settings
are installed by the discovery command.

The coordinator ran:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen prove-guides
```

This uses the actual shared recipe installer with pip directed into an isolated
directory under ignored `build/`. It then imports that installed package outside
the checkout and executes each topic command. All three resources and commands
passed. The active Pixi environment was not modified. This is a local installed
source-package proof, not a published Conda-package or all-kit artifact proof.
At that point, a subsequent package build and per-kit guide delivery remained
separate work. Published 0.3.x lacks this command.

Initial proof runs failed because the root development interpreter lacked pip,
then because startup output preceded JSON. The proof now uses the integration
environment and disables auto-PCH during its isolated import/CLI checks. The
activation environment stays in memory and is not written into evidence.

The subsequent integration delivered the three task guides and per-kit
overview/API resources in local 0.4.0 Conda artifacts. Its independent checks
covered 11 artifacts, 23 exact byte comparisons and CLI reads outside the
checkout. Fresh external Pixi installation also checked numeric kernels,
diagnostics and serialized same-handle publication. The other nine wrapper
artifacts skipped native recipe tests, so resource delivery does not establish
native bringup for every domain kit. PCL, OpenCV and OMPL native behavior was
checked separately. See the [integration validation table](../../EXPERIMENT_INTEGRATION_2026-10-04.md#validation-and-upstream-status)
for the full scope; channel publication remains a separate release step.

## Review and practical limits

The independent [safety review](review/SAFETY_REVIEW.md) reproduced four defects
and verified their corrections: unaligned nanoflann input, unaligned native
speed-query input, empty speed-query buffers, and overflow in squared-speed
arithmetic. Typed extraction also gained a footer/truncation regression check.
Nanoflann concurrent cold setup was unsupported in this recorded experiment.
The later integration coordinates helper-managed headers and build cache
entries; that does not establish arbitrary application-state concurrency.

Local results support different conclusions for different operations:

- Native configuration and driver parity support reproducible experiments.
- Generated tests reduce a deliberately faulty native reset implementation to
  three operations. They do not prove absence of other defects.
- The small native worker is slower overall than NumPy. Its demonstrated value
  is bounded storage, lifecycle, and Python supervision during native work.
- oneTBB/xsimd show no established repeatable advantage over the compiler's
  vectorized columns. Preparation dominates this detection workload.
- Ruckig's retained-input operation is useful native composition. Input/output
  conversion can remove its timing advantage. Mock control is not physical or
  wall-clock real-time validation.
- Ceres/manif custom residuals work in the tested toolchain. Existing Python
  bindings were inspected; no pyceres performance comparison is claimed.
- Typed MCAP has explicit schema/CDR/encoding limits and offline compression
  conversion. Public input provenance and attribution are saved in its evidence.
- Buffer ownership and lifecycle protections apply to documented wrappers.
  Neither these checks nor a Python wrapper give arbitrary C++ Rust-equivalent
  memory-safety or data-race guarantees. Sanitizer validation remains future work.

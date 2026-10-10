# Python and C++ integration experiments

Start with one compiled C++ pose filter. Validate its configuration in Python,
test its state transitions, tune it on synthetic episodes, and reproduce the
selected settings in a standalone C++ driver. The filter is a demonstration
library written for these experiments, not a production estimator.

## Run the connected workflow

These commands require this repository checkout and Pixi. Run them from the
repository root. The integration manifest imports local code through PYTHONPATH.

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen check-reverse
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen python -m roscon_uk_2026.next_steps.reverse_core.demo
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen python -m roscon_uk_2026.next_steps.tuning.tune --output roscon_uk_2026/next_steps/build/tuning
```

The checks exercise the compiled filter, Python validation, generated inputs,
tuning, and report integration. The demo checks standalone-driver parity. The
tuning command writes trials, resolved settings, and held-out metrics under
`roscon_uk_2026/next_steps/build/tuning`. Held-out episodes do not select settings.
Grid search and Optuna have equal trial budgets; either can produce the better
settings for this small problem.

To generate a typed ROS 2 MCAP fixture and an image report:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen python -c "from roscon_uk_2026.next_steps.typed_mcap.extract import generate_fixture; generate_fixture('roscon_uk_2026/next_steps/build/episode.mcap')"
pixi run --manifest-path roscon_uk_2026/next_steps/pixi.toml --frozen python -m roscon_uk_2026.next_steps.reports.report roscon_uk_2026/next_steps/build/episode.mcap --output roscon_uk_2026/next_steps/build/report
```

The generated input has independent synthetic truth and typed PoseStamped/Image
messages. The result contains five selected windows, image matches, traces, and
an HTML report at `roscon_uk_2026/next_steps/build/report/report.html`. Missing
images remain visible. Reports on unreferenced data require `--diagnostic` and
identify their score as a heuristic.

## Choose an experiment

| Priority | Task and guide | Useful result |
|---|---|---|
| 1 | [Validated native configuration and state](reverse_core/GUIDE.md) | Pydantic validation, explicit C++ field mapping, reset and batch processing, independent driver |
| 1 | [Generated native tests](generated_tests/README.md) | Hypothesis properties, stateful sequences, and a reduced deliberate native failure |
| 2 | [Native parameter tuning](tuning/README.md) | Equal-budget searches, held-out checks, persisted trials, exported configuration |
| 2 | [Typed MCAP extraction](typed_mcap/GUIDE.md) | Native selected columns and image payloads with independent decoder parity |
| 2 | [Dataset reports](reports/GUIDE.md) | Integer-nanosecond image joins, selected windows, portable HTML and provenance |
| 2 | [Bring in nanoflann](nanoflann/LIBRARY_RECIPE.md) | Retained exact index with custom metadata selection and centroids; no new kit |
| 3 | [Buffer interoperability](buffers/GUIDE.md) | NumPy/Eigen views, explicit copying and ownership, CPU DLPack sharing |
| 3 | [Native workers](workers/README.md) | Bounded queues, declared overload, synchronized settings, joined cleanup |
| 3 | [Existing C++ extension points](extensions/GUIDE.md) | Python validity policy invoked by OMPL's native planner |
| 4 | [Pydantic, oneTBB and xsimd](parallel/GUIDE.md) | Validated records, serial/parallel/SIMD comparisons, bounded thread counts |
| 4 | [Ceres and manif](ceres_manif/GUIDE.md) | Rigid-transform calibration, automatic derivatives, matching SciPy objective |
| 4 | [Ruckig composition](ruckig/GUIDE.md) | Persistent trajectories, native tracking, official-binding parity and mock control |

Each directory contains a guide, exact prompt, unfinished skeleton, saved
implementation, independent checks, and RESULTS.md. Follow its manifest when
additional libraries are required. Some examples use installed cppyy-kit 0.3.0;
others explicitly use this checkout's newer APIs. Their guides distinguish these.

## Give the agent explicit guidance

The base package now provides `python -m cppyy_kit guide` with `accelerate`,
`bring-library`, and `existing-cpp` topics. This is read-only guide discovery,
not automatic skill installation. Current 0.4.0 source also provides installed
kit overview/API guides and `python -m cppyy_kit status --environment` diagnostics.
The [integration report](../../EXPERIMENT_INTEGRATION_2026-10-04.md) records local
Conda artifact checks outside the checkout: 11 artifacts and 23 exact resource
comparisons. Channel publication remains separate; published 0.3.x lacks these
guide commands. See [guide setup and limits](../../docs/GUIDES.md) before using
them in a standalone installed project.

For a specific experiment, read its exact prompt and guide. Do not give the agent
the saved missing-logic implementation when evaluating completion. Keep acceptance
checks unchanged. The coordinator's [evaluation summary](EVALUATION.md) separates
implementation checks from fresh-session completions and installation proofs.

## Scope and remaining work

These are tested experiments. They do not establish generic wrappers for arbitrary
libraries, arbitrary ROS schemas, GPU DLPack, or physical real-time controllers.
Typed extraction currently has an explicit schema and encoding subset. Its guide
records compression restrictions and a reproducible public-recording slice.
The native worker's small arithmetic task does not beat NumPy. Parallel/SIMD
measurements do not establish a repeatable benefit over compiler-vectorized
columns. Conversion can remove Ruckig's retained-input timing advantage.

Managed ownership and lifecycle checks apply to each documented wrapper contract.
They do not give arbitrary C++ memory safety or data-race guarantees. Writable
aliases and raw native access still require caller discipline.

The [implementation plan](../IMPLEMENTATION_PLAN.md) records ownership, priorities,
and review gates. The [next-steps document](../CPP_PYTHON_NEXT_STEPS.md) records the
broader design and experiments that remain outside this implementation.

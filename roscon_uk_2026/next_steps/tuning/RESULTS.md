# Native-estimator tuning results

Measured on 4 October 2026 in this checkout. This evaluates a synthetic fixture
and a Python experiment harness. It does not evaluate an external estimator,
a physical controller, or a fresh-agent completion.

## Commands and evidence

Commands require the repository checkout and run from its root:

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml tune --output build/frozen-run
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml replay evidence
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml python roscon_uk_2026/next_steps/tuning/check_solution.py
```

The generated JSON and resolved configurations were copied from
`build/frozen-run` to [evidence](evidence/results.json). Native binaries,
driver CSV inputs/outputs, and SQLite remain under ignored build directories.
Checked-in trial JSON contains every proposal and its resolved settings.
[Provenance](evidence/provenance.json) records source and dataset hashes, compiler,
flags, versions, parameters, seeds and sample counts.

The locked environment used Python 3.12, cppyy 3.5.0, NumPy 2.4.6,
Pydantic 2.12.5, Optuna 4.5.0, GCC/G++ 14.3.0, and libgcc/libstdc++ 15.2.0.
Pixi installation succeeded. Cold dependency download/setup time was not measured.
The native library was already compiled; the recorded build timing is a cache
lookup. The [shared fixture results](../reverse_core/RESULTS.md) measure its build.

## Accuracy and selection

Four training seeds and four disjoint held-out seeds each contain 500 samples.
The parameter bounds, sampler seed, objective and search budgets are fixed in
[the guide](README.md). All 24 Optuna trials and all 24 grid trials completed.
The default receives one extra evaluation outside both search budgets.

| Configuration | tau_s (s) | Training RMSE (m) | Held-out RMSE (m) | Mean held-out alignment lag (s) |
|---|---:|---:|---:|---:|
| Default | 0.0800000000 | 0.0498564228 | 0.0600192517 | 0.06875 |
| Optuna training selection | 0.0406899327 | 0.0381399803 | 0.0408156373 | 0.03250 |
| Grid training selection | 0.0370284235 | 0.0380818044 | 0.0401491672 | 0.03125 |

Training RMSE selected the grid setting for
[chosen.config](evidence/chosen.config). The held-out results were computed after
selection and did not choose the export. The simple grid slightly outperformed
Optuna on this run. Both chosen search settings reduced error relative to the
default on these synthetic held-out episodes. These observations do not establish
an improvement guarantee or a preference across other data.

Lag is the bounded interpolated signal-alignment shift defined in
[metrics.py](metrics.py). It has 5 ms resolution and a 300 ms upper limit. Noise,
amplitude distortion, trajectory length, and interpolation limit its meaning.
No selected configuration's held-out lag reached the upper bound. It is not
wall-clock processing latency or physical-controller response time.

All 12 standalone-driver comparisons, covering every held-out episode and all
three configurations, had maximum absolute difference `0 m`. Acceptance tolerance
was `1e-12 m` absolute and `1e-12` relative.

## Observed costs

These are one process's measured costs on a shared development machine. They are
not a speed comparison. Search time includes Python metrics, JSON writes and,
for Optuna, sampling and SQLite work. The alignment scan dominates metric work.

| Startup operation | Seconds |
|---|---:|
| Dependency imports | 0.207584 |
| Native build cache lookup | 0.002643 |
| cppyy library/header loading | 0.293485 |
| Eight synthetic episodes generated | 0.008115 |
| First constructor/adapter/native processing call | 0.063893 |

| Post-startup operation | Seconds |
|---|---:|
| Optuna, 24 complete training trials, total | 2.374546 |
| Optuna resets, summed | 0.000371 |
| Optuna process calls including validation and adapter, summed | 0.009994 |
| Optuna metrics, summed | 0.326121 |
| Grid, 24 complete training trials, total | 0.210268 |
| Grid resets, summed | 0.000092 |
| Grid process calls including validation and adapter, summed | 0.003042 |
| Grid metrics, summed | 0.183690 |
| Complete experiment after startup, including held-out/parity/export | 2.780414 |

Input arrays already have the required contiguous dtypes. No dtype/layout copy
is required. The adapter validates arrays, inspects native state, allocates an
owning output buffer and calls native processing. Those costs are included in
`process_including_adapter`; native-only and conversion-only time were not
isolated. This example makes no zero-copy output or speedup claim.

Maximum process RSS was `267.49 MiB`, including imports, cppyy, sampling, metrics,
and persistence. The recorded child maximum was also `267.49 MiB`. Linux child
maximum RSS can inherit the parent's resident footprint at fork; this is not
a sum of child memory and is not native-estimator allocation size.

## Correctness, failures, and evaluation scope

The nine pytest checks passed. They cover an independent scalar recurrence,
Cartesian RMSE units, exact known signal delay, boundary censoring, interpolation
gap exclusion, reset/episode/trial order isolation, seeded Optuna reproduction,
configuration export and C++ parity, and complete failure records.

The evidence replay reproduced all 49 complete training evaluations and all
three held-out evaluations. Source and dataset hashes matched. The chosen export
was loaded and replayed by C++ on all held-out episodes within tolerance. Repeated
runs selected the same settings and reproduced numerical metrics; their timings
varied and were excluded from reproducibility checks.

[Two saved failure trials](evidence/failure_trials.json) use a deliberately
injected Python replay exception. Their
[fixture metadata](evidence/failure_fixture.json) records the seed, budget, bounds,
dataset hash and dependency versions. Both were persisted as `FAIL` and told to
Optuna as failed trials in the correctness check. This fixture does not indicate
a failure of the real estimator.

The saved exercise solution passed its independent recurrence, reset/order,
seeded trial, selection and persistence checks. The unmodified skeleton failed
with `NotImplementedError: implement native replay, isolation and metrics`, as
expected. No fresh-agent skeleton completion has been run; agent elapsed time,
agent repairs, and installed-package proof are not measured here.

Two implementation issues were preserved in the work record. An explicit
`pixi run ... python check_solution.py` command initially ran from the repository
root and could not find the file. Using the repository-relative script path
fixed it. An initial evidence replay rejected a changed shared adapter source
hash while `reverse_core` was still being completed. The experiment was rerun
after the shared source was finalized. Source hashes were not bypassed.

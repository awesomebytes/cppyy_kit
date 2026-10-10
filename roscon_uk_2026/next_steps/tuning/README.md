# Tune a compiled pose estimator from Python

Find a smoothing setting using known Cartesian truth. Compare the default,
Optuna, and a fixed log-grid search on separate held-out episodes. Export the
training-selected setting and replay it with the standalone C++ driver.

The estimator is a separately compiled demonstration fixture from
[reverse_core](../reverse_core/GUIDE.md). It is not an external production
estimator. The input episodes are synthetic observations with known truth.
Python proposes settings, computes metrics, and saves evidence. C++ executes each
whole episode through the same estimator used by the standalone driver.

## Setup and task

These commands require this repository checkout. Run them from its root. Pixi
installs the isolated dependencies declared and locked in this directory.

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml tune --output build/my-run
```

The output path for the `tune` task is relative to this directory. Use an empty
output directory. Existing results are preserved. Expected output names the
three settings, their training and held-out RMSE in metres, and their held-out
signal-alignment lag in seconds. The default run saves 24 Optuna trials, 24 grid
trials, one default evaluation, and three held-out evaluations.

Replay the checked-in evidence without optimization:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml replay evidence
```

This verifies saved source and dataset hashes, all complete training metrics,
held-out metrics, configuration export, and the chosen export's C++ driver
parity. Saved source hashes are intentionally strict. A source change requires
a new experiment in a new output directory.

## Experiment declared before selection

The default budget is 24 proposals per search. Both searches replay identical
training seeds `0, 1, 2, 3`, with 500 samples per episode. Held-out seeds
`100, 101, 102, 103` never supply a selection metric. Every episode starts with
`reset()`. A trial constructs its own native estimator. Trials run sequentially.
Each complete episode uses one native `process` batch call.

Only `tau_s` varies, with logarithmic bounds `[0.005, 0.5]` seconds. The other
resolved fields stay at `max_gap_s=0.5` and `frame_id=world`. Optuna uses
`TPESampler(seed=20261004, n_startup_trials=8)` and
[ask-and-tell](https://optuna.readthedocs.io/en/v4.5.0/tutorial/20_recipes/009_ask_and_tell.html).
The grid uses 24 geometrically spaced points including both bounds. The default
configuration receives one separate evaluation and is available to final
selection. Both search budgets count failed proposals. No pruning, retries, or
parallel trials are used.

The objective is full-episode pooled Cartesian position RMSE:
`sqrt(sum(||estimate - truth||²) / total_samples)`, in metres. Startup samples
and samples after the deliberate long gap remain in this error metric. Each
search selects its smallest training RMSE. The final export selects the smallest
training RMSE among defaults and both search selections. Equal scores retain
order defaults, Optuna, grid. All three are then evaluated on held-out seeds.
Held-out results do not choose the exported setting. CLI overrides `--budget`,
`--samples`, and `--seed` are recorded in provenance and describe a different run.

The lag measure searches positive shifts from `0` to `0.30` seconds in `0.005`
second steps. It compares `truth(t)` with linearly interpolated `estimate(t+shift)`
on one common set of truth timestamps for every candidate. It removes the first
`0.5` seconds and excludes all timestamps whose candidate queries could cross a
gap larger than `0.05` seconds. A known delayed synthetic curve checks the shift
measurement. Per-episode lag and the arithmetic mean across episodes are saved.

This is a bounded signal-alignment measurement. Noise, smoothing amplitude
changes, finite trajectories, and interpolation can bias it. The resolution is
5 ms; an upper-bound minimum is censored and saved as `at_upper_bound=true`.
The measure does not identify wall-clock processing latency or robot response.
It is descriptive and is excluded from the tuning objective.

## Saved outputs and checks

`provenance.json` records seeds, bounds, budget, dataset content hashes, source
hashes, package versions, compiler version, flags, platform, and startup timings.
`optuna_trials.json` and `grid_trials.json` contain every resolved configuration,
state, per-episode metrics, and replay timings. Failed native replays include the
exception type, message, and traceback. Optuna also stores its study in SQLite.
The JSON histories are sufficient to replay complete trials without SQLite.
`selection.json` is saved before the first held-out replay.

`results.json` includes all three configurations' training/held-out results,
per-episode bounded-lag flags, C++ parity, timing and maximum resident memory.
`defaults.config`, `optuna.config`, `grid.config`, and `chosen.config` use the
[shared driver's resolved format](../reverse_core/CONTRACT.md). The driver inputs
and outputs remain in the ignored output directory. The native library and
driver remain in the shared fixture's ignored build directory.

The independent checks use a Python recurrence to verify native outputs. They
check analytic RMSE units, known alignment shifts, gap exclusion, repeated
resets, episode/trial order, seeded ask-and-tell reproducibility, durable failure
records, dataset hashing, export round-trip, and native-driver parity. Metrics
replay tolerance is `1e-12 m` absolute and `1e-12` relative. Lag grid values must
match exactly. Timing is not included in numerical reproducibility comparisons.

Sequential sampling is deliberate. Optuna's
[reproducibility guidance](https://optuna.readthedocs.io/en/v4.5.0/faq.html#how-can-i-obtain-reproducible-optimization-results)
requires sampler seeding and warns that concurrent trials introduce nondeterminism.
The manifest pins compatible GCC and runtime versions because this repository's
[cppyy package recipe](../../../recipe/cppyy-kit/recipe.yaml) documents a Cling
startup failure with a different compiler/runtime combination.

## Agent exercise

Read [PROMPT.md](PROMPT.md) for the exact prompt. The missing logic is in
[skeleton.py](skeleton.py). The saved implementation is [tune.py](tune.py), exposed
through [solution.py](solution.py). Run the independent exercise check from the
repository root:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml python roscon_uk_2026/next_steps/tuning/check_solution.py
pixi run --manifest-path roscon_uk_2026/next_steps/tuning/pixi.toml python roscon_uk_2026/next_steps/tuning/check_solution.py roscon_uk_2026/next_steps/tuning/skeleton.py
```

The second command fails until the skeleton is completed. To connect a repaired
skeleton to the complete experiment, import `tune` and the repaired module, then
assign its `replay_trial`, `ask_and_tell`, and `select_training` to
`tune.evaluate`, `tune.search`, and `tune.best_complete` before calling `tune.run`.
Keep the checks unchanged. These are explicit guides, not installed skills.
A fresh-agent completion has not been evaluated. [RESULTS.md](RESULTS.md) records
implementation checks and observed results.

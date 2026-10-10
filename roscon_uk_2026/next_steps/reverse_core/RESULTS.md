# Configuration and native-filter results

Measured on 4 October 2026 in this checkout on Linux x86_64, kernel `6.17.0-1032-oem`, glibc 2.39. The compiled component is a demonstration library created for this experiment. It is not an external production estimator.

## Commands and checks

Run these commands from the repository root. They require this checkout:

```bash
pixi install --locked --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml python -m roscon_uk_2026.next_steps.reverse_core.check_exercise
pixi run --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml demo --benchmark
```

Actual correctness result: **31 passed in 0.46 s**. Checks cover invalid settings before native loading, revalidation of unchecked Pydantic models, resolved text/JSON round-trips, explicit native fields and units, loader rejection, exact gap threshold, signed int64 timestamp limits, extreme finite positions, transactional native rejection, empty batches, dtype and shape rejection, read-only/unaligned/noncontiguous inputs, owning output lifetime, reset and chunk equality, independent instances, repeated closure, deterministic episodes, driver rejection, and cppyy/driver equality for empty, one-sample, and 125-sample inputs.

The saved exercise solution printed:

```text
PASS: validation before native loading, explicit field/units mapping, default export, native-loader replay
```

The checked-in skeleton remains incomplete. It is an exercise artifact. No fresh-agent completion or installed-package proof is claimed here.

## Versions and setup

| Component | Verified version |
|---|---|
| Python | 3.12.14 |
| NumPy | 2.5.3 |
| Pydantic | 2.13.5 |
| pytest | 8.4.2 |
| cppyy | 3.5.0 |
| cppyy Cling package | 6.32.8 |
| Conda C++ compiler | GCC 14.3.0, build 20 |
| cxx-compiler metapackage | 1.11.0 |
| libgcc / libstdcxx | 15.2.0 |

The manifest pins direct dependencies. [pixi.lock](pixi.lock) records transitive versions, builds, URLs, and checksums. `PYTHONPATH` explicitly resolves `cppyy_kit` from this checkout. The actual import was `/home/sapf/playground/ros_project/cppyy_kit/cppyy_kit/__init__.py`.

An initial unpinned compiler/runtime resolution crashed during `import cppyy` in Cling `AddHostArguments`, with exit status 139. It selected GCC/G++ 15.3 and libstdcxx 16.2. Applying the compatibility pins already documented in [the repository recipe](../../../recipe/cppyy-kit/recipe.yaml) produced successful imports and native replays. This is a recorded environment failure and correction.

An already-installed locked Pixi setup was measured with:

```bash
/usr/bin/time -f 'elapsed_s=%e peak_rss_kib=%M' pixi install --locked --manifest-path roscon_uk_2026/next_steps/reverse_core/pixi.toml
```

Actual result: `elapsed_s=0.04 peak_rss_kib=37892`. This is a warm setup check. The initial download/install duration was not measured. A separate new Python process importing the shared Python module took 0.1515 s; it did not load cppyy or native symbols.

## Replay evidence and measured costs

[evidence.json](evidence.json) saves the actual benchmark result, source SHA-256 values, configuration, episode identity, versions, absolute build paths, and timing values. Regeneration writes `build/demo/results.json`, `resolved.cfg`, `input.csv`, and `output.csv`.

The benchmark used `synthetic-pose-v1-seed-0-n-500`, with Cartesian values in metres and caller-declared frame `world`. It used `tau_s=0.08` and `max_gap_s=0.5`. The noisy observation RMSE was **0.082599 m**. Filtered RMSE was **0.072077 m**. The maximum absolute difference from standalone C++ replay was **0 m**, with exact array equality. There was one long-gap reset.

| Phase | Measured time |
|---|---:|
| Synthetic episode generation | 3.995 ms |
| Forced compilation of library and executable | 3.203 s |
| Native import/header/library loading | 237.1 ms |
| First native construction through the adapter | 28.06 ms |
| First validated Python batch replay | 11.14 ms |
| Driver configuration/CSV export, subprocess, and output parse | 9.666 ms |
| Warm direct native batch, 500 samples, median | 5.363 microseconds |
| Warm direct native batch, 500 samples, 95th percentile | 6.217 microseconds |
| Warm validated Python batch, 500 samples, median | 16.01 microseconds |
| Warm validated Python batch, 500 samples, 95th percentile | 18.29 microseconds |
| Contiguous input validation, median | 3.643 microseconds |
| Noncontiguous validation and copies, median | 7.218 microseconds |

Warm measurements used 100 repetitions. Reset ran before each replay and was outside the measured operation. Direct native replay reused preallocated input/output buffers. Python replay includes validation, state inspection, and output allocation. Each replay makes one native batch call. These measurements show local boundary and validation costs. They are not claims of speedup over another estimator or of real-time suitability. Native loading used an already available cppyy precompiled header; a cold cppyy PCH build is excluded.

`sizeof(reverse_demo::PoseEstimator)` was **104 bytes**. The frame string can use separate bounded storage. Input and output arrays scale with batch size. The measured process peak RSS was **227828 KiB**, including Python, NumPy, Pydantic, cppyy, and its interpreter. This peak is not an estimator-only memory figure. Native allocation counts and a steady-state RSS profile were not measured. The implementation retains no batch or sample history.

## Limits

Episodes are seeded synthetic trajectories and noise, with independent training seeds `(0,1,2,3)` and held-out seeds `(100,101,102,103)`. This scope defines their identity and separation. It does not select parameters on held-out data or claim real recording quality. The tuning consumer reports its separate train/test evaluation.

The component filters position only. It does not estimate orientation, uncertainty, sensor bias, transform frames, or model robot dynamics. `max_gap_s >= tau_s` is a demonstration validation policy. One native instance is single-threaded. Direct C++ callers must supply valid buffer lengths and non-overlapping storage. Compiler/ABI behavior was checked only in the pinned Linux x86_64 environment.

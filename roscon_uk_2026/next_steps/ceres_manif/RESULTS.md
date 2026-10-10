# Ceres and manif feasibility results

Measured on 4 October 2026 in this Linux x86-64 checkout. The corrected direct
cppyy JIT path and compiled adapter both solve the full synthetic calibration
and pass the independent checks. This establishes a bounded feasibility result
for custom native residuals driven from Python. It is not a general kit or a
comparison against existing pyceres performance.

## Reproduction

All commands require this repository checkout. They use the independent
[manifest](pixi.toml) and [lock](pixi.lock), not the main workspace environment.

```bash
pixi install --locked --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml probe
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml demo
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml demo-jit
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml check-jit
```

Both check tasks exit 0. Saved numerical records are
[compiled results](evidence/results_compiled.json),
[JIT results](evidence/results_jit.json),
[compiled checks](evidence/acceptance_compiled.json), and
[JIT checks](evidence/acceptance_jit.json).

## Dependency and ABI gate

| Dependency | Version | Locked build |
|---|---|---|
| Python | 3.12.13 | h8ab3286_1_cpython |
| cppyy | 3.5.0 | py312h0a2e395_0 |
| cppyy-kit | 0.3.0 | pyh4616a5c_2 |
| Ceres Solver | 2.2.0 | cpugplhc142d66_210 |
| manif | 0.0.5 | hca8cc02_6 |
| Eigen | 3.4.0 | h54a6638_2 |
| gcc_linux-64 | 14.3.0 | h50e9bb6_28 |
| gxx_linux-64 | 14.3.0 | h3ba8f88_28 |
| libgcc | 15.2.0 | he0feb66_20 |
| libstdcxx | 15.2.0 | h934c35e_20 |
| NumPy | 2.5.3 | py312he827f4e_0 |
| SciPy | 1.17.1 | py312h54fa4ab_1 |

The lock contains package URLs and SHA256 checksums. No downloaded source tree
or pip installation was required. Ceres is the CPU build. Both C++ paths report
`__cplusplus=201703`, `EIGEN_MAX_ALIGN_BYTES=16`, and
`_GLIBCXX_USE_CXX11_ABI=1`. The adapter compiles with the activated compiler,
`-std=c++17 -O2 -DGLOG_USE_GLOG_EXPORT`, the environment's Eigen headers, and
`-lceres -lglog -lgflags`. Its RPATH uses that same environment's `lib` directory.
The Ceres installed CMake target declares C++17 and links glog, gflags, and Eigen.
The adapter passes primitive buffers and a plain report struct across cppyy;
it exposes no Eigen object layout to Python.

The small corrected [subprocess probe](evidence/probe.json) took 2.115 s and
returned cost `2.6495e-27`. The full JIT implementation also passed. The following
failed probes remain reproducible:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml probe --missing-exports
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml probe --missing-manif-helper
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml probe --compile --missing-manif-helper
```

The probe launcher records each child's exit status and intentionally exits 0
so a failed feasibility probe can be inspected. The missing glog definition
returns child status 1 after 0.605 s with `GLOG_EXPORT` errors. See
[full parse failure](evidence/probe_missing_exports.json). The missing manif
Ceres helper crashes Cling's `DeclUnloader` with child status 129 after 5.052 s.
See [full subprocess failure](evidence/probe_missing_manif_helper.json).
The equivalent ordinary compiler probe returns status 1 after 2.017 s: manif's
unspecialized `Constants<ceres::Jet<double,6>>` uses non-literal constexpr data.
See [compiler diagnostic](evidence/probe_missing_manif_helper_compile.json).
Including the manif Ceres helper fixes the template compatibility issue.

An early adapter edit also hit C++'s function-declaration ambiguity when
constructing a templated point with parentheses. Brace initialization fixed it.
The successful timings and source reflect that repair. The compiled adapter
remains available to keep large template headers out of the live interpreter;
it is not required by the corrected JIT path.

## Recovery and independent checks

The generator uses seed 2026, 20 noncoplanar landmarks, 12 varied poses, and 240
paired points. Translation truth is `[0.30,-0.20,0.50]` m. Rotation truth is
`[0.25,-0.35,0.18]` rad as an SO(3) rotation vector. See
[frame and objective conventions](GUIDE.md#frame-rotation-and-objective-conventions).

| Case | Translation error, m | Rotation error, rad | Ceres objective | SciPy objective |
|---|---:|---:|---:|---:|
| Exact | 8.79e-16 | 5.95e-16 | 1.18e-28 | 1.26e-30 |
| 0.005 m noise | 5.45e-4 | 4.85e-4 | 0.00946967241156334 | 0.00946967241156333 |
| Noise plus 10% outliers, linear | 0.005224 | 0.015434 | 6.596746767570225 | 6.596746767569609 |
| Noise plus 10% outliers, Huber | 3.78e-4 | 0.001159 | 0.4837619957300614 | 0.4837619957300608 |

The exact transform also matches independent Kabsch registration to `1e-9`.
Noisy and robust cases satisfy declared limits of 0.004 m and 0.006 rad. Huber
uses one norm per 3D correspondence with threshold 0.03 m. SciPy uses a radial
residual transformation and linear loss to reproduce that scalar objective.
The robust parameter vectors differ by `2.47e-9` in Euclidean six-vector norm.
The check requires objective agreement below `1e-10` and parameter agreement
below `2e-7`. Both solves start from zero and have no bounds. Tolerance values
are `1e-12`, but algorithms, stop tests, and budgets differ. These are matching
objective solutions, not identical solver iterations.

Ceres autodiff differs from independent central differences by at most
`2.27e-10` over zero, truth, and a larger rotation. The finite-difference step is
`1e-6`; the check allows `2e-8` absolute and relative error. manif's analytic
right SE(3) action Jacobian differs from independent SciPy matrix-exponential
perturbations by `1.32e-10`. A finite right perturbation also matches the
independent result to `1e-12`. The analytic Jacobian has tangent coordinates;
it is not substituted for the additive rotation-vector Jacobian.

Tests reject coincident and collinear points, nonfinite observations, shape
mismatches, noncontiguous buffers, and invalid loss options. A noncollinear
planar fixture succeeds. With zero solver iterations the result is termination
1, `usable=true`, and `converged=false`. The initial/final cost remains 4.95038
and the transform remains zero. This preserves a concrete unsuccessful solve.

## Costs and limits

These are single-host observations. Runs occurred in a shared development
container; timings do not establish a production speed guarantee. BLAS and OMP
thread counts are fixed to 1. Ceres uses one thread and dense QR.

| Operation | Compiled adapter | Direct JIT |
|---|---:|---:|
| Adapter native compile | 8.954 s | Not used |
| Adapter load and declarations | 0.260 s | Not used |
| Full header JIT and library setup | Not used | 6.163 s |
| Warm native Huber solve, median of 20 | 0.574 ms | 0.724 ms |
| Warm one-point evaluation including boundary | 2.53 microseconds | 2.53 microseconds |
| Explicit noncontiguous 240-point copy | 1.11 microseconds | 1.00 microseconds |
| Full check process wall time | 0.63 s | 7.09 s |
| Full check process peak RSS | 253080 KiB | 672836 KiB |

The warm solve excludes validation, parameter copying, report extraction, and
independent objective computation. It includes native problem construction,
point copies, solver execution, and the cppyy crossing. Ceres' own
`solve_seconds` is recorded separately. The first crossing can cost more:
the exact compiled case used 5.74 ms including lazy call setup. SciPy's robust
solve took 7.03 ms in the saved compiled-run comparison. It computes finite
differences and uses a different step model, so the timing is not a binding-only
comparison. Compilation is cached by source, lock, compiler identity, and
environment path.

Memory/time records use `/usr/bin/time -f` around the corresponding Pixi check
task. See [compiled resources](evidence/resource_compiled.json) and
[JIT resources](evidence/resource_jit.json). A repeated `pixi install --locked`
on the already populated environment took 0.04 s. See
[installed lock check](evidence/installed_lock_check.json). Initial downloading
and cold environment setup were not timed. The initial cppyy import also built
its standard precompiled header before these reported startup runs.

The native residual is templated and composed with manif operations. Python
controls initial values, arrays, loss scale, stopping options, and result checks.
[pyceres](https://github.com/cvg/pyceres) and
[manif's Python wrappers](https://github.com/artivis/manif) already cover general
bindings. Their modules were absent in this environment. They were inspected
but not benchmarked or reimplemented here. This study's value is the custom
native residual and direct composition with Ceres autodiff.

The [missing-residual skeleton](skeleton/residual.hpp) fails the frozen checks;
the saved solution passes after copying into the exercise. See
[exercise validation](evidence/exercise_validation.json). This was implementation
validation, not a fresh-agent task completion. The exact prompt is in
[PROMPT.md](PROMPT.md). No agent completion time is claimed.

Point matching, time-offset estimation, hand-eye motion calibration, real data,
large rotations across chart boundaries, sparse/GPU solving, installed-package
distribution, and other cppyy/Eigen/compiler combinations remain untested.

## Primary references

- [Ceres modeling and autodiff](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html).
- [Ceres solver options and termination](https://ceres-solver.readthedocs.io/latest/nnls_solving.html).
- [manif group operations and tangent Jacobians](https://github.com/artivis/manif).
- [manif Ceres helper header](https://github.com/artivis/manif/blob/devel/include/manif/ceres/ceres.h).
- [SciPy least_squares objective, loss, and tolerances](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html).

These online references can track newer versions. The executed versions and
actual outputs are pinned above and in the saved evidence.

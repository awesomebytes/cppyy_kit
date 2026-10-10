# Exact agent prompt

Read `roscon_uk_2026/next_steps/ceres_manif/GUIDE.md` first. Work only in
`roscon_uk_2026/next_steps/ceres_manif/build/exercise`. Do not use Git worktrees,
new checkouts, pip, skills installation, or additional agents.

Implement the missing templated `PointResidual::operator()` in `residual.hpp`.
Do not read or copy the saved residual implementation outside the exercise.
Use manif's SO(3) exponential and SE(3) group action. The six parameters are
target-frame translation in metres followed by a rotation vector in radians.
The residual is predicted target point minus measured target point. Preserve
the struct fields and public interface. Do not change the dataset, solver,
tests, tolerances, or check hashes. Preserve the required glog definition and
manif Ceres helper inclusion. Inspect the supplied native solve and Python
orchestration before editing.

Run the frozen checks using this repository-root command:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml python roscon_uk_2026/next_steps/ceres_manif/build/exercise/acceptance.py
```

Run its direct JIT variant in a separate process:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml python roscon_uk_2026/next_steps/ceres_manif/build/exercise/acceptance.py --jit
```

Check SHA256 hashes against `checksums.json`. Report edited files, elapsed time,
commands, derivative errors, noisy recovery errors, solver termination, and any
failed probes or manual repairs. Do not claim a fresh-agent evaluation from the
saved solution's results.

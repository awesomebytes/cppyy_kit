# Work diary — 2026-09-28

- Added a UR5 RRT-Connect comparison using RoboPlan's bundled model and one collision scene for both planners. It validates endpoints, bounds, and sampled path segments, then ranks median solve time only if every measured trial validates.
- Added an optional Pixi environment, tutorial, and documentation links. RoboPlan 0.7 is the intended dependency; the existing OMPL environment resolves OMPL 1.7.
- Local Pixi syntax compilation and Flake8 passed. The benchmark did not run: the existing OMPL environment is missing `libcblas.so.3`, while Pixi cannot download packages needed to build the combined environment (`conda-forge` DNS request failed). `pixi.lock` remains unchanged. No measured speed ranking is available yet.
- Next: restore local Pixi package access, update `pixi.lock`, run the comparison locally, review the observed paths and timing, and revise the tutorial with the measured result.

## Local repair follow-up

- Restored the missing OpenBLAS library in the gitignored OMPL Pixi prefix from the exact package in the local Pixi cache. `pixi run --frozen --no-install -e ompl demo-ompl-plan` now passes and finds a valid path.
- `pixi run --frozen --no-install -e ompl python -c 'import roboplan'` fails with `ModuleNotFoundError`. No RoboPlan package or checkout is present locally. The command shell explicitly has `CODEX_SANDBOX_NETWORK_DISABLED=1`; direct HTTPS by IP also fails. A writable Pixi cache under `/tmp` does not change this. The RoboPlan comparison and speed ranking remain unmeasured until this shell can download the package or receives it locally.

## Comparison completed after local network access was restored

- `pixi install -e roboplan-ompl` succeeded and updated `pixi.lock`. The local environment has RoboPlan 0.7.0, OMPL 1.7.0, and Python 3.12.13.
- Ten fresh x86-64 processes each ran one unmeasured warmup and five measured trials per planner. Both planners returned paths that passed the shared validation in all 50 measured trials. Median of process medians: RoboPlan 0.48 ms (range 0.47–0.49 ms), OMPL 1.36 ms (range 1.21–1.52 ms). RoboPlan ranked first in all ten processes. This measures the Python-facing integrations, including OMPL's Python collision callback; it does not isolate planner kernels.
- Updated the tutorial with the measured result and corrected the API excerpts against RoboPlan 0.7. The runnable comparison reports the speed ratio and states its warmup and random-seed protocol.
- Final audit found that OMPL's first `solve()` on each new setup included lazy initialization. The demo now calls `setup.setup()` before timing, and reports solve failures separately from invalid returned paths. After this correction, ten fresh processes again validated 50/50 paths per planner. Median of process medians: RoboPlan 0.48 ms (0.47–0.49 ms), OMPL 1.33 ms (1.13–1.41 ms). RoboPlan ranked first in all ten processes.

## World-obstacle tutorial and planner sweep

- Added a fixed box to the UR5 RoboPlan scene. A separately loaded box-free scene uses the same URDF and SRDF; bounded, seeded endpoint selection accepts only endpoints valid in both scenes and a 0.05-rad sampled direct edge that is clear without the box and blocked with it. The same obstacle scene and endpoints are used by both planners.
- Ten fresh-process comparisons validated all 50 returned paths per planner. Median of process medians: RoboPlan 1.19 ms (range 1.18–1.29 ms), OMPL 1.495 ms (range 1.19–3.43 ms). RoboPlan was faster in nine runs; one run tied at 1.19 ms. A single local demo run also validated 5/5 paths each (1.22 ms RoboPlan, 1.58 ms OMPL). These are Python-facing integration timings: OMPL calls a Python validity callback while RoboPlan checks collisions in C++; they do not isolate planner kernels and apply only to this model, scene, endpoint pair, settings, and machine.
- Added an optional 2D OMPL sweep for RRTConnect, RRTstar, and PRM. Each planner runs in a fresh subprocess with the same seed; results include validity, solve time, and path length, and valid paths are drawn to [ompl_planner_sweep.svg](docs/tutorials/ompl_planner_sweep.svg). The full default run (1 s cap per planner) returned valid paths for all three and generated the linked SVG; a separate short-cap Pixi task smoke run also returned valid paths for all three. RRTstar uses its time budget to improve its path, so its solve time is not directly comparable with the other planners.
- Validation performed: the local UR5 demo, full default and shifted-obstacle 2D sweeps, and `bench-ompl --micro-n 200000` passed; `pixi run -e roboplan-ompl test-ompl` passed 9/9; Flake8 passed on both demo scripts; strict documentation build passed. In a separate 2D collision-check microbenchmark, the Python callback measured 300.8 ns/check and the JIT-compiled C++ checker 15.4 ns/check; these microbenchmark results are separate from planner timings.

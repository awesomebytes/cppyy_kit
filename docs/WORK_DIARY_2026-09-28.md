# Work diary — 2026-09-28

- Added a UR5 RRT-Connect comparison using RoboPlan's bundled model and one collision scene for both planners. It validates endpoints, bounds, and sampled path segments, then ranks median solve time only if every measured trial validates.
- Added an optional Pixi environment, tutorial, and documentation links. RoboPlan 0.7 is the intended dependency; the existing OMPL environment resolves OMPL 1.7.
- Local Pixi syntax compilation and Flake8 passed. The benchmark did not run: the existing OMPL environment is missing `libcblas.so.3`, while Pixi cannot download packages needed to build the combined environment (`conda-forge` DNS request failed). `pixi.lock` remains unchanged. No measured speed ranking is available yet.
- Next: restore local Pixi package access, update `pixi.lock`, run the comparison locally, review the observed paths and timing, and revise the tutorial with the measured result.

## Local repair follow-up

- Restored the missing OpenBLAS library in the gitignored OMPL Pixi prefix from the exact package in the local Pixi cache. `pixi run --frozen --no-install -e ompl demo-ompl-plan` now passes and finds a valid path.
- `pixi run --frozen --no-install -e ompl python -c 'import roboplan'` fails with `ModuleNotFoundError`. No RoboPlan package or checkout is present locally. The command shell explicitly has `CODEX_SANDBOX_NETWORK_DISABLED=1`; direct HTTPS by IP also fails. A writable Pixi cache under `/tmp` does not change this. The RoboPlan comparison and speed ranking remain unmeasured until this shell can download the package or receives it locally.

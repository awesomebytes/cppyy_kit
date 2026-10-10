# Calibrate a rigid sensor transform

Estimate `T_target_source` from paired 3D points. The example generates 240
correspondences, fits a transform with Ceres and manif, and checks the same
objective with SciPy. This helps developers try a custom native residual while
Python controls datasets, options, and validation.

Run these commands from this repository checkout:

```bash
pixi install --locked --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml guide
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml probe
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml demo
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml check
```

The expected exact solution is translation `[0.30, -0.20, 0.50]` metres and
rotation vector `[0.25, -0.35, 0.18]` radians. The check also requires translation
error below 4 mm and rotation error below 0.006 rad for the declared noisy cases.
Output is saved under this directory's ignored `build/` directory.

`demo` uses a compiled adapter. `demo-jit` and `check-jit` run the full native
implementation through direct cppyy JIT in a separate process. Run `probe` first
when changing dependencies. [RESULTS.md](RESULTS.md) records both successful
paths and missing-header failures.

## Frame, rotation, and objective conventions

All points use metres. The transform predicts a target-frame point from a
source-frame point: `q = Exp_SO3(w) p + t`. The six optimization parameters are
`[t_x, t_y, t_z, w_x, w_y, w_z]`. Translation is expressed in the target frame.
This vector is not an SE(3) exponential coordinate: translation is independent
of the rotation vector. The study uses known point correspondences, not
hand-eye calibration from unknown relative motions.

The generator observes 20 noncoplanar landmarks under 12 independently varied
rotations and translations. Noise is independent Gaussian noise with coordinate
standard deviation 0.005 m. The outlier case adds coordinate noise with standard
deviation 0.4 m to 24 selected correspondences. Seed 2026 fixes all cases.

Each correspondence produces one three-component residual block. Ceres applies
Huber loss to the block's squared Euclidean norm with threshold 0.03 m. The
objective is `0.5 sum rho(||r_i||²)`, where `rho(s)=s` below the squared threshold
and `rho(s)=2*a*sqrt(s)-a²` above it. This follows the
[Ceres residual-block definition](https://ceres-solver.readthedocs.io/latest/nnls_modeling.html).

SciPy receives `sqrt(rho(s)/s)*r_i` and uses linear loss. This has exactly the same
scalar objective. Passing ordinary coordinates to SciPy's `loss="huber"` would
robustify each coordinate separately and produce a different objective. See
[SciPy loss semantics](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html).

Both solvers start at zero, have no bounds, and set function, parameter, and
gradient tolerances to `1e-12`. Ceres permits 100 iterations; SciPy permits 100
function evaluations. Their stopping tests, scaling, and step models differ.
Equivalent final objectives do not imply identical iterations or solver budgets.

## Use your own observations

From this example directory inside its Pixi environment:

```python
import numpy as np
from calibration import native_solve
from native import load

api, startup = load()
# source and target are N-by-3 arrays in the declared frames.
source = np.ascontiguousarray(source, dtype=np.float64)
target = np.ascontiguousarray(target, dtype=np.float64)
x, report = native_solve(api, source, target, scale=0.03)
assert report["converged"], report
```

The wrapper requires matching finite C-contiguous float64 arrays. It retains
them during the synchronous solve. Native code reads these buffers directly;
Ceres copies each point into its residual object. A noncontiguous input is
rejected until the caller makes an explicit copy. The initial parameter vector
is copied so the caller's vector is not modified. Source points that coincide
or lie on one line are rejected because rotation is unobservable. Planar,
noncollinear correspondences can determine a rigid transform and are accepted.

`usable` and `converged` are separate report fields. A zero-iteration solve can
return usable parameters with termination code 1 and `converged=false`.
Never accept a calibration solely because `usable` is true.

## Read before implementing a residual

[residual.hpp](residual.hpp) is the saved solution. Ceres instantiates its
templated operator with `ceres::Jet`. manif supplies the SO(3) exponential and
SE(3) group action inside that operator. [adapter.cpp](adapter.cpp) also exposes
manif's analytic action Jacobian in right SE(3) tangent coordinates. The check
compares that Jacobian against independent matrix-exponential perturbations.
It separately compares Ceres' six-parameter autodiff Jacobian against SciPy
finite differences. The two Jacobians use different coordinates.

Define `GLOG_USE_GLOG_EXPORT` before including Ceres. Include the manif group
header before `manif/ceres/ceres.h`. The latter provides Ceres Jet support.
See [manif's helper header](https://github.com/artivis/manif/blob/devel/include/manif/ceres/ceres.h).

## Missing-residual exercise

Prepare an isolated file exercise within this directory. This makes no checkout
and does not replace the saved solution:

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/ceres_manif/pixi.toml python roscon_uk_2026/next_steps/ceres_manif/exercise.py
```

Read [PROMPT.md](PROMPT.md) explicitly and give its text to the agent. The
exercise writes frozen check hashes into `build/exercise/checksums.json`.
Its unimplemented residual returns false and the checks fail with
`residual evaluation failed`. The implementation checks and skeleton rejection
have been run. A fresh-agent completion has not been evaluated.

## Existing bindings and scope

[pyceres](https://github.com/cvg/pyceres) already exposes Ceres and supports Python
cost functions. [manif](https://github.com/artivis/manif) already has Python
wrappers and Jacobian operations. Neither Python module is installed in this
experiment's environment. The study adds a task-specific native residual and
native solve composition; it does not provide a replacement general binding.
The comparison here uses SciPy, not a measured pyceres/manifpy implementation.

No time offset, real sensor data, GPU execution, deployment package, or
real-time claim is included. Package/build details and timing limits are in
[RESULTS.md](RESULTS.md).

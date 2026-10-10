# Transform and filter a point batch through native buffer views

Transform model-produced Cartesian points into a target frame. Keep points
inside a sphere and compute the sum of their squared ranges in a second native
operation. This avoids Python objects per point and passes the filtered output
between native operations without another buffer conversion.

These commands require this repository checkout. From the repository root:

```bash
cd roscon_uk_2026/next_steps/buffers
pixi install --locked
pixi run guide
pixi run probe
pixi run check
pixi run demo
```

Expected result: the probe returns zero, 31 checks pass, and the demo writes
`measurement.json`. The seeded 100,000-point batch retains 36,994 points inside
a sphere of radius 1.4 metres. Pointer checks report shared input, CPU DLPack
storage, and filtered output storage. Transform and filter outputs have separate
allocations. Dependency installation happens through the local Pixi manifest.
No installed-package-only workflow is claimed: `buffers.py` imports the core
package from this checkout.

Run this smaller example from the same directory:

```bash
pixi run python - <<'PY'
import numpy as np
from buffers import BorrowedPoints, process

# Source coordinates in metres. Rows are points; columns are x, y, z.
points = np.array([[1., 0., 0.], [3., 0., 0.]])
rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
translation = np.array([0.5, 0., 0.])
points.flags.writeable = False
with BorrowedPoints(points) as lease:
    result = process(lease, rotation, translation, radius=2.)
print(result.points)
print(result.energy)
PY
```

Expected coordinates are `[[0.5, 1.0, 0.0]]`. Energy is `1.25` square metres.
The convention is `target_column = rotation * source_column + translation`.
The rotation must be orthogonal with determinant +1, within `1e-12`. Filtering
uses the target-frame origin and an inclusive squared-radius comparison. Row
order is preserved. Empty input or an empty selection produces `(0, 3)` and
energy `0.0`. Input coordinates and rigid parameters must be finite. Native
filtering rejects any nonfinite transformed row, including arithmetic overflow.

## Storage and conversion rules

The default policy borrows native-endian, aligned, C-contiguous `float64` storage
with shape `(N, 3)`. For a nonempty ordinary array, byte strides are `(24, 8)`.
Both read-only and writable inputs use `ConstNDArray[np.float64]`. The existing
typed API validates the scalar buffer. This experiment validates point shape
and the rotation convention. Native outputs use `NDArray[np.float64]`.

| Input condition | Default | `BorrowedPoints(points, copy=True)` |
|---|---|---|
| Compatible `float64`, including read-only | Borrow | Borrow |
| Noncontiguous, negative strides, or Fortran layout | Reject | Copy to C order |
| Unaligned storage | Reject | Copy to aligned storage |
| `float32` | Reject | Convert to native `float64` |
| Non-native-endian `float32` or `float64` | Reject | Convert byte order and dtype |
| Integer, object, complex, or `float16` | Reject | Reject |
| Wrong shape or nonfinite coordinates | Reject | Reject |

`copy=True` permits a necessary copy. It does not force one for compatible
input. Dtype conversion changes representation and can change precision. The
checks use float32-to-float64 conversion, which preserves finite float32 values.

The transform writes into a new NumPy allocation. Filtering compacts rows into
another allocation with capacity for all input rows. `result.points` is a
read-only prefix view of that allocation. It retains the full allocation even
when few rows remain. The downstream energy kernel maps the same prefix pointer.
There is no new copy between filtering and energy. Pointer identity and
`np.shares_memory` are checked separately. Zero-sized arrays have no elements to
share, so pointer equality alone does not prove useful sharing in that case.

## Ownership and resize policy

`BorrowedPoints` holds the ndarray and a Python buffer export. The ndarray keeps
its base owners alive. A retained lease survives deletion of the producer's
ordinary Python reference. Native `Eigen::Map` objects exist only during each
synchronous call. No pointer is retained in C++ after the call.

Do not mutate or resize storage, including through aliases, while a lease is
active. Do not use `resize(refcheck=False)` or external storage reallocation.
Default NumPy resize checks reject reallocation while the lease references the
array. Before each pipeline call, the adapter checks address, shape, strides,
dtype, byte size, alignment, and layout against the lease's initial metadata.
Changing metadata invalidates the lease. Close it and create a new lease after
changing storage. The checks intentionally change only shape metadata and never
dereference a released pointer. Concurrent mutation and external producer
reallocation are outside this synchronous experiment.

Close releases the lease's references. A result remains valid after its input
lease closes. Its prefix view remains valid after the `PipelineResult` is
deleted because NumPy retains the output allocation through the view's base.
Read-only views restrict writes through that view. They do not freeze other
aliases to the same allocation.

## CPU DLPack

Use `dlpack_points(producer)` for a CPU producer implementing `__dlpack__` and
`__dlpack_device__`. It calls `np.from_dlpack(producer, copy=False)` before applying
the same shape and layout checks. Copy-requiring imports raise an error. A
noncontiguous imported view is rejected by the point adapter. To opt into a
copy, import it explicitly and pass it to `BorrowedPoints(..., copy=True)`.

The checked producer is NumPy itself. This tests CPU DLPack protocol sharing and
capsule ownership without a second tensor dependency. It is not evidence for
another framework, a device transfer, GPU execution, or stream synchronization.
NumPy 2.5.1 returned writable imported views in this run. Const input works for
both writable and read-only views. The producer survived garbage collection
until the imported lease closed.

## Missing-logic exercise

Read [AGENT_PROMPT.txt](AGENT_PROMPT.txt) explicitly. Copy [skeleton.py](skeleton.py)
to `candidate.py`, then implement its two C++ bodies. Keep the tests unchanged.
From this directory:

```bash
cp skeleton.py candidate.py
pixi run env BUFFERS_KERNEL_MODULE=candidate pytest -q test_buffers.py
```

The saved solution is [buffers.py](buffers.py) and [native.hpp](native.hpp).
The incomplete skeleton fails the checked transform task. An independent fresh
agent evaluation has not been run. Passing implementation checks is recorded
separately in [RESULTS.md](RESULTS.md).

## Maintainer notes and sources

The C++17 probe builds an executable with the locked compiler and starts a
separate Cling process. Both report `__cplusplus == 201703` and Eigen 5.0.1.
Point maps declare `Eigen::RowMajor` and `Eigen::Unaligned`; NumPy scalar
alignment is still required by the typed API. The point arrays are not mapped
using Eigen's default column-major matrix layout.

The wrappers use `@cpp(cached=False)` after `cppyy.include("native.hpp")`.
Declarations are available in Cling. A separate cached `@cpp` translation unit
does not inherit those declarations. No core helper was duplicated or changed.
The wrappers compile on each process start; warmed measurements exclude this.
The core package may build its environment-local Cling PCH on first import.
It installed its existing auto-PCH hook in this experiment's local environment.
No agent skill or settings were installed.

Primary references:

- [Eigen Map](https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialMapClass.html)
  describes const maps, raw pointers, row-major layout, alignment, and strides.
- [NumPy from_dlpack](https://numpy.org/doc/stable/reference/generated/numpy.from_dlpack.html)
  defines CPU imports and `copy=False` failure behavior.
- [NumPy resize](https://numpy.org/doc/stable/reference/generated/numpy.ndarray.resize.html)
  explains reference checks and the caller's responsibility with `refcheck=False`.
- [DLPack Python specification](https://dmlc.github.io/dlpack/latest/python_spec.html)
  specifies exporter, capsule, and consumer ownership.
- [Typed array API](../../../cppyy_kit/_cpp.py) documents borrowing, dtype,
  byte order, alignment, writability, and the pointer's call scope.

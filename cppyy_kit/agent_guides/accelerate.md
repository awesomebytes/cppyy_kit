# Accelerate a measured Python operation

Use this guide after the Python task and its expected behavior are defined.
In a Pixi environment containing cppyy-kit, ask the agent to run:

```sh
python -m cppyy_kit guide accelerate
```

The command prints guidance for the agent to follow. Use the same environment
for profiling, native compilation, and correctness checks.

## Work sequence

1. Run the existing correctness checks and measure the complete operation.
   Identify the measured computation, input sizes, and required numerical tolerance.
2. Check whether NumPy or an existing library already performs the operation well.
   Compare with that useful baseline. Preserve the user's algorithm and API.
3. Move a complete loop or batch operation into C++. Keep decoding, validation,
   configuration, and reporting in Python when they are not the measured cost.
4. Validate shape, dtype, alignment, units, and finite-value rules before the kernel.
   Retain borrowed input storage until the synchronous call returns.
5. Run the original acceptance checks unchanged. Include empty inputs and boundary
   cases. Use an independent reference, rather than checking two copies of the kernel.
6. Report compilation/first-call time, warmed time, conversion costs, and total
   time separately. Do not imply that faster arithmetic accelerates file decoding.

## A small native kernel

Save this as a Python file. The C++ body is the function docstring. The example
computes squared distance in metres squared for Cartesian positions in metres.

```python
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray, NDArray

@cpp(nogil=True)
def squared_distance(points: ConstNDArray[np.float64],
                     output: NDArray[np.float64], count: int) -> None:
    """
    for (int i = 0; i < count; ++i) {
        const double x = points[3*i];
        const double y = points[3*i+1];
        const double z = points[3*i+2];
        output[i] = x*x + y*y + z*z;
    }
    """

points = np.array([[1., 2., 2.], [0., 0., 3.]], dtype=np.float64)
output = np.empty(len(points), dtype=np.float64)
assert points.ndim == 2 and points.shape[1] == 3
squared_distance(points, output, len(points))
np.testing.assert_allclose(output, [9., 9.])
```

Typed ndarray parameters borrow exact-dtype, native-endian, aligned, C-contiguous NumPy
storage. A mutable NDArray requires writable storage. ConstNDArray accepts
read-only storage. Shape relationships and output capacity remain the caller's
responsibility. Reject unsupported layouts or make a deliberate, measured copy.
For a deliberate normalization, use `np.require(array, dtype=np.float64,
requirements=["C", "A"])` and retain the returned owner through the call.
`int` maps to C++ `int`; verify that counts fit its range. Typed `@cpp` accepts
empty arrays, but the kernel must handle a zero count without dereferencing them.
Raw cppyy pointer calls may reject empty NumPy buffers before entering C++.
Handle empty input explicitly in such wrappers, and use exact native dtype
matches, for example `np.int64` for `std::int64_t`.

`nogil=True` releases the GIL around the compiled body. Native code must not touch
Python objects without acquiring it. Do not change or resize shared storage during
the call. Python callbacks reacquire the GIL and add boundary costs.

## Compilation and repeated runs

`@cpp` uses Cling for runtime compilation. The optional shared-library cache
needs a C++ compiler and compatible development headers. Direct compilation
selects `$CXX`, or `c++` when unset. `$CXX` supports quoted arguments and
compiler launchers such as `ccache c++`; execution does not use a shell.
Keep compiler and runtime dependencies in the active environment.
Read `python -m cppyy_kit status --environment` for dependency diagnostics.

For explicitly cached glue, provide both the implementation and bodiless
declarations. `prebuild(code, decls=decls, **options)` prepares the artifact;
`cppdef_cached(code, decls=decls, **options)` loads it. Use identical code,
declarations, include/link directories, libraries, and compiler options.
On Linux, cold compilation of the same artifact is serialized across processes
and threads. Complete warm artifacts can load without a compiler or writable
cache. This does not make arbitrary native calls thread-safe. Cache keys include
explicit build inputs and the cppyy version tag; they do not inspect compiler
versions or recursively hash headers. After changing a toolchain or replacing
headers/libraries in place, use `clear_cache()` and start a fresh process.

For uncertain template declarations, run a short probe in a separate process.
If Cling cannot parse the library headers, consider a compiled adapter with a
small public header rather than repeating a process-crashing declaration.

Agent prompt: "Profile this task, read the accelerate guide, and use cppyy_kit
for the measured operation. Preserve the API and checks. Report first-call,
warmed, conversion, and complete-operation timings."

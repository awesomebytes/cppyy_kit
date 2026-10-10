# Writing an inline C++ kernel with cppyy_kit

Read this guide only when explicitly asked to use cppyy_kit. This is a local
rehearsal guide, not an automatically installed agent skill.

## Environment

Run the exact Pixi environment or Python interpreter provided with the task.
In roscon_uk_2026, `pixi run python your_script.py` uses the current repository
checkout through explicit Pixi `PYTHONPATH` activation. The 3 October 2026
evaluation used published 0.3 packages and is preserved separately.
Preserve Pixi activation, including its CXX compiler setting. Calling an
interpreter by absolute path alone does not reproduce that activation.
Do not change the environment to solve a numerical task.

## Supported pattern

```python
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray, NDArray

@cpp(nogil=True)
def sum_sq(data: ConstNDArray[np.float64]) -> float:
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

answer = sum_sq(np.array([1, 2, 3], dtype=np.float64))
```

The docstring contains the C++ function body. The Python function body is not
executed. NumPy annotations select checked, borrowed buffers.
`from __future__ import annotations` is supported; names must be available in
the function's module globals. Literal C++ type strings remain available for
advanced declarations.

- Python int/float/bool annotations map to C++ int/double/bool.
- `ConstNDArray[np.float64]` receives a const pointer and an element count, named
  `data` and `data_size` in the example. A multidimensional input must be
  flattened explicitly; its size is the element count, not the row count.
- `out: NDArray[np.int32]` passes an int32 NumPy buffer.
  `out: NDArray[np.float64]` passes float64.
  Allocate the output in Python and return it after calling a void kernel.
- Checked annotations require the exact dtype, native byte order, alignment,
  and C-contiguous layout. Mutable `NDArray` also requires writable storage;
  `ConstNDArray` accepts read-only inputs. They do not copy an existing ndarray.
  Validate shapes and mathematical preconditions in the Python wrapper.
  Use np.ascontiguousarray when accepting other dtypes or strided inputs, and
  account for any conversion cost in end-to-end measurements.
  Keep the owning arrays alive for the entire call.
- Return None maps to void. Defaults and keyword arguments work.
- `nogil=True` releases the GIL during the native body. Do not access Python
  objects there. This does not make a shared output buffer thread-safe.
- Caching is enabled by default. The first call can compile an artifact.
  Warm the callable before measuring processing time and report its first call
  separately. Do not add cached=False just to avoid understanding cache costs.
- Keep the existing function's signature, validation, units, and behavior.
  Test edge cases and compare with the original output before timing.
- Avoid one Python-to-C++ call per sample. Pass the whole array once.
- If NumPy already handles the work efficiently, compare honestly.

For C++ library integration, the decorator accepts include_paths, library_paths,
and libraries. Load the required native library, include its headers, and probe
uncertain declarations with cppyy_kit.probe_cppdef before using them in a
long-running process. A header-only package may use cppyy_kit.require, which
prefers installed headers and requires a SHA-256 for source downloads.

Sources: [implementation](../../cppyy_kit/_cpp.py),
[patterns](../../docs/COMMON_PATTERNS.md#26-cpp-write-a-c-kernel-in-python-compiled-cached-auto-marshaled),
[cache behavior](../../docs/FREEZE.md).

Current task guides are also readable without changing agent configuration:
`pixi run python -m cppyy_kit guide accelerate` and
`pixi run python -m cppyy_kit guide bring-library`.

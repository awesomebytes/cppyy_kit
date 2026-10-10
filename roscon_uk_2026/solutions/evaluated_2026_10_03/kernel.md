# Writing an inline C++ kernel with cppyy_kit

Read this guide only when explicitly asked to use cppyy_kit. This is a local
rehearsal guide, not an automatically installed agent skill.

## Environment

Run the exact Pixi environment or Python interpreter provided with the task.
In roscon_uk_2026, `pixi run python your_script.py` uses published packages.
Preserve Pixi activation, including its CXX compiler setting. Calling an
interpreter by absolute path alone does not reproduce that activation.
Do not change the environment to solve a numerical task.

## Supported pattern

```python
import numpy as np
from cppyy_kit import cpp

@cpp(nogil=True)
def sum_sq(data: cpp.arr("double")) -> float:
    "double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i]*data[i]; return s;"

answer = sum_sq(np.array([1, 2, 3], dtype=np.float64))
```

The docstring contains the C++ function body. The Python function body is not
executed. Use double-quoted strings for C++ pointer annotations. Do not add
`from __future__ import annotations`: this decorator expects actual Python
scalar annotations and cpp.arr markers.

- Python int/float/bool annotations map to C++ int/double/bool.
- `cpp.arr("double")` receives a pointer and an element count, named
  `data` and `data_size` in the example. A multidimensional input must be
  flattened explicitly; its size is the element count, not the row count.
- `out: "int*"` passes an int32 NumPy buffer. `out: "double*"` passes float64.
  Allocate the output in Python and return it after calling a void kernel.
- Raw-pointer paths do not validate dtype, length, contiguity, or writability.
  Check these in the Python wrapper. Use np.ascontiguousarray when needed.
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

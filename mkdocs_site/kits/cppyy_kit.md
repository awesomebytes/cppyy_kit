# cppyy_kit: inline C++ and shared helpers

Use `cppyy_kit` to write C++ functions in Python or to prepare a C++ library for
Python calls. The library kits use these same helpers for loading libraries,
passing callbacks and arrays, and keeping objects alive.

## Write a C++ function

The numeric annotations below are new in `cppyy-kit` 0.4.0. Published 0.3.x
packages do not support them. Before 0.4.0 is published, use a clone of this
repository: save the example below as `kernel.py` in the checkout root, then
run `pixi run python kernel.py` from that root. For standalone use after
publication, follow [Getting Started](../getting-started.md) and install
`cppyy-kit>=0.4.0` with `pixi add`.

```python
import numpy as np
from numpy.typing import NDArray
from cppyy_kit import cpp

@cpp
def sum_sq(data: NDArray[np.float32]) -> float:
    """
    double s = 0;
    for (std::size_t i = 0; i < data_size; ++i) {
        s += data[i] * data[i];
    }
    return s;
    """

print(sum_sq(np.array([1, 2, 3], dtype=np.float32)))  # 14.0
```

Run `pixi run python kernel.py`. It prints `14.0`. The annotation supplies a typed
array pointer and element count; the docstring is compiled as C++.

The compiled function is cached between runs. Use `@cpp(nogil=True)` when
independent Python threads should run while the C++ function is working.

## Use C++ libraries and callbacks

| Task | Helpers and reference |
|---|---|
| Load C++ libraries and prepare include paths | `load_libraries`; [library setup](../docs/COMMON_PATTERNS.md) |
| Pass a Python function to C++ | `callback`; [callbacks and lifetime](../docs/COMMON_PATTERNS.md) |
| Keep a callback or buffer alive while C++ uses it | `keep_alive` |
| Compile reusable C++ helpers | `cppdef_cached`; [compile cache](../docs/FREEZE.md) |
| Make header-only libraries available | `require` |
| Release C++ resources in a defined order | `register_teardown`, `shutdown` |

A kit's usage page shows how it applies these helpers to its C++ library. See the
[library list](../index.md#choose-a-library) to choose a kit.

## Reduce repeated startup work

`@cpp` uses the compile cache by default. Library kits can also register headers
for automatic precompiled-header (PCH) caching. The caches save different work:
compiled C++ functions and parsed library headers.

See [Compile cache and cached headers](../docs/FREEZE.md) for setup, status
commands, and cache controls.

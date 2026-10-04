# Use Python around an existing C++ implementation

Use this guide to test, configure, tune, or report on a compiled C++ component.
Read it in a Pixi environment containing cppyy-kit:

```sh
python -m cppyy_kit guide existing-cpp
```

The component needs compatible public headers, shared libraries, and dependencies.
cppyy cannot recover an arbitrary application API from a binary alone.

## Load the component

With paths resolved from your installation or build:

```python
import cppyy

cppyy.add_include_path(str(include_directory))
cppyy.load_library(str(shared_library))
cppyy.include("my_library/estimator.hpp")
Estimator = cppyy.gbl.my_library.Estimator
```

Replace these names with the library's actual declarations. Loading a library
does not compile it. Rebuilding experiment scripts is unnecessary when only
Python orchestration changes. Changes to the compiled implementation still
require its build process.

## Define the experiment boundary

- Validate configuration in Python before native construction. Pydantic can
  provide defaults, constraints, cross-field checks, and resolved export.
  Map validated fields explicitly into the existing C++ configuration type.
  `cppyy_kit.pydantic_structs` generates new structs for a supported model subset;
  it does not automatically adapt arbitrary existing C++ configuration types.
- Run complete native replays for tuning. Reset state for every trial. Separate
  training and held-out inputs. Save parameters, seeds, metrics, input hashes,
  and dependency versions. Reproduce exported settings with the C++ driver.
- Use Hypothesis for edge cases and operation sequences. Define independent
  invariants and tolerances first. Save reduced failures as deterministic tests.
  A native crash needs a subprocess; it is not a shrinkable Python assertion.
- Implement supported virtual interfaces or callbacks in Python for experiments.
  Match exact signatures and required virtual methods. Pin objects until the
  native engine no longer references them. Measure callback frequency and define
  exception propagation and shutdown order. Python callbacks acquire the GIL.
- Join native outputs with metadata and images using an explicit clock and
  matching tolerance. Report unmatched records. Distinguish a diagnostic score
  from error measured against independent ground truth.

Keep borrowed buffers alive and unchanged throughout native processing. A
returned view needs an owner that outlives it. Prefer an owning output when
retained-view semantics are unnecessary. Do not assume shared state is thread-safe.
For raw pointer inputs, check native dtype, byte order, alignment, contiguity,
shape, and allocation capacity. Handle empty buffers explicitly, and match
fixed-width C++ types with their exact NumPy dtype. Typed `@cpp` validates
dtype and layout, but shape relationships remain the caller's responsibility.

Agent prompt: "Read the existing-cpp guide. Use this compiled implementation
from Python to validate settings, test edge cases, and evaluate an experiment.
Preserve its native API and make the saved results reproducible in its C++ driver."

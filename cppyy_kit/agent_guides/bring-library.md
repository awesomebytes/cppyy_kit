# Bring a C++ library into Python

Use this guide when a task needs a library that has no existing kit. In a Pixi
environment containing cppyy-kit, read it with:

```sh
python -m cppyy_kit guide bring-library
```

Install the library's development package through Pixi when available. Keep the
manifest and lock. Headers and compatible binary dependencies are needed; a
Python binding package alone may not supply them. Check the library's existing
Python bindings before adding an adapter.

## Load an installed header-only library

For example, an environment containing Eigen headers can run:

```python
import cppyy
import os
from cppyy_kit import require

library = require("eigen", "Eigen/Core", search_paths=[
    os.path.join(os.environ["CONDA_PREFIX"], "include", "eigen3")])
cppyy.include("Eigen/Core")
print(library["source"])
```

The expected result is `conda` when installed headers are located. `require`
registers their include directory. It does not compile a library or discover
arbitrary transitive dependencies. The Eigen include layout may vary by package.

If the header is unavailable, `require` can fetch a single header or archive
when supplied `url=` and `sha256=`. Pin the source revision and verify the
checksum. Its default download cache is under `build/cppyy_kit_require`.
Use trusted sources; a checksum pins the selected source bytes. It does not
establish that the native implementation is safe for arbitrary inputs.
Fetched include paths are `<cache>/<name>/<request hash>/include`. The request
hash includes the URL, checksum, header, and strip prefix. Complete entries and
their extracted file hashes are verified before offline reuse. Different pins
use separate directories. Failed fetches leave earlier good entries intact;
legacy unverified entries are fetched again. Cold fetches are serialized between
Linux processes and Python threads. Archive extraction rejects absolute paths,
parent traversal, links, special files, and duplicate files.

For a compiled library, add its include paths with `cppyy.add_include_path` and
load the actual shared library with `cppyy.load_library`. Use a path resolved
from the active environment or your documented build output. Check dependent
libraries and ABI compatibility. An include path does not load binary symbols.

## Implement one useful operation

Expose a batch operation or a persistent native component. Keep indexed storage,
callbacks, and returned views alive for their complete native use. Declare whether
inputs are copied, borrowed synchronously, or retained. Document whether storage
can change after constructing an index or native view.
For NumPy pointer inputs, validate shape and capacity, native dtype and byte
order, alignment, and contiguity. Use `np.require(array, dtype=np.float64,
requirements=["C", "A"])` when an explicit normalization is appropriate.
Match fixed-width declarations exactly, such as `std::int64_t` with `np.int64`.
Handle zero-length inputs before raw cppyy pointer conversion; empty buffers
may be rejected by the binding. Typed `@cpp` supports empty arrays when the
native kernel itself handles zero counts.

Check `python -m cppyy_kit status --environment` before compiling an adapter.
Direct compilation selects `$CXX`, or `c++` when unset. `$CXX` supports quoted
arguments and compiler launchers, and execution does not use a shell.
Keep development headers, binary libraries, and compiler/runtime
dependencies compatible in the active environment.

Probe headers and template instantiations in a subprocess first. Record compiler,
runtime, library version, and failure output. A small separately compiled adapter
is an option when direct template parsing is incompatible. Describe the adapter
as such; do not claim the library worked through direct JIT.

Compare results with an independent reference and any existing Python binding.
Measure construction and repeated use separately. A new kit is warranted only
after the operation, ownership rules, and agent workflow are useful and tested.

Agent prompt: "Use this C++ library through cppyy_kit for the specified batch
operation. Read the bring-library guide, retain ownership correctly, compare
against the reference, and document reproducible setup and timings."

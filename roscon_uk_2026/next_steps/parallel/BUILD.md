# Native adapter and measurement reference

The manifest and lock belong to this directory. Direct dependencies are pinned.
GCC/G++ 14.3.0 and libgcc/libstdcxx 15.2.0 match the compatibility pins in the
repository's [package recipe](../../../recipe/cppyy-kit/recipe.yaml). The
environment disables the package's optional auto-PCH setup. No shared manifest
or environment is modified by this experiment.

`build.py` calls the existing Pydantic prototype's `emit_cpp(Detection)` and
writes a struct header with an alias named `DetectionType`. It compiles
`native.cpp` into `build/libdetection.so`. All kernels use the same C++17, `-O3`,
`-march=native`, and `-ffp-contract=off` flags. The library is specific to the
local CPU. The source, command, and generated schema determine the build cache
key. GCC writes its actual vectorization decisions to `build/vectorization.txt`.

`processing.py` uses `cpp_vector_columnar` to fill an owning native vector from
validated numeric columns. It uses the existing general `cpp_vector` path for
empty input because the prototype's raw strided column fill cannot represent a
zero-length array at a nonzero member offset. The check compares general and
columnar conversion on real records.

The native adapter presents small C declarations. `ctypes` resolves its function
addresses once and retains the library handle. The existing `@cpp(nogil=True)`
wrapper calls those typed native function pointers. The wrapper retains the
arguments for the call, and `Batch` owns all input buffers. There are no Python
callbacks and no user-supplied pointer arguments in the public operation.

The pointer bridge avoids placing a header with an `extern "C"` linkage block
inside an `@cpp` function body. The first attempted include recipe failed this
way; [evidence/FAILURES.md](evidence/FAILURES.md) preserves the diagnostic and
correction. The saved wrapper source is compiled with the helper's ordinary
cache settings. Its bridge compilation is measured separately from kernel
compilation in [evidence/startup.json](evidence/startup.json).

`probe.py` parses and instantiates minimal oneTBB and xsimd templates in separate
processes. Both probes succeeded here. The precompiled adapter remains useful
for comparing actual native optimization without template parsing in each
Python process. If a template probe fails on another toolchain, retain its exit
code and diagnostic before trying this adapter. A precompiled adapter does not
repair an incompatible cppyy startup or STL ABI. Use compatible toolchain pins
and rerun the subprocess checks before claiming feasibility.

`benchmark.py` generates raw Python dictionaries with seed 712 outside the
measured input-to-result operation. It validates and prepares the shared batch
before three warmups and 21 calls per candidate. Every call is checked. Candidate
order is fixed, so load and thermal effects can bias comparison. It also measures
one fresh input-to-result operation per candidate with the required layout.
These one-shot totals are single observations, not distributions.

Preparation component timers do not include all Python model cleanup, wrapper
construction, or build-cache checks. `one_shot_prepare_ms` and the end-to-end
timer include those costs. The one-shot runtime has its own timer. No input
validation or conversion cost is inferred from a speed ratio.

Counts use a serial histogram in every method. A histogram reduction could
improve parallel scaling, but would add thread-local storage and a merge. It has
not been implemented or measured. The measurements do not support a generic
parallel helper or a combined TBB/SIMD path.

Set `DETECTION_SOURCE=skeleton/native.cpp` to build the exercise source against
the unchanged adapter and tests in a new Python process. This overwrites only
the ignored native artifact. The next ordinary command rebuilds the saved
solution because its source digest differs. Do not rebuild the library while a
process using it is still running.

# Read task and library guides

Choose the instructions for the task you are doing:

```bash
pixi run python -m cppyy_kit guide
pixi run python -m cppyy_kit guide accelerate
pixi run python -m cppyy_kit guide bring-library
pixi run python -m cppyy_kit guide existing-cpp
```

`accelerate` covers moving a measured Python operation to C++. `bring-library`
covers loading headers and shared libraries. `existing-cpp` covers configuration,
tests, tuning and reports around an existing compiled implementation.

For an installed kit, read its overview and API reference:

```bash
pixi run python -m cppyy_kit guide ompl_kit
pixi run python -m cppyy_kit guide ompl_kit api
```

The command lists kit import names and their Pixi package names. A missing kit
produces an installation message. Printing a guide does not import its C++
library. The packaged references come from the same documents as this website.

These commands are included in published `cppyy-kit` 0.4.1 or later. Create a
Pixi project with the `awesomebytes` and `conda-forge` channels, then run
`pixi add "cppyy-kit>=0.4.1"`. Kit guides require their own installed packages;
see [Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/).

## Check an environment before compiling

```bash
pixi run python -m cppyy_kit status --environment
pixi run python -m cppyy_kit status --environment --json
```

The report locates the selected compiler, installed runtime versions, Python
and CPyCppyy development headers, and `libcppyy`. It can help explain a missing
dependency or a compiler selected outside the active environment. It does not
load Cling or test binary compatibility.

Direct compilation uses `$CXX`, or `c++` when unset. Keep headers, linked
libraries and the compiler/runtime in a compatible Pixi environment. See
[compile caching](FREEZE.md) for building adapters and clearing stale artifacts.

## Try a complete workflow

- [Configure and test a native component](tutorials/native_component.md).
- [Implement an OMPL validity checker in Python](tutorials/ompl_callbacks.md).
- [Use callbacks, arrays and C++ objects](COMMON_PATTERNS.md).

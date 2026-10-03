# wbc_kit rationale

## The problem

Crocoddyl provides optimal control through DDP. pinocchio provides rigid-body
computations, and tsid provides task-space inverse dynamics. All three have Python
bindings on conda-forge. A cppyy kit is useful only when it adds a capability that
these bindings do not provide.

Crocoddyl supports custom action models in Python and C++. A Python model is called
many times by the DDP solver, through `calc` and `calcDiff`, so those calls add
interpreter and NumPy overhead. A C++ model avoids that overhead, but normally needs
a CMake project linked to `libcrocoddyl` and a rebuild after changes.

## What cppyy adds

With cppyy, users can write a C++ action model in a `cppyy.cppdef` string in a Python
script. cppyy compiles it at runtime. The DDP solver calls the C++ methods directly,
without Python calls in the solve loop or a separate build system.

On Crocoddyl's unicycle problem (see REPORT.md), the inline C++ model ran in 0.32 ms.
The compiled built-in model ran in 0.34 ms. The Python-derived model ran in 6.84 ms.
All three reached the same cost, 250.039320. The benchmark used the best of seven
runs after warm-up on a shared machine, so the timings are provisional.

This use case follows the same approach as the OMPL validity checker and the
ros2_control controller examples: write a prototype, then move a frequently called
virtual method to C++.

## Run this example

From this checkout, run `pixi run -e wbc demo-wbc-lower`. It compares Python-derived,
inline-C++, and built-in C++ Crocoddyl action models on the same unicycle solve; the
recorded result reaches cost `250.039320` in 8 iterations for all three. For
installation or source-development instructions, see
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/). The
published package is [`wbc-kit`](https://repo.prefix.dev/awesomebytes).

## Scope

- Use Crocoddyl's binding to prototype a model. Use wbc_kit to compile its C++ version.
  Both can load `libcrocoddyl.so` in one process, but their proxy objects cannot be
  passed between runtimes.
- This kit does not instantiate pinocchio models with new scalar types. That attempt
  is blocked by the Boost variant arity limit in this environment. pinocchio already
  provides a binding for the CasADi scalar. See REPORT.md, section 4.
- Use a separate environment from ROS. The conda-forge whole-body-control packages
  and the robostack ROS packages require incompatible Boost versions.

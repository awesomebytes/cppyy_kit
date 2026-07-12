# wbc_kit — why

## The problem

Whole-body / legged-robot control leans on **Crocoddyl** (optimal control via DDP),
**pinocchio** (rigid-body dynamics), and **tsid** (task-space inverse dynamics). All
three ship on conda-forge **with** good Python bindings — so, unlike OMPL (no easy
bindings) or PCL (templated bulk data), the bar for a cppyy kit is high: it has to do
something the bindings genuinely can't.

For Crocoddyl there is exactly such a thing, and it is central to the library's own
workflow. Crocoddyl tells you to **prototype your custom dynamics/cost ("action")
model in Python, then rewrite the hot model in C++ for production.** The Python
prototype is easy but slow — the DDP solver calls the model's `calc`/`calcDiff`
thousands of times per solve, each crossing the Python boundary. The C++ rewrite is
fast but heavy: a CMake project that links `libcrocoddyl` and rebuilds every time you
tweak a cost weight.

## What cppyy adds

cppyy collapses that rewrite. You write the C++ action model in a `cppyy.cppdef`
string **in the same Python script**, it is JIT-compiled at runtime, and the DDP
solver calls its `calc`/`calcDiff` **natively** — no Python in the hot loop, no build
system. You get the fast path with the prototype's convenience.

Measured on Crocoddyl's canonical unicycle problem (see REPORT.md): the inline-C++
model solves at the **exact speed of Crocoddyl's compiled built-in model** and
**~21x faster** than the Python-derived model — converging to a **bit-identical**
cost. That last point is the contract: the lowered model is not an approximation, it
is the same math at C++ speed.

This is the same "prototype in Python, lower the hot virtual to C++ in one script"
pattern ompl_kit proved for OMPL validity checkers and control_kit for ros2_control
controllers — here applied to a new domain, trajectory optimization, where the hot
virtual genuinely dominates the solve.

## What it is not

- **Not a re-wrap of the bindings.** Use Crocoddyl's own binding to prototype; wbc_kit
  is for the lowering step. Both live in one script (they share `libcrocoddyl.so`).
- **Not a pinocchio-scalar kit.** pinocchio's templated-scalar surface (the other
  candidate cppyy angle) is env-blocked here by a boost `variant` arity wall, and its
  main autodiff scalar (casadi) is already a shipped binding. REPORT.md S4.
- **Not for mixing with ROS in one env.** conda-forge WBC libs and the robostack ROS
  stack pin incompatible boost; the `wbc` env is standalone.

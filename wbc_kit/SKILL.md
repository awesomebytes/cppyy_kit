# wbc_kit — SKILL (LLM-facing cheat sheet)

Drive **Crocoddyl** (optimal control / DDP) from Python via cppyy. Headline
capability: **author a custom action model in inline C++, JIT'd at runtime, no build
system** — the DDP solver calls it at native speed (~21x a Python-derived model,
bit-identical result). Env: `pixi run -e wbc ...` (standalone, conda-forge).

## When to use
- You have a **custom Crocoddyl action / cost model** and the Python-subclass version
  is too slow in the DDP hot loop, but you don't want a CMake project to write it in C++.
- You want to **prototype in Python then lower to C++ in the same script** (the ROSCon arc).
- Not for pinocchio templated scalars (env-blocked, casadi already bound) or tsid (works
  via cross-inheritance but same pattern as ompl/control — see REPORT).

## Bringup
```python
import wbc_kit
cr = wbc_kit.bringup_crocoddyl()      # returns cppyy.gbl.crocoddyl; use its API verbatim
m  = cr.ActionModelUnicycle()          # built-in models, ShootingProblem, SolverFDDP, ...
```
Hides: pinocchio 4.0's soname split (`libpinocchio_default.so`, there is no
`libpinocchio.so`), the `pinocchio/fwd.hpp`-first include order, and the `.so` loads.
Idempotent. `with_solvers=False` skips the shooting/solver JIT.

## Author a custom C++ action model (the win)
```python
wbc_kit.bringup_crocoddyl()
wbc_kit.safe_cppdef(r'''
namespace mywbc {
  using crocoddyl::ActionModelAbstract; using crocoddyl::ActionDataAbstract;
  struct Model : ActionModelAbstract {
    Model() : ActionModelAbstract(std::make_shared<crocoddyl::StateVector>(3), 2, 5) {}
    void calc(const std::shared_ptr<ActionDataAbstract>& d,
              const Eigen::Ref<const Eigen::VectorXd>& x,
              const Eigen::Ref<const Eigen::VectorXd>& u) override { /* xnext,r,cost */ }
    void calc(const std::shared_ptr<ActionDataAbstract>& d,
              const Eigen::Ref<const Eigen::VectorXd>& x) override { /* terminal */ }
    void calcDiff(const std::shared_ptr<ActionDataAbstract>& d,
                  const Eigen::Ref<const Eigen::VectorXd>& x,
                  const Eigen::Ref<const Eigen::VectorXd>& u) override { /* Fx,Fu,Lx.. */ }
    void calcDiff(const std::shared_ptr<ActionDataAbstract>& d,
                  const Eigen::Ref<const Eigen::VectorXd>& x) override { /* terminal */ }
''' + wbc_kit.ACTION_MODEL_CLONES.format(cls="Model") + r'''
  };
  std::shared_ptr<ActionModelAbstract> make(){ return std::make_shared<Model>(); }
}''')
import cppyy
model = cppyy.gbl.mywbc.make()         # native C++ model; hand to a C++-built solve
```
Full worked model + FDDP driver: `wbc_kit/wbc_kit/cpp/unicycle_model.cpp`.
Demo + benchmark: `pixi run -e wbc demo-wbc-lower`.

## Gotchas (each cost a real dead-end)
- **`safe_cppdef`, not raw `cppyy.cppdef`.** A wrong override signature or a missing
  clone makes the class abstract -> a failed `cppdef` that **crashes Cling** with no
  traceback (Pattern 9). `safe_cppdef` probes out-of-process and raises `CppyyKitError`.
- **`ACTION_MODEL_CLONES` is mandatory.** Crocoddyl 3.2's `CROCODDYL_BASE_CAST` adds
  pure-virtual `cloneAsDouble`/`cloneAsFloat`; omit them and your model won't instantiate.
- **Match `calc`/`calcDiff` signatures exactly** — `const Eigen::Ref<const VectorXs>&`
  (== `const Eigen::Ref<const Eigen::VectorXd>&` for the double scalar).
- **Build the solve in C++** (the running-models `std::vector`, `ShootingProblem`,
  `SolverFDDP`), Pattern 6. A cppyy-created C++ model **cannot** be passed to Crocoddyl's
  boost::python `ShootingProblem` (two proxy runtimes). Prototype with the binding, lower
  with cppyy — both in one script, but don't pass objects across the two.
- **Standalone env.** Don't expect the ROS-touching kits in the same environment
  (boost 1.86 vs 1.90 pin clash).

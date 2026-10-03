# wbc_kit quick reference

Use Crocoddyl's DDP solver from Python with cppyy. To put a custom action model in
the DDP loop without Python calls or a separate CMake build, write it in inline C++.
In the unicycle benchmark, the C++ model ran about 21 times faster than the
Python-derived model and reached the same cost. Use the standalone environment with
`pixi run -e wbc ...`.

## Use this kit when

- A custom Crocoddyl action or cost model written in Python is too slow in the DDP loop.
- You want to prototype a model in Python and compile its C++ implementation in the
  same script.
- Do not use it to instantiate pinocchio models with new scalar types. This is blocked
  in the current environment. Crocoddyl and tsid cross-inheritance are described in
  the report.

## Bringup

```python
import wbc_kit
cr = wbc_kit.bringup_crocoddyl()      # returns cppyy.gbl.crocoddyl; use its API verbatim
m  = cr.ActionModelUnicycle()          # built-in models, ShootingProblem, SolverFDDP, ...
```

The helper loads pinocchio's `libpinocchio_default.so`, includes `pinocchio/fwd.hpp`
before Crocoddyl headers, and loads the required shared libraries. It is idempotent.
Set `with_solvers=False` to skip JIT-compiling the shooting and solver headers.

## Define a custom C++ action model

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

The complete model and FDDP driver are in `wbc_kit/wbc_kit/cpp/unicycle_model.cpp`.
Run the demo and benchmark with `pixi run -e wbc demo-wbc-lower`.

## Notes

- Use `safe_cppdef` instead of calling `cppyy.cppdef` directly. An incorrect override
  or missing clone method can make Cling crash without a Python traceback. The helper
  probes the code in a subprocess and raises `CppyyKitError` on failure.
- `ACTION_MODEL_CLONES` is required. Crocoddyl 3.2's `CROCODDYL_BASE_CAST` macro adds
  the pure virtual methods `cloneAsDouble` and `cloneAsFloat`.
- The `calc` and `calcDiff` signatures must match the base class. For the double scalar,
  use `const Eigen::Ref<const Eigen::VectorXd>&`, equivalent to
  `const Eigen::Ref<const VectorXs>&`.
- Build the running-model vector, `ShootingProblem`, and `SolverFDDP` in C++. A C++
  model created by cppyy cannot be passed to Crocoddyl's boost::python
  `ShootingProblem`. You can use both bindings in one script, but keep their objects
  separate.
- The `wbc` environment is standalone. It cannot share a process with the ROS packages
  because they require different Boost versions.

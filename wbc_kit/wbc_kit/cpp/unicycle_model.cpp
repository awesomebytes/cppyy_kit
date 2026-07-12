// A custom Crocoddyl action model authored in C++ and JIT-compiled by cppyy at
// runtime -- NO build system, NO CMake project. This is the "lowered" endpoint of
// the wbc_kit story: the DDP solver calls calc/calcDiff natively (no Python in the
// hot loop). It reproduces Crocoddyl's canonical unicycle model so the result can
// be verified bit-for-bit against the compiled built-in crocoddyl::ActionModelUnicycle.
//
// Deliberately mirrors crocoddyl/core/actions/unicycle.hxx. The two clone overrides
// at the end are mandatory in Crocoddyl 3.2 (CROCODDYL_BASE_CAST adds them as pure
// virtuals) -- see wbc_kit.ACTION_MODEL_CLONES.
namespace wbc_demo {
using crocoddyl::ActionModelAbstract;
using crocoddyl::ActionDataAbstract;
using crocoddyl::StateVector;
typedef Eigen::VectorXd VectorXd;

struct Unicycle : public ActionModelAbstract {
  double dt_ = 0.1, wx_ = 10.0, wu_ = 1.0;                 // costWeights = [10, 1]
  Unicycle() : ActionModelAbstract(std::make_shared<StateVector>(3), 2, 5) {}

  void calc(const std::shared_ptr<ActionDataAbstract>& d,
            const Eigen::Ref<const VectorXd>& x,
            const Eigen::Ref<const VectorXd>& u) override {
    const double c = std::cos(x[2]), s = std::sin(x[2]);
    d->xnext << x[0] + c * u[0] * dt_, x[1] + s * u[0] * dt_, x[2] + u[1] * dt_;
    d->r.head<3>() = wx_ * x;
    d->r.tail<2>() = wu_ * u;
    d->cost = 0.5 * d->r.dot(d->r);
  }
  void calc(const std::shared_ptr<ActionDataAbstract>& d,
            const Eigen::Ref<const VectorXd>& x) override {   // terminal (no control)
    d->xnext = x;
    d->r.head<3>() = wx_ * x;
    d->r.tail<2>().setZero();
    d->cost = 0.5 * d->r.head<3>().dot(d->r.head<3>());
  }
  void calcDiff(const std::shared_ptr<ActionDataAbstract>& d,
                const Eigen::Ref<const VectorXd>& x,
                const Eigen::Ref<const VectorXd>& u) override {
    const double c = std::cos(x[2]), s = std::sin(x[2]);
    const double wx = wx_ * wx_, wu = wu_ * wu_;
    d->Lx = x * wx;
    d->Lu = u * wu;
    d->Lxx.diagonal().setConstant(wx);
    d->Luu.diagonal().setConstant(wu);
    d->Fx.setIdentity();
    d->Fx(0, 2) = -s * u[0] * dt_;
    d->Fx(1, 2) = c * u[0] * dt_;
    d->Fu.setZero();
    d->Fu(0, 0) = c * dt_;
    d->Fu(1, 0) = s * dt_;
    d->Fu(2, 1) = dt_;
  }
  void calcDiff(const std::shared_ptr<ActionDataAbstract>& d,
                const Eigen::Ref<const VectorXd>& x) override {
    const double wx = wx_ * wx_;
    d->Lx = x * wx;
    d->Lxx.diagonal().setConstant(wx);
  }

  // Mandatory in Crocoddyl 3.2 (see wbc_kit.ACTION_MODEL_CLONES).
  std::shared_ptr<crocoddyl::ActionModelBase> cloneAsDouble() const override {
    return std::make_shared<Unicycle>(*this);
  }
  std::shared_ptr<crocoddyl::ActionModelBase> cloneAsFloat() const override {
    return nullptr;
  }
};

std::shared_ptr<ActionModelAbstract> make_unicycle() {
  return std::make_shared<Unicycle>();
}

// Build the shooting problem + run FDDP entirely in C++ (containers built here,
// cppyy Pattern 6). Returns the converged cost + iteration count.
struct SolveResult { double cost; int iters; bool converged; };
SolveResult solve(std::shared_ptr<ActionModelAbstract> model, int T, int maxiter) {
  Eigen::VectorXd x0(3);
  x0 << -1.0, -1.0, 1.0;
  std::vector<std::shared_ptr<ActionModelAbstract>> running(T, model);
  auto problem = std::make_shared<crocoddyl::ShootingProblem>(x0, running, model);
  crocoddyl::SolverFDDP solver(problem);
  std::vector<Eigen::VectorXd> xs(T + 1, x0), us(T, Eigen::VectorXd::Zero(2));
  bool ok = solver.solve(xs, us, maxiter, false, 1e-9);
  return SolveResult{solver.get_cost(), static_cast<int>(solver.get_iter()), ok};
}
}  // namespace wbc_demo

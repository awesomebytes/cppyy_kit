"""
wbc_kit provides Crocoddyl setup helpers and safe C++ model compilation through cppyy.

Crocoddyl bindings support Python subclasses of ``crocoddyl.ActionModelAbstract``.
The DDP solver calls ``calc`` and ``calcDiff`` many times, so a Python model adds
Python and NumPy overhead. A C++ model avoids those calls but usually needs a CMake
project linked to libcrocoddyl. With cppyy, define the C++ subclass in a
``cppyy.cppdef`` string and compile it at runtime. The solve loop then calls the C++
methods directly. The canonical unicycle benchmark is about 21 times faster than the
Python-derived model and reaches the same cost. See ``docs/wbc/REPORT.md``.

The kit provides three helpers:

* ``bringup_crocoddyl()`` loads ``libpinocchio_default.so`` and ``libcrocoddyl.so``
  and includes ``pinocchio/fwd.hpp`` before Crocoddyl headers. It returns the
  ``cppyy.gbl.crocoddyl`` namespace.
* ``safe_cppdef(code)`` checks a custom C++ action model in a subprocess before
  compiling it in the main process. This avoids a Cling crash if compilation fails.
* ``ACTION_MODEL_CLONES`` contains the two clone methods required by Crocoddyl 3.2's
  ``CROCODDYL_BASE_CAST`` macro. A subclass that omits them is abstract.

Example: define a custom action model and solve it::

    import wbc_kit
    cr = wbc_kit.bringup_crocoddyl()                 # Crocoddyl's own namespace

    wbc_kit.safe_cppdef(r'''
    namespace mywbc {
      using crocoddyl::ActionModelAbstract; using crocoddyl::ActionDataAbstract;
      struct MyModel : crocoddyl::ActionModelAbstract {
        MyModel() : ActionModelAbstract(
            std::make_shared<crocoddyl::StateVector>(3), 2, 5) {}
        void calc(const std::shared_ptr<ActionDataAbstract>& d,
                  const Eigen::Ref<const Eigen::VectorXd>& x,
                  const Eigen::Ref<const Eigen::VectorXd>& u) override { /* ... */ }
        // ... the other calc/calcDiff overrides ...
    ''' + wbc_kit.ACTION_MODEL_CLONES.format(cls="MyModel") + r'''
      };
      std::shared_ptr<ActionModelAbstract> make(){ return std::make_shared<MyModel>(); }
    }''')

    model = cr  # then build the ShootingProblem + SolverFDDP in C++ (see demo);
                # the solver calls MyModel::calc natively, no Python in the loop.

Notes and limits:
    * Build the C++ solve containers because a cppyy-created model cannot be passed
      to Crocoddyl's boost::python ``ShootingProblem``. The proxy runtimes are separate.
    * Use the standalone ``wbc`` environment. Crocoddyl and tsid require the Boost
      1.86 line here, while the robostack ROS packages require Boost 1.90.
    * Instantiating pinocchio's ``ModelTpl<Scalar>`` for a new scalar is blocked by
      the 25-type ``boost::variant`` arity limit in Boost 1.90. The existing
      ``pinocchio.casadi`` binding covers the main autodiff scalar. See REPORT.md.
"""
import glob
import os

import cppyy

import cppyy_kit

# pinocchio/fwd.hpp MUST come first (it configures Eigen/boost macros the rest of
# pinocchio + crocoddyl rely on). Then the action base to derive, a Euclidean state,
# and a built-in model for reference. Solvers/problem are gated behind with_solvers.
_CORE_HEADERS = (
    "pinocchio/fwd.hpp",
    "crocoddyl/core/action-base.hpp",
    "crocoddyl/core/states/euclidean.hpp",
    "crocoddyl/core/actions/unicycle.hpp",
)
_SOLVER_HEADERS = (
    "crocoddyl/core/optctrl/shooting.hpp",
    "crocoddyl/core/solvers/fddp.hpp",
    "crocoddyl/core/solvers/ddp.hpp",
)

# The clone overrides Crocoddyl 3.2's CROCODDYL_BASE_CAST makes mandatory on any
# ActionModelAbstract subclass. Format with the derived class name: a double-scalar
# model clones itself as double and declines the float cast (returns nullptr).
ACTION_MODEL_CLONES = (
    "  std::shared_ptr<crocoddyl::ActionModelBase> cloneAsDouble() const override "
    "{{ return std::make_shared<{cls}>(*this); }}\n"
    "  std::shared_ptr<crocoddyl::ActionModelBase> cloneAsFloat() const override "
    "{{ return nullptr; }}"
)

_CROCODDYL = None
_CORE_DONE = False
_SOLVERS_DONE = False


def _prefix():
    conda = os.environ.get("CONDA_PREFIX", "")
    if not conda or not glob.glob(os.path.join(conda, "include", "crocoddyl")):
        raise RuntimeError(
            "Crocoddyl headers not found under $CONDA_PREFIX/include. "
            "Install the wbc environment first: pixi install -e wbc"
        )
    return conda


def include_paths():
    """The include dirs a custom-model ``cppdef`` needs (env include + eigen3).

    Handy for passing to ``cppyy_kit.probe_cppdef`` / ``cppdef_cached`` yourself;
    ``safe_cppdef`` uses them for you.
    """
    conda = _prefix()
    return [os.path.join(conda, "include"), os.path.join(conda, "include", "eigen3")]


def _pinocchio_soname(conda):
    """pinocchio 4.0 has no ``libpinocchio.so``; find ``libpinocchio_default.so``."""
    hits = glob.glob(os.path.join(conda, "lib", "libpinocchio_default.so"))
    if hits:
        return "libpinocchio_default.so"
    # Older single-library pinocchio, just in case.
    if glob.glob(os.path.join(conda, "lib", "libpinocchio.so")):
        return "libpinocchio.so"
    raise RuntimeError("No libpinocchio*.so found under $CONDA_PREFIX/lib")


def _ensure_core():
    global _CROCODDYL, _CORE_DONE
    if _CORE_DONE:
        return
    conda = _prefix()
    for path in include_paths():
        cppyy.add_include_path(path)
    for header in _CORE_HEADERS:
        cppyy.include(header)
    cppyy_kit.load_libraries(
        [_pinocchio_soname(conda), "libcrocoddyl.so"],
        [os.path.join(conda, "lib")],
    )
    _CROCODDYL = cppyy.gbl.crocoddyl
    _CORE_DONE = True


def _ensure_solvers():
    global _SOLVERS_DONE
    if _SOLVERS_DONE:
        return
    for header in _SOLVER_HEADERS:
        cppyy.include(header)
    _SOLVERS_DONE = True


def bringup_crocoddyl(with_solvers=True):
    """
    Bring up Crocoddyl under cppyy and return its ``cppyy.gbl.crocoddyl`` namespace.
    Idempotent.

    Discovers the install (``$CONDA_PREFIX/include`` + eigen3), JIT-includes the core
    headers (``pinocchio/fwd.hpp`` first, the action base, a Euclidean state, the
    built-in unicycle) and -- with ``with_solvers=True`` (default) -- the shooting
    problem + FDDP/DDP solvers, then loads ``libpinocchio_default.so`` and
    ``libcrocoddyl.so`` so calls resolve without ``LD_LIBRARY_PATH``.

    Use Crocoddyl's own API on the returned namespace directly
    (``cr.ActionModelUnicycle()``, ``cr.ShootingProblem(...)``, ``cr.SolverFDDP(...)``).
    Any other header (a differential action model, an integrator, ...) is one more
    ``cppyy.include`` away -- the kit does not pre-parse the whole library.
    """
    _ensure_core()
    if with_solvers:
        _ensure_solvers()
    return _CROCODDYL


def crocoddyl():
    """The ``cppyy.gbl.crocoddyl`` namespace (brings up the core if needed)."""
    _ensure_core()
    return _CROCODDYL


def safe_cppdef(code, extra_include_paths=(), libraries=()):
    """
    Compile ``code`` (a custom C++ action model + factory) into the live interpreter,
    but probe it out-of-process *first* so a compile error is reported cleanly
    instead of crashing Cling.

    Authoring an ``ActionModelAbstract`` subclass is precisely where a ``cppdef``
    fails to parse (a wrong override signature, a forgotten clone -- see
    ``ACTION_MODEL_CLONES``), and a failed ``cppdef`` can SIGSEGV the interpreter
    during transaction revert with no Python traceback (cppyy_kit Pattern 9). This
    runs ``cppyy_kit.probe_cppdef`` in a throwaway subprocess with the Crocoddyl
    headers/paths pre-loaded; on failure it raises ``CppyyKitError`` with the
    compiler message, on success it runs ``cppyy.cppdef``.

    Bring Crocoddyl up first (``bringup_crocoddyl``) so the headers are on the path.
    """
    _ensure_core()
    conda = _prefix()
    inc = include_paths() + list(extra_include_paths)
    ok, message = cppyy_kit.probe_cppdef(
        code,
        include_paths=inc,
        library_paths=[os.path.join(conda, "lib")],
        headers=list(_CORE_HEADERS) + list(_SOLVER_HEADERS),
        libraries=[_pinocchio_soname(conda), "libcrocoddyl.so"] + list(libraries),
    )
    if not ok:
        raise cppyy_kit.CppyyKitError(
            "custom action model failed to compile (probed out-of-process, so the "
            "interpreter is intact):\n" + message)
    cppyy.cppdef(code)

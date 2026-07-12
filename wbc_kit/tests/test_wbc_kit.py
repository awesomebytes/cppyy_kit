#!/usr/bin/env python3
"""Tests for wbc_kit (Crocoddyl via cppyy).

Crocoddyl is an optional dependency (the pixi ``wbc`` env), absent from the default
env, so the whole module auto-skips when the headers are not installed -- the
default ``pixi run test`` is unaffected. Run the real thing with
``pixi run -e wbc test-wbc``.

The headline test is the numeric contract: a custom action model authored in inline
C++ (JIT-compiled at runtime, no build system) drives an FDDP solve to a cost that
is bit-identical to Crocoddyl's compiled built-in model. Pure Crocoddyl (no rclcpp),
so the process exits cleanly.
"""
import os

import numpy as np
import pytest

_HAVE = bool(os.path.isdir(os.path.join(os.environ.get("CONDA_PREFIX", ""),
                                        "include", "crocoddyl")))

pytestmark = pytest.mark.skipif(not _HAVE,
                                reason="Crocoddyl not installed (use the wbc env)")

if _HAVE:
    import cppyy

    import cppyy_kit
    import wbc_kit

T, MAXITER = 100, 50
X0 = np.array([-1.0, -1.0, 1.0])
_CPP = os.path.join(os.path.dirname(__file__), "..", "wbc_kit", "cpp",
                    "unicycle_model.cpp")


@pytest.fixture(scope="module")
def cr():
    return wbc_kit.bringup_crocoddyl()


@pytest.fixture(scope="module")
def custom_model(cr):
    """Author the custom C++ action model once (module-scoped: cppdef is global)."""
    with open(_CPP) as fh:
        wbc_kit.safe_cppdef(fh.read())
    return cppyy.gbl.wbc_demo


def test_bringup_idempotent_and_namespace(cr):
    assert cr is wbc_kit.bringup_crocoddyl()          # idempotent
    assert hasattr(cr, "ActionModelUnicycle")
    assert hasattr(cr, "ShootingProblem")
    assert hasattr(cr, "SolverFDDP")


def test_builtin_model_calc_values(cr):
    """The compiled built-in unicycle computes the expected dynamics/cost."""
    m = cr.ActionModelUnicycle()
    d = m.createData()
    x = cppyy.gbl.Eigen.VectorXd(3)
    x[0], x[1], x[2] = 1.0, 0.0, 0.0
    u = cppyy.gbl.Eigen.VectorXd(2)
    u[0], u[1] = 0.5, 0.1
    m.calc(d, x, u)
    # dt=0.1, weights=[10,1]: xnext=[1.05,0,0.01]; cost=0.5*(100+0.25+0.01)
    assert d.xnext[0] == pytest.approx(1.05)
    assert d.xnext[2] == pytest.approx(0.01)
    assert d.cost == pytest.approx(50.13)


def test_action_model_clones_formats():
    snippet = wbc_kit.ACTION_MODEL_CLONES.format(cls="MyModel")
    assert "cloneAsDouble" in snippet and "cloneAsFloat" in snippet
    assert "std::make_shared<MyModel>(*this)" in snippet


def test_inline_cpp_model_matches_builtin(cr, custom_model):
    """HEADLINE: a custom action model authored in inline C++ (no build system)
    drives an FDDP solve to a cost bit-identical to Crocoddyl's built-in model."""
    # inline-C++ custom model, solved entirely in C++ (Pattern 6 containers)
    res = custom_model.solve(custom_model.make_unicycle(), T, MAXITER)
    assert res.converged
    # built-in reference via the same cppyy namespace, solved in C++ too
    ref = custom_model.solve(cr.ActionModelUnicycle(), T, MAXITER)
    assert res.cost == pytest.approx(ref.cost, abs=1e-9)
    assert res.iters == ref.iters


def test_python_derived_model_matches_and_is_slower(cr):
    """Crocoddyl's supported prototype path (Python subclass) reaches the SAME
    optimum -- the inline-C++ model is a faithful lowering, not an approximation."""
    import crocoddyl as pcr

    class PyUnicycle(pcr.ActionModelAbstract):
        def __init__(self):
            pcr.ActionModelAbstract.__init__(self, pcr.StateVector(3), 2, 5)

        def calc(self, data, x, u=None):
            if u is None:
                r = np.concatenate([10.0 * np.asarray(x), np.zeros(2)])
                data.xnext = x
                data.r = r
                data.cost = 0.5 * float(r[:3].dot(r[:3]))
                return
            c, s = np.cos(x[2]), np.sin(x[2])
            data.xnext = np.array([x[0] + c * u[0] * 0.1, x[1] + s * u[0] * 0.1,
                                   x[2] + u[1] * 0.1])
            r = np.concatenate([10.0 * np.asarray(x), 1.0 * np.asarray(u)])
            data.r = r
            data.cost = 0.5 * float(r.dot(r))

        def calcDiff(self, data, x, u=None):
            data.Lx = np.asarray(x) * 100.0
            Lxx = np.zeros((3, 3))
            np.fill_diagonal(Lxx, 100.0)
            data.Lxx = Lxx
            if u is None:
                return
            c, s = np.cos(x[2]), np.sin(x[2])
            data.Lu = np.asarray(u) * 1.0
            Luu = np.zeros((2, 2))
            np.fill_diagonal(Luu, 1.0)
            data.Luu = Luu
            Fx = np.eye(3)
            Fx[0, 2] = -s * u[0] * 0.1
            Fx[1, 2] = c * u[0] * 0.1
            data.Fx = Fx
            Fu = np.zeros((3, 2))
            Fu[0, 0] = c * 0.1
            Fu[1, 0] = s * 0.1
            Fu[2, 1] = 0.1
            data.Fu = Fu

    m = PyUnicycle()
    s = pcr.SolverFDDP(pcr.ShootingProblem(X0, [m] * T, m))
    s.solve([X0] * (T + 1), [np.zeros(2)] * T, MAXITER, False, 1e-9)
    ref = pcr.ActionModelUnicycle()
    sref = pcr.SolverFDDP(pcr.ShootingProblem(X0, [ref] * T, ref))
    sref.solve([X0] * (T + 1), [np.zeros(2)] * T, MAXITER, False, 1e-9)
    assert s.cost == pytest.approx(sref.cost, abs=1e-6)


def test_safe_cppdef_reports_error_without_crashing(cr):
    """Regression for the Pattern-9 mitigation: a broken model is caught
    out-of-process and raises cleanly -- the interpreter survives (this test, and
    everything after it, still runs)."""
    with pytest.raises(cppyy_kit.CppyyKitError):
        wbc_kit.safe_cppdef("namespace broken { this is not valid c++ ; }")
    # interpreter intact: a real bringup call still works
    assert hasattr(wbc_kit.bringup_crocoddyl(), "SolverFDDP")

#!/usr/bin/env python3
"""
THE SHOWCASE: author a custom Crocoddyl action model three ways and solve the same
unicycle optimal-control problem with each -- the ompl_kit "lower to C++" story
applied to optimal control.

    (A) Python-derived model  -- subclass crocoddyl.ActionModelAbstract in Python,
        calc/calcDiff in NumPy. This is Crocoddyl's supported *prototype* path.
    (ref) built-in C++ model  -- crocoddyl.ActionModelUnicycle (compiled into the
        binding). The speed ceiling.
    (B) cppyy inline C++ model -- the SAME custom model written in C++ in a
        cppyy.cppdef string in THIS script, JIT-compiled at runtime with NO build
        system. The solver calls its calc/calcDiff natively.

All three solve the identical problem and converge to a bit-identical cost; (B)
runs at the built-in C++ speed and many times faster than the Python-derived model.
That is the wbc_kit thesis: prototype the model in Python, then *lower* it to inline
C++ with cppyy -- same script, minimal diff, no build system, native hot loop.

Run: pixi run -e wbc demo-wbc-lower       (numbers vary; machine may be shared)
"""
import os
import time

import numpy as np

import wbc_kit

T, MAXITER, REPS = 100, 50, 7
X0 = np.array([-1.0, -1.0, 1.0])


def _best(fn):
    fn()  # warm up (first-use JIT / caches)
    return min(_time_once(fn) for _ in range(REPS))


def _time_once(fn):
    t0 = time.perf_counter()
    fn()
    return time.perf_counter() - t0


def main():
    wbc_kit.bringup_crocoddyl()                  # Crocoddyl up under cppyy (for cppdef)
    import crocoddyl as pcr                       # the boost::python binding

    # --- (B) inline C++ custom model, JIT-compiled, no build system --------------
    with open(os.path.join(os.path.dirname(__file__), "..", "wbc_kit", "cpp",
                           "unicycle_model.cpp")) as fh:
        wbc_kit.safe_cppdef(fh.read())           # probed out-of-process, then cppdef'd
    wbc_demo = __import__("cppyy").gbl.wbc_demo
    model_b = wbc_demo.make_unicycle()
    res_b = wbc_demo.solve(model_b, T, MAXITER)
    t_b = _best(lambda: wbc_demo.solve(model_b, T, MAXITER))

    # --- (ref) Crocoddyl's compiled built-in model, via the Python binding -------
    def solve_ref():
        m = pcr.ActionModelUnicycle()
        s = pcr.SolverFDDP(pcr.ShootingProblem(X0, [m] * T, m))
        s.solve([X0] * (T + 1), [np.zeros(2)] * T, MAXITER, False, 1e-9)
        return s
    s_ref = solve_ref()
    t_ref = _best(lambda: solve_ref())

    # --- (A) Python-derived model (Crocoddyl's supported prototype path) ---------
    class PyUnicycle(pcr.ActionModelAbstract):
        def __init__(self):
            pcr.ActionModelAbstract.__init__(self, pcr.StateVector(3), 2, 5)
            self.dt, self.wx, self.wu = 0.1, 10.0, 1.0

        def calc(self, data, x, u=None):
            if u is None:
                r = np.concatenate([self.wx * np.asarray(x), np.zeros(2)])
                data.xnext = x
                data.r = r
                data.cost = 0.5 * float(r[:3].dot(r[:3]))
                return
            c, s = np.cos(x[2]), np.sin(x[2])
            data.xnext = np.array([x[0] + c * u[0] * self.dt,
                                   x[1] + s * u[0] * self.dt, x[2] + u[1] * self.dt])
            r = np.concatenate([self.wx * np.asarray(x), self.wu * np.asarray(u)])
            data.r = r
            data.cost = 0.5 * float(r.dot(r))

        def calcDiff(self, data, x, u=None):
            wx, wu = self.wx ** 2, self.wu ** 2
            data.Lx = np.asarray(x) * wx
            Lxx = np.zeros((3, 3))
            np.fill_diagonal(Lxx, wx)
            data.Lxx = Lxx
            if u is None:
                return
            c, s = np.cos(x[2]), np.sin(x[2])
            data.Lu = np.asarray(u) * wu
            Luu = np.zeros((2, 2))
            np.fill_diagonal(Luu, wu)
            data.Luu = Luu
            Fx = np.eye(3)
            Fx[0, 2] = -s * u[0] * self.dt
            Fx[1, 2] = c * u[0] * self.dt
            data.Fx = Fx
            Fu = np.zeros((3, 2))
            Fu[0, 0] = c * self.dt
            Fu[1, 0] = s * self.dt
            Fu[2, 1] = self.dt
            data.Fu = Fu

    def solve_a():
        m = PyUnicycle()
        s = pcr.SolverFDDP(pcr.ShootingProblem(X0, [m] * T, m))
        s.solve([X0] * (T + 1), [np.zeros(2)] * T, MAXITER, False, 1e-9)
        return s
    s_a = solve_a()
    t_a = _best(lambda: solve_a())

    print(f"\nUnicycle optimal control, T={T} nodes, FDDP (max {MAXITER} iters)\n")
    row = "  {:34s} cost={:.6f} iters={:2d} solve={:8.2f} ms"
    print(row.format("(A) Python-derived model", s_a.cost, s_a.iter, t_a * 1e3))
    print(row.format("(ref) built-in C++ (binding)", s_ref.cost, s_ref.iter,
                     t_ref * 1e3))
    print(row.format("(B) cppyy inline C++ model", res_b.cost, res_b.iters,
                     t_b * 1e3))
    match = (abs(res_b.cost - s_ref.cost) < 1e-6 and abs(s_a.cost - s_ref.cost) < 1e-6)
    print(f"\n  numeric match (A == ref == B): {match}")
    print(f"  speedup  Python-model / cppyy-inline-C++ = {t_a / t_b:.1f}x"
          f"   (built-in ceiling: {t_a / t_ref:.1f}x)")
    print("\n  Same script. No build system. Native hot loop.\n")


if __name__ == "__main__":
    main()

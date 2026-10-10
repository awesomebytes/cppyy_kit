"""Load the compiled adapter through cppyy. Inputs are explicit copied vectors."""
import time
from build import ROOT, LIBRARY, build

_LOAD = None


def load():
    global _LOAD
    if _LOAD is not None:
        return _LOAD
    build_result = build()
    start = time.perf_counter()
    import cppyy
    import_s = time.perf_counter() - start
    start = time.perf_counter()
    cppyy.load_library(str(LIBRARY))
    cppyy.include(str(ROOT / 'tracking.hpp'))
    _LOAD = {'namespace': cppyy.gbl.trajectory_demo, 'cppyy': cppyy,
             'import_s': import_s, 'adapter_declarations_s': time.perf_counter() - start,
             'build': build_result}
    return _LOAD


def vector(values):
    return load()['cppyy'].gbl.std.vector['double'](values)


def session(dofs=3, dt=.001, kp=12.0):
    return load()['namespace'].Session(dofs, dt, vector([1.] * dofs),
                                     vector([2.] * dofs), vector([8.] * dofs), kp)


def set_target(engine, positions, velocities=None, accelerations=None):
    n = len(positions)
    engine.target(vector(positions), vector([0.] * n if velocities is None else velocities),
                  vector([0.] * n if accelerations is None else accelerations))


def state_tuple(engine):
    state = engine.state()
    return tuple(tuple(getattr(state, field)) for field in
                 ('position', 'velocity', 'acceleration', 'command')) + (state.time, state.duration, state.result)

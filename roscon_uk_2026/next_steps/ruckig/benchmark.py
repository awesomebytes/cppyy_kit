"""Separate native work, boundary, conversion, and official-binding composition."""
from pathlib import Path
import importlib.metadata
import json
import platform
import resource
import time
import numpy as np
import ruckig
from build import ROOT, BUILD
from native import load, session, set_target, vector
from test_acceptance import baseline


def distribution(ns):
    data = np.array(ns, dtype=float) / 1000
    return {'count': len(ns), 'unit': 'us', 'p50': float(np.percentile(data, 50)),
            'p95': float(np.percentile(data, 95)), 'p99': float(np.percentile(data, 99)),
            'max': float(np.max(data))}


def measure(operation, repeats=3000):
    for _ in range(100):
        operation()
    times = []
    for _ in range(repeats):
        start = time.perf_counter_ns()
        operation()
        times.append(time.perf_counter_ns() - start)
    return distribution(times)


def run(n):
    engine = session(n)
    measured_list = [0.] * n
    measured_native = vector(measured_list)
    set_target(engine, [.5] * n)
    for _ in range(100):
        engine.step(measured_native)
    wall, internal, recalculation = [], [], []
    for k in range(5000):
        changing = k % 500 == 0
        if changing:
            set_target(engine, [(.5 if k % 1000 == 0 else -.5)] * n)
        start = time.perf_counter_ns()
        sample = engine.step(measured_native)
        elapsed = time.perf_counter_ns() - start
        (recalculation if changing else wall).append(elapsed)
        if not changing:
            internal.append(sample.native_ns)
    pyengine, inp, out = baseline(n)
    inp.target_position = [.5] * n

    def python_composed():
        pyengine.update(inp, out)
        commands = [q + .001*max(-1., min(1., v+12*(p-q))) for p, v, q in
                    zip(out.new_position, out.new_velocity, measured_list)]
        out.pass_to_input(inp)
        return commands

    result = {'dofs': n, 'native_step_internal': distribution(internal),
              'native_step_boundary_retained_input': distribution(wall),
              'native_recalculation_boundary_retained_input': distribution(recalculation),
              'python_binding_plus_python_tracking': measure(python_composed),
              'python_list_to_native_vector': measure(lambda: vector(measured_list)),
              'native_outputs_to_python_lists': measure(lambda: [list(getattr(sample, field)) for field in
                                                                   ['position', 'velocity', 'acceleration', 'command']]),
              'native_step_with_list_conversion': measure(lambda: engine.step(vector(measured_list)))}
    return result


def main():
    setup = load()
    result = {'versions': {name: importlib.metadata.version(name) for name in ['cppyy', 'numpy', 'ruckig', 'pytest']},
              'python': platform.python_version(), 'platform': platform.platform(),
              'setup': {k: v for k, v in setup.items() if k not in ['namespace', 'cppyy']},
              'last_full_build': json.loads((BUILD / 'build.json').read_text()),
              'measurements': [run(3), run(6)],
              'peak_process_rss_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
              'timing_scope': 'single process, warmed calls, no scheduling or real-time guarantee'}
    (ROOT / 'measurements.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

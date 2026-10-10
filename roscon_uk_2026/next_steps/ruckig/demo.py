"""Retarget a persistent trajectory and mirror its native tracking commands."""
import json
import numpy as np
from native import session, set_target, vector


def main():
    engine = session(3)
    measured = np.zeros(3)
    changes = {0: [.6, -.3, .2], 400: [-.2, .4, -.1], 900: [.1, 0., .2]}
    samples = []
    for tick in range(4000):
        if tick in changes:
            set_target(engine, changes[tick])
        sample = engine.step(vector(measured.tolist()))
        measured = np.array(sample.command)
        if tick in changes or sample.result == 1:
            samples.append({'tick': tick, 'new_calculation': sample.new_calculation,
                            'reference': list(sample.position), 'velocity': list(sample.velocity),
                            'acceleration': list(sample.acceleration), 'command': measured.tolist(),
                            'result': sample.result})
        if tick > 900 and sample.result == 1:
            break
    assert sample.result == 1
    # Allow the tracking law to settle after the reference reaches its rest target.
    for _ in range(1000):
        measured = np.array(engine.step(vector(measured.tolist())).command)
    error = float(np.max(np.abs(measured - changes[900])))
    assert error < 1e-6
    result = {'dofs': 3, 'dt_s': .001, 'updates': samples, 'settled_position_error': error,
              'hardware': 'numerical command mirroring only; see mock_control.py for control-kit probe'}
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

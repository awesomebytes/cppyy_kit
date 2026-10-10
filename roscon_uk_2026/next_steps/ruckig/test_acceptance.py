"""Independent binding parity, phase-polynomial bounds, and state transaction checks."""
import math
import numpy as np
import pytest
import ruckig
from native import load, session, set_target, vector, state_tuple

TOL = 2e-8  # absolute position/velocity/acceleration tolerance in SI units


def baseline(n, dt=.001):
    engine = ruckig.Ruckig(n, dt)
    inp, out = ruckig.InputParameter(n), ruckig.OutputParameter(n)
    inp.current_position = inp.current_velocity = inp.current_acceleration = [0.] * n
    inp.target_position = inp.target_velocity = inp.target_acceleration = [0.] * n
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = [1.] * n, [2.] * n, [8.] * n
    return engine, inp, out


def check_phases(native):
    """Check each constant-jerk interval from endpoint and midpoint states.

    Acceleration extrema are at interval ends. Velocity extrema also occur
    where acceleration is zero. Jerk is a secant of acceleration inside each
    interval. A jerk jump between intervals is allowed; it is not snap-limited.
    """
    knots = list(native.knots())
    for left, right in zip(knots, knots[1:]):
        h = right - left
        if h < 1e-10:
            continue
        s0, s1 = native.at_time(left), native.at_time(right)
        sm = native.at_time((left + right) / 2)
        v0, a0 = np.array(s0.velocity), np.array(s0.acceleration)
        a1 = np.array(s1.acceleration)
        jerk = (a1 - a0) / h
        assert np.max(np.abs(jerk)) <= 8. + TOL
        assert max(np.max(np.abs(a0)), np.max(np.abs(a1))) <= 2. + TOL
        np.testing.assert_allclose(sm.acceleration, a0 + jerk*h/2, atol=TOL, rtol=0)
        np.testing.assert_allclose(sm.velocity, v0 + a0*h/2 + jerk*h*h/8, atol=TOL, rtol=0)
        np.testing.assert_allclose(sm.jerk, jerk, atol=TOL, rtol=0)
        assert max(np.max(np.abs(v0)), np.max(np.abs(s1.velocity))) <= 1. + TOL
        for axis, j in enumerate(jerk):
            if abs(j) > 1e-10:
                t = -a0[axis] / j
                if 0 < t < h:
                    extremum = v0[axis] + a0[axis]*t + j*t*t/2
                    assert abs(extremum) <= 1. + TOL


@pytest.mark.parametrize('n', [3, 6])
def test_parity_changing_targets_reversal_finished_and_tracking(n):
    native = session(n)
    engine, inp, out = baseline(n)
    measured = np.zeros(n)
    updates = {0: [(.5 if i % 2 == 0 else -.25) for i in range(n)],
               300: [(-.4 if i % 2 == 0 else .3) for i in range(n)],
               700: [(.1 if i % 2 == 0 else -.1) for i in range(n)]}
    finished = False
    for k in range(5000):
        if k in updates:
            before = state_tuple(native)
            target = updates[k]
            set_target(native, target)
            assert state_tuple(native) == before  # changing a goal does not reset p/v/a
            inp.target_position = target
        result = engine.update(inp, out)
        sample = native.step(vector(measured.tolist()))
        assert sample.result == int(result)
        assert sample.new_calculation == out.new_calculation
        for field, expected in [('position', out.new_position), ('velocity', out.new_velocity),
                                ('acceleration', out.new_acceleration)]:
            np.testing.assert_allclose(getattr(sample, field), expected, atol=TOL, rtol=0)
        assert abs(sample.time - out.time) <= 1e-12
        rates = np.clip(np.array(out.new_velocity) + 12*(np.array(out.new_position)-measured), -1, 1)
        commands = measured + .001*rates
        np.testing.assert_allclose(sample.command, commands, atol=2e-12, rtol=0)
        if sample.new_calculation:
            check_phases(native)
            for t in np.linspace(0, sample.duration, 19):
                p, v, a = out.trajectory.at_time(float(t))
                ns = native.at_time(float(t))
                for name, values in [('position', p), ('velocity', v), ('acceleration', a)]:
                    np.testing.assert_allclose(getattr(ns, name), values, atol=TOL, rtol=0)
        # GenericSystem mirroring is a command transport model, not plant dynamics.
        measured = commands
        out.pass_to_input(inp)
        if k > 700 and result == ruckig.Result.Finished:
            finished = True
            np.testing.assert_allclose(sample.position, updates[700], atol=TOL, rtol=0)
            for _ in range(5):
                held = native.step(vector(measured.tolist()))
                assert held.result == 1
                np.testing.assert_allclose(held.velocity, 0, atol=TOL, rtol=0)
            break
    assert finished


@pytest.mark.parametrize('n', [3, 6])
def test_nonzero_target_state_and_random_changes(n):
    native = session(n)
    engine, inp, out = baseline(n)
    rng = np.random.default_rng(721)
    measured = vector([0.] * n)
    for segment in range(12):
        p, v, a = rng.uniform(-.8, .8, n), rng.uniform(-.2, .2, n), rng.uniform(-.3, .3, n)
        set_target(native, p.tolist(), v.tolist(), a.tolist())
        inp.target_position, inp.target_velocity, inp.target_acceleration = p, v, a
        for k in range(50):
            result = engine.update(inp, out)
            ns = native.step(measured)
            assert ns.result == int(result)
            for field in ['position', 'velocity', 'acceleration']:
                np.testing.assert_allclose(getattr(ns, field), getattr(out, 'new_' + field), atol=TOL, rtol=0)
            if k == 0:
                check_phases(native)
            out.pass_to_input(inp)


def test_retarget_preserves_derivatives_but_reset_discards_them():
    moving = session()
    set_target(moving, [.7]*3)
    for _ in range(250):
        moving.step(vector([0.]*3))
    state = moving.state()
    assert abs(state.velocity[0]) > .1 and abs(state.acceleration[0]) > .1
    set_target(moving, [-.4]*3)
    reset = session()
    reset.reset(vector(list(state.position)), vector([0.]*3), vector([0.]*3))
    set_target(reset, [-.4]*3)
    m, r = moving.step(vector([0.]*3)), reset.step(vector([0.]*3))
    assert abs(m.velocity[0] - state.velocity[0]) <= 2*.001 + TOL
    assert abs(m.acceleration[0] - state.acceleration[0]) <= 8*.001 + TOL
    assert abs(m.velocity[0] - r.velocity[0]) > .1
    fresh = session()
    set_target(fresh, [-.4]*3)
    moving.reset(vector([0.]*3), vector([0.]*3), vector([0.]*3))
    np.testing.assert_allclose(moving.step(vector([0.]*3)).position,
                               fresh.step(vector([0.]*3)).position, atol=TOL, rtol=0)


@pytest.mark.parametrize('operation', ['target', 'limits', 'reset', 'step'])
@pytest.mark.parametrize('bad', [[], [0., 0.], [0., 0., 0., 0.], [math.nan]*3, [math.inf]*3])
def test_invalid_vector_is_transactional(operation, bad):
    native, clean = session(), session()
    for s in [native, clean]:
        set_target(s, [.5]*3)
        for _ in range(20):
            s.step(vector([0.]*3))
    before = state_tuple(native)
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        if operation == 'step':
            native.step(vector(bad))
        else:
            getattr(native, operation)(vector(bad), vector([0.]*3), vector([0.]*3))
    assert state_tuple(native) == before
    np.testing.assert_allclose(native.step(vector([0.]*3)).position,
                               clean.step(vector([0.]*3)).position, atol=TOL, rtol=0)


@pytest.mark.parametrize('values', [[0.]*3, [-1.]*3, [math.nan]*3, [math.inf]*3])
def test_invalid_limits_and_constructor(values):
    native = session()
    before = state_tuple(native)
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.limits(vector(values), vector([2.]*3), vector([8.]*3))
    assert state_tuple(native) == before
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        load()['namespace'].Session(3, .001, vector(values), vector([2.]*3), vector([8.]*3))


@pytest.mark.parametrize('dt', [0, -.001, math.nan, math.inf])
def test_invalid_timestep(dt):
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        session(dt=dt)


@pytest.mark.parametrize('n', [0, 2, 4, 7])
def test_unsupported_dofs(n):
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        session(n)


def test_out_of_limit_states_rejected_then_previous_goal_continues():
    native, clean = session(), session()
    for instance in [native, clean]:
        set_target(instance, [.5]*3)
        instance.step(vector([0.]*3))
    before = state_tuple(native)
    for v, a in [([2.]*3, [0.]*3), ([0.]*3, [3.]*3), ([1.]*3, [-.2]*3)]:
        with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
            set_target(native, [.1]*3, v, a)
        assert state_tuple(native) == before
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.at_time(math.nan)
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.at_time(-.1)
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.at_time(native.state().duration + 1)
    measured = vector([0.]*3)
    for _ in range(2000):
        continued, expected = native.step(measured), clean.step(measured)
        np.testing.assert_allclose(continued.position, expected.position, atol=TOL, rtol=0)
    np.testing.assert_allclose(continued.position, [.5]*3, atol=TOL, rtol=0)


def test_nonzero_terminal_state_is_not_a_hold():
    native = session()
    set_target(native, [.2]*3, [.1]*3, [.1]*3)
    sample = native.step(vector([0.]*3))
    final = native.at_time(sample.duration)
    np.testing.assert_allclose(final.velocity, .1, atol=TOL, rtol=0)
    np.testing.assert_allclose(final.acceleration, .1, atol=TOL, rtol=0)
    while sample.result != 1:
        sample = native.step(vector([0.]*3))
    previous = sample
    sample = native.step(vector([0.]*3))
    np.testing.assert_allclose(np.array(sample.velocity) - np.array(previous.velocity), .0001, atol=TOL, rtol=0)


def test_tightening_limits_rejects_infeasible_current_state():
    native = session()
    set_target(native, [.7]*3)
    for _ in range(250):
        native.step(vector([0.]*3))
    before = state_tuple(native)
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.limits(vector([.05]*3), vector([2.]*3), vector([8.]*3))
    assert state_tuple(native) == before
    with pytest.raises(load()['cppyy'].gbl.std.invalid_argument):
        native.reset(vector([0.]*3), vector([1.]*3), vector([.2]*3))
    assert state_tuple(native) == before
    native.limits(vector([1.2]*3), vector([2.5]*3), vector([10.]*3))
    assert state_tuple(native) == before
    assert native.step(vector([0.]*3)).new_calculation

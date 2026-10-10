import numpy as np
import pytest
from analyze import sweep


def test_two_hands_thresholds_and_duration():
    t = np.arange(5, dtype=float)
    states = np.zeros((5, 12))
    states[:, 0] = [0., .2, .4, .6, .6]
    states[:, 6] = [0., 0., 0., .3, .6]
    counts, seconds = sweep(t, states, np.array([.1, .25]), hold_seconds=1.5)
    assert counts.dtype == np.int32
    assert seconds.dtype == np.float64
    np.testing.assert_array_equal(counts, [[1, 0], [0, 0]])
    np.testing.assert_allclose(seconds, [[1., 0.], [0., 0.]])


def test_empty():
    counts, seconds = sweep(np.array([]), np.zeros((0, 12)), np.array([.1]))
    np.testing.assert_array_equal(counts, [[0, 0]])
    np.testing.assert_array_equal(seconds, [[0., 0.]])


@pytest.mark.parametrize("t,states,thresholds", [
    ([0., 0.], np.zeros((2, 12)), [.1]),
    ([0.], np.zeros((1, 6)), [.1]),
    ([0.], np.zeros((1, 12)), [-.1]),
])
def test_invalid(t, states, thresholds):
    with pytest.raises(ValueError):
        sweep(t, states, thresholds)

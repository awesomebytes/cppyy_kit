"""Acceptance cases check timing, hysteresis, validation, and input layout."""
import numpy as np
import pytest
from task import motion_mask


def positions(t, speeds):
    xyz = np.zeros((len(t), 3))
    xyz[1:, 0] = np.cumsum(np.asarray(speeds) * np.diff(t))
    return xyz


@pytest.mark.parametrize("t,speeds,hold,expected", [
    ([0., .1, .2, .3, .4, .5], [.1, .1, .1, .03, .09], .15, [0, 0, 0, 1, 0, 0]),
    ([0., .1, .2, .3, .4], [.1, .06, .05, .01], 0., [0, 1, 1, 1, 0]),
    ([0., .1, .2, .3, .4], [.1, .01, .1, .1], .15, [0, 0, 0, 0, 0]),
    ([0., .05, .11, .25], [.12, .12, .12], .17, [0, 0, 0, 1]),
    ([0.], [], .12, [0]),
    ([], [], .12, []),
])
def test_events(t, speeds, hold, expected):
    t = np.asarray(t)
    got = motion_mask(t, positions(t, speeds), hold_seconds=hold)
    assert got.dtype == np.int32
    np.testing.assert_array_equal(got, expected)


def test_strided_float32_input_and_three_axes():
    t = np.arange(10, dtype=np.float32)[::2]
    xyz = np.zeros((10, 3), dtype=np.float32)[::2]
    xyz[:, 2] = t * .2
    np.testing.assert_array_equal(motion_mask(t, xyz, hold_seconds=0), [0, 1, 1, 1, 1])


@pytest.mark.parametrize("t,xyz", [
    ([0., 0.], np.zeros((2, 3))),
    ([1., 0.], np.zeros((2, 3))),
    ([0., float('nan')], np.zeros((2, 3))),
    ([0., 1.], np.zeros((2, 2))),
    ([0.], [[float('inf'), 0., 0.]]),
])
def test_invalid(t, xyz):
    with pytest.raises(ValueError):
        motion_mask(t, xyz)

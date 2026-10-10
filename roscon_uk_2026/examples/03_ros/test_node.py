import numpy as np
from node import rolling_query


def test_window_rule():
    t = np.arange(5,dtype=np.float64)
    states = np.zeros((5,12))
    states[:,0] = [0,.2,.4,.6,.6]
    counts, seconds = rolling_query(t,states,np.array([.1,.25]),1.5)
    np.testing.assert_array_equal(counts,[[1,0],[0,0]])
    np.testing.assert_allclose(seconds,[[1,0],[0,0]])


def test_singleton():
    counts, seconds = rolling_query(np.array([0.]),np.zeros((1,12)),np.array([.1]))
    np.testing.assert_array_equal(counts,[[0,0]])
    np.testing.assert_array_equal(seconds,[[0.,0.]])

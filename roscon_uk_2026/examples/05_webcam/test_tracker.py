import numpy as np
import pytest
from tracker import track


def test_translation():
    rng = np.random.default_rng(17)
    a = rng.integers(0, 256, (80,100), dtype=np.uint8)
    b = np.roll(a, (3,-2), (0,1))
    assert track(a,b,40,30) == (38,33)


def test_tie_and_boundary():
    a = np.zeros((20,20), dtype=np.uint8)
    assert track(a,a,5,5,5,8) == (5,5)
    assert track(a,a,10,10,2,3) == (7,7)


def test_strided():
    a = np.random.default_rng(3).integers(0,256,(80,100),dtype=np.uint8)[:,::2]
    assert track(a,a,20,30) == (20,30)


def test_invalid():
    with pytest.raises(ValueError):
        track(np.zeros((20,20),dtype=np.uint8), np.zeros((20,20),dtype=np.uint8), 0,0)

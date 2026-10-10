"""Saved Python policy. Read GUIDE.md before changing the native contract."""
import math


def make_policy(ob):
    class BiasedKeepOutPolicy(ob.StateValidityChecker):
        def __init__(self, si, bias_x=0.0, fail_at=None):
            super().__init__(si)
            self.bias_x = float(bias_x)
            if not math.isfinite(self.bias_x):
                raise ValueError("bias must be finite")
            self.fail_at = fail_at
            self.calls = 0

        def isValid(self, state):
            self.calls += 1
            if self.calls == self.fail_at:
                raise ValueError("scripted validity policy failure")
            x, y = float(state[0]), float(state[1])
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
                return False
            dx, dy = x + self.bias_x - 0.5, y - 0.5
            return dx * dx + dy * dy > 0.25 * 0.25

    return BiasedKeepOutPolicy

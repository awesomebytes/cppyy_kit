"""Implement the factory contract in GUIDE.md. Keep native.hpp unchanged."""
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
            raise NotImplementedError("implement the existing OMPL validity policy")

    return BiasedKeepOutPolicy

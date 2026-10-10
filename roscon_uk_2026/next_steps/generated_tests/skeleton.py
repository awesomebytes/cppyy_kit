"""Missing-logic exercise. Complete against CONTRACT.md and test_filter.py.

This file is not the saved solution and is not collected by pytest.
"""


def assert_reset_isolation(factory, config, history, replay):
    """Compare a reused native instance after reset with a fresh one.

    history/replay each hold (int64 timestamps, float64 Nx3 positions).
    Retain each instance through calls and close both on all exit paths.
    """
    raise NotImplementedError("Implement reset isolation and cleanup")


def assert_rejection_preserves_state(instance, timestamps, positions, exception):
    """Reject a batch with the given exception and compare the entire snapshot."""
    raise NotImplementedError("Implement invalid-batch state isolation")

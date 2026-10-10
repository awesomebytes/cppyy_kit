"""Implement the temporal query. Typed extraction and fixtures are supplied."""


def count_speed_above(batch, threshold_m_s=0.3):
    """Count consecutive intervals where Cartesian speed exceeds threshold_m_s.

    batch.log_time_ns is uint64[N]. batch.position_m is float64[N,3] in metres.
    Keep integer time differences until conversion to seconds. Skip zero-time
    duplicate intervals. Reject decreasing timestamps, invalid shapes, nonfinite
    positions and nonfinite/negative thresholds. Empty input returns zero.
    Accept unaligned/strided NumPy arrays through an explicit aligned copy.
    Perform the repeated numerical loop in C++, using one batch boundary call.
    """
    raise NotImplementedError("implement validated batch speed query")

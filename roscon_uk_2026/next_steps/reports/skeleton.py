"""Agent exercise. Complete only window ranking and integer image alignment.

Read GUIDE.md, PROMPT.md, report_core.py function contracts, and test_report.py.
The saved solution is report_core.py. Tests use REPORT_CORE_MODULE to select this
module. Do not copy the saved implementation for a fresh-agent evaluation.
"""
from .report_core import Window, integer_times, check_frames, trace_svg


def rank_windows(times, scores, duration_ns, count=5):
    """Implement full-duration half-open candidates and greedy disjoint ranking.

    Return Window objects. Rank descending mean rounded to 1e-12 metres, then
    earliest start. Peak-score sample is representative; tie uses earliest time.
    Validate finite nonnegative scores and strictly increasing integer ns.
    """
    raise NotImplementedError('Implement native-result window selection')


def match_images(queries, image_times, *, query_clock, image_clock,
                 direction='nearest', tolerance_ns=60_000_000):
    """Implement exact integer-ns as-of matching without converting times to float.

    Queries may be in rank order. Images must be sorted but can repeat. Return
    one dict per query: query_ns, image_index, image_ns, delta_ns, status.
    Reject clock mismatch. Retain unmatched records. See GUIDE.md tie policies.
    """
    raise NotImplementedError('Implement bounded timestamp matching')

"""Position error and bounded sampled signal alignment, with explicit units."""
import numpy as np


def position_rmse(estimate, truth):
    estimate, truth = np.asarray(estimate), np.asarray(truth)
    if estimate.shape != truth.shape or estimate.ndim != 2 or estimate.shape[1] != 3:
        raise ValueError("expected matching N by 3 Cartesian arrays")
    if not estimate.size or not np.isfinite(estimate).all() or not np.isfinite(truth).all():
        raise ValueError("expected nonempty finite arrays")
    return float(np.sqrt(np.mean(np.sum((estimate - truth) ** 2, axis=1))))


def alignment_lag(timestamps_s, estimate, truth, max_lag_s=0.30,
                  resolution_s=0.005, settle_s=0.50, max_interp_gap_s=0.05):
    """Align truth[t] with interpolated estimate[t+shift] on a fixed interval.

    The positive shift measures signal alignment, not processing latency. Noise,
    interpolation, and amplitude distortion can bias it. A boundary minimum is
    censored. Exclude times whose candidate queries could cross a large gap.
    """
    t = np.asarray(timestamps_s, dtype=np.float64)
    dt = np.diff(t)
    if len(dt) == 0 or not np.isfinite(t).all() or np.any(dt <= 0):
        raise ValueError("alignment requires strictly increasing finite timestamps")
    position_rmse(estimate, truth)
    if len(t) != len(truth):
        raise ValueError("timestamp count does not match positions")
    if max_lag_s <= 0 or resolution_s <= 0 or settle_s < 0 or max_interp_gap_s <= 0:
        raise ValueError("invalid alignment bounds")
    shifts = np.arange(int(np.floor(max_lag_s / resolution_s + 1e-9)) + 1) * resolution_s
    mask = (t >= t[0] + settle_s) & (t <= t[-1] - shifts[-1])
    for gap in np.flatnonzero(dt > max_interp_gap_s):
        mask &= ~((t > t[gap] - shifts[-1]) & (t < t[gap+1]))
    if not mask.any():
        raise ValueError("episode too short for common alignment interval")
    query = t[mask]
    errors = []
    for shift in shifts:
        aligned = np.column_stack([np.interp(query+shift, t, estimate[:, axis]) for axis in range(3)])
        errors.append(float(np.mean(np.sum((aligned-truth[mask])**2, axis=1))))
    best = int(np.argmin(errors))
    return {"lag_s": float(shifts[best]), "resolution_s": resolution_s,
            "at_upper_bound": best == len(shifts)-1, "max_lag_s": float(shifts[-1]),
            "common_samples": int(mask.sum()), "aligned_rmse_m": float(np.sqrt(errors[best]))}

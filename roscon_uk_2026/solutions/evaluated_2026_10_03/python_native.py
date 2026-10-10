"""Complete motion_mask. The surrounding input contract is supplied boilerplate."""
import numpy as np
from cppyy_kit import cpp


@cpp(nogil=True)
def _motion_mask_kernel(t: cpp.arr("double"), xyz: cpp.arr("double"),
                        n: int, enter_speed: float, exit_speed: float,
                        hold_seconds: float, mask: "int*") -> None:
    """double candidate_time = 0.0;
    int candidate = -1;
    bool active = false;
    for (int i = 1; i < n; ++i) {
        const double dx = xyz[3 * i] - xyz[3 * (i - 1)];
        const double dy = xyz[3 * i + 1] - xyz[3 * (i - 1) + 1];
        const double dz = xyz[3 * i + 2] - xyz[3 * (i - 1) + 2];
        const double elapsed = t[i] - t[i - 1];
        const double speed = std::sqrt(dx * dx + dy * dy + dz * dz) / elapsed;
        if (active) {
            if (speed < exit_speed) {
                active = false;
                candidate = -1;
            } else {
                mask[i] = 1;
            }
        } else if (speed < enter_speed) {
            candidate = -1;
        } else {
            if (candidate < 0) {
                candidate = i;
                candidate_time = t[i];
            }
            if (t[i] - candidate_time >= hold_seconds) {
                active = true;
                mask[i] = 1;
            }
        }
    }
    return;"""


def prepare(t, xyz, enter_speed, exit_speed, hold_seconds):
    t = np.ascontiguousarray(t, dtype=np.float64)
    xyz = np.ascontiguousarray(xyz, dtype=np.float64)
    if t.ndim != 1 or xyz.shape != (len(t), 3):
        raise ValueError("expected seconds (N,) and Cartesian positions (N,3)")
    if not np.isfinite(t).all() or not np.isfinite(xyz).all():
        raise ValueError("timestamps and positions must be finite")
    if np.any(np.diff(t) <= 0):
        raise ValueError("timestamps must increase strictly")
    if not np.isfinite([enter_speed, exit_speed, hold_seconds]).all():
        raise ValueError("parameters must be finite")
    if not (enter_speed >= exit_speed >= 0 and hold_seconds >= 0):
        raise ValueError("expected enter >= exit >= 0 and hold >= 0")
    return t, xyz


def motion_mask(t, xyz, enter_speed=0.08, exit_speed=0.04, hold_seconds=0.12):
    """Return an int32 mask for sustained Cartesian motion.

    Positions are metres; timestamps are seconds. Sample zero is inactive.
    Speed at i is Euclidean displacement from i-1 divided by elapsed time.
    While inactive, start a candidate when speed >= enter_speed. Reset it on
    any subsequent speed < enter_speed. Activate when t[i]-t[candidate] >=
    hold_seconds. Do not mark earlier candidate samples retroactively.
    While active, stay active until speed < exit_speed, then reset the candidate.
    """
    t, xyz = prepare(t, xyz, enter_speed, exit_speed, hold_seconds)
    n = len(t)
    mask = np.zeros(n, dtype=np.int32)
    if n < 2:
        return mask

    _motion_mask_kernel(t, xyz.reshape(-1), n, enter_speed, exit_speed,
                        hold_seconds, mask)

    return mask


def synthetic(n=250_000):
    t = np.arange(n, dtype=np.float64) / 50.0
    speed = np.where(np.arange(n) % 1000 < 400, 0.12, 0.01)
    xyz = np.zeros((n, 3), dtype=np.float64)
    xyz[:, 0] = np.cumsum(speed / 50.0)
    return t, xyz


def intervals(t, mask):
    """Return active [start,end] sample timestamps; no extrapolation at edges."""
    mask = np.asarray(mask, dtype=bool)
    edges = np.diff(np.r_[False, mask, False].astype(np.int8))
    return [(float(t[a]), float(t[b - 1]))
            for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]


if __name__ == "__main__":
    import time
    t, xyz = synthetic()
    started = time.perf_counter()
    mask = motion_mask(t, xyz)
    print({"samples": len(t), "events": len(intervals(t, mask)),
           "first_call_ms": 1000 * (time.perf_counter() - started)})

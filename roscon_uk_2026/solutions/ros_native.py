"""A supplied rclpy monitor. Accelerate only rolling_query()."""
import argparse
from collections import deque
import hashlib
import json
import time
import numpy as np
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray, NDArray


@cpp(nogil=True)
def _rolling_query_kernel(t: ConstNDArray[np.float64], states: ConstNDArray[np.float64],
                          thresholds: ConstNDArray[np.float64], hold_seconds: float,
                          counts: NDArray[np.int32], seconds: NDArray[np.float64]) -> None:
    """
    const std::size_t n = t_size;
    const std::size_t k_count = thresholds_size;
    for (std::size_t k = 0; k < k_count; ++k) {
        for (std::size_t hand = 0; hand < 2; ++hand) {
            counts[k * 2 + hand] = 0;
            seconds[k * 2 + hand] = 0.0;
        }
    }
    for (std::size_t hand = 0; hand < 2; ++hand) {
        const std::size_t offset = hand * 6;
        for (std::size_t k = 0; k < k_count; ++k) {
            bool active = false;
            std::size_t candidate = 0;
            bool has_candidate = false;
            for (std::size_t i = 1; i < n; ++i) {
                const double dx = states[i * 12 + offset] - states[(i - 1) * 12 + offset];
                const double dy = states[i * 12 + offset + 1] - states[(i - 1) * 12 + offset + 1];
                const double dz = states[i * 12 + offset + 2] - states[(i - 1) * 12 + offset + 2];
                const double dt = t[i] - t[i - 1];
                const double speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
                const double enter = thresholds[k];
                if (active) {
                    if (speed < enter / 2.0) {
                        active = false;
                        has_candidate = false;
                    }
                } else if (speed < enter) {
                    has_candidate = false;
                } else {
                    if (!has_candidate) {
                        candidate = i;
                        has_candidate = true;
                    }
                    if (t[i] - t[candidate] >= hold_seconds) {
                        active = true;
                        ++counts[k * 2 + hand];
                    }
                }
                if (active) {
                    seconds[k * 2 + hand] += dt;
                }
            }
        }
    }
    """


def rolling_query(t, states, thresholds, hold_seconds=.02):
    """Recompute motion events on a rolling window, resetting state at its start.

    t(N) seconds, states(N,12), thresholds(K), all contiguous float64.
    Contract and return values match the offline sweep. These are window-local
    counts, not cumulative episode counts. The node supplies validated arrays.
    """
    t = np.ascontiguousarray(t, dtype=np.float64)
    states = np.ascontiguousarray(states.reshape(-1), dtype=np.float64)
    thresholds = np.ascontiguousarray(thresholds, dtype=np.float64)
    counts = np.empty((len(thresholds), 2), dtype=np.int32)
    seconds = np.empty((len(thresholds), 2), dtype=np.float64)
    _rolling_query_kernel(t, states, thresholds, hold_seconds, counts, seconds)
    return counts, seconds


def main():
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected", type=int, default=2735)
    parser.add_argument("--timeout", type=float, default=80)
    args = parser.parse_args()
    thresholds = np.linspace(.02,.20,41)
    rolling_query(np.array([0.,.02]),np.zeros((2,12)),thresholds)
    rclpy.init()
    node = Node("roscon_motion_monitor")
    history = deque(maxlen=256)
    ns0, received, timings, digest = [None], [0], [], hashlib.sha256()
    def callback(message):
        started = time.perf_counter()
        payload = json.loads(message.data)
        if payload["sequence"] != received[0]:
            raise RuntimeError("missing, repeated, or reordered replay message")
        if ns0[0] is None:
            ns0[0] = payload["recorded_ns"]
        t = (payload["recorded_ns"]-ns0[0])/1e9
        if history and t <= history[-1][0]:
            raise ValueError("recorded times must increase")
        state = np.asarray(payload["ee_state"],dtype=np.float64)
        if state.shape != (12,) or not np.isfinite(state).all():
            raise ValueError("invalid recorded Cartesian state")
        history.append((t,state))
        times = np.ascontiguousarray([item[0] for item in history])
        states = np.ascontiguousarray([item[1] for item in history])
        counts, seconds = rolling_query(times,states,thresholds)
        digest.update(counts.tobytes())
        # Floating durations are compared independently offline, not byte-hashed.
        timings.append(1000*(time.perf_counter()-started))
        received[0] += 1
    subscription = node.create_subscription(String,"/roscon/poses",callback,100)
    print("READY",flush=True)
    deadline = time.monotonic()+args.timeout
    try:
        while received[0] < args.expected and time.monotonic() < deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        print("RESULT " + json.dumps({"received": received[0], "expected": args.expected,
              "counts_sha256": digest.hexdigest(),
              "callback_median_ms": float(np.median(timings)) if timings else None,
              "callback_p99_ms": float(np.percentile(timings,99)) if timings else None}))
        if received[0] != args.expected:
            raise RuntimeError("incomplete replay")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

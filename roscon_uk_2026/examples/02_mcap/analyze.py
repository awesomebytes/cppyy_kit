"""MCAP loading is supplied; implement the threshold-sweep query."""
import argparse
import importlib.util
import json
from pathlib import Path
import statistics
import time

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


def load_poses(path):
    """Decode recorded EE states, preserving sample order and integer log times.

    /wbc_lerobot is std_msgs/msg/String containing JSON. ee_state contains
    [left xyz, left rotvec, right xyz, right rotvec]. Analysis uses only xyz.
    The source does not name the coordinate frame here. Do not relabel it world.
    """
    timestamps, states = [], []
    with open(path, "rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        for schema, channel, message, decoded in reader.iter_decoded_messages(topics=["/wbc_lerobot"]):
            if schema.name != "std_msgs/msg/String":
                raise ValueError("unexpected pose-topic schema")
            payload = json.loads(decoded.data)
            state = payload["ee_state"]
            if len(state) != 12:
                raise ValueError("expected 12 end-effector state coordinates")
            timestamps.append(message.log_time)
            states.append(state)
    if not timestamps:
        raise ValueError("no /wbc_lerobot observations found")
    timestamps = np.asarray(timestamps, dtype=np.int64)
    t = (timestamps - timestamps[0]).astype(np.float64) / 1e9
    return timestamps, t, np.ascontiguousarray(states, dtype=np.float64)


def sweep(t, states, thresholds, hold_seconds=0.12):
    """Return (counts int32[K,2], active_seconds float64[K,2]).

    Use the milestone-1 motion state machine independently for both hands and
    each enter threshold. exit_speed = enter_speed/2. Count inactive-to-active
    transitions. Active duration is sum(t[i]-t[i-1]) for active samples i>0,
    not the difference between interval endpoint timestamps. Do not change the
    candidate confirmation rule or mark candidate samples retroactively.
    """
    t = np.ascontiguousarray(t, dtype=np.float64)
    states = np.ascontiguousarray(states, dtype=np.float64)
    thresholds = np.ascontiguousarray(thresholds, dtype=np.float64)
    if t.ndim != 1 or states.shape != (len(t), 12) or thresholds.ndim != 1:
        raise ValueError("expected t(N), states(N,12), thresholds(K)")
    if not all(np.isfinite(a).all() for a in (t, states, thresholds)):
        raise ValueError("all inputs must be finite")
    if np.any(np.diff(t) <= 0) or np.any(thresholds < 0) or not np.isfinite(hold_seconds) or hold_seconds < 0:
        raise ValueError("invalid timestamps, thresholds, or hold duration")
    raise NotImplementedError("implement the bimanual threshold sweep")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--query", help="optional generated query module with sweep()")
    args = parser.parse_args()
    query = sweep
    if args.query:
        spec = importlib.util.spec_from_file_location("generated_query", args.query)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        query = module.sweep
    started = time.perf_counter()
    ns, t, states = load_poses(args.path)
    load_ms = 1000 * (time.perf_counter() - started)
    thresholds = np.linspace(.02, .20, 41)
    started = time.perf_counter()
    counts, seconds = query(t, states, thresholds)
    first_ms = 1000 * (time.perf_counter() - started)
    times = []
    for _ in range(7):
        started = time.perf_counter()
        query(t, states, thresholds)
        times.append(1000 * (time.perf_counter() - started))
    print(json.dumps({"samples": len(t), "duration_s": float(t[-1]),
                      "decode_and_load_ms": load_ms, "query_first_ms": first_ms,
                      "query_warm_median_ms": statistics.median(times),
                      "thresholds": thresholds.tolist(), "counts": counts.tolist(),
                      "active_seconds": seconds.tolist()}, indent=2))


if __name__ == "__main__":
    main()

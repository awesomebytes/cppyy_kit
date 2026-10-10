"""Run actual ROS playback, retain delivery counts, and compare count hashes."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from verify import load, ROOT, provenance, write_report
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", action="store_true")
    parser.add_argument("--domain-id", type=int,
                        help="use an unused DDS domain (default: baseline 77, native 78)")
    parser.add_argument("--output", type=Path,
                        help="default: build/current-checkout/ros_replay[_baseline].json")
    args = parser.parse_args()
    if args.domain_id is not None and not 0 <= args.domain_id <= 232:
        parser.error("--domain-id must be in [0, 232]")
    task = ROOT / ("examples/03_ros/node.py" if args.baseline else "solutions/ros_native.py")
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = str(args.domain_id if args.domain_id is not None else (77 if args.baseline else 78))
    player = None
    node = subprocess.Popen([sys.executable,str(task),"--timeout","100"],
                            stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,env=env)
    try:
        preamble = []
        for line in node.stdout:
            preamble.append(line)
            if line.strip() == "READY":
                break
        else:
            raise RuntimeError("node exited before READY: " + node.stderr.read())
        player = subprocess.Popen(["ros2","bag","play","-i",str(ROOT / "data/poses_replay.mcap"),
                                   "mcap","--rate","4","--delay","1","--topics","/roscon/poses",
                                   "--disable-keyboard-controls"],env=env,
                                  stdout=subprocess.DEVNULL,stderr=subprocess.PIPE,text=True)
        stdout, stderr = node.communicate(timeout=105)
        player.communicate(timeout=30)
        if node.returncode or player.returncode:
            raise RuntimeError(stdout + stderr + "player exit " + str(player.returncode))
        result = json.loads(next(line[7:] for line in stdout.splitlines() if line.startswith("RESULT ")))
    finally:
        for proc in (node,player):
            if proc and proc.poll() is None:
                proc.terminate()
                proc.communicate(timeout=10)
    mcap = load(ROOT / "solutions/mcap_native.py")
    ns, t, states = mcap.load_poses(ROOT / "data/hiw_pillow_episode_0002.mcap")
    digest = hashlib.sha256()
    thresholds = np.linspace(.02,.20,41)
    # The numerical kernel has separate Python-oracle parity checks.
    for i in range(len(t)):
        start = max(0,i-255)
        counts, seconds = mcap.sweep(t[start:i+1],states[start:i+1],thresholds,.02)
        digest.update(counts.tobytes())
    assert result["received"] == len(t)
    assert result["counts_sha256"] == digest.hexdigest()
    label = "ros_replay_baseline" if args.baseline else "ros_replay"
    result.update({"playback_rate": 4, "recorded_time_semantics": True,
                   "offline_counts_hash_match": True, "provenance": provenance(),
                   "ros_domain_id": int(env["ROS_DOMAIN_ID"])})
    write_report(result, args.output or ROOT / f"build/current-checkout/{label}.json")
    print(json.dumps(result,indent=2))


if __name__ == "__main__":
    main()

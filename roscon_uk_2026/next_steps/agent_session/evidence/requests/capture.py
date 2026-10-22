"""Run through marimo pair after each requested revision, in the live kernel."""
import hashlib
import json
import os
from pathlib import Path
from roscon_uk_2026.next_steps.agent_session.perception import save_overlay

capture_dir = Path("roscon_uk_2026/next_steps/agent_session/evidence")
capture_index = 1 + len(list(capture_dir.glob("iteration-*.json")))
capture_report = {
    "request_index": capture_index,
    "pid": os.getpid(),
    "snapshot_id": id(snapshot),
    "points_id": id(snapshot.points),
    "rgb_id": id(snapshot.rgb),
    "points_sha256": hashlib.sha256(snapshot.points.tobytes()).hexdigest(),
    "rgb_sha256": hashlib.sha256(snapshot.rgb.tobytes()).hexdigest(),
    "result": serializable(result),
}
save_overlay(snapshot, result, capture_dir / f"iteration-{capture_index}.png")
(capture_dir / f"iteration-{capture_index}.json").write_text(
    json.dumps(capture_report, indent=2) + "\n"
)
print(json.dumps(capture_report, indent=2))

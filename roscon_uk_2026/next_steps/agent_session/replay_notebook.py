"""Run the accepted notebook from source in a fresh interpreter."""
import hashlib
import json
import os
from pathlib import Path

from roscon_uk_2026.next_steps.agent_session.notebook import app
from roscon_uk_2026.next_steps.agent_session.perception import serializable


def main():
    _, definitions = app.run()
    snapshot = definitions["snapshot"]
    result = definitions["result"]
    report = {
        "pid": os.getpid(),
        "points_sha256": hashlib.sha256(snapshot.points.tobytes()).hexdigest(),
        "rgb_sha256": hashlib.sha256(snapshot.rgb.tobytes()).hexdigest(),
        "result": serializable(result),
    }
    destination = Path(__file__).parent / "evidence" / "replay.json"
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()

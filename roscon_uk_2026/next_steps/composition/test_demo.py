"""Run native ROS in a subprocess so crashes and shutdown hangs are observable."""
import json
import os
from pathlib import Path
import subprocess
import sys


def test_composed_native_stages():
    environment = dict(os.environ, ROS_DOMAIN_ID=str(100 + os.getpid() % 100))
    completed = subprocess.run(
        [sys.executable, str(Path(__file__).with_name("demo.py"))],
        capture_output=True, text=True, timeout=180, env=environment)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    # cppyy and ROS may print startup diagnostics before the JSON report.
    report = json.loads(completed.stdout[completed.stdout.index("{\n"):])
    assert report["direct_parity_cases"] == report["ros_parity_cases"] == 3
    assert report["native_progress_while_python_busy"]
    assert report["joined_shutdown"]
    stage = report["stages"]["affine"]
    assert stage["received"] == 67
    assert stage["processed"] + stage["coalesced"] == stage["received"]
    assert stage["coalesced"] > 0
    assert report["stages"]["sink"]["processed"] == stage["processed"]

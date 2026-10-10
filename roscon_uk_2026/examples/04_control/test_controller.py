"""Check the actual controller-manager path, not a substituted simulator."""
import json
from pathlib import Path
import subprocess
import sys


def test_real_mock_controller():
    result = subprocess.run([sys.executable, str(Path(__file__).with_name("controller.py")),
                             "--rate", "1000", "--seconds", "1.5"],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    row = json.loads(next(line[7:] for line in result.stdout.splitlines() if line.startswith("RESULT ")))
    assert row["cycles"] == 1500
    assert row["hardware"] == "mock_components/GenericSystem"
    assert row["tail_max_error_rad"] < .025
    # Rate is evidence, not a timing assertion on a shared development host.

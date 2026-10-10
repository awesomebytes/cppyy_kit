"""Bound the complete native test process, including failure probes."""
import os
from pathlib import Path
import subprocess
import sys


def main():
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD="1")
    try:
        proc = subprocess.run([sys.executable, "-m", "pytest", "-q",
                               str(Path(__file__).parent / "tests")],
                              env=env, timeout=90)
    except subprocess.TimeoutExpired:
        print("Worker checks exceeded the 90 second process deadline", file=sys.stderr)
        return 124
    return proc.returncode


if __name__ == "__main__":
    raise SystemExit(main())

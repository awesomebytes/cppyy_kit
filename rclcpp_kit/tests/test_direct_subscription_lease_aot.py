import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent


def format_output(process):
    return (
        f"exit={process.returncode}\n--- stdout ---\n{process.stdout}"
        f"\n--- stderr ---\n{process.stderr}"
    )


def test_direct_subscription_leases_interoperate_with_aot_publishers(tmp_path):
    build = subprocess.run(
        [
            str(HERE / "build_direct_subscription_lease_aot_peer.sh"),
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert build.returncode == 0, format_output(build)
    peer = tmp_path / "direct_subscription_lease_aot_peer"
    assert peer.is_file() and os.access(peer, os.X_OK)

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    helper = subprocess.run(
        [
            sys.executable,
            str(HERE / "_direct_subscription_lease_aot_helper.py"),
            str(peer),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=env,
    )
    assert helper.returncode == 0, format_output(helper)
    assert "DIRECT_SUBSCRIPTION_LEASE_AOT_OK" in helper.stdout
    assert "DIRECT_SUBSCRIPTION_LEASE_AOT_RETAINED_OK" in helper.stdout

import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent


def _format(proc):
    return (
        f"exit={proc.returncode}\n--- stdout ---\n{proc.stdout}"
        f"\n--- stderr ---\n{proc.stderr}"
    )


def test_managed_native_lifecycle_interoperates_with_aot_client(tmp_path):
    build = subprocess.run(
        [str(HERE / "build_native_lifecycle_aot_peer.sh"), str(tmp_path)],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert build.returncode == 0, _format(build)
    assert "warning:" not in (build.stdout + build.stderr).lower(), _format(build)
    peer = tmp_path / "native_lifecycle_aot_peer"
    assert peer.is_file() and os.access(peer, os.X_OK)

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    helper = subprocess.run(
        [
            sys.executable,
            str(HERE / "_native_lifecycle_aot_interop_helper.py"),
            str(peer),
        ],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=env,
    )
    assert helper.returncode == 0, _format(helper)
    assert "NATIVE_LIFECYCLE_AOT_INTEROP_OK" in helper.stdout
    assert "NATIVE_LIFECYCLE_AOT_TEARDOWN_OK" in helper.stdout

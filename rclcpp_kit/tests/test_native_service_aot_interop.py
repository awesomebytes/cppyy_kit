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


def test_managed_native_services_and_clients_interoperate_with_aot_peer(tmp_path):
    build = subprocess.run(
        [str(HERE / "build_native_service_aot_peer.sh"), str(tmp_path)],
        capture_output=True,
        text=True,
        timeout=180,
        check=False,
        env=os.environ.copy(),
    )
    assert build.returncode == 0, _format(build)
    peer = tmp_path / "native_service_aot_peer"
    assert peer.is_file() and os.access(peer, os.X_OK)

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(100 + os.getpid() % 100)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    results = {}
    for scenario in ("service", "python-service", "client"):
        helper = subprocess.run(
            [
                sys.executable,
                str(HERE / "_native_service_aot_interop_helper.py"),
                str(peer),
                scenario,
            ],
            capture_output=True,
            text=True,
            timeout=180,
            check=False,
            env=env,
        )
        assert helper.returncode == 0, _format(helper)
        assert f"NATIVE_AOT_INTEROP_OK scenario={scenario}" in helper.stdout
        results[scenario] = helper.stdout
    assert "MANAGED_SERVICE_TO_AOT_CLIENT_OK" in results["service"]
    assert "PYTHON_SERVICE_TO_AOT_CLIENT_OK" in results["python-service"]
    assert "MANAGED_CLIENT_TO_AOT_SERVICE_OK" in results["client"]

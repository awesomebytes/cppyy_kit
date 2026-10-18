"""Exercise the launch mutex with dummy processes; never start a simulator."""

import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


GUARD = Path(__file__).with_name("simulator_guard.sh")


def main():
    with tempfile.TemporaryDirectory(prefix="cppyy-kit-guard-") as cache:
        env = {
            **os.environ,
            "XDG_CACHE_HOME": cache,
            # Copied environment state alone must never bypass the lock.
            "CPPYY_KIT_SIMULATOR_OWNER": str(os.getpid()),
        }

        def invoke(*args):
            return subprocess.run(
                ["bash", str(GUARD), "--", *args],
                env=env, capture_output=True, text=True, timeout=5,
            )

        assert invoke("true").returncode == 0
        assert invoke("bash", str(GUARD), "--", "true").returncode == 0
        for stop_signal in (signal.SIGINT, signal.SIGTERM):
            ready = Path(cache) / "ready"
            ready.unlink(missing_ok=True)
            child = subprocess.Popen(
                ["bash", str(GUARD), "--", "bash", "-c",
                 'touch "$1"; exec sleep 30', "dummy", str(ready)],
                env=env,
            )
            try:
                deadline = time.monotonic() + 5
                while not ready.exists():
                    assert child.poll() is None, "dummy failed to start"
                    assert time.monotonic() < deadline, "startup timed out"
                    time.sleep(0.02)
                denied = invoke("true")
                assert denied.returncode == 75, denied
                assert "Another cppyy_kit simulator" in denied.stderr
                child.send_signal(stop_signal)
                assert child.wait(timeout=5) == -stop_signal
                assert invoke("true").returncode == 0
            finally:
                if child.poll() is None:
                    child.kill()
                    child.wait(timeout=5)
        descendant_pid_file = Path(cache) / "descendant"
        descendant_pid = None
        try:
            result = subprocess.run(
                ["bash", str(GUARD), "--", "bash", "-c",
                 'sleep 30 & echo $! > "$1"', "dummy", str(descendant_pid_file)],
                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=5,
            )
            assert result.returncode == 0
            descendant_pid = int(descendant_pid_file.read_text())
            assert invoke("true").returncode == 75, "child must retain lock"
            os.kill(descendant_pid, signal.SIGTERM)
            descendant_pid = None
            deadline = time.monotonic() + 5
            while invoke("true").returncode != 0:
                assert time.monotonic() < deadline, "child lock not released"
                time.sleep(0.02)
        finally:
            if descendant_pid is not None:
                os.kill(descendant_pid, signal.SIGKILL)
        print("PASS: normal release, nested use, overlap refusal, SIGINT/SIGTERM release, child lifetime")


if __name__ == "__main__":
    main()

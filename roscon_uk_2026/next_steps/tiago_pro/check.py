"""Run the simulation and smoke checks, then stop only our own process group."""
import os
from pathlib import Path
import signal
import subprocess
import time

root = Path(__file__).resolve().parent
os.chdir(root)
os.environ.setdefault("ROS_DOMAIN_ID", "77")
(root / "artifacts").mkdir(exist_ok=True)
with (root / "artifacts/simulator.log").open("w") as log:
    simulation = subprocess.Popen(
        ["ros2", "launch", str(root / "playground.launch.py"), "moveit:=false", "navigation:=false", "rviz:=false", "gui:=false"], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, pass_fds=(9,))
    try:
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            if simulation.poll() is not None:
                raise RuntimeError("Simulator exited; see artifacts/simulator.log")
            if "TIAGO_PRO_READY" in (root / "artifacts/simulator.log").read_text():
                break
            time.sleep(1)
        else:
            raise RuntimeError("Controller readiness timeout; see artifacts/simulator.log")
        subprocess.run(["python", "smoke.py"], check=True, timeout=50)
    finally:
        if simulation.poll() is None:
            os.killpg(simulation.pid, signal.SIGINT)
        try:
            simulation.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(simulation.pid, signal.SIGKILL)
            simulation.wait()

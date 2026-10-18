"""Start, validate and stop a dedicated simulator. No existing sim may use port 11363."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

root = Path(__file__).resolve().parent
log_path = root / 'smoke_simulation.log'
with log_path.open('w') as log:
    simulator = subprocess.Popen(['bash', 'run.sh'], cwd=root, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True,
                                 pass_fds=(9,))
    returncode = 1
    try:
        deadline = time.monotonic() + 100
        while time.monotonic() < deadline:
            if simulator.poll() is not None:
                raise RuntimeError(f'Simulator exited: {simulator.returncode}; see {log_path}')
            text = log_path.read_text(errors='replace')
            if text.count('Configured and activated') >= 8:
                break
            time.sleep(0.25)
        else:
            raise RuntimeError(f'Controllers not ready; see {log_path}')
        returncode = subprocess.run(['bash', 'check.sh'], cwd=root, timeout=90,
                                    pass_fds=(9,)).returncode
    finally:
        if simulator.poll() is None:
            os.killpg(simulator.pid, signal.SIGINT)
        forced = False
        try:
            simulator.wait(timeout=20)
        except subprocess.TimeoutExpired:
            forced = True
            os.killpg(simulator.pid, signal.SIGKILL)
            simulator.wait()
        shutdown = {'forced_kill': forced, 'launch_returncode': simulator.returncode}
        (root / 'shutdown.json').write_text(json.dumps(shutdown, indent=2) + '\n')
        print('Shutdown:', shutdown)
        if forced:
            returncode = 1
sys.exit(returncode)

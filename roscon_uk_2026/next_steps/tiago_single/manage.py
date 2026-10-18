"""Own and stop only this workspace's launch process group."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parent
STATE=ROOT/'.run.json'

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['run','view','stop'])
    args=parser.parse_args()
    if args.action=='stop':
        if not STATE.exists():
            print('No managed TIAGo single process.');return
        state=json.loads(STATE.read_text());pid=state['supervisor_pid']
        try:
            command=Path(f'/proc/{pid}/cmdline').read_bytes()
            if b'manage.py' not in command:
                raise RuntimeError(f'PID {pid} no longer belongs to this manager')
            os.kill(pid,signal.SIGINT)
        except ProcessLookupError:
            STATE.unlink(missing_ok=True)
            return
        deadline=time.monotonic()+30
        while STATE.exists() and time.monotonic()<deadline:
            time.sleep(.2)
        if STATE.exists():
            raise RuntimeError('Shutdown did not finish within 30 seconds; inspect the launch log')
        print('TIAGo single stopped.')
        return
    if STATE.exists():
        state=json.loads(STATE.read_text())
        if Path(f"/proc/{state['supervisor_pid']}").exists():
            raise RuntimeError('Managed single-arm simulation is already running; use pixi run stop')
    command=['bash',str(ROOT/'run.sh')]
    if args.action=='view':command+=['gui:=true']
    # Preserve the shared guard's actual lock descriptor in the launch child.
    inherited_fds=()
    if os.environ.get('CPPYY_KIT_SIMULATOR_OWNER'):
        try:
            os.fstat(9)
            inherited_fds=(9,)
        except OSError:
            pass
    process=subprocess.Popen(command,cwd=ROOT,start_new_session=True,pass_fds=inherited_fds)
    STATE.write_text(json.dumps({'supervisor_pid':os.getpid(),'launch_pid':process.pid}))
    stop_deadline=None
    def stop(signum,frame):
        nonlocal stop_deadline
        if stop_deadline is None:stop_deadline=time.monotonic()+15
        if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    try:
        while process.poll() is None:
            if stop_deadline is not None and time.monotonic()>stop_deadline:break
            time.sleep(.2)
    finally:
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGTERM)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        STATE.unlink(missing_ok=True)
    return process.returncode

if __name__=='__main__':sys.exit(main())

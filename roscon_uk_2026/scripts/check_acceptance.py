"""Run the original numerical acceptance contracts on current saved solutions.

Each subprocess stages the supplied test and solution under its expected import
name. This leaves the unfinished audience exercises and historical evidence intact.
The default checks require no ROS process, controller manager, camera, download,
or agent. Pass --control in the control environment for the actual mock rig.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CASES = (
    ("01_python", "task", "test_task.py", "python_baseline.py"),
    ("01_python", "task", "test_task.py", "python_native.py"),
    ("02_mcap", "analyze", "test_analyze.py", "mcap_native.py"),
    ("03_ros", "node", "test_node.py", "ros_native.py"),
    ("05_webcam", "tracker", "test_tracker.py", "webcam_native.py"),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--control", action="store_true",
                        help="only run the actual mock controller-manager check (control Pixi environment)")
    args = parser.parse_args()
    cases = (("04_control", "controller", "test_controller.py", "control_native.py"),) if args.control else CASES
    for folder, module, test, solution in cases:
        target = ROOT / "build/current-checkout/acceptance" / Path(solution).stem
        target.mkdir(parents=True, exist_ok=True)
        shutil.copy2(ROOT / "examples" / folder / test, target / test)
        shutil.copy2(ROOT / "solutions" / solution, target / (module + ".py"))
        print(f"Acceptance: {solution}", flush=True)
        result = subprocess.run([sys.executable, "-m", "pytest", test, "-q"], cwd=target,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        (target / "acceptance.txt").write_text(result.stdout)
        print(result.stdout, end="", flush=True)
        if result.returncode:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Run fresh Luna-high Codex sessions against the current checkout and preserve evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=["implement", "accelerate", "mcap", "ros", "control", "webcam"])
    parser.add_argument("--run", required=True)
    parser.add_argument("--baseline", help="completed implement run for acceleration")
    parser.add_argument("--timeout", type=float, default=300)
    args = parser.parse_args()
    if not args.run or Path(args.run).name != args.run or args.run in (".", ".."):
        parser.error("--run must be a new directory name")
    if args.baseline and (Path(args.baseline).name != args.baseline or args.baseline in (".", "..")):
        parser.error("--baseline must name a completed run directory")
    run = ROOT / "eval_runs" / args.run
    if run.exists():
        parser.error("run already exists; use a new name to preserve evidence")
    run.mkdir(parents=True)
    cases = {"implement": ("01_python", "task.py", "test_task.py", "default"),
             "accelerate": ("01_python", "task.py", "test_task.py", "default"),
             "mcap": ("02_mcap", "analyze.py", "test_analyze.py", "default"),
             "ros": ("03_ros", "node.py", "test_node.py", "ros"),
             "control": ("04_control", "controller.py", "test_controller.py", "control"),
             "webcam": ("05_webcam", "tracker.py", "test_tracker.py", "vision")}
    folder, task_name, test_name, environment = cases[args.phase]
    source = ROOT / "examples" / folder
    task_source = source / task_name
    if args.phase == "accelerate":
        if not args.baseline:
            parser.error("accelerate requires --baseline")
        task_source = ROOT / "eval_runs" / args.baseline / "task.py"
    shutil.copy2(task_source, run / task_name)
    shutil.copy2(source / test_name, run / test_name)
    # A fresh process uses the same explicit checkout activation as this rehearsal.
    python = ROOT / f".pixi/envs/{environment}/bin/python"
    if not python.exists():
        parser.error("install roscon_uk_2026/pixi.toml first")
    # Read Pixi's activation environment in memory. Do not write credentials or
    # the complete environment into the evaluation artifacts.
    activation = subprocess.run(
        ["pixi", "run", "--manifest-path", str(ROOT / "pixi.toml"), "-e", environment, "python",
         "-c", "import json,os; print(json.dumps(dict(os.environ)))"],
        capture_output=True, text=True, check=True)
    env = json.loads(activation.stdout)
    # Make fresh startup caches writable inside this run's sandbox and keep
    # compilation independent of artifacts produced by another rehearsal.
    env["XDG_CACHE_HOME"] = str(run / "build/xdg-cache")
    env["CPPYY_KIT_CACHE_DIR"] = str(run / "build/cppyy_kit_cache")
    package = subprocess.run(
        [str(python), "-c", "import cppyy_kit; print(cppyy_kit.__file__)"],
        env=env, capture_output=True, text=True, check=True)
    package_path = Path(package.stdout.strip()).resolve()
    if package_path != REPO / "cppyy_kit/__init__.py":
        parser.error(f"expected current checkout activation, found {package_path}")
    # Build cache compiler discovery uses the conda prefix.
    env["CONDA_PREFIX"] = str(python.parents[1])
    prompt_files = {"implement": "01_implement", "accelerate": "01_accelerate",
                    "mcap": "02_mcap", "ros": "03_ros", "control": "04_control", "webcam": "05_webcam"}
    prompt = (ROOT / f"prompts/{prompt_files[args.phase]}.txt").read_text()
    prompt += (f"\nPython: {python}\nCheck: {python} -m pytest {test_name} -q\n"
               f"cppyy_kit source: {package_path}. Preserve the provided Pixi activation.\n")
    if args.phase in ("accelerate", "mcap", "ros", "webcam"):
        prompt += f"Read {ROOT / 'guides/kernel.md'} before editing.\n"
    if args.phase == "mcap":
        prompt += f"Motion rule: {ROOT / 'examples/01_python/task.py'}\nRecording: {ROOT / 'data/hiw_pillow_episode_0002.mcap'}\n"
    if args.phase == "control":
        prompt += f"Read {REPO / 'control_kit/SKILL.md'} and {REPO / 'control_kit/demos/d02_python_controller.py'} before editing.\n"
    (run / "prompt.txt").write_text(prompt)
    cmd = ["codex", "exec", "--ignore-user-config", "--ephemeral", "--json",
           "--model", "gpt-6-luna", "-c", 'model_reasoning_effort="high"',
           "-c", 'approval_policy="never"', "--sandbox",
           "danger-full-access" if args.phase in ("control", "ros") else "workspace-write",
           "--add-dir", str(python.parents[1]), "--cd", str(run), "--output-last-message", str(run / "answer.txt"), "-"]
    before = hashlib.sha256((run / test_name).read_bytes()).hexdigest()
    started = time.perf_counter()
    timed_out = False
    with (run / "events.jsonl").open("w") as out, (run / "stderr.txt").open("w") as err:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=out, stderr=err,
                                text=True, env=env, start_new_session=True)
        try:
            proc.communicate(prompt, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            import signal
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=15)
            timed_out = True
    elapsed = time.perf_counter() - started
    check = subprocess.run([str(python), "-m", "pytest", test_name, "-q"],
                           cwd=run, env=env, capture_output=True, text=True, timeout=90)
    (run / "acceptance.txt").write_text(check.stdout + check.stderr)
    events = []
    for line in (run / "events.jsonl").read_text().splitlines():
        try:
            events.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    commands = [e["item"] for e in events if e.get("type") == "item.completed"
                and e.get("item", {}).get("type") == "command_execution"]
    usage = [e.get("usage") for e in events if e.get("type") == "turn.completed"]
    report = {"model": "gpt-6-luna", "reasoning_effort": "high",
              "phase": args.phase, "run": args.run, "baseline": args.baseline,
              "sandbox": "danger-full-access" if args.phase in ("control", "ros") else "workspace-write",
              "agent_seconds": elapsed, "agent_exit_code": proc.returncode,
              "timed_out": timed_out, "command_count": len(commands),
              "failed_commands": sum(c.get("exit_code", 0) != 0 for c in commands),
              "usage": usage, "acceptance_exit_code": check.returncode,
              "tests_unchanged": before == hashlib.sha256((run / test_name).read_bytes()).hexdigest(),
              "human_repairs": 0,
              "task_sha256": hashlib.sha256((run / task_name).read_bytes()).hexdigest()}
    report["package_mode"] = "checkout"
    report["package_source"] = str(package_path)
    report["pixi_lock_sha256"] = hashlib.sha256((ROOT / "pixi.lock").read_bytes()).hexdigest()
    report["cache_condition"] = "run-local compilation and auto-PCH caches; no shared rehearsal artifacts"
    (run / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if check.returncode == 0 and report["tests_unchanged"] and proc.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

"""Checkout-only fresh-session completion checks for selected next-steps skeletons."""
import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
NEXT = ROOT / "next_steps"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=["config", "buffers", "nanoflann", "tuning"])
    parser.add_argument("--run", required=True)
    parser.add_argument("--model", default="gpt-6.1-sol")
    parser.add_argument("--timeout", type=float, default=600)
    args = parser.parse_args()
    if not args.run or Path(args.run).name != args.run or args.run in (".", ".."):
        parser.error("--run must be a new directory name")
    scope = NEXT / ("reverse_core" if args.case == "config" else args.case)
    run = ROOT / "next_steps_evaluation" / args.run
    run.mkdir(parents=True, exist_ok=False)
    activation = subprocess.run(
        ["pixi", "run", "--frozen", "--manifest-path", str(scope / "pixi.toml"),
         "python", "-c", "import os,json; print(json.dumps(dict(os.environ)))"],
        capture_output=True, text=True, check=True)
    # Activation stays in memory. It is never copied into evidence.
    env = json.loads(activation.stdout)
    python = Path(env["CONDA_PREFIX"]) / "bin/python"
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    env["PYTHONPATH"] = os.pathsep.join([str(run), str(scope), str(REPO)])
    candidate = run / "candidate.py"
    text = (scope / "skeleton.py").read_text()
    if args.case == "config":
        # Adapt only the relative import so this skeleton runs outside its package.
        text = text.replace("from . import ",
                            "from roscon_uk_2026.next_steps.reverse_core import ")
        check = [str(python), "-m", "roscon_uk_2026.next_steps.reverse_core.check_exercise",
                 "--module", "candidate"]
        checks = [scope / "check_exercise.py"]
        guides = [scope / "GUIDE.md", scope / "CONTRACT.md"]
        instruction = "Implement only validate_and_export. Validate before native loading, adapt fields, export defaults."
    elif args.case == "buffers":
        env["BUFFERS_KERNEL_MODULE"] = "candidate"
        check = [str(python), "-m", "pytest", str(scope / "test_buffers.py"), "-q"]
        checks = [scope / "test_buffers.py"]
        guides = [scope / "GUIDE.md", scope / "AGENT_PROMPT.txt"]
        instruction = "Implement only the two missing C++ docstring bodies. Do not forward to the saved native kernels."
    elif args.case == "nanoflann":
        env["NN_SOLUTION"] = str(candidate)
        check = [str(python), str(scope / "acceptance.py")]
        checks = [scope / "acceptance.py"]
        guides = [scope / "README.md", scope / "LIBRARY_RECIPE.md", scope / "PROMPT.txt"]
        instruction = "Implement the retained native Index. Put new native sources under this evaluation directory's build/."
    else:
        check = [str(python), str(scope / "check_solution.py"), str(candidate)]
        checks = [scope / "check_solution.py", scope / "test_tuning.py"]
        guides = [scope / "README.md", scope / "PROMPT.md", NEXT / "reverse_core/CONTRACT.md"]
        instruction = "Implement only replay_trial, ask_and_tell, select_training; this evaluation checks those functions."
    candidate.write_text(text)
    before = {str(p.relative_to(REPO)): digest(p) for p in checks}
    initial = subprocess.run(check, cwd=run, env=env, capture_output=True, text=True, timeout=120)
    (run / "initial_failure.txt").write_text(initial.stdout + initial.stderr)
    if initial.returncode == 0:
        raise RuntimeError("unfinished skeleton unexpectedly passed")
    prompt = (
        f"Complete {candidate}. {instruction}\n"
        "Read the listed guides explicitly. Their original working-directory instructions are replaced by this one: "
        f"work only in {run}. Do not edit other files outside it or change acceptance checks.\n"
        "Do not read saved solutions, index.py, native.cpp/native.hpp implementation bodies, solution.py, "
        "or forward to saved missing-logic implementations. You may use documented scaffold helpers and installed "
        "dependency headers. Do not inspect or print credentials or the complete environment. "
        "Do not create worktrees, another checkout, install skills, change settings, or spawn agents.\n"
        + "\n".join("Read: " + str(p) for p in guides)
        + f"\nPython: {python}\n"
        + "Acceptance argv (run directly): " + json.dumps(check)
        + "\nPreserve contracts. Run the checks and fix actual logic failures without changing them. "
        "Report commands and results. This is an implementation completion exercise, not a speed guarantee.\n")
    (run / "prompt.txt").write_text(prompt)
    (run / "acceptance_command.json").write_text(json.dumps(check, indent=2) + "\n")
    cmd = ["codex", "exec", "--ignore-user-config", "--ephemeral", "--json",
           "--model", args.model, "-c", 'model_reasoning_effort="high"',
           "-c", 'approval_policy="never"', "--sandbox", "danger-full-access",
           "--cd", str(run), "--output-last-message", str(run / "answer.txt"), "-"]
    started = time.perf_counter()
    timed_out = False
    with (run / "events.jsonl").open("w") as events, (run / "stderr.txt").open("w") as errors:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=events, stderr=errors,
                                text=True, env=env, start_new_session=True)
        try:
            proc.communicate(prompt, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
    elapsed = time.perf_counter() - started
    checked = subprocess.run(check, cwd=run, env=env, capture_output=True, text=True, timeout=120)
    (run / "acceptance.txt").write_text(checked.stdout + checked.stderr)
    commands = []
    usage = []
    with (run / "events.jsonl").open() as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("type") == "item.completed" and event.get("item", {}).get("type") == "command_execution":
                commands.append(event["item"])
            if event.get("type") == "turn.completed":
                usage.append(event.get("usage"))
    report = dict(case=args.case, model=args.model, reasoning_effort="high",
                  agent_seconds=elapsed, agent_exit_code=proc.returncode, timed_out=timed_out,
                  acceptance_exit_code=checked.returncode, initial_exit_code=initial.returncode,
                  checks_unchanged=before == {str(p.relative_to(REPO)): digest(p) for p in checks},
                  check_sha256=before, candidate_sha256=digest(candidate), command_count=len(commands),
                  failed_commands=sum(c.get("exit_code", 0) != 0 for c in commands), usage=usage,
                  human_logic_repairs=0, context="checkout scaffolding with documented helpers; saved solutions excluded")
    (run / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    generated = run / "build" / "agent_native"
    if generated.is_dir():
        saved = run / "native_sources"
        saved.mkdir(exist_ok=True)
        native_hashes = {}
        for source in generated.iterdir():
            if source.suffix in (".h", ".hpp", ".cc", ".cpp"):
                target = saved / source.name
                shutil.copy2(source, target)
                native_hashes[source.name] = digest(target)
        (run / "native_sources.json").write_text(json.dumps(native_hashes, indent=2) + "\n")
    with (run / "events.jsonl").open("rb") as original, gzip.open(run / "events.jsonl.gz", "wb") as compressed:
        shutil.copyfileobj(original, compressed)
    (run / "events.jsonl").unlink()
    print(json.dumps(report, indent=2))
    return 0 if checked.returncode == 0 and report["checks_unchanged"] and proc.returncode == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

# Exact missing-logic prompt

Provide this text to a fresh agent. A user must read and invoke it explicitly.

```text
Work in the existing repository checkout. Do not create another checkout or
worktree. Read AGENTS.md and roscon_uk_2026/next_steps/extensions/GUIDE.md.
Use the existing root Pixi ompl environment and lock. Do not change manifests,
native.hpp, demo.py, check.py, test_extensions.py, the saved solution policy.py,
or files outside roscon_uk_2026/next_steps/extensions/build/. Do not install skills.

Copy policy_skeleton.py to build/candidate_policy.py. Implement only its missing
isValid method. Keep the existing factory, class constructor, and field names.
The native base is ompl::base::StateValidityChecker. Your override must be called
by OMPL's existing native planner, rather than by a Python simulation of planning.

Increment self.calls on every call. When it equals self.fail_at, raise
ValueError("scripted validity policy failure"). Return false for nonfinite or
out-of-bounds coordinates. Return true only for a state in the closed unit square
whose squared distance from (0.5 - self.bias_x, 0.5) is greater than 0.25 squared.
The circle boundary is invalid. Treat state as borrowed and do not retain it.

Run from the repository root:
CPPYY_KIT_NO_AUTOPCH=1 pixi run --locked -e ompl python
roscon_uk_2026/next_steps/extensions/demo.py --policy
roscon_uk_2026/next_steps/extensions/build/candidate_policy.py --output
roscon_uk_2026/next_steps/extensions/build/candidate_evidence.json

The command above is one shell command with whitespace between arguments.
Run the same command for --bias 0 and --bias -0.05. Check the independent checker
and preserve each output. Report changed files, commands, elapsed time, and any
failure or manual repair. Do not weaken the checks to accommodate a policy.
```

The saved solution is [policy.py](policy.py). The unfinished skeleton is tested
to ensure that it fails. No fresh-agent evaluation has been performed. Running
the saved solution is an implementation check, not an agent-completion result.

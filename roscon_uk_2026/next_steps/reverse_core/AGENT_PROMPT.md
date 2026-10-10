# Configuration exercise prompt

Use this exact prompt in a fresh agent session:

> Read `roscon_uk_2026/next_steps/reverse_core/GUIDE.md` and `CONTRACT.md`. Implement only `validate_and_export` in `skeleton.py`. Validate the supplied mapping with Pydantic before loading or constructing native objects. Populate the existing C++ `reverse_demo::Config` fields explicitly, without creating a generated replacement struct. Export all resolved defaults and supplied values in the driver's documented format. Keep the native header, implementation, acceptance checker, and manifest unchanged. Run the acceptance checker with `--module roscon_uk_2026.next_steps.reverse_core.skeleton` using this directory's Pixi manifest. Record commands, results, and any failure. Work in this checkout. Do not create worktrees or install skills.

[solution.py](solution.py) is a saved implementation. [check_exercise.py](check_exercise.py) is the independent checker. The checked-in skeleton intentionally raises `NotImplementedError`. No fresh-agent exercise result is claimed in this scope's results. The coordinator owns fresh-session evaluation.

# Work diary: 2026-10-03

- On branch `docs/first-use-walkthrough`, corrected first-use guidance in `README.md`, `mkdocs_site/index.md`, and `mkdocs_site/getting-started.md`: added prominent Getting Started links, clarified cppyy vs the base package vs domain kits, supplied a complete built-in `AlwaysSuccess` tree, and documented the Pixi install-and-run steps and supported platform scope. The inline array example now says NumPy is an additional dependency.
- Verified the literal Getting Started manifest and example against isolated published `cppyy-kit` and `ros-jazzy-bt-kit` 0.3.0 packages with `PYTHONPATH` unset. The tree status is numerically `2`, so the example prints the boolean check `True` for `status == bt.NodeStatus.SUCCESS`.
- Added a note that cppyy may prepare its compilation cache on first use and later runs can reuse it.
- `git diff --check` and `pixi run -e docs docs-build` passed. Verification covered the installed-package example and documentation build.

### Newcomer-facing performance wording

- Kept the user's README edits removing the blanket machine/workload caveat and the NumPy dependency disclaimer; mirrored those removals on the Home page.
- Reworded benchmark summaries to name the measured work and retain direct links: Python TF callback CPU relative to the C++ listener, and rclcpp initialization cold/warm timing. The acceleration guidance now explains where Python loops, callbacks, and data copies leave room for gains, with the webcam workload's NCC patch-tracking and ORB/RANSAC comparisons.
- Simplified the `nogil` explanation to describe the Python interpreter lock and independent jobs writing to separate slots. Kept the Getting Started note that cppyy may prepare its cache on first use and later runs can reuse it.
- `git diff --check` and `pixi run -e docs docs-build` passed for these prose edits.

### Documentation and docstring style review

- Rewrote rhetorical wording and em dashes across the repository documentation and docstrings in favor of concise technical descriptions. Preserved measurements, units, commands, behavior, and executable `@cpp` bodies.
- Removed exact-phrasing checks for public claims and added `AGENTS.md` with contributor writing rules.
- Corrected benchmark descriptions: the 1 kHz loop reports wakeup latency; NCC tracking permits documented tie differences; the retarget kernel has a small nonzero numerical difference. Updated renamed section links and obsolete source paths.
- Reviewed all 57 tracked documentation pages, 1,062 ordinary Python docstrings, and 2,107 Python comment lines. The final scan found no em dashes or flagged rhetorical phrases in authored prose. Added writing rules for future changes.
- Verification passed: `pixi run lint`, `git diff --check`, a strict MkDocs build with anchor validation set to warning, and the documentation integrity audit. Python AST comparisons preserved executable code and `@cpp` bodies, with four reviewed exceptions for explanatory output text. Benchmark table values and executable examples were preserved. The audit script and report remain in ignored `.pixi/docs-style-audit/`.
- Changes are local. No push was made.

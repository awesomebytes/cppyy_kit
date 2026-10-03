# Work diary — 2026-10-03

- On branch `docs/first-use-walkthrough`, corrected first-use guidance in `README.md`, `mkdocs_site/index.md`, and `mkdocs_site/getting-started.md`: added prominent Getting Started links, clarified cppyy vs the base package vs domain kits, supplied a complete built-in `AlwaysSuccess` tree, and documented the Pixi install-and-run steps and supported platform scope. The inline array example now says NumPy is an additional dependency.
- Verified the literal Getting Started manifest and example against isolated published `cppyy-kit` and `ros-jazzy-bt-kit` 0.3.0 packages with `PYTHONPATH` unset. The tree status is numerically `2`, so the example prints the boolean check `True` for `status == bt.NodeStatus.SUCCESS`.
- Added a note that cppyy may prepare its compilation cache on first use and later runs can reuse it; no universal timing is claimed.
- `git diff --check` and `pixi run -e docs docs-build` passed. Verification covered the installed-package example and documentation build.

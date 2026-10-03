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

### Reader tasks and first-use order

- The user identified that the prior review left packaging details ahead of useful installation and examples. Nine Luna reviews covered reader priorities across the documentation, including an independent review of the revised entry pages. I selected and checked the changes before committing them.
- Rebuilt README, Home, and Getting Started around two equally prominent uses: calling BehaviorTree.CPP and writing an inline C++ function. Each example has setup, run instructions, and expected output. Removed package counts, noarch details, bridge internals, and distribution terminology from those entry pages. Package names remain in the installation guide; build details remain in the packaging guide.
- Put existing Python examples before C++ build comparisons in the kit usage pages. Marked source-checkout demo commands, corrected stale paths and the control initialization call, and fixed a retarget `--follow` command that conflicted with the task's built-in `--replay` option.
- Moved tutorials ahead of advanced references in the site navigation, added the WBC usage/API pages to the site mirror, and gave contributor/design pages explicit audience context. Added these reader-priority rules to `AGENTS.md`.
- Ran both literal README examples in the isolated installed-package environment with the repository `PYTHONPATH` unset: `tree.py` printed `True`; `kernel.py` printed `14.0`. The strict docs build with broken anchors treated as warnings, the local Markdown link audit, and `git diff --check` passed. Numerical results in the evidence reports were not rerun or changed.
- Changes are local. No push was made.

### Inline C++ example formatting

- Reformatted the repeated `sum_sq` documentation example with a triple-quoted multiline docstring and an indented C++ loop. Documentation only; no code was executed.

### Numeric `@cpp` argument annotations and 0.4.0 metadata

- Updated the inline C++ examples in the README and site entry pages to use `NDArray[np.float32]`, preserving the multiline C++ body and expected output. Put a source-checkout availability note before the install steps, with the checkout commands for both examples; published 0.3.x packages do not support the numeric API.
- Integrated the numeric argument reference into `docs/COMMON_PATTERNS.md` §26: accepted annotation forms and type mappings, inference, specialization cache reuse, sequence conversion/copy/lifetime behavior, direct array borrowing and error conditions, flat pointer/element-count semantics for multidimensional contiguous arrays, plus return annotation and `cpp.arr` notes.
- Bumped the suite metadata to 0.4.0 with `recipe/bump_version.sh`; made the script explicitly skip the upstream `cppyy` package. Added NumPy to the base package runtime dependencies and reset the changed base/rclcpp recipe build numbers to zero. Updated release SBOM versions, build strings and the base package dependency list, rclcpp build-proof helper pins, recipe version guidance, and the release-version test expectation.

### Numeric annotation documentation and 0.4.0 validation

- The strict MkDocs Python-API build passed with `strict=True` and link-anchor validation set to WARNING.
- `pixi run -e default pytest -q cppyy_kit/tests/test_release_version.py` passed: **8 passed**. `pixi run -e default python scripts/verify_release_version.py v0.4.0` printed `RELEASE_VERSION_OK 0.4.0`.
- Extracted the preferred README kernel snippet verbatim into ignored `build/docs_numeric_kernel.py` and ran it from the source checkout with `pixi run python build/docs_numeric_kernel.py`; it printed `14.0`.

### Numeric annotation implementation and final validation

- Normalized supported Python and NumPy annotations into concrete C++ numeric signatures. Omitted input annotations and bare arrays infer supported types; typed sequences copy into call-scoped native storage, while compatible NumPy arrays are borrowed. Fixed cached specialization symbol registration and preserved existing string-form C++ `float` annotations. An independent review was completed.
- Final core regression passed: `pixi run -e default pytest -q cppyy_kit/tests examples/parallel_demo/test_parallel.py --durations=10` reported **180 passed, 49 skipped in 7.21s**. The skips were optional BehaviorTree.CPP, Pydantic v2, and opt-in Auto-PCH end-to-end coverage. The focused test run reported **93 passed in 3.81s** with no new skips.
- Full Pixi lint passed after test-style fixes. The strict MkDocs build, release-version test and `v0.4.0` verification, README kernel example, and scoped diff check also passed (details above).
- Work remains local on `feat/numeric-cpp-annotations`; no push, ARM tests, or release publication occurred.

### Numeric annotation final audit and installed-package proof coverage

- Final audit found stale standalone install commands that could resolve pre-0.4.0 packages and a packaging table that omitted NumPy; both documentation gaps were corrected.
- Updated the `prove_all.sh` and `prove_rclcpp.sh` installed-package lanes to expect base recipe build number 0 and exercise the installed `@cpp` API with an annotated float32 array, typed float sequence, and inferred float32/float64 arrays. The proofs verify that `cppyy_kit` imports from the installed prefix; `PYTHONPATH` is cleared for the throwaway environments.
- Added a release test that reads the recipe build number and checks both proof scripts against it. Validation passed: `pixi run -m cppyy_kit pytest cppyy_kit/cppyy_kit/tests/test_release_version.py -q` (**9 passed**) and `bash -n recipe/prove_all.sh recipe/prove_rclcpp.sh`. Installed artifact proofs were not run locally; CI will provide that evidence.

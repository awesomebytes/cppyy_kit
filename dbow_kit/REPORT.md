# dbow_kit — REPORT (seed)

> Seed. dbow_kit and cv_kit are the **vision pair**; their shared evidence
> (probe matrix, ingest/ORB/query benchmarks, loop precision/recall) currently
> lives in [`cv_kit/REPORT.md`](../cv_kit/REPORT.md) and the tutorial
> [`docs/tutorials/vision_loop_closure.md`](../docs/tutorials/vision_loop_closure.md).
> A dbow-specific report is split out during a later documentation pass.

## Status

- **DBoW2 via cppyy: works.** Vocabulary train/save/load and database add/query
  are driven from Python; descriptors stay as C++ `cv::Mat`. Vendored + compiled
  from source (no conda-forge package, no Python binding) via
  `dbow_kit/cpp/build_dbow2.py` -> `build/vendor/libDBoW2.so`.
- **Golden test:** `cv_kit/tests/test_vision_loop.py` trains a small vocabulary
  on the deterministic synthetic sequence (zero download) and asserts the
  recorded loop-closure baseline -- the differential contract for the pipeline.
- **Real vocabulary:** the 145 MB ORBvoc.txt loads and is transparently cached to
  a `.dbow2` binary (~1 s reloads thereafter).

See `cv_kit/REPORT.md` for the measured numbers.

# dbow_kit report

Most DBoW2 measurements and probe results are documented in
[`cv_kit/REPORT.md`](../cv_kit/REPORT.md) and the
[vision loop-closure tutorial](../docs/tutorials/vision_loop_closure.md).

## Status

- DBoW2 vocabulary training, save/load, database insertion, and queries work from
  Python through cppyy. Descriptor data stays in C++ `cv::Mat` objects.
- `dbow_kit/cpp/build_dbow2.py` vendors and compiles DBoW2 from source into
  `build/vendor/libDBoW2.so`. DBoW2 has no conda-forge package or Python binding.
- `cv_kit/tests/test_vision_loop.py` trains a vocabulary on a deterministic
  synthetic sequence and checks loop-closure results. The test needs no download.
- The 145 MB ORBvoc text file loads and is cached as a `.dbow2` binary. Reloads
  take about 0.4 seconds.

See [`cv_kit/REPORT.md`](../cv_kit/REPORT.md) for measured results.

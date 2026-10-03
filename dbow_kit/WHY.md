# Why dbow_kit exists

DBoW2 is a bag-of-binary-words library used for place recognition in ORB-SLAM. It
has no Python binding and is not packaged on conda-forge. Using it from Python
would normally require writing and maintaining a binding.

`dbow_kit` uses cppyy to compile DBoW2's headers and call its ORB API from Python.
It vendors the source and builds a small shared library with `build-dbow2`.
Descriptors from `cv_kit` arrive as C++ `cv::Mat` objects and stay in C++ during
the DBoW2 query. Python controls the pipeline. See [`REPORT.md`](REPORT.md) and
the [vision loop-closure tutorial](../docs/tutorials/vision_loop_closure.md).

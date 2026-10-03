# Why cv_kit exists

OpenCV has a mature Python binding in `cv2`. `cv_kit` is useful when a pipeline
needs to keep image data in C++:

- A ROS 2 subscription delivers a C++ `sensor_msgs::msg::Image`. `cv_kit` wraps
  its `data` buffer as a `cv::Mat` without copying it.
- C++ `cv::ORB` extracts features from that Mat. The descriptor Mat can then go
  to DBoW2 through [`dbow_kit`](../dbow_kit/WHY.md), still in C++.
- Python controls the pipeline, while the image and descriptors stay in C++.

This avoids converting the ROS image to NumPy before using OpenCV. A CUDA-enabled
OpenCV build can also be selected through `create_orb(..., use_cuda=...)` without
changing the rest of the pipeline. See [`REPORT.md`](REPORT.md) and the
[vision loop-closure tutorial](../docs/tutorials/vision_loop_closure.md) for details.

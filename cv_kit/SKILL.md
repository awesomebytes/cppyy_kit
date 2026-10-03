# cv_kit

Use OpenCV's C++ API from Python through cppyy. `cv_kit` is useful when a Python
pipeline needs to keep image pixels and later results in C++, including a ROS 2
image callback or a custom C++ vision operation. Ordinary `cv2` remains a good
choice when NumPy and OpenCV's Python API fit the pipeline.

## Install and try the demo

The published Pixi package is `ros-jazzy-cv-kit`. For package setup and the
supported Pixi environment, start with the [Getting Started guide](https://awesomebytes.github.io/cppyy_kit/getting-started/).

From this repository checkout, the existing feature demo needs the `vision`
environment:

```bash
pixi install -e vision
pixi run -e vision demo-vision-features
```

The demo reports `cv::ORB (CPU)` with the default OpenCV build, about 1,000
keypoints per frame, and an `N x 32` descriptor matrix: one 32-byte binary ORB
descriptor per row. It opens a Rerun
viewer when a display is available; headless runs save a recording under
`build/vision/`.

## API shapes

- `numpy_to_mat(frame)` takes an existing NumPy image frame, such as the frames
  used by `demo-vision-features`, and returns a C++ `cv::Mat` view.
- `msg_to_mat(msg)` takes a C++ `sensor_msgs::msg::Image` from an
  `rclcpp_kit` subscription callback. It views the message's pixel buffer without
  copying. Keep the Mat within the callback, while the message owns that buffer.
- `create_orb(nfeatures)` creates the C++ ORB detector; its
  `detect_and_compute(mat)` result contains keypoints and a descriptor Mat.

The complete ROS-to-ORB-to-DBoW2 example is the [vision loop-closure
tutorial](../docs/tutorials/vision_loop_closure.md). For GPU OpenCV, see
[`CUDA_OPENCV.md`](CUDA_OPENCV.md). Probe and benchmark details are in
[`REPORT.md`](REPORT.md).

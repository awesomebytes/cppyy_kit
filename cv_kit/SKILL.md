# cv_kit API reference

Use OpenCV's C++ API from Python through cppyy. `cv_kit` is useful when a Python
pipeline needs to keep image pixels and later results in C++, including a ROS 2
image callback or a custom C++ vision operation. Ordinary `cv2` remains a good
choice when NumPy and OpenCV's Python API fit the pipeline.

`cv = cv_kit.bringup_cv()` returns the C++ `cv` namespace, including `Mat`,
`cvtColor`, and feature detectors. The kit adds image-buffer conversions and an
ORB helper.

In a Pixi project configured with the channels in
[Getting Started](https://awesomebytes.github.io/cppyy_kit/getting-started/), install
`pixi add ros-jazzy-cv-kit numpy` and run scripts with
`pixi run python your_script.py`. ROS image examples additionally need
`ros-jazzy-rclcpp-kit` and `ros-jazzy-sensor-msgs`. See the
[kit overview](https://awesomebytes.github.io/cppyy_kit/cv_kit/WHY/).

## Repository demo

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

- `numpy_to_mat(frame)` accepts `(H,W)` or `(H,W,3)` `uint8` arrays. It makes a
  contiguous copy when needed, returns a C++ `cv::Mat` view, and retains the
  backing array. Do not resize or reallocate that array while using the Mat.
- `msg_to_mat(msg)` takes a C++ `sensor_msgs::msg::Image` from an
  `rclcpp_kit` subscription callback. It views the message's pixel buffer without
  copying and retains the owning message. Do not resize its `data` vector while
  using the Mat. Dimensions, row step, and payload length must agree. Supported
  16-bit encodings must have the host's byte order. Invalid layouts and unsupported
  encodings raise `ValueError`.
- `mat_to_numpy(mat, copy=True)` accepts two-dimensional unsigned 8-bit Mats.
  It returns `(H,W)` for one channel or `(H,W,C)` for multiple channels. The default
  copies pixels; `copy=False` respects the row step and retains the Mat and its
  owners. Do not reallocate Mat storage while using a view. Other depths, including
  a `mono16` Mat returned by `msg_to_mat`, raise `ValueError`.
- `create_orb(nfeatures)` creates the C++ ORB detector; its
  `detect_and_compute(mat)` result contains keypoints and a descriptor Mat.

The complete ROS-to-ORB-to-DBoW2 example is the [vision loop-closure
tutorial](https://awesomebytes.github.io/cppyy_kit/docs/tutorials/vision_loop_closure/).
For GPU OpenCV, see the
[CUDA guide](https://awesomebytes.github.io/cppyy_kit/cv_kit/CUDA_OPENCV/).
Probe and benchmark details are in the
[report](https://awesomebytes.github.io/cppyy_kit/cv_kit/REPORT/).

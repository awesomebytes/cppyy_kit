# cv_kit

Use OpenCV's C++ API from Python through cppyy. `cv_kit` can wrap a C++ ROS 2
`sensor_msgs::msg::Image` buffer as a `cv::Mat` without copying the image data. It
also provides C++ ORB feature extraction. Use it with [`dbow_kit`](../dbow_kit/WHY.md)
for place recognition and loop closure.

## Start

```python
import cv_kit

cv = cv_kit.bringup_cv()          # JIT-includes opencv4 and loads libopencv_*.so
orb = cv_kit.create_orb(500)      # Uses CUDA when available; otherwise uses CPU ORB
mat = cv_kit.msg_to_mat(image)    # View of the message data; does not copy pixels
```

`msg_to_mat` and `mat_to_numpy(copy=False)` return views into C++ or message
storage. Keep the owner alive while using the view. For a ROS message, use the Mat
inside the callback that owns the message.

See [`REPORT.md`](REPORT.md) for probes and benchmarks, and
[`CUDA_OPENCV.md`](CUDA_OPENCV.md) for the CUDA OpenCV setup. The full pipeline is
described in the [vision loop-closure tutorial](../docs/tutorials/vision_loop_closure.md).

The demos are in `cv_kit/demos/`. The CUDA build script is
`cv_kit/cpp/build_opencv_cuda.py`.

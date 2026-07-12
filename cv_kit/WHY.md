# cv_kit — WHY (seed)

> Seed. The full human pitch (side-by-side + what you gain) is folded in with
> a later SKILL.md pass. See [`REPORT.md`](REPORT.md) for the measured evidence
> and the tutorial [`docs/tutorials/vision_loop_closure.md`](../docs/tutorials/vision_loop_closure.md)
> for the end-to-end story.

OpenCV already ships a mature Python binding (`cv2`), so cv_kit is **not** an
"impossible → possible" kit like pcl_kit. Its reason to exist is **composition
without copies**:

- A ROS 2 subscription delivers a **C++** `sensor_msgs::msg::Image`. cv_kit wraps
  its `data` buffer as a `cv::Mat` with **no copy** (the Mat's storage *is* the
  message buffer).
- C++ `cv::ORB` runs on that Mat; the resulting descriptor Mat is handed to DBoW2
  (`dbow_kit`) still in C++.
- The entire front-end (ingest → features → place recognition) lives in one C++
  address space. Python only orchestrates.

With `cv2` you serialize/copy at every hop (message → numpy → cv2 → back). cv_kit
keeps the pixels in C++ and lets you drop in a CUDA-enabled OpenCV build with a
single branch point (`create_orb(..., use_cuda=…)`) and no other code change.

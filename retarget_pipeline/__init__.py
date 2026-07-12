"""
retarget_pipeline (M6f) -- a minimal-code human-demonstration capture rig.

Two processes, coupled by a landmark stream file:

  * Process A -- ``perceive.py`` (pixi env ``pipeline``): webcam -> MediaPipe
    HolisticLandmarker (body + hands) -> a landmark stream (record/replay) -> TF
    frames via ``rclcpp_kit`` -> live Rerun.
  * Process B -- ``retarget.py`` (pixi env ``wbc``): the landmark stream -> a
    whole-body CLIK retarget onto a humanoid (Talos, or G1) via pinocchio -> a
    Talos configuration per frame -> Rerun + a recorded "policy-kickstart" dataset.

The two live in separate pixi environments on purpose: pinocchio's conda stack pins
libboost 1.86 and the robostack ROS stack pins 1.90, so the retarget half cannot
share a process (or env) with ROS (see docs/wbc/REPORT.md). The stream file is the
seam -- tailable for live coupling, replayable for CI. Both processes log to one
Rerun viewer.

Only ``landmark_stream`` (the record/replay contract) is imported by both envs, so
it depends on nothing but stdlib + numpy.
"""

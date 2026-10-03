"""Perception and humanoid retargeting pipeline.

Process A, `perceive.py`, detects body and hand landmarks with MediaPipe, writes an
optional stream, publishes TF through rclcpp_kit, and logs to Rerun. Process B,
`retarget.py`, reads the stream and computes humanoid configurations with pinocchio.
It also logs to Rerun and can write a policy-kickstart dataset.

The offline workflow uses separate pixi environments because the pinocchio and ROS
packages in `wbc` require incompatible Boost versions. The landmark stream is the
interface between processes. It supports live tailing and replay in tests. Both
processes can log to the same Rerun viewer.

`landmark_stream` is imported by both environments. It uses only the standard library
and NumPy.
"""

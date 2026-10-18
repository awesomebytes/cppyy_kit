"""Check Stretch 4 control, three image streams, IMU and lidar without ROS."""
import json
import os
from pathlib import Path
import time

os.environ.setdefault("MUJOCO_GL", "egl")


def main():
    import mujoco
    import numpy as np
    from stretch4_mujoco import Stretch4MujocoSimulator
    from stretch4_mujoco.enums.stretch_cameras import StretchCameras

    artifacts = Path(__file__).parent / "artifacts"
    artifacts.mkdir(exist_ok=True)
    sim = Stretch4MujocoSimulator(
        cameras_to_use=[StretchCameras.cam_gripper_se4_left_rgb,
                        StretchCameras.cam_gripper_se4_stereo_depth,
                        StretchCameras.cam_nav_rgb_se4_center], camera_hz=5
    )
    try:
        sim.start(headless=True)
        assert sim.is_running(), "Upstream simulation server exited"
        time.sleep(1)
        initial = sim.pull_status()
        target = max(0.2, initial.lift.pos - 0.1)
        sim.lift.move_to(target)
        arm_target = min(0.25, initial.arm.pos + 0.1)
        sim.arm.move_to(arm_target)
        sim.base.set_velocity(vx_m=0.08, vy_m=0.0, w_r=0.0)
        time.sleep(3)
        sim.base.set_velocity(vx_m=0.0, vy_m=0.0, w_r=0.0)
        final = sim.pull_status()
        deadline = time.monotonic() + 8
        while (abs(final.arm.pos-arm_target) > 0.015 or abs(final.lift.pos-target) > 0.015) and time.monotonic() < deadline:
            time.sleep(0.1)
            final = sim.pull_status()
        camera = sim.pull_camera_data()
        image = camera.cam_gripper_se4_left_rgb
        head_image = camera.cam_nav_rgb_se4_center
        depth = camera.cam_gripper_se4_stereo_depth
        sensors = sim.pull_sensor_data()
        lidar = sim.pull_lidar_points()
        assert final.time > initial.time
        assert np.isfinite([final.base.x, final.base.y, final.lift.pos]).all()
        base_distance = float(np.hypot(final.base.x-initial.base.x, final.base.y-initial.base.y))
        assert base_distance > 0.01, f"Base did not move: {base_distance}"
        assert abs(final.lift.pos-target) < 0.015
        assert abs(final.arm.pos-arm_target) < 0.015
        assert final.arm.pos-initial.arm.pos > 0.05
        assert image is not None and image.ndim == 3
        assert head_image is not None and head_image.ndim == 3
        assert depth is not None and depth.ndim == 2
        finite_depth = depth[np.isfinite(depth) & (depth > 0)]
        assert finite_depth.size > 0, "No finite positive depth"
        for field in (sensors.base_gyro, sensors.base_imu):
            assert field is not None and np.asarray(field).shape == (3,)
            assert np.isfinite(field).all()
        for points in lidar.values():
            assert points.ndim == 2 and points.shape[1] == 3
            assert len(points) > 0 and np.isfinite(points).all()
        assert set(lidar) == {"left", "right"}
        from PIL import Image
        Image.fromarray(image).save(artifacts / "gripper_rgb.png")
        Image.fromarray(head_image).save(artifacts / "head_rgb.png")
        np.save(artifacts / "gripper_depth.npy", depth)
        report = {
            "mujoco": mujoco.__version__,
            "initial": initial.to_dict(), "final": final.to_dict(),
            "lift_target_m": target, "base_distance_m": base_distance,
            "arm_target_m": arm_target,
            "camera_shape": list(image.shape), "camera_time_s": camera.time,
            "head_camera_shape": list(head_image.shape),
            "depth_shape": list(depth.shape),
            "positive_depth_min_max_m": [float(finite_depth.min()), float(finite_depth.max())],
            "gyro": sensors.base_gyro.tolist(), "accelerometer": sensors.base_imu.tolist(),
            "lidar_point_counts": {name: len(points) for name, points in lidar.items()},
        }
        (artifacts / "result.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
    finally:
        sim.stop()


if __name__ == "__main__":
    main()

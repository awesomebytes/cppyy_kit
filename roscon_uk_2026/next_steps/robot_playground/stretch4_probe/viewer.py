"""Show the stock Stretch 4 playground until Ctrl+C or the viewer closes."""
import os
import time

os.environ["MUJOCO_GL"] = "glfw"
os.environ.pop("PYOPENGL_PLATFORM", None)


def main():
    from stretch4_mujoco import Stretch4MujocoSimulator
    from stretch4_mujoco.enums.stretch_cameras import StretchCameras

    sim = Stretch4MujocoSimulator(
        cameras_to_use=[StretchCameras.cam_gripper_se4_left_rgb,
                        StretchCameras.cam_gripper_se4_stereo_depth,
                        StretchCameras.cam_nav_rgb_se4_center], camera_hz=5
    )
    try:
        sim.start(headless=False, show_viewer_ui=True, viewer_look_at_body="stretch4")
        print("Stretch 4 ready. Drag to orbit; scroll to zoom. Ctrl+C stops.", flush=True)
        while sim.is_running():
            time.sleep(0.2)
    finally:
        sim.stop()


if __name__ == "__main__":
    main()

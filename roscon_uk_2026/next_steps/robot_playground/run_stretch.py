"""Run the stock Stretch simulator and verify motion and sensor delivery."""

import argparse
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=0,
                        help="Viewer lifetime after checks; 0 runs until closed.")
    args = parser.parse_args()
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

    import mujoco
    import numpy as np
    from PIL import Image
    from stretch_mujoco import StretchMujocoSimulator
    from stretch_mujoco.enums.stretch_cameras import StretchCameras

    output = Path(__file__).parent / "artifacts"
    output.mkdir(exist_ok=True)
    sim = StretchMujocoSimulator(camera_hz=10, cameras_to_use=StretchCameras.all())
    started = time.monotonic()
    try:
        sim.start(headless=args.headless, show_viewer_ui=True)
        if not sim.is_running():
            raise RuntimeError("Simulator failed during startup")
        before = sim.pull_status()
        previous_camera_time = sim.pull_camera_data().time
        previous_sensor_time = sim.pull_sensor_data().time
        # Validate one command at a time against observed state. Upstream shares
        # commands through a read-modify-write proxy, without acknowledgements.
        for joint, target in (("lift", 0.8), ("arm", 0.15)):
            sim.move_to(joint, target)
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                status = sim.pull_status()
                if abs(getattr(status, joint).pos - target) < 0.03:
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError(f"Pose timeout: {joint}={getattr(status, joint).pos}")
        sim.set_base_velocity(0.08, 0.0)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            status = sim.pull_status()
            if np.hypot(status.base.x - before.base.x, status.base.y - before.base.y) > 0.08:
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("Base did not move 0.08 m within 20 seconds")
        sim.set_base_velocity(0.0, 0.0)
        cameras = sim.pull_camera_data()
        sensors = sim.pull_sensor_data()
        if cameras.time <= previous_camera_time or sensors.time <= previous_sensor_time:
            raise RuntimeError("Sensor timestamps did not advance during motion")
        camera_info = {}
        for camera in StretchCameras.all():
            data = cameras.get_camera_data(camera, auto_correct_rgb=False)
            if not np.isfinite(data).all():
                raise RuntimeError(f"Nonfinite camera data: {camera.name}")
            if camera.is_depth and not (data > 0).any():
                raise RuntimeError(f"Depth camera has no positive ranges: {camera.name}")
            camera_info[camera.name] = {
                "shape": list(data.shape), "dtype": str(data.dtype),
                "min": float(data.min()), "max": float(data.max()),
                "positive_fraction": float(np.mean(data > 0)) if camera.is_depth else None,
            }
            if not camera.is_depth:
                Image.fromarray(data).save(output / f"{camera.name}.png")
            else:
                np.save(output / f"{camera.name}.npy", data)
        delta = float(np.hypot(status.base.x - before.base.x, status.base.y - before.base.y))
        if delta < 0.02:
            raise RuntimeError(f"Base moved only {delta:.4f} m")
        sensor_info = {}
        for name in ("base_gyro", "base_imu", "lidar"):
            data = np.asarray(getattr(sensors, name))
            if data.size == 0 or not np.isfinite(data).all():
                raise RuntimeError(f"Invalid sensor output: {name}")
            sensor_info[name] = {"shape": list(data.shape), "values": data.tolist() if data.size < 10 else None}
        report = {
            "mujoco_version": mujoco.__version__, "headless": args.headless,
            "startup_and_checks_wall_seconds": time.monotonic() - started,
            "simulation_time_seconds": status.time,
            "base_displacement_metres": delta,
            "arm_position_metres": status.arm.pos,
            "lift_position_metres": status.lift.pos,
            "camera_timestamp": cameras.time, "sensor_timestamp": sensors.time,
            "camera_timestamp_advanced": cameras.time > previous_camera_time,
            "sensor_timestamp_advanced": sensors.time > previous_sensor_time,
            "cameras": camera_info, "sensors": sensor_info,
        }
        mode = "headless" if args.headless else "gui"
        (output / f"validation_{mode}.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
        print(f"Checks passed. Sensor captures: {output}", flush=True)
        deadline = time.monotonic() + args.seconds if args.seconds else float("inf")
        if args.headless and not args.seconds:
            deadline = time.monotonic()
        while sim.is_running() and time.monotonic() < deadline:
            time.sleep(0.2)
    finally:
        sim.stop()
        # Upstream stop() joins its simulator process but leaves its Manager alive.
        # Private API confined to this checkout-only validation harness.
        sim._manager.shutdown()


if __name__ == "__main__":
    main()

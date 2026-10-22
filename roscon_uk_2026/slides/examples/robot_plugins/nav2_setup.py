"""Call before starting controller_server with the resulting parameters."""
import json
import os
from pathlib import Path

from nav2_kit.python_critic import install_python_critic_plugin


def configure_forward_navigation(controller_parameters, repo, artifacts):
    repo = Path(repo).resolve()
    prefix = install_python_critic_plugin(Path(artifacts) / "python_critic")
    os.environ["AMENT_PREFIX_PATH"] = (
        str(prefix) + ":" + os.environ.get("AMENT_PREFIX_PATH", "")
    )
    follow = controller_parameters["FollowPath"]
    follow["critics"] += ["ObstacleFootprint", "ForwardCamera"]
    follow.update({
        "min_vel_x": 0.0, "min_vel_y": -0.2, "max_vel_y": 0.2,
        "ObstacleFootprint.scale": 0.02,
        "ForwardCamera.class": "nav2_kit_python::PythonCritic",
        "ForwardCamera.scale": 1.0,
        "ForwardCamera.python_module": "forward_camera_policy",
        "ForwardCamera.python_class": "ForwardCameraPolicy",
        "ForwardCamera.python_path": str(repo / "nav2_kit/demos"),
        "ForwardCamera.python_options": json.dumps({
            "camera_half_angle": 0.5236, "goal_tolerance": 0.25,
            "heading_weight": 80.0, "translation_epsilon": 0.0001,
        }),
    })

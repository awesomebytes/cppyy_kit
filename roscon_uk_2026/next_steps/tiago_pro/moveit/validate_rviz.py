"""Reject invalid saved MoveIt topics before the native RViz plugin loads."""
import yaml
from rclpy.validate_topic_name import validate_topic_name


def validate(path):
    config = yaml.safe_load(path.read_text())
    for display in config.get("Visualization Manager", {}).get("Displays", []):
        if display.get("Class") != "moveit_rviz_plugin/MotionPlanning":
            continue
        properties = {
            "Planning Scene Topic": display.get("Planning Scene Topic", "/monitored_planning_scene"),
            "Planned Path/Trajectory Topic": display.get("Planned Path", {}).get("Trajectory Topic", "/display_planned_path"),
        }
        for name, topic in properties.items():
            if not isinstance(topic, str) or not topic:
                raise ValueError(f"Invalid RViz {name}: expected a nonempty topic string")
            try:
                validate_topic_name(topic)
            except Exception as error:
                raise ValueError(f"Invalid RViz {name} {topic!r}: {error}") from error

"""Print explicit rehearsal guidance without importing cppyy or installing skills."""
import argparse
from pathlib import Path


def main():
    root = Path(__file__).resolve().parents[1]
    repo = root.parent
    guides = {
        "kernel": root / "guides/kernel.md",
        "rclcpp": repo / "rclcpp_kit/SKILL.md",
        "control": repo / "control_kit/SKILL.md",
        "vision": repo / "cv_kit/SKILL.md",
        "pcl": repo / "pcl_kit/SKILL.md",
        "moveit": repo / "moveit_kit/SKILL.md",
    }
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", nargs="?", choices=guides)
    parser.add_argument("--path", action="store_true")
    args = parser.parse_args()
    if args.topic:
        path = guides[args.topic]
        print(path if args.path else path.read_text())
    else:
        print("Rehearsal guide locations. Read the topic relevant to your task.")
        for topic, path in guides.items():
            print(f"{topic}: {path}")
        print("No skills or agent configuration have been installed.")
        print("These aliases use this checkout. Current 0.4 builds also package guides.")
        print("Core guidance: python -m cppyy_kit guide accelerate")
        print("Kit API: python -m cppyy_kit guide control_kit api")


if __name__ == "__main__":
    main()

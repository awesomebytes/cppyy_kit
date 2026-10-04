"""Read packaged agent guidance without installing skills or changing settings."""
import argparse
from importlib.resources import files
from importlib.util import find_spec
from pathlib import Path


TOPICS = {
    "accelerate": "Profile Python code and move a measured operation into C++",
    "bring-library": "Load a C++ library without an existing kit",
    "existing-cpp": "Use Python around an existing compiled C++ implementation",
}

# Import names are also the CLI topics. Resource discovery does not import kits.
KITS = {
    "bt_kit": ("ros-jazzy-bt-kit", "BehaviorTree.CPP", "WHY.md"),
    "pcl_kit": ("ros-jazzy-pcl-kit", "Point Cloud Library", "WHY.md"),
    "ompl_kit": ("ros-jazzy-ompl-kit", "Open Motion Planning Library", "WHY.md"),
    "nav2_kit": ("ros-jazzy-nav2-kit", "Navigation2", "WHY.md"),
    "moveit_kit": ("ros-jazzy-moveit-kit", "MoveIt", "WHY.md"),
    "control_kit": ("ros-jazzy-control-kit", "ros2_control", "WHY.md"),
    "cv_kit": ("ros-jazzy-cv-kit", "OpenCV", "WHY.md"),
    "dbow_kit": ("ros-jazzy-dbow-kit", "DBoW2", "WHY.md"),
    "rclcpp_kit": ("ros-jazzy-rclcpp-kit", "ROS 2 C++ components", "README.md"),
    "wbc_kit": ("wbc-kit", "Whole-body control", "WHY.md"),
}
PAGES = ("overview", "api")


def _kit_guide(topic, page):
    distribution, _, overview = KITS[topic]
    spec = find_spec(topic)
    if spec is None:
        raise ValueError(
            "%s is not installed; add %s to your Pixi environment"
            % (topic, distribution))
    reader_factory = getattr(spec.loader, "get_resource_reader", None)
    reader = reader_factory(topic) if reader_factory else None
    if reader is not None and hasattr(reader, "files"):
        resource = reader.files().joinpath("agent_guides", page + ".md")
        if resource.is_file():
            return resource.read_text(encoding="utf-8")
    # A checkout keeps canonical documents beside the nested import package.
    # Installed packages and zip resources use only the packaged files above.
    if spec.origin:
        package = Path(spec.origin).parent
        if package.name == topic and package.parent.name == topic:
            document = package.parent / (overview if page == "overview" else "SKILL.md")
            if document.is_file():
                return document.read_text(encoding="utf-8")
    raise ValueError(
        "%s has no packaged %s guide; install a build of %s containing guides"
        % (topic, page, distribution))


def read_guide(topic, page=None):
    """Read a core topic or a kit's overview/API without importing the kit.

    Invalid topics, pages, and unavailable kit guides raise ValueError.
    """
    if topic not in TOPICS and topic not in KITS:
        raise ValueError("unknown guide %r; choose %s" % (
            topic, ", ".join((*TOPICS, *KITS))))
    if topic in KITS:
        if page is not None and page not in PAGES:
            raise ValueError("unknown guide page %r; choose overview or api" % page)
        return _kit_guide(topic, page or "overview")
    if page is not None:
        raise ValueError("core guide %s has no pages; omit the page argument" % topic)
    return files("cppyy_kit").joinpath("agent_guides", topic + ".md").read_text(
        encoding="utf-8")


def _main(argv=None):
    parser = argparse.ArgumentParser(
        prog="python -m cppyy_kit guide",
        description="Print packaged guidance. No skills or settings are installed.")
    parser.add_argument("topic", nargs="?", choices=(*TOPICS, *KITS))
    parser.add_argument("page", nargs="?", choices=PAGES,
                        help="kit document (default: overview)")
    args = parser.parse_args(argv)
    if args.topic:
        try:
            text = read_guide(args.topic, args.page)
        except ValueError as exc:
            parser.error(str(exc))
        print(text, end="")
    else:
        for topic, description in TOPICS.items():
            print("%s: %s" % (topic, description))
        for topic, (distribution, description, _) in KITS.items():
            print("%s: %s (install %s)" % (topic, description, distribution))
        print("Read one with: python -m cppyy_kit guide <topic>")
        print("Read a kit API with: python -m cppyy_kit guide <kit> api")
    return 0

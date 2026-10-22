"""PCL geometry and OpenCV color silhouettes for recorded or native ROS input.

Cloud coordinates must use metres with Z up. Image coordinates remain pixels.
The two results require camera calibration and TF before spatial association.
"""
import os
from pathlib import Path

_NATIVE = Path(__file__).with_name("native")
_CLOUD = _IMAGE = None


def _compile(name, includes, libraries):
    import cppyy
    import cppyy_kit
    prefix = Path(os.environ["CONDA_PREFIX"])
    # Include source contents in the cache key, including local declarations.
    header = _NATIVE / f"{name}.hpp"
    declaration = f'#include "{header}"\n'
    source = (_NATIVE / f"{name}.cpp").read_text()
    source = source.replace(f'#include "{name}.hpp"', header.read_text().replace("#pragma once", ""))
    options = dict(decls=declaration, name=f"pick_place_{name}",
        include_paths=[str(prefix / "include"), *map(str, includes)],
        library_paths=[str(prefix / "lib")], libraries=libraries)
    # PCL segmentation headers can fail inside Cling on a cache miss. Compile
    # the implementation first, then expose only its declarations to Cling.
    from cppyy_kit.cache import prebuild
    prebuild(source, **options)
    cppyy_kit.cppdef_cached(source, **options)
    return cppyy.gbl.pick_place


def load_cloud(with_ros=False):
    """Load the cached native PCL stage and optionally the ROS bridge."""
    global _CLOUD
    import pcl_kit
    pcl_kit.bringup_pcl(with_ros=with_ros)
    if _CLOUD is None:
        prefix = Path(os.environ["CONDA_PREFIX"])
        _CLOUD = _compile("cloud", [Path(pcl_kit._pcl_include_dir()),
                                   prefix / "include/eigen3"],
                          ["pcl_common", "pcl_filters", "pcl_segmentation",
                           "pcl_sample_consensus", "pcl_search", "pcl_kdtree"])
    return _CLOUD


def load_image(with_ros=False):
    """Load the cached native OpenCV stage."""
    global _IMAGE
    import cv_kit
    cv_kit.bringup_cv()
    if _IMAGE is None:
        prefix = Path(os.environ["CONDA_PREFIX"])
        _IMAGE = _compile("image", [prefix / "include/opencv4"],
                          ["opencv_core", "opencv_imgproc"])
    return _IMAGE


def ros_pipeline_options():
    """Compiler arguments for native stages including the ROS wrapper headers."""
    import pcl_kit
    from rclcpp_kit.bringup_rclcpp import add_ros2_include_paths, ros2_include_paths
    load_cloud(with_ros=True)
    load_image()
    add_ros2_include_paths()
    prefix = Path(os.environ["CONDA_PREFIX"])
    ros_includes = ros2_include_paths()
    return dict(include_paths=[str(_NATIVE), str(prefix / "include"),
                               pcl_kit._pcl_include_dir(),
                               str(prefix / "include/eigen3"),
                               str(prefix / "include/opencv4"), *ros_includes],
                library_paths=[str(prefix / "lib")],
                libraries=["pcl_common", "pcl_filters", "pcl_segmentation",
                           "pcl_sample_consensus", "pcl_search", "pcl_kdtree",
                           "opencv_core", "opencv_imgproc"])


class Perception:
    """Load each modality on first use. Results own all returned native buffers."""

    def __init__(self, cloud_config=None):
        self.cloud_config = cloud_config

    def cloud(self, points):
        """Accept an Nx3 array or native PCL XYZ cloud; return CloudResult.

        Bounds describe visible points after removing the table. Occluded faces
        and contact surfaces are not reconstructed.
        """
        native = load_cloud()
        import pcl_kit
        if not hasattr(points, "__cpp_name__"):
            points = pcl_kit.cloud_from_numpy(points)
        config = self.cloud_config if self.cloud_config is not None else native.CloudConfig()
        return native.process_cloud(points, config)

    def image(self, image, encoding="rgb8", min_area=40):
        """Accept uint8 HxWx3 or cv::Mat. Return owned masks and silhouettes."""
        if encoding not in ("rgb8", "bgr8"):
            raise ValueError("encoding must be rgb8 or bgr8")
        native = load_image()
        import cv_kit
        if not hasattr(image, "__cpp_name__"):
            image = cv_kit.numpy_to_mat(image)
        return native.process_image(image, encoding == "rgb8", min_area)

"""Replay sample.npz without ROS or a running simulator.

From this repository checkout:
    pixi run -e pick-place python roscon_uk_2026/next_steps/pick_place/replay.py

Expected result: three clusters associated with red, blue and yellow silhouettes.
Timings exclude file loading, coordinate transformation and initial compilation.
"""
import argparse
import json
from pathlib import Path
import time

import numpy as np

from perception import Perception, load_cloud


def numpy_voxel(points, leaf):
    """Vectorized voxel centroids using the same float32 grid as PCL."""
    keys = np.floor(points * np.float32(1 / leaf)).astype(np.int64)
    _, index, count = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([
        np.bincount(index, weights=points[:, axis]) / count for axis in range(3)
    ]).astype(np.float32)


def measure(function, repeats):
    function()
    elapsed = []
    for _ in range(repeats):
        start = time.perf_counter()
        result = function()
        elapsed.append((time.perf_counter() - start) * 1000)
    return result, dict(median_ms=float(np.median(elapsed)),
                        p95_ms=float(np.percentile(elapsed, 95)))


def save_overlay(sample_path, result, path):
    """Save measured associations as a slide image; no background contours."""
    import cv2
    sample = np.load(sample_path)
    rgb = sample["rgb"]
    bgr = rgb[..., ::-1] if str(sample["encoding"]) == "rgb8" else rgb
    image = cv2.resize(bgr, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    height, width = image.shape[:2]
    image = cv2.copyMakeBorder(image, 0, 150, 0, 0, cv2.BORDER_CONSTANT, value=(24, 24, 24))
    colors = {"red": (70, 70, 255), "blue": (255, 170, 60), "yellow": (40, 230, 255)}
    cv2.putText(image, "Centers: base_link [m]   |   Visible bounds: XYZ [mm]",
                (20, height + 30), cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1, cv2.LINE_AA)
    for index, obj in enumerate(sorted(result["objects"], key=lambda o: o["pixel"][0])):
        color = colors[obj["color"]]
        x, y, w, h = [2 * n for n in obj["bbox"]]
        label = f'{obj["color"]} {obj["silhouette"]}'
        cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        for thickness, ink in [(4, (0, 0, 0)), (1, color)]:
            cv2.putText(image, label, (x - 30, y - 12), cv2.FONT_HERSHEY_SIMPLEX,
                        .6, ink, thickness, cv2.LINE_AA)
        lines = [label, "Center: " + ", ".join(f"{v:.3f}" for v in obj["center_base_m"]),
                 "Bounds: " + " x ".join(f"{1000*v:.0f}" for v in obj["size_visible_m"])]
        for row, line in enumerate(lines):
            cv2.putText(image, line, (20 + index * (width // 3), height + 65 + row * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, .6, color if row == 0 else (230, 230, 230),
                        1, cv2.LINE_AA)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), image):
        raise OSError(f"could not write overlay: {path}")


def replay(path, repeats=30, expected=3):
    import pcl_kit
    sample = np.load(path)
    points = sample["points"]
    finite = np.isfinite(points).all(axis=1)
    points = points[finite].astype(np.float64)
    # The recording stores the cloud body frame -> base_link rigid transform.
    quaternion = sample["base_quaternion_xyzw"]
    vector, scalar = quaternion[:3], quaternion[3]
    rotate = lambda p: p + 2 * np.cross(vector, np.cross(vector, p) + scalar * p)
    rotation = rotate(np.eye(3)).T
    translation = sample["base_translation"]
    base = np.ascontiguousarray(points @ rotation.T + translation, dtype=np.float32)

    start = time.perf_counter()
    native = load_cloud()
    config = native.CloudConfig()
    config.lower, config.upper = [.5, -.5, .85], [1.3, .5, 1.3]
    perception = Perception(config)
    perception.cloud(base)
    perception.image(sample["rgb"], str(sample["encoding"]))
    startup_ms = (time.perf_counter() - start) * 1000
    cloud, cloud_time = measure(lambda: perception.cloud(base), repeats)
    image, image_time = measure(
        lambda: perception.image(sample["rgb"], str(sample["encoding"])), repeats)

    intrinsics = sample["camera_k"].reshape(3, 3)
    objects = []
    for obj in cloud.objects:
        center = np.array(list(obj.center))
        camera = (center - translation) @ rotation
        pixel = [float(intrinsics[0, 2] - intrinsics[0, 0] * camera[1] / camera[0]),
                 float(intrinsics[1, 2] - intrinsics[1, 1] * camera[2] / camera[0])]
        candidates = [o for o in image.objects
                      if o.x - 5 <= pixel[0] <= o.x + o.width + 5
                      and o.y - 5 <= pixel[1] <= o.y + o.height + 5]
        label = min(candidates, key=lambda o:
                    (pixel[0] - o.x - o.width / 2)**2 +
                    (pixel[1] - o.y - o.height / 2)**2) if candidates else None
        objects.append(dict(center_base_m=center.tolist(), size_visible_m=list(obj.size),
                            color=str(label.color) if label else None,
                            silhouette=str(label.shape) if label else None, pixel=pixel,
                            bbox=[label.x, label.y, label.width, label.height] if label else None))

    lower, upper = np.array(list(config.lower)), np.array(list(config.upper))
    roi = np.ascontiguousarray(base[((base >= lower) & (base <= upper)).all(axis=1)])
    leaf = float(config.leaf)
    reference, numpy_time = measure(lambda: numpy_voxel(roi, leaf), repeats)
    # Both paths start and end with NumPy. Include both PCL storage copies.
    filtered, pcl_time = measure(lambda: pcl_kit.cloud_to_numpy(
        pcl_kit.voxel_downsample(pcl_kit.cloud_from_numpy(roi), leaf)), repeats)
    order = lambda a: np.lexsort(np.floor(a * np.float32(1 / leaf)).T[::-1])
    error = float(np.max(np.abs(reference[order(reference)] - filtered[order(filtered)]))) \
        if reference.shape == filtered.shape else None
    result = dict(input=str(path), input_points=int(len(finite)), finite_points=int(finite.sum()),
                  frame="base_link", table_height_m=float(cloud.table_height),
                  objects=objects, image_contours=len(image.objects),
                  native_loading_and_first_calls_ms=startup_ms,
                  warmed=dict(calls=repeats, cloud=cloud_time, image=image_time,
                      scope="cloud receives transformed finite points; image receives full RGB frame"),
                  voxel_comparison=dict(input_points=len(roi), leaf_m=leaf,
                      numpy_points=len(reference), pcl_points=len(filtered),
                      max_centroid_difference_m=error, numpy=numpy_time, pcl=pcl_time,
                      scope="same finite ROI input; NumPy arrays in and out; PCL copies included"))
    if len(objects) != expected or {o["color"] for o in objects} != {"red", "blue", "yellow"}:
        raise RuntimeError(f"expected {expected} clusters with red, blue and yellow labels: {objects}")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample", type=Path, default=Path(__file__).with_name("sample.npz"))
    parser.add_argument("--repeats", type=int, default=30)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overlay", type=Path, help="write matched RGB boxes and 3D measurements")
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    report = replay(args.sample, args.repeats)
    if args.overlay:
        save_overlay(args.sample, report, args.overlay)
    result = json.dumps(report, indent=2)
    print(result)
    if args.output:
        args.output.write_text(result + "\n")

"""Download models and measure retained inference on a saved webcam frame."""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import torch

from models import DepthModel, ObjectModel, ROOT


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("model", choices=("depth", "objects"))
    parser.add_argument("--image", type=Path, default=ROOT / "build/webcam_session/rotation_0.jpg")
    parser.add_argument("--runs", type=int, default=20)
    args = parser.parse_args()
    output = ROOT / "build/perception_toolbox/evidence"
    output.mkdir(parents=True, exist_ok=True)
    rgb = cv2.cvtColor(cv2.imread(str(args.image)), cv2.COLOR_BGR2RGB)
    torch.cuda.reset_peak_memory_stats()
    start = time.perf_counter()
    model = DepthModel() if args.model == "depth" else ObjectModel()
    torch.cuda.synchronize()
    load_seconds = time.perf_counter() - start
    samples = []
    for _ in range(args.runs + 1):
        start = time.perf_counter()
        result = model(rgb)
        torch.cuda.synchronize()
        samples.append((time.perf_counter() - start) * 1000)
    stats = {
        "model": args.model, "image": str(args.image),
        "torch": torch.__version__, "gpu": torch.cuda.get_device_name(),
        "load_seconds": load_seconds, "first_ms": samples[0],
        "warm_median_ms": float(np.median(samples[1:])),
        "warm_p95_ms": float(np.percentile(samples[1:], 95)),
        "warm_samples_ms": samples[1:],
        "peak_allocated_MiB": torch.cuda.max_memory_allocated() / 2**20,
        "peak_reserved_MiB": torch.cuda.max_memory_reserved() / 2**20,
    }
    if args.model == "depth":
        np.savez_compressed(output / "depth.npz", **result)
        depth = result["depth"]
        lo, hi = np.percentile(depth, (2, 98))
        grey = np.uint8(np.clip((depth - lo) / max(hi - lo, 1e-6), 0, 1) * 255)
        cv2.imwrite(str(output / "depth.jpg"), cv2.applyColorMap(grey, cv2.COLORMAP_TURBO))
        stats["shape"] = list(depth.shape)
        stats["finite_fraction"] = float(np.isfinite(depth).mean())
    else:
        cv2.imwrite(str(output / "objects.jpg"), result["overlay_rgb"][..., ::-1])
        stats["labels"] = result["labels"]
        stats["scores"] = result["scores"].tolist()
        np.savez_compressed(output / "objects.npz", **result)
    (output / f"{args.model}.json").write_text(json.dumps(stats, indent=2) + "\n")
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()

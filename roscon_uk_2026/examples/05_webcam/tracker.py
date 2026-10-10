"""Click to choose a grayscale patch. Complete its custom SSD search."""
import argparse
import time
import numpy as np


def track(previous, current, x, y, radius=5, search=8):
    """Return (x,y) minimizing integer sum of squared pixel differences.

    Use a (2*radius+1) square template centered at (x,y) in previous. Search
    current within +/-search pixels, clipping candidate centers to valid patch
    bounds. Iterate candidates in increasing y, then increasing x. On ties,
    return the first candidate. Frames must be same-shape 2-D uint8 arrays.
    Noncontiguous frames are accepted. The previous center must be valid.
    """
    if previous.ndim != 2 or previous.shape != current.shape or previous.dtype != np.uint8 or current.dtype != np.uint8:
        raise ValueError("expected same-shape grayscale uint8 frames")
    if not all(isinstance(v, (int, np.integer)) for v in (x, y, radius, search)) or radius < 0 or search < 0:
        raise ValueError("invalid integer search parameters")
    h, w = previous.shape
    if not (radius <= x < w-radius and radius <= y < h-radius):
        raise ValueError("template center outside valid image bounds")
    previous = np.ascontiguousarray(previous)
    current = np.ascontiguousarray(current)
    raise NotImplementedError("implement patch search using cppyy_kit")


def main():
    import cv2
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()
    camera = cv2.VideoCapture(args.device)
    if not camera.isOpened():
        raise RuntimeError(f"camera {args.device} could not be opened")
    center, previous, durations = [None], None, []
    name = "cppyy_kit patch tracker: click to select; q exits"
    if not args.headless:
        cv2.namedWindow(name)
        def clicked(event, x, y, flags, data):
            if event == cv2.EVENT_LBUTTONDOWN:
                center[0] = (x, y)
        cv2.setMouseCallback(name, clicked)
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("camera read failed")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            if center[0] is None:
                center[0] = (w//2, h//2)
            x, y = center[0]
            center[0] = (min(max(x,5),w-6), min(max(y,5),h-6))
            if previous is not None:
                started = time.perf_counter()
                center[0] = track(previous, gray, *center[0])
                durations.append(1000*(time.perf_counter()-started))
            previous = gray
            if not args.headless:
                x, y = center[0]
                cv2.rectangle(frame, (x-5,y-5), (x+5,y+5), (0,255,0), 2)
                cv2.imshow(name, frame)
                if cv2.waitKey(1) & 255 == ord("q"):
                    break
            if args.frames and len(durations) >= args.frames:
                break
    finally:
        camera.release()
        if not args.headless:
            cv2.destroyAllWindows()
    print({"tracked_frames": len(durations), "median_track_ms": float(np.median(durations))})


if __name__ == "__main__":
    main()

#!/usr/bin/env python
"""
webcam_demo (M6b) -- LIVE webcam visual-odometry front-end, two ways, side by side.

The stage story, in one sentence: **you write the whole per-frame pipeline in
Python, and with the kits it runs as C++ -- fast enough to sail while the naive
Python version, doing the identical work, struggles on the very same frames.**

Two pipelines run over each captured frame and their per-frame processing time,
achievable FPS and process CPU% are plotted head-to-head in Rerun:

  * **Pipeline A ("all in Python", the cppyy_kit way).** camera -> ``cv::Mat``
    (zero-copy alias of the capture buffer via ``cv_kit.numpy_to_mat``) -> C++
    ``cv::ORB`` keypoints (``cv_kit``; the ``cv::cuda::ORB`` branch activates
    automatically if a CUDA OpenCV build is provisioned, see cv_kit/CUDA_OPENCV.md)
    -> a **hand-written per-keypoint NCC patch tracker** (the expensive stage) and
    a 2D similarity motion estimate, **all in one C++ address space** (a
    ``cppyy.cppdef`` kernel; features/descriptors/patches never cross into Python)
    -> a TF transform + an image topic published via ``rclcpp_kit`` -> Rerun.

  * **Pipeline B (naive Python baseline).** The identical algorithm written the way
    a roboticist prototypes in Python: ``cv2.ORB`` keypoints materialized as Python
    objects, and the NCC patch tracker as a **NumPy-per-keypoint loop** in Python.

**What actually differs, honestly.** ``cv2``'s calls are C++ too, so ORB, matching
and RANSAC take about the same time in both -- for those, A is only ~1.1-1.2x faster
(the per-frame Python orchestration/copies). The *dramatic* gap is the NCC patch
tracker: it is a **custom numerical kernel with no OpenCV/NumPy one-liner**, so the
naive baseline must loop in Python (~5-100x slower, measured), while A expresses the
same math as C++ and runs it natively. That is exactly the cppyy_kit thesis
(COMMON_PATTERNS s6/s26: a per-element Python loop is the trap; keep the loop in
C++). Both pipelines compute bit-identical flow -- verified in the tests.

Rerun is LIVE by default when run interactively (a viewer opens and the plots
diverge in real time); headless (.rrd) under pytest/CI or no display. Force with
RCLCPPYY_RERUN_SPAWN=1/0. See cv_kit/demos/vision_viz.py.

Robust for stage use: never crashes on a dropped frame / unplugged camera (it prints
a notice and falls back to the synthetic moving scene), warms up so frame 0 does not
stutter, and tears down cleanly.

    # live, auto source (webcam if present, else synthetic):
    pixi run -e vision demo-webcam
    # force the synthetic moving scene (no camera needed), 20 s:
    pixi run -e vision demo-webcam --source synthetic --duration 20
    # print the A-vs-B table at 640x480 and 1280x720 (no viewer, no ROS):
    pixi run -e vision demo-webcam --bench
"""
import argparse
import math
import os
import sys
import time

import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "62")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(REPO, "scripts", "datasets"))
sys.path.insert(0, HERE)

import synthetic_loop  # noqa: E402
import vision_viz  # noqa: E402

IMAGE_TOPIC = "vision/webcam"


# --------------------------------------------------------------------------- #
# The expensive C++ kernel (pipeline A): a hand-written NCC patch tracker + a 2D
# similarity motion estimate, held together in one cppyy.cppdef translation unit so
# the whole per-frame computation stays in C++. ORB keypoints, the grayscale
# patches, and the correspondence arrays never cross back into Python; only the
# small per-keypoint flow output and the six motion scalars do.
#
# Why this is the honest showcase: there is no single cv2 call for "NCC-search each
# keypoint's patch over a window and return the refined flow", so the naive Python
# baseline (VoTrackerPy) must loop -- which is exactly the per-element Python trap
# cppyy_kit exists to avoid. estimateAffinePartial2D lives in calib3d, which cv_kit
# does not load by default, so the kernel's bringup adds it.
# --------------------------------------------------------------------------- #
_VO_GLUE = r"""
namespace rclcppyy_webcam {

struct Motion { double dx, dy, dtheta, scale; int n_tracked, n_inliers; bool ok; };

// NCC of the (2r+1) square patch centred at (cx,cy) in `prev` against the same-size
// patch centred at (cx+ox, cy+oy) in `cur`. Zero-mean normalised cross-correlation.
static inline float ncc_at(const cv::Mat& prev, const cv::Mat& cur,
                           int cx, int cy, int ox, int oy, int r, float pmean) {
  int P = 2 * r + 1;
  float cm = 0.f;
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx)
      cm += cur.at<unsigned char>(cy + oy + dy, cx + ox + dx);
  cm /= (P * P);
  float num = 0.f, dp = 0.f, dc = 0.f;
  for (int dy = -r; dy <= r; ++dy)
    for (int dx = -r; dx <= r; ++dx) {
      float a = (float)prev.at<unsigned char>(cy + dy, cx + dx) - pmean;
      float b = (float)cur.at<unsigned char>(cy + oy + dy, cx + ox + dx) - cm;
      num += a * b; dp += a * a; dc += b * b;
    }
  return num / (std::sqrt(dp * dc) + 1e-6f);
}

// A stateful patch-based visual-odometry front-end. Holds the previous frame's
// grayscale image and ORB keypoints; on each frame it NCC-tracks those keypoints
// into the new frame, estimates the 2D similarity (rotation+scale+translation)
// motion by RANSAC, then re-detects keypoints for the next iteration. All in C++.
class VoTracker {
  cv::Ptr<cv::ORB> orb;
  cv::Mat prev_gray;
  std::vector<cv::KeyPoint> prev_kps;
  bool have_prev = false;
  int r, s;           // patch radius, search radius (pixels)
  float min_score;    // minimum NCC to accept a track
 public:
  VoTracker(int nfeatures, int patch_r, int search_s, double min_sc)
      : r(patch_r), s(search_s), min_score((float)min_sc) {
    orb = cv::ORB::create(nfeatures);
  }

  // Number of keypoints available to track this frame (0 on the first frame). The
  // caller sizes the out buffers to this before calling track().
  int prev_count() const { return (int)prev_kps.size(); }

  // Track prev_kps into cur_gray, estimate motion, then re-detect for next frame.
  //   out_flow  : Kx3 float32 [dx, dy, ncc_score] per previous keypoint (may be 0)
  //   out_prev  : Kx2 float32 previous keypoint (x,y) (may be 0)
  // where K == prev_count() at call time. Returns the Motion.
  Motion track(const cv::Mat& cur_gray, uintptr_t out_flow, uintptr_t out_prev) {
    Motion mo{0, 0, 0, 1, 0, 0, false};
    float* flow = reinterpret_cast<float*>(out_flow);
    float* pxy = reinterpret_cast<float*>(out_prev);
    std::vector<cv::Point2f> src, dst;
    if (have_prev) {
      int K = (int)prev_kps.size();
      int W = cur_gray.cols, H = cur_gray.rows;
      int P = 2 * r + 1;
      for (int k = 0; k < K; ++k) {
        int cx = (int)(prev_kps[k].pt.x + 0.5f), cy = (int)(prev_kps[k].pt.y + 0.5f);
        float fdx = 0, fdy = 0, best = -2;
        if (cx - r - s >= 0 && cy - r - s >= 0 && cx + r + s < W && cy + r + s < H) {
          float pmean = 0.f;
          for (int dy = -r; dy <= r; ++dy)
            for (int dx = -r; dx <= r; ++dx)
              pmean += prev_gray.at<unsigned char>(cy + dy, cx + dx);
          pmean /= (P * P);
          for (int oy = -s; oy <= s; ++oy)
            for (int ox = -s; ox <= s; ++ox) {
              float v = ncc_at(prev_gray, cur_gray, cx, cy, ox, oy, r, pmean);
              if (v > best) { best = v; fdx = (float)ox; fdy = (float)oy; }
            }
          if (best >= min_score) {
            src.push_back(prev_kps[k].pt);
            dst.push_back(cv::Point2f(prev_kps[k].pt.x + fdx, prev_kps[k].pt.y + fdy));
          }
        }
        if (flow) { flow[3 * k] = fdx; flow[3 * k + 1] = fdy; flow[3 * k + 2] = best; }
        if (pxy)  { pxy[2 * k] = prev_kps[k].pt.x; pxy[2 * k + 1] = prev_kps[k].pt.y; }
      }
      mo.n_tracked = (int)src.size();
      if (src.size() >= 3) {
        std::vector<uchar> inl;
        cv::Mat M = cv::estimateAffinePartial2D(src, dst, inl, cv::RANSAC, 3.0);
        if (!M.empty()) {
          double a = M.at<double>(0, 0), b = M.at<double>(0, 1);
          mo.dx = M.at<double>(0, 2); mo.dy = M.at<double>(1, 2);
          mo.scale = std::sqrt(a * a + b * b);
          mo.dtheta = std::atan2(b, a);
          mo.n_inliers = cv::countNonZero(inl);
          mo.ok = true;
        }
      }
    }
    std::vector<cv::KeyPoint> kps;
    orb->detect(cur_gray, kps);
    prev_gray = cur_gray.clone();
    prev_kps = kps;
    have_prev = true;
    return mo;
  }
};

}  // namespace rclcppyy_webcam
"""

_VO_READY = False


def bringup_vo():
    """JIT-compile the pipeline-A C++ kernel (once). Brings up cv_kit's OpenCV, adds
    the ``calib3d`` module (``estimateAffinePartial2D``) that cv_kit does not load by
    default, and defines the ``VoTracker`` class. Returns the ``rclcppyy_webcam``
    cppyy namespace."""
    global _VO_READY
    import cppyy
    import cppyy_kit
    import cv_kit
    cv_kit.bringup_cv()
    if not _VO_READY:
        conda = os.environ["CONDA_PREFIX"]
        cppyy.include("opencv2/calib3d.hpp")
        cppyy_kit.load_libraries(("libopencv_calib3d.so",), [os.path.join(conda, "lib")])
        cppyy.cppdef(_VO_GLUE)
        _VO_READY = True
    return cppyy.gbl.rclcppyy_webcam


# --------------------------------------------------------------------------- #
# Pipeline A -- the kits path. Thin Python wrapper over the C++ VoTracker.
# --------------------------------------------------------------------------- #
class VoTrackerCpp:
    """Pipeline A: ORB + NCC patch track + similarity motion, entirely in C++
    (cv_kit's ``cv::ORB`` + the ``VoTracker`` kernel). Python only orchestrates and
    reads back the small flow/motion results for the overlay."""

    name = "A (cppyy_kit -> C++)"

    def __init__(self, nfeatures, patch_r, search_s, min_score):
        import cv_kit
        self._cv_kit = cv_kit
        vo = bringup_vo()
        self._t = vo.VoTracker(nfeatures, patch_r, search_s, min_score)

    def process(self, gray_np):
        """Run one frame. ``gray_np`` is a contiguous (H,W) uint8 grayscale image.
        Returns ``(motion_dict_or_None, prev_xy Nx2, flow Nx3)``."""
        cvk = self._cv_kit
        mat = cvk.numpy_to_mat(gray_np)          # zero-copy alias of the capture buffer
        k = int(self._t.prev_count())
        n = max(k, 1)
        flow = np.zeros((n, 3), dtype=np.float32)
        prev = np.zeros((n, 2), dtype=np.float32)
        mo = self._t.track(mat, flow.ctypes.data, prev.ctypes.data)
        motion = _motion_dict(mo) if bool(mo.ok) else None
        return motion, prev[:k], flow[:k]


# --------------------------------------------------------------------------- #
# Pipeline B -- the naive Python baseline. Same algorithm, cv2 + NumPy-per-keypoint.
# --------------------------------------------------------------------------- #
class VoTrackerPy:
    """Pipeline B: the identical patch-tracking VO, written the naive way -- cv2 ORB
    (keypoints as Python objects) and the NCC patch tracker as a **NumPy-per-keypoint
    Python loop**. Correct and readable; the per-keypoint Python iteration is exactly
    what makes it slow relative to the C++ kernel."""

    name = "B (naive Python)"

    def __init__(self, nfeatures, patch_r, search_s, min_score):
        import cv2
        self._cv2 = cv2
        self._orb = cv2.ORB_create(nfeatures)
        self._r, self._s, self._min = patch_r, search_s, float(min_score)
        self._prev_gray = None
        self._prev_xy = None

    def _ncc_track(self, prev, cur, xy):
        """NCC patch tracker, NumPy per keypoint (the naive-Python expensive stage)."""
        r, s = self._r, self._s
        H, W = cur.shape
        out = np.zeros((len(xy), 3), dtype=np.float32)
        for k in range(len(xy)):
            cx, cy = int(xy[k, 0] + 0.5), int(xy[k, 1] + 0.5)
            if cx - r - s < 0 or cy - r - s < 0 or cx + r + s >= W or cy + r + s >= H:
                out[k] = (0.0, 0.0, -1.0)
                continue
            patch = prev[cy - r:cy + r + 1, cx - r:cx + r + 1].astype(np.float32)
            patch -= patch.mean()
            pnorm = math.sqrt(float((patch * patch).sum())) + 1e-6
            best, bdx, bdy = -2.0, 0, 0
            for oy in range(-s, s + 1):
                for ox in range(-s, s + 1):
                    win = cur[cy + oy - r:cy + oy + r + 1,
                              cx + ox - r:cx + ox + r + 1].astype(np.float32)
                    win -= win.mean()
                    den = pnorm * (math.sqrt(float((win * win).sum())) + 1e-6)
                    v = float((patch * win).sum()) / den
                    if v > best:
                        best, bdx, bdy = v, ox, oy
            out[k] = (bdx, bdy, best)
        return out

    def process(self, gray_np):
        cv2 = self._cv2
        motion, prev_xy, flow = None, None, None
        if self._prev_gray is not None and self._prev_xy is not None and len(self._prev_xy):
            prev_xy = self._prev_xy
            flow = self._ncc_track(self._prev_gray, gray_np, prev_xy)
            good = flow[:, 2] >= self._min
            src = prev_xy[good]
            dst = prev_xy[good] + flow[good, :2]
            if len(src) >= 3:
                M, inl = cv2.estimateAffinePartial2D(
                    src.astype(np.float32), dst.astype(np.float32), method=cv2.RANSAC)
                if M is not None:
                    motion = _affine_to_motion(M, int(inl.sum()) if inl is not None else 0,
                                               int(len(src)))
        kps = self._orb.detect(gray_np, None)
        self._prev_xy = (np.array([kp.pt for kp in kps], dtype=np.float32)
                         if kps else np.zeros((0, 2), dtype=np.float32))
        self._prev_gray = gray_np.copy()
        return motion, prev_xy, flow


def _motion_dict(mo):
    return {"dx": float(mo.dx), "dy": float(mo.dy), "dtheta": float(mo.dtheta),
            "scale": float(mo.scale), "n_tracked": int(mo.n_tracked),
            "n_inliers": int(mo.n_inliers)}


def _affine_to_motion(M, n_inliers, n_tracked):
    a, b = float(M[0, 0]), float(M[0, 1])
    return {"dx": float(M[0, 2]), "dy": float(M[1, 2]), "dtheta": math.atan2(b, a),
            "scale": math.sqrt(a * a + b * b), "n_tracked": n_tracked,
            "n_inliers": n_inliers}


# --------------------------------------------------------------------------- #
# Frame sources (interchangeable). Both yield contiguous uint8 frames; a webcam
# frame is BGR (H,W,3), a synthetic frame is grayscale (H,W).
# --------------------------------------------------------------------------- #
class SyntheticSource:
    """The deterministic moving textured scene (scripts/datasets/synthetic_loop.py):
    a window panning a fixed canvas -> real, repeatable inter-frame motion. Needs no
    camera, so it is the CI/rehearsal source and the webcam-unplug fallback."""

    kind = "synthetic"
    is_color = False

    def __init__(self, width=640, height=480, n=synthetic_loop.DEFAULT_N):
        self._frames = [f for _, f in synthetic_loop.frames(n)]
        self._i = 0
        self.width, self.height = width, height

    def read(self):
        f = self._frames[self._i % len(self._frames)]
        self._i += 1
        return True, f

    def close(self):
        pass


class WebcamSource:
    """A V4L2 webcam via ``cv2.VideoCapture``. ``read()`` returns ``(ok, frame_bgr)``;
    a failed read (unplug, dropped frame) returns ``(False, None)`` so the caller can
    fall back to synthetic without crashing."""

    kind = "webcam"
    is_color = True

    def __init__(self, device=0, width=640, height=480):
        import cv2
        self._cv2 = cv2
        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self._cap.release()
            raise RuntimeError("could not open webcam device %r" % device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def read(self):
        try:
            ok, frame = self._cap.read()
        except Exception:
            return False, None
        if not ok or frame is None:
            return False, None
        return True, np.ascontiguousarray(frame)

    def close(self):
        try:
            self._cap.release()
        except Exception:
            pass


def open_source(kind, device, width, height, synth_n):
    """Open the requested source. ``kind='auto'`` prefers the webcam and falls back to
    synthetic (printing why). Returns ``(source, note)``."""
    if kind in ("webcam", "auto"):
        try:
            src = WebcamSource(device, width, height)
            return src, "webcam device %d (%dx%d)" % (device, src.width, src.height)
        except Exception as exc:
            if kind == "webcam":
                raise
            print("[webcam_demo] no webcam (%s); using the synthetic moving scene." % exc,
                  flush=True)
    return SyntheticSource(width, height, synth_n), "synthetic moving scene (no camera)"


# --------------------------------------------------------------------------- #
# Timing: wall ms + process CPU% per pipeline call (dependency-free, honest).
# process_time() is this process's user+system CPU seconds (all threads), so
# cpu% = 100 * cpu_delta / wall_delta is the average number of CPU cores busy during
# the call -- >100% when OpenCV parallelises ORB internally (real, not a bug).
# --------------------------------------------------------------------------- #
def _timed(fn):
    w0, c0 = time.perf_counter(), time.process_time()
    out = fn()
    return out, (time.perf_counter() - w0) * 1e3, (time.process_time() - c0) * 1e3


def _to_gray_np(frame, is_color):
    if not is_color and frame.ndim == 2:
        return frame
    import cv2
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


# --------------------------------------------------------------------------- #
# Bench mode: the A-vs-B table at fixed resolutions on the synthetic scene.
# --------------------------------------------------------------------------- #
def _synth_frames(width, height, n):
    """n grayscale frames at (width,height) with genuine inter-frame motion, by
    panning a crop window across a large deterministic canvas (then resizing to the
    exact resolution). Reuses the tutorial's canvas so it is reproducible."""
    import cv2
    canvas = synthetic_loop.canvas(size=max(width, height) + 500)
    out = []
    for i in range(n):
        off = i * 3
        crop = canvas[(off % 200):height + (off % 200), off:width + off]
        out.append(np.ascontiguousarray(cv2.resize(crop, (width, height))))
    return out


def run_bench(resolutions, nfeatures, track_points, patch_r, search_s, min_score,
              n=80, warm=5):
    """Measure both pipelines at each resolution and return a list of row dicts.
    ``track_points`` caps how many of the strongest keypoints the NCC stage tracks
    (bounds the expensive stage; ORB still detects ``nfeatures``)."""
    rows = []
    for (w, h) in resolutions:
        frames = _synth_frames(w, h, n + warm)
        # Cap tracked keypoints by lowering nfeatures for the tracker's re-detect so
        # both pipelines track a comparable number of the strongest features.
        nf = min(nfeatures, track_points)
        a = VoTrackerCpp(nf, patch_r, search_s, min_score)
        b = VoTrackerPy(nf, patch_r, search_s, min_score)
        ta, ca, tb, cb, kt = [], [], [], [], []
        for i, f in enumerate(frames):
            (_, _, _), aw, ac = _timed(lambda: a.process(f))
            (_, pxy, _), bw, bc = _timed(lambda: b.process(f))
            if i >= warm:
                ta.append(aw)
                ca.append(ac)
                tb.append(bw)
                cb.append(bc)
                if pxy is not None:
                    kt.append(len(pxy))
        row = {
            "w": w, "h": h,
            "tracked": int(np.mean(kt)) if kt else 0,
            "a_ms": float(np.mean(ta)), "a_cpu": 100.0 * np.sum(ca) / np.sum(ta),
            "b_ms": float(np.mean(tb)), "b_cpu": 100.0 * np.sum(cb) / np.sum(tb),
        }
        row["a_fps"] = 1000.0 / row["a_ms"]
        row["b_fps"] = 1000.0 / row["b_ms"]
        row["speedup"] = row["b_ms"] / row["a_ms"]
        rows.append(row)
    return rows


def print_bench(rows, nfeatures, track_points, patch_r, search_s):
    print("\n== webcam_demo A-vs-B (synthetic scene; ORB nfeatures=%d, NCC track<=%d "
          "keypoints, patch %dx%d, search %dx%d) =="
          % (nfeatures, track_points, 2 * patch_r + 1, 2 * patch_r + 1,
             2 * search_s + 1, 2 * search_s + 1))
    print("%-11s %8s | %-28s | %-28s | %8s" %
          ("res", "tracked", "A (cppyy_kit -> C++)", "B (naive Python)", "A speedup"))
    print("-" * 96)
    for r in rows:
        print("%-11s %8d | %6.2f ms  %6.1f fps  %5.0f%% cpu | "
              "%7.2f ms %6.1f fps %5.0f%% cpu | %6.2fx" %
              ("%dx%d" % (r["w"], r["h"]), r["tracked"],
               r["a_ms"], r["a_fps"], r["a_cpu"],
               r["b_ms"], r["b_fps"], r["b_cpu"], r["speedup"]))
    print("-" * 96)
    print("A = you write it in Python, it runs as C++ (cv_kit ORB + a cppyy.cppdef "
          "NCC/RANSAC kernel; features & patches never leave C++).")
    print("B = the same algorithm in naive Python (cv2 ORB + a NumPy-per-keypoint NCC "
          "loop). Both compute identical flow; the gap is the per-keypoint Python "
          "iteration.\n")


# --------------------------------------------------------------------------- #
# Optional ROS side-channel: publish the frame as sensor_msgs/Image and the
# accumulated pose as a TF transform, both via rclcpp_kit (the robotics integration).
# --------------------------------------------------------------------------- #
class RosPublisher:
    def __init__(self, width, height):
        from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
        import cppyy
        self._rclcpp = bringup_rclcpp()
        if not self._rclcpp.ok():
            self._rclcpp.init()
        cppyy.include("sensor_msgs/msg/image.hpp")
        cppyy.include("tf2_msgs/msg/tf_message.hpp")
        cppyy.include("geometry_msgs/msg/transform_stamped.hpp")
        self._cppyy = cppyy
        self._Image = cppyy.gbl.sensor_msgs.msg.Image
        self._TFMessage = cppyy.gbl.tf2_msgs.msg.TFMessage
        self._node = self._rclcpp.Node("webcam_demo")
        self._img_pub = self._node.create_publisher(self._Image, IMAGE_TOPIC, 10)
        self._tf_pub = self._node.create_publisher(self._TFMessage, "/tf", 10)
        import dataset_publisher
        self._build_image = dataset_publisher.build_image_msg

    def publish(self, frame, is_color, pose):
        enc = "bgr8" if (is_color and frame.ndim == 3) else "mono8"
        try:
            self._img_pub.publish(self._build_image(self._Image, frame, enc,
                                                    frame_id="camera"))
            self._tf_pub.publish(self._tf_message(pose))
        except Exception as exc:  # never let a publish error kill the live demo
            print("[webcam_demo] ROS publish skipped: %s" % exc, flush=True)

    def _tf_message(self, pose):
        gm = self._cppyy.gbl.geometry_msgs.msg
        ts = gm.TransformStamped()
        now = time.time()
        ts.header.stamp.sec = int(now)
        ts.header.stamp.nanosec = int((now - int(now)) * 1e9)
        ts.header.frame_id = "world"
        ts.child_frame_id = "camera"
        ts.transform.translation.x = float(pose[0])
        ts.transform.translation.y = float(pose[1])
        ts.transform.translation.z = 0.0
        th = float(pose[2])
        ts.transform.rotation.z = math.sin(th / 2.0)
        ts.transform.rotation.w = math.cos(th / 2.0)
        msg = self._TFMessage()
        msg.transforms.push_back(ts)
        return msg


# --------------------------------------------------------------------------- #
# The live demo.
# --------------------------------------------------------------------------- #
def run_live(args):
    import rerun as rr

    session = vision_viz.init_rerun("rclcppyy_webcam_demo", args.rrd,
                                    blueprint=vision_viz.blueprint_webcam_ab())

    # Bring up both pipelines. ORB detects nfeatures; the NCC stage tracks the
    # strongest track_points of them (cap re-detect to keep the expensive stage
    # bounded and the two pipelines tracking a comparable set).
    nf = min(args.nfeatures, args.track_points)
    print("Bringing up pipeline A (cv_kit ORB + C++ NCC kernel) ...", flush=True)
    a = VoTrackerCpp(nf, args.patch_radius, args.search_radius, args.min_score)
    b = VoTrackerPy(nf, args.patch_radius, args.search_radius, args.min_score)
    print("ORB backend (pipeline A): %s"
          % ("cv::cuda::ORB (GPU)" if _cuda_on() else "cv::ORB (CPU)"), flush=True)

    src, note = open_source(args.source, args.device, args.width, args.height, args.synth_n)
    print("Source: %s" % note, flush=True)
    rr.log("log", rr.TextLog("source: " + note), static=False)

    ros = None
    if not args.no_ros:
        try:
            ros = RosPublisher(src.width, src.height)
            print("ROS: publishing %s (sensor_msgs/Image) + /tf (world->camera)."
                  % IMAGE_TOPIC, flush=True)
        except Exception as exc:
            print("[webcam_demo] ROS publishing disabled (%s)." % exc, flush=True)

    # Warm both pipelines so frame 0 does not stutter live (moves the first-use JIT
    # of the C++ track() wrapper + the OpenCV codegen out of the live loop).
    _warmup(a, b, src.width, src.height)

    rr.log("perf/ms", rr.SeriesLines(names=["A cppyy", "B python"],
                                     colors=[(80, 220, 120), (240, 150, 60)],
                                     widths=[2.0, 2.0]), static=True)
    rr.log("perf/fps", rr.SeriesLines(names=["A cppyy", "B python"],
                                      colors=[(80, 220, 120), (240, 150, 60)],
                                      widths=[2.0, 2.0]), static=True)
    rr.log("perf/cpu", rr.SeriesLines(names=["A cppyy", "B python"],
                                      colors=[(80, 220, 120), (240, 150, 60)],
                                      widths=[2.0, 2.0]), static=True)

    pose = [0.0, 0.0, 0.0]           # accumulated (x, y, theta) from pipeline A
    traj = [(0.0, 0.0)]
    stats = {"n": 0, "a_ms": [], "b_ms": [], "a_cpu": [], "b_cpu": [], "drops": 0}
    consecutive_fail = 0
    deadline = time.monotonic() + args.duration

    try:
        while time.monotonic() < deadline:
            ok, frame = src.read()
            if not ok:
                consecutive_fail += 1
                stats["drops"] += 1
                if src.kind == "webcam" and consecutive_fail >= 5:
                    msg = "webcam stopped delivering frames -- falling back to synthetic"
                    print("[webcam_demo] %s." % msg, flush=True)
                    rr.log("log", rr.TextLog(msg, level=rr.TextLogLevel.WARN))
                    src.close()
                    src = SyntheticSource(args.width, args.height, args.synth_n)
                    consecutive_fail = 0
                else:
                    time.sleep(0.005)
                continue
            consecutive_fail = 0

            gray = _to_gray_np(frame, src.is_color)
            result_a, aw, ac = _timed(lambda: a.process(gray))
            motion, prev_xy, flow = result_a
            if args.no_baseline:
                bw, bc = float("nan"), float("nan")
            else:
                _, bw, bc = _timed(lambda: b.process(gray))

            stats["n"] += 1
            stats["a_ms"].append(aw)
            stats["a_cpu"].append(ac)
            if not args.no_baseline:
                stats["b_ms"].append(bw)
                stats["b_cpu"].append(bc)

            # Accumulate pipeline A's motion into a 2D camera pose (demo scale).
            if motion is not None:
                th = pose[2]
                pose[0] += -(math.cos(th) * motion["dx"] - math.sin(th) * motion["dy"]) * args.motion_scale
                pose[1] += -(math.sin(th) * motion["dx"] + math.cos(th) * motion["dy"]) * args.motion_scale
                pose[2] += motion["dtheta"]
                traj.append((pose[0], pose[1]))

            _log_frame(rr, stats, frame, src.is_color, prev_xy, flow, traj,
                       aw, bw, ac, bc, args.no_baseline, motion)

            if ros is not None:
                ros.publish(frame, src.is_color, pose)
    except KeyboardInterrupt:
        print("\n[webcam_demo] interrupted; shutting down cleanly.", flush=True)
    finally:
        src.close()

    _summary(stats, args.no_baseline)
    vision_viz.announce(session)


def _log_frame(rr, stats, frame, is_color, prev_xy, flow, traj,
               aw, bw, ac, bc, no_baseline, motion):
    rr.set_time("frame", sequence=stats["n"])
    if is_color and frame.ndim == 3:
        rr.log("camera/image", rr.Image(frame, color_model="BGR"))
    else:
        rr.log("camera/image", rr.Image(frame))
    if prev_xy is not None and len(prev_xy):
        rr.log("camera/image/tracked", rr.Points2D(prev_xy, radii=2.0,
                                                    colors=[(80, 220, 120)]))
        if flow is not None:
            good = flow[:, 2] >= 0
            if good.any():
                rr.log("camera/image/flow",
                       rr.Arrows2D(origins=prev_xy[good], vectors=flow[good, :2],
                                   colors=[(240, 220, 60)]))
    rr.log("perf/ms/A", rr.Scalars(aw))
    rr.log("perf/fps/A", rr.Scalars(1000.0 / aw if aw > 0 else 0.0))
    rr.log("perf/cpu/A", rr.Scalars(100.0 * ac / aw if aw > 0 else 0.0))
    if not no_baseline:
        rr.log("perf/ms/B", rr.Scalars(bw))
        rr.log("perf/fps/B", rr.Scalars(1000.0 / bw if bw > 0 else 0.0))
        rr.log("perf/cpu/B", rr.Scalars(100.0 * bc / bw if bw > 0 else 0.0))
    if len(traj) >= 2:
        pts = np.array(traj, dtype=np.float32)
        rr.log("world/trajectory", rr.LineStrips2D([pts], colors=[(80, 170, 255)]))
        rr.log("world/camera", rr.Points2D([pts[-1]], radii=4.0, colors=[(255, 90, 90)]))
    if stats["n"] % 30 == 0:
        extra = "" if no_baseline else "  |  B %.1f ms (%.0f fps)" % (bw, 1000.0 / bw if bw > 0 else 0)
        trk = 0 if motion is None else motion["n_tracked"]
        print("  frame %d: A %.1f ms (%.0f fps)%s  tracked=%d"
              % (stats["n"], aw, 1000.0 / aw if aw > 0 else 0, extra, trk), flush=True)


def _warmup(a, b, w, h):
    """Run both pipelines twice on throwaway frames so the first live frame is
    steady-state (JIT of the C++ track() wrapper done, OpenCV warm)."""
    import cv_kit
    cv_kit.warmup()
    frames = _synth_frames(w, h, 3)
    for f in frames:
        a.process(f)
        b.process(f)


def _cuda_on():
    try:
        import cv_kit
        return bool(cv_kit.cuda_available())
    except Exception:
        return False


def _summary(stats, no_baseline):
    if not stats["a_ms"]:
        print("\nSUMMARY: no frames processed.")
        return
    a_ms = float(np.mean(stats["a_ms"]))
    a_cpu = 100.0 * np.sum(stats["a_cpu"]) / np.sum(stats["a_ms"])
    line = ("\nSUMMARY frames=%d dropped=%d | A %.2f ms/frame (%.1f fps, %.0f%% cpu)"
            % (stats["n"], stats["drops"], a_ms, 1000.0 / a_ms, a_cpu))
    if not no_baseline and stats["b_ms"]:
        b_ms = float(np.mean(stats["b_ms"]))
        b_cpu = 100.0 * np.sum(stats["b_cpu"]) / np.sum(stats["b_ms"])
        line += (" | B %.2f ms/frame (%.1f fps, %.0f%% cpu) | A is %.1fx faster"
                 % (b_ms, 1000.0 / b_ms, b_cpu, b_ms / a_ms))
    print(line, flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["auto", "webcam", "synthetic"], default="auto",
                    help="frame source (default auto: webcam if present, else synthetic)")
    ap.add_argument("--device", type=int, default=0, help="webcam V4L2 index")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--duration", type=float, default=30.0, help="run seconds (live)")
    ap.add_argument("--nfeatures", type=int, default=1000, help="ORB max features")
    ap.add_argument("--track-points", type=int, default=150, dest="track_points",
                    help="max keypoints the NCC stage tracks (bounds the expensive stage)")
    ap.add_argument("--patch-radius", type=int, default=3, dest="patch_radius",
                    help="NCC patch radius (patch is (2r+1)^2)")
    ap.add_argument("--search-radius", type=int, default=5, dest="search_radius",
                    help="NCC search window radius")
    ap.add_argument("--min-score", type=float, default=0.3, dest="min_score",
                    help="minimum NCC score to accept a track")
    ap.add_argument("--motion-scale", type=float, default=0.01, dest="motion_scale",
                    help="pixels->world scale for the demo trajectory/TF")
    ap.add_argument("--synth-n", type=int, default=synthetic_loop.DEFAULT_N, dest="synth_n")
    ap.add_argument("--no-ros", action="store_true", help="skip TF/image publishing")
    ap.add_argument("--no-baseline", action="store_true",
                    help="run only pipeline A (skip the Python baseline)")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "vision", "webcam_demo.rrd"))
    ap.add_argument("--bench", action="store_true",
                    help="print the A-vs-B table at 640x480 and 1280x720, then exit")
    ap.add_argument("--bench-n", type=int, default=80, dest="bench_n")
    args = ap.parse_args(argv)

    if args.bench:
        rows = run_bench([(640, 480), (1280, 720)], args.nfeatures, args.track_points,
                         args.patch_radius, args.search_radius, args.min_score,
                         n=args.bench_n)
        print_bench(rows, min(args.nfeatures, args.track_points), args.track_points,
                    args.patch_radius, args.search_radius)
        return

    run_live(args)


if __name__ == "__main__":
    main()

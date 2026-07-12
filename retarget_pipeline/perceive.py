#!/usr/bin/env python
"""
perceive.py (Process A) -- the perception half of the capture rig.

    webcam -> MediaPipe HolisticLandmarker (body + hands) -> a landmark stream
    (record) -> TF frames via rclcpp_kit -> live Rerun.

The ML inference is a **library primitive** (MediaPipe, commodity Python -- the
webcam demo's honest-headline lesson: do not wrap inference in cppyy). What cppyy_kit owns here
is the *glue*: publishing the ~75 detected landmark frames onto ``/tf`` every video
frame. That message is **built in C++** by a ``cppyy.cppdef`` broadcaster that fills
all the translations from one flat address (COMMON_PATTERNS s6), instead of
constructing 75 ``TransformStamped`` proxies field-by-field in a Python loop. The
``--bench`` mode measures exactly that A-vs-B difference on a recorded stream.

Record + replay from day one:
    * live:    ``demo-perceive [--record build/pipeline/demo.jsonl]``
    * replay:  ``demo-perceive --replay build/pipeline/demo.jsonl`` (no camera, no
               MediaPipe -- re-renders the recorded landmarks to Rerun + /tf)
    * headless/CI: no webcam or no model -> the synthetic waving-skeleton scene, so
               it runs with no camera and no network (``--source synthetic``).

The retargeting half (retarget.py) consumes the stream from the standalone ``wbc``
env; the two never share a process (pinocchio vs ROS boost clash).
"""
import argparse
import os
import sys
import time

import numpy as np

os.environ.setdefault("ROS_DOMAIN_ID", "62")

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
for _p in (REPO, os.path.join(REPO, "rclcpp_kit")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from retarget_pipeline import fetch_models              # noqa: E402
from retarget_pipeline import landmark_stream as ls     # noqa: E402
from retarget_pipeline import viz                       # noqa: E402

PARENT_FRAME = "human_root"


# --------------------------------------------------------------------------- #
# MediaPipe holistic detector (the library-primitive inference).
# --------------------------------------------------------------------------- #
class HolisticDetector:
    """Wraps MediaPipe's HolisticLandmarker (Tasks API, VIDEO mode). ``detect``
    takes a BGR frame + a monotonic timestamp (ms) and returns a landmark dict
    (``pose_world`` (33,3) metres, ``pose_image`` (33,4), hands (21,3) or ``None``)."""

    def __init__(self, model_path):
        import mediapipe as mp
        from mediapipe.tasks import python as mtp
        from mediapipe.tasks.python import vision as mtv
        self._mp = mp
        opts = mtv.HolisticLandmarkerOptions(
            base_options=mtp.BaseOptions(model_asset_path=model_path),
            running_mode=mtv.RunningMode.VIDEO)
        self._landmarker = mtv.HolisticLandmarker.create_from_options(opts)

    def detect(self, bgr, t_ms):
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB,
                               data=np.ascontiguousarray(rgb))
        res = self._landmarker.detect_for_video(image, int(t_ms))
        return _result_to_landmarks(res)

    def close(self):
        try:
            self._landmarker.close()
        except Exception:
            pass


def _lm_xyz(landmarks, n):
    if not landmarks:
        return None
    return np.array([[lm.x, lm.y, lm.z] for lm in landmarks[:n]], dtype=np.float32)


def _result_to_landmarks(res):
    pose_world = _lm_xyz(res.pose_world_landmarks, ls.N_POSE)
    pose_image = None
    if res.pose_landmarks:
        pose_image = np.array([[lm.x, lm.y, lm.z, getattr(lm, "visibility", 1.0)]
                               for lm in res.pose_landmarks[:ls.N_POSE]],
                              dtype=np.float32)
    return {
        "pose_world": pose_world,
        "pose_image": pose_image,
        "left_hand": _lm_xyz(res.left_hand_landmarks, ls.N_HAND),
        "right_hand": _lm_xyz(res.right_hand_landmarks, ls.N_HAND),
    }


# --------------------------------------------------------------------------- #
# Frame sources.
# --------------------------------------------------------------------------- #
class WebcamSource:
    """A V4L2 webcam via cv2. ``read`` returns ``(ok, frame_bgr)``; a failed read
    returns ``(False, None)`` so the caller can fall back without crashing."""

    kind = "webcam"

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


def open_webcam(device, width, height):
    try:
        src = WebcamSource(device, width, height)
        return src, "webcam device %d (%dx%d)" % (device, src.width, src.height)
    except Exception as exc:
        return None, str(exc)


# --------------------------------------------------------------------------- #
# The cppyy_kit glue win: build the /tf message in C++ (COMMON_PATTERNS s6).
# --------------------------------------------------------------------------- #
_TF_GLUE = r"""
#include <string>
#include <vector>
#include <sstream>
namespace landmark_tf {

// A /tf broadcaster whose TFMessage is built and refilled entirely in C++. The
// child frame names are fixed at construction (one per tracked landmark); each
// frame the translations are memcpy-style filled from a flat (3N) float64 address
// (Pattern 6: keep the container work in C++, cross only a raw pointer). The
// Python side never touches a per-frame TransformStamped.
class Broadcaster {
  tf2_msgs::msg::TFMessage msg_;
 public:
  Broadcaster(const std::string& child_names_nl, const std::string& parent) {
    std::stringstream ss(child_names_nl);
    std::string name;
    while (std::getline(ss, name)) {
      if (name.empty()) continue;
      geometry_msgs::msg::TransformStamped ts;
      ts.header.frame_id = parent;
      ts.child_frame_id = name;
      ts.transform.rotation.w = 1.0;
      msg_.transforms.push_back(ts);
    }
  }
  void update(uintptr_t xyz, int n, int sec, unsigned int nsec) {
    const double* p = reinterpret_cast<const double*>(xyz);
    int m = (int)msg_.transforms.size();
    if (n > m) n = m;
    for (int i = 0; i < n; ++i) {
      auto& t = msg_.transforms[i];
      t.header.stamp.sec = sec;
      t.header.stamp.nanosec = nsec;
      t.transform.translation.x = p[3 * i + 0];
      t.transform.translation.y = p[3 * i + 1];
      t.transform.translation.z = p[3 * i + 2];
    }
  }
  tf2_msgs::msg::TFMessage& message() { return msg_; }
  int size() const { return (int)msg_.transforms.size(); }
};

}  // namespace landmark_tf
"""

_TF_READY = False


def _bringup_tf_glue():
    global _TF_READY
    import cppyy
    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    if not _TF_READY:
        cppyy.include("tf2_msgs/msg/tf_message.hpp")
        cppyy.include("geometry_msgs/msg/transform_stamped.hpp")
        cppyy.cppdef(_TF_GLUE)
        _TF_READY = True
    return rclcpp


class TfPublisher:
    """Publishes the landmark frames on ``/tf``. The message is built in C++
    (``landmark_tf::Broadcaster``); Python passes a flat xyz array address per frame."""

    def __init__(self, frame_names, parent=PARENT_FRAME):
        import cppyy
        import cppyy_kit
        self._rclcpp = _bringup_tf_glue()
        self._cppyy = cppyy
        self._cppyy_kit = cppyy_kit
        self._TFMessage = cppyy.gbl.tf2_msgs.msg.TFMessage
        self._node = self._rclcpp.Node("retarget_perceive")
        self._pub = self._node.create_publisher(self._TFMessage, "/tf", 10)
        self._bc = cppyy.gbl.landmark_tf.Broadcaster("\n".join(frame_names), parent)
        self.n_frames = int(self._bc.size())
        # Warm the update() call-wrapper JIT so frame 0 does not stutter.
        warm = np.zeros((self.n_frames, 3), dtype=np.float64)
        with cppyy_kit.suppress_first_use_notice():
            self._bc.update(warm.ctypes.data, self.n_frames, 0, 0)

    def publish(self, xyz):
        """xyz is (N,3) float; N should be self.n_frames. Missing rows stay 0."""
        a = np.ascontiguousarray(xyz, dtype=np.float64)
        now = time.time()
        try:
            self._bc.update(a.ctypes.data, int(a.shape[0]), int(now),
                            int((now - int(now)) * 1e9))
            self._pub.publish(self._bc.message())
        except Exception as exc:
            print("[perceive] /tf publish skipped: %s" % exc, flush=True)


def build_tf_message_python(TFMessage, TransformStamped, names, xyz, parent, sec, nsec):
    """The naive baseline for --bench: build the SAME TFMessage by constructing each
    TransformStamped proxy and setting its fields in a Python loop (the per-element
    crossing this glue exists to avoid). Returns the message."""
    msg = TFMessage()
    a = np.ascontiguousarray(xyz, dtype=np.float64)
    for i, name in enumerate(names):
        ts = TransformStamped()
        ts.header.frame_id = parent
        ts.header.stamp.sec = sec
        ts.header.stamp.nanosec = nsec
        ts.child_frame_id = name
        ts.transform.translation.x = float(a[i, 0])
        ts.transform.translation.y = float(a[i, 1])
        ts.transform.translation.z = float(a[i, 2])
        ts.transform.rotation.w = 1.0
        msg.transforms.push_back(ts)
    return msg


# --------------------------------------------------------------------------- #
# Frame-name set for the /tf tree (pose + both hands).
# --------------------------------------------------------------------------- #
def frame_names(with_hands=True):
    names = ["pose/" + n for n in ls.POSE_LANDMARK_NAMES]
    if with_hands:
        names += ["left_hand/%d" % i for i in range(ls.N_HAND)]
        names += ["right_hand/%d" % i for i in range(ls.N_HAND)]
    return names


def landmarks_to_xyz(lm, with_hands=True):
    """Flatten a landmark dict into the (N,3) robot-frame array the /tf frames map
    to (pose 33, then left hand 21, then right hand 21). Missing parts -> zeros."""
    n = ls.N_POSE + (2 * ls.N_HAND if with_hands else 0)
    out = np.zeros((n, 3), dtype=np.float64)
    if lm.get("pose_world") is not None:
        out[:ls.N_POSE] = ls.mediapipe_world_to_robot(lm["pose_world"])
    if with_hands:
        for arr, off in ((lm.get("left_hand"), ls.N_POSE),
                         (lm.get("right_hand"), ls.N_POSE + ls.N_HAND)):
            if arr is not None:
                out[off:off + len(arr)] = ls.mediapipe_world_to_robot(arr)
    return out


# --------------------------------------------------------------------------- #
# Rerun rendering (shared by live + replay).
# --------------------------------------------------------------------------- #
def log_frame(rr, i, lm, frame_bgr, detect_ms):
    rr.set_time("frame", sequence=i)
    if frame_bgr is not None:
        rr.log("camera/image", rr.Image(frame_bgr, color_model="BGR"))
        pi = lm.get("pose_image")
        if pi is not None and frame_bgr.ndim == 3:
            h, w = frame_bgr.shape[:2]
            pts = np.column_stack([pi[:, 0] * w, pi[:, 1] * h])
            rr.log("camera/image/pose", rr.Points2D(pts, radii=3.0,
                                                    colors=[(80, 220, 120)]))
            segs = [[pts[a], pts[b]] for (a, b) in ls.POSE_CONNECTIONS]
            rr.log("camera/image/skeleton", rr.LineStrips2D(segs,
                                                            colors=[(80, 220, 120)]))
    pw = lm.get("pose_world")
    if pw is not None:
        robot = ls.mediapipe_world_to_robot(pw)
        viz.log_skeleton_3d(rr, "human/pose", robot, ls.POSE_CONNECTIONS,
                            color=(120, 200, 255))
    if detect_ms is not None:
        rr.log("perf/detect", rr.Scalars(float(detect_ms)))


# --------------------------------------------------------------------------- #
# Modes.
# --------------------------------------------------------------------------- #
def run_live(args):
    session = None
    if not args.no_viz:
        import rerun as rr
        session = viz.init_rerun("retarget_perceive", args.rrd,
                                 blueprint=viz.blueprint_perceive())
    else:
        rr = None

    # Source: webcam (needs a model) or synthetic (needs neither camera nor model).
    src, detector, note = _open_live_source(args)
    print("Source: %s" % note, flush=True)
    if rr is not None:
        rr.log("log", rr.TextLog("source: " + note))

    writer = None
    if args.record:
        writer = ls.StreamWriter(args.record, source=src.kind if src else "synthetic",
                                 fps_target=args.fps)
        print("Recording landmark stream -> %s" % args.record, flush=True)

    tfpub = None
    if not args.no_ros:
        try:
            tfpub = TfPublisher(frame_names(with_hands=True))
            print("ROS: publishing %d landmark frames on /tf (built in C++)."
                  % tfpub.n_frames, flush=True)
        except Exception as exc:
            print("[perceive] /tf disabled (%s)." % exc, flush=True)

    stats = {"n": 0, "detect_ms": [], "wall": [], "cpu": [], "detected": 0, "drops": 0}
    deadline = time.monotonic() + args.duration
    synth = _synthetic_iter(args) if detector is None else None
    consecutive_fail = 0

    try:
        while time.monotonic() < deadline:
            w0, cpu0 = time.perf_counter(), time.process_time()
            if detector is not None:
                ok, frame = src.read()
                if not ok:
                    consecutive_fail += 1
                    stats["drops"] += 1
                    if consecutive_fail >= 5:
                        print("[perceive] webcam stopped; switching to synthetic.",
                              flush=True)
                        src.close()
                        src = None
                        detector.close()
                        detector = None
                        synth = _synthetic_iter(args)
                        consecutive_fail = 0
                    else:
                        time.sleep(0.005)
                    continue
                consecutive_fail = 0
                t_ms = stats["n"] * (1000.0 / args.fps)
                c0 = time.perf_counter()
                lm = detector.detect(frame, t_ms)
                detect_ms = (time.perf_counter() - c0) * 1e3
                frame_bgr = frame
            else:
                t, pw, pi, lh, rh, w, h = next(synth)
                lm = {"pose_world": pw, "pose_image": pi, "left_hand": lh,
                      "right_hand": rh}
                detect_ms = 0.0
                frame_bgr = None

            if lm.get("pose_world") is not None:
                stats["detected"] += 1

            if writer is not None:
                writer.write(t=time.monotonic(), pose_world=lm.get("pose_world"),
                             pose_image=lm.get("pose_image"),
                             left_hand=lm.get("left_hand"),
                             right_hand=lm.get("right_hand"),
                             w=(frame_bgr.shape[1] if frame_bgr is not None else 640),
                             h=(frame_bgr.shape[0] if frame_bgr is not None else 480))
            if tfpub is not None:
                tfpub.publish(landmarks_to_xyz(lm, with_hands=True))
            if rr is not None:
                log_frame(rr, stats["n"], lm, frame_bgr, detect_ms)

            stats["n"] += 1
            stats["detect_ms"].append(detect_ms)
            stats["wall"].append((time.perf_counter() - w0) * 1e3)
            stats["cpu"].append((time.process_time() - cpu0) * 1e3)
            if stats["n"] % 30 == 0:
                _progress(stats)
            if detector is None:            # pace the synthetic scene to fps
                time.sleep(max(0.0, 1.0 / args.fps - (time.perf_counter() - w0)))
    except KeyboardInterrupt:
        print("\n[perceive] interrupted; shutting down cleanly.", flush=True)
    finally:
        if src is not None:
            src.close()
        if detector is not None:
            detector.close()
        if writer is not None:
            writer.close()

    _summary(stats, args)
    if session is not None:
        viz.announce(session)


def _open_live_source(args):
    """Return ``(src, detector, note)``. Tries the webcam + a MediaPipe model; on
    any failure (no camera, no model) returns ``(None, None, note)`` and the caller
    uses the synthetic scene."""
    if args.source == "synthetic":
        return None, None, "synthetic waving-skeleton scene (forced)"
    if args.source in ("webcam", "auto"):
        src, why = open_webcam(args.device, args.width, args.height)
        if src is not None:
            model = fetch_models.ensure(args.model)
            if model is not None:
                try:
                    return src, HolisticDetector(model), \
                        "%s + MediaPipe %s" % (why, args.model)
                except Exception as exc:
                    src.close()
                    if args.source == "webcam":
                        raise
                    print("[perceive] MediaPipe init failed (%s); synthetic." % exc,
                          flush=True)
            else:
                src.close()
                if args.source == "webcam":
                    raise RuntimeError("no MediaPipe model available")
                print("[perceive] no MediaPipe model; synthetic scene.", flush=True)
        elif args.source == "webcam":
            raise RuntimeError("no webcam: %s" % why)
        else:
            print("[perceive] no webcam (%s); synthetic scene." % why, flush=True)
    return None, None, "synthetic waving-skeleton scene (no camera / no MediaPipe)"


def _synthetic_iter(args):
    def gen():
        i = 0
        dt = 1.0 / args.fps
        while True:
            t = i * dt
            pw, pi = ls.synthetic_pose(t)
            yield (t, pw, pi, None, None, 640, 480)
            i += 1
    return gen()


def run_replay(args):
    import rerun as rr
    reader = ls.StreamReader(args.replay)
    print("Replaying %s (recorded from source=%s, %d landmarks/pose)."
          % (args.replay, reader.meta.get("source"), reader.meta.get("n_pose", 0)),
          flush=True)
    session = None
    if not args.no_viz:
        session = viz.init_rerun("retarget_perceive_replay", args.rrd,
                                 blueprint=viz.blueprint_perceive())
    tfpub = None
    if not args.no_ros:
        try:
            tfpub = TfPublisher(frame_names(with_hands=True))
        except Exception as exc:
            print("[perceive] /tf disabled (%s)." % exc, flush=True)
    n = 0
    for fr in reader.frames():
        lm = {"pose_world": fr["pose_world"], "pose_image": fr["pose_image"],
              "left_hand": fr["left_hand"], "right_hand": fr["right_hand"]}
        if tfpub is not None:
            tfpub.publish(landmarks_to_xyz(lm, with_hands=True))
        if not args.no_viz:
            log_frame(rr, n, lm, None, None)
        n += 1
        if args.realtime:
            time.sleep(1.0 / args.fps)
    print("Replayed %d frames." % n, flush=True)
    if session is not None:
        viz.announce(session)


def run_bench(args):
    """A-vs-B: build the /tf message for the landmark set, C++ helper vs the
    per-field Python loop, over a recorded (or synthetic) stream."""
    import cppyy
    _bringup_tf_glue()
    TransformStamped = cppyy.gbl.geometry_msgs.msg.TransformStamped
    TFMessage = cppyy.gbl.tf2_msgs.msg.TFMessage
    names = frame_names(with_hands=True)
    n_frames = len(names)

    # Frames to bench over: replay stream if given, else synthetic.
    if args.replay:
        rows = [landmarks_to_xyz({"pose_world": fr["pose_world"],
                                  "left_hand": fr["left_hand"],
                                  "right_hand": fr["right_hand"]}, with_hands=True)
                for fr in ls.StreamReader(args.replay).frames()]
    else:
        rows = [landmarks_to_xyz({"pose_world": pw, "left_hand": None,
                                  "right_hand": None}, with_hands=True)
                for (_, pw, _, _, _, _, _) in ls.synthetic_frames(args.bench_n)]
    if not rows:
        print("bench: no frames.")
        return

    bc = cppyy.gbl.landmark_tf.Broadcaster("\n".join(names), PARENT_FRAME)
    # Warmup both paths.
    bc.update(rows[0].ctypes.data, n_frames, 0, 0)
    build_tf_message_python(TFMessage, TransformStamped, names, rows[0], PARENT_FRAME, 0, 0)

    def bench(fn, reps=3):
        best = float("inf")
        for _ in range(reps):
            t0 = time.perf_counter()
            for r in rows:
                fn(r)
            best = min(best, (time.perf_counter() - t0) / len(rows) * 1e3)
        return best

    a_ms = bench(lambda r: bc.update(r.ctypes.data, n_frames, 0, 0))
    b_ms = bench(lambda r: build_tf_message_python(TFMessage, TransformStamped,
                                                   names, r, PARENT_FRAME, 0, 0))
    print("\n== /tf-build A-vs-B (%d frames/message, %d messages) =="
          % (n_frames, len(rows)))
    print("  A  cppyy_kit C++ builder (Pattern 6): %8.4f ms/message" % a_ms)
    print("  B  per-field Python loop (cppyy proxies): %8.4f ms/message" % b_ms)
    print("  A speedup: %.1fx\n" % (b_ms / a_ms if a_ms > 0 else float("nan")))


def _progress(stats):
    ms = np.mean(stats["detect_ms"][-30:]) if stats["detect_ms"] else 0.0
    wall = np.mean(stats["wall"][-30:]) if stats["wall"] else 0.0
    fps = 1000.0 / wall if wall > 0 else 0.0
    print("  frame %d: detect %.1f ms, loop %.1f ms (%.0f fps), detected %d/%d"
          % (stats["n"], ms, wall, fps, stats["detected"], stats["n"]), flush=True)


def _summary(stats, args):
    if not stats["wall"]:
        print("\nSUMMARY: no frames.")
        return
    wall = float(np.mean(stats["wall"]))
    detect = float(np.mean(stats["detect_ms"]))
    cpu_pct = (100.0 * np.sum(stats["cpu"]) / np.sum(stats["wall"])
               if stats["wall"] and np.sum(stats["wall"]) > 0 else 0.0)
    print("\nSUMMARY frames=%d detected=%d drops=%d | loop %.2f ms/frame (%.1f fps,"
          " %.0f%% cpu) | detect %.2f ms/frame"
          % (stats["n"], stats["detected"], stats["drops"], wall,
             1000.0 / wall if wall > 0 else 0.0, cpu_pct, detect), flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["auto", "webcam", "synthetic"], default="auto")
    ap.add_argument("--device", type=int, default=0, help="webcam V4L2 index")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=float, default=30.0, help="target/synthetic fps")
    ap.add_argument("--duration", type=float, default=30.0, help="run seconds (live)")
    ap.add_argument("--model", choices=["holistic"], default="holistic")
    ap.add_argument("--record", metavar="PATH", help="write the landmark stream here")
    ap.add_argument("--replay", metavar="PATH", help="replay a recorded stream")
    ap.add_argument("--realtime", action="store_true", help="pace replay to fps")
    ap.add_argument("--no-ros", action="store_true", help="skip /tf publishing")
    ap.add_argument("--no-viz", action="store_true", help="skip Rerun")
    ap.add_argument("--bench", action="store_true",
                    help="A-vs-B /tf-build micro-bench, then exit")
    ap.add_argument("--bench-n", type=int, default=120, dest="bench_n")
    ap.add_argument("--rrd", default=os.path.join(REPO, "build", "pipeline",
                                                  "perceive.rrd"))
    args = ap.parse_args(argv)

    if args.bench:
        run_bench(args)
    elif args.replay:
        run_replay(args)
    else:
        run_live(args)


if __name__ == "__main__":
    main()

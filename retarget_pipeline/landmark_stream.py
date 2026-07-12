"""
landmark_stream -- the record/replay contract between the two pipeline processes.

Process A (perception) WRITES a stream of per-frame human landmarks; Process B
(retargeting) READS it. The stream is newline-delimited JSON (JSONL): one metadata
header line, then one frame object per line. JSONL was chosen deliberately:

  * **tailable** -- for live coupling, B follows A's file as it grows (``follow``);
  * **replayable** -- CI/headless reads a finished file (``StreamReader``);
  * **inspectable** -- ``head -1 stream.jsonl`` shows the schema; and
  * **dependency-free** -- stdlib ``json`` + numpy only, so this module imports in
    BOTH pixi envs (the ROS/MediaPipe perception env and the pinocchio ``wbc`` env,
    which share no C++ stack). Nothing here imports ROS, cppyy, mediapipe or cv2.

Coordinates follow MediaPipe's world-landmark convention: metres, origin at the
midpoint of the hips, **x** to the subject's left, **y** down, **z** toward the
camera. The retarget stage converts this to its robot frame once (see
``mediapipe_world_to_robot``); everything upstream stays in MediaPipe's frame so the
synthetic generator and the real detector produce interchangeable streams.
"""
import datetime
import json
import os
import time

import numpy as np

FORMAT = "cppyy_kit.retarget.landmarks"
VERSION = 1

# MediaPipe pose landmark indices (33-point BlazePose topology).
NOSE = 0
LEFT_EAR, RIGHT_EAR = 7, 8
LEFT_SHOULDER, RIGHT_SHOULDER = 11, 12
LEFT_ELBOW, RIGHT_ELBOW = 13, 14
LEFT_WRIST, RIGHT_WRIST = 15, 16
LEFT_HIP, RIGHT_HIP = 23, 24
LEFT_KNEE, RIGHT_KNEE = 25, 26
LEFT_ANKLE, RIGHT_ANKLE = 27, 28

POSE_LANDMARK_NAMES = [
    "NOSE", "LEFT_EYE_INNER", "LEFT_EYE", "LEFT_EYE_OUTER", "RIGHT_EYE_INNER",
    "RIGHT_EYE", "RIGHT_EYE_OUTER", "LEFT_EAR", "RIGHT_EAR", "MOUTH_LEFT",
    "MOUTH_RIGHT", "LEFT_SHOULDER", "RIGHT_SHOULDER", "LEFT_ELBOW", "RIGHT_ELBOW",
    "LEFT_WRIST", "RIGHT_WRIST", "LEFT_PINKY", "RIGHT_PINKY", "LEFT_INDEX",
    "RIGHT_INDEX", "LEFT_THUMB", "RIGHT_THUMB", "LEFT_HIP", "RIGHT_HIP",
    "LEFT_KNEE", "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE", "LEFT_HEEL",
    "RIGHT_HEEL", "LEFT_FOOT_INDEX", "RIGHT_FOOT_INDEX",
]
N_POSE = 33
N_HAND = 21

# The BlazePose skeleton edges and the 21-point hand skeleton edges, captured from
# MediaPipe so the viz can draw a skeleton without importing mediapipe (Process B's
# env has no mediapipe). These are fixed topology, not model outputs.
POSE_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 7), (0, 4), (4, 5), (5, 6), (6, 8), (9, 10),
    (11, 12), (11, 13), (13, 15), (15, 17), (15, 19), (15, 21), (17, 19),
    (12, 14), (14, 16), (16, 18), (16, 20), (16, 22), (18, 20), (11, 23),
    (12, 24), (23, 24), (23, 25), (24, 26), (25, 27), (26, 28), (27, 29),
    (28, 30), (29, 31), (30, 32), (27, 31), (28, 32),
]
HAND_CONNECTIONS = [
    (0, 1), (1, 5), (9, 13), (13, 17), (5, 9), (0, 17), (1, 2), (2, 3), (3, 4),
    (5, 6), (6, 7), (7, 8), (9, 10), (10, 11), (11, 12), (13, 14), (14, 15),
    (15, 16), (17, 18), (18, 19), (19, 20),
]


# --------------------------------------------------------------------------- #
# Encoding helpers: numpy <-> JSON-friendly nested lists (rounded, compact).
# --------------------------------------------------------------------------- #
def _enc(arr, ndigits=5):
    """A (N,K) float array -> rounded nested list, or ``None`` passthrough."""
    if arr is None:
        return None
    a = np.asarray(arr, dtype=np.float64)
    return [[round(float(v), ndigits) for v in row] for row in a]


def _dec(obj, dtype=np.float32):
    """Nested list -> float32 array, or ``None`` passthrough."""
    if obj is None:
        return None
    return np.asarray(obj, dtype=dtype)


# --------------------------------------------------------------------------- #
# Writer / Reader.
# --------------------------------------------------------------------------- #
class StreamWriter:
    """Write a landmark stream to ``path`` (JSONL). Use as a context manager.

    ``source`` is a short free-text note (``"webcam"`` / ``"synthetic"`` / ...).
    Each :meth:`write` flushes, so a tailing reader sees frames immediately.
    """

    def __init__(self, path, source, fps_target=30.0, extra=None):
        self.path = path
        d = os.path.dirname(os.path.abspath(path))
        if d:
            os.makedirs(d, exist_ok=True)
        self._f = open(path, "w")
        self.meta = {
            "kind": "meta", "format": FORMAT, "version": VERSION, "source": source,
            "fps_target": float(fps_target),
            "created": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "n_pose": N_POSE, "n_hand": N_HAND,
        }
        if extra:
            self.meta.update(extra)
        self._write_line(self.meta)
        self._i = 0

    def _write_line(self, obj):
        self._f.write(json.dumps(obj, separators=(",", ":")) + "\n")
        self._f.flush()

    def write(self, t, pose_world=None, pose_image=None, left_hand=None,
              right_hand=None, w=0, h=0):
        """Append one frame. ``pose_world`` is (33,3) metres; ``pose_image`` is
        (33,4) [x,y,visibility,presence] in normalized image coords; the hands are
        (21,3) each. Any of them may be ``None`` (not detected this frame).
        Returns the frame index written."""
        rec = {
            "kind": "frame", "i": self._i, "t": round(float(t), 4),
            "present": pose_world is not None, "w": int(w), "h": int(h),
            "pose_world": _enc(pose_world), "pose_image": _enc(pose_image),
            "left_hand": _enc(left_hand), "right_hand": _enc(right_hand),
        }
        self._write_line(rec)
        self._i = self._i + 1
        return self._i - 1

    @property
    def n_written(self):
        return self._i

    def close(self):
        if self._f is not None and not self._f.closed:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def _decode_frame(obj):
    """A parsed frame JSON object -> a dict with numpy arrays (or ``None``)."""
    return {
        "i": int(obj.get("i", 0)),
        "t": float(obj.get("t", 0.0)),
        "present": bool(obj.get("present", False)),
        "w": int(obj.get("w", 0)),
        "h": int(obj.get("h", 0)),
        "pose_world": _dec(obj.get("pose_world")),
        "pose_image": _dec(obj.get("pose_image")),
        "left_hand": _dec(obj.get("left_hand")),
        "right_hand": _dec(obj.get("right_hand")),
    }


class StreamReader:
    """Read a finished landmark stream. ``reader.meta`` is the header dict;
    iterate :meth:`frames` (or :meth:`read_all`) for decoded frame dicts."""

    def __init__(self, path):
        self.path = path
        with open(path) as f:
            first = f.readline()
        if not first:
            raise ValueError("empty landmark stream: %r" % path)
        self.meta = json.loads(first)
        if self.meta.get("format") != FORMAT:
            raise ValueError("not a %s stream: %r (got format=%r)"
                             % (FORMAT, path, self.meta.get("format")))

    def frames(self):
        with open(self.path) as f:
            f.readline()  # skip meta
            for line in f:
                line = line.strip()
                if not line:
                    continue
                obj = json.loads(line)
                if obj.get("kind") == "frame":
                    yield _decode_frame(obj)

    def read_all(self):
        return list(self.frames())


def follow(path, idle_timeout=2.0, poll=0.02):
    """Tail a landmark stream that another process is still writing (live
    coupling). Yields decoded frame dicts as they appear; returns when no new line
    arrives for ``idle_timeout`` seconds (the writer stopped). Skips the meta line.
    """
    # Wait for the file to exist and its meta line to be flushed.
    deadline = time.monotonic() + idle_timeout
    while not os.path.exists(path):
        if time.monotonic() > deadline:
            return
        time.sleep(poll)
    with open(path) as f:
        f.readline()  # meta (may be partial on first pass; re-read below if so)
        last = time.monotonic()
        buf = ""
        while True:
            line = f.readline()
            if not line:
                if time.monotonic() - last > idle_timeout:
                    return
                time.sleep(poll)
                continue
            if not line.endswith("\n"):     # partial write; stash and retry
                buf += line
                continue
            line = (buf + line).strip()
            buf = ""
            last = time.monotonic()
            if not line:
                continue
            obj = json.loads(line)
            if obj.get("kind") == "frame":
                yield _decode_frame(obj)


# --------------------------------------------------------------------------- #
# Coordinate conversion: MediaPipe world frame -> a robot-friendly frame.
# --------------------------------------------------------------------------- #
def mediapipe_world_to_robot(points):
    """Convert MediaPipe world landmarks (x left, y down, z toward camera) into a
    robot frame (x forward, y left, z up), origin unchanged (hip midpoint).

    Mapping: robot_x = -mp_z (camera-toward -> back, so negate for forward),
    robot_y = mp_x (subject-left stays left), robot_z = -mp_y (image-down -> up).
    ``points`` is (N,3); returns (N,3) float32."""
    p = np.asarray(points, dtype=np.float32)
    out = np.empty_like(p)
    out[:, 0] = -p[:, 2]
    out[:, 1] = p[:, 0]
    out[:, 2] = -p[:, 1]
    return out


# --------------------------------------------------------------------------- #
# Synthetic landmark source: a standing person waving both arms. Produces
# MediaPipe-world-convention landmarks with NO camera and NO mediapipe, so the
# perception demo runs headless (CI / rehearsal / no-webcam), and the retarget has
# real inter-frame motion to track. The pose_image projection is orthographic.
# --------------------------------------------------------------------------- #
# A neutral standing skeleton in MediaPipe world convention (metres, hip-centered,
# x=subject-left, y=down, z=toward-camera). Only the retarget-relevant joints need
# to be accurate; the rest are placed plausibly so the skeleton looks right.
# Rows are indexed by the BlazePose landmark index (0..32); see POSE_LANDMARK_NAMES.
_BASE_POSE = np.array([
    (0.0, -0.62, 0.06),      # 0  NOSE
    (0.03, -0.65, 0.05),     # 1  LEFT_EYE_INNER
    (0.035, -0.65, 0.05),    # 2  LEFT_EYE
    (0.04, -0.65, 0.05),     # 3  LEFT_EYE_OUTER
    (-0.03, -0.65, 0.05),    # 4  RIGHT_EYE_INNER
    (-0.035, -0.65, 0.05),   # 5  RIGHT_EYE
    (-0.04, -0.65, 0.05),    # 6  RIGHT_EYE_OUTER
    (0.07, -0.63, 0.0),      # 7  LEFT_EAR
    (-0.07, -0.63, 0.0),     # 8  RIGHT_EAR
    (0.03, -0.58, 0.05),     # 9  MOUTH_LEFT
    (-0.03, -0.58, 0.05),    # 10 MOUTH_RIGHT
    (0.18, -0.48, 0.0),      # 11 LEFT_SHOULDER
    (-0.18, -0.48, 0.0),     # 12 RIGHT_SHOULDER
    (0.30, -0.25, 0.0),      # 13 LEFT_ELBOW
    (-0.30, -0.25, 0.0),     # 14 RIGHT_ELBOW
    (0.34, -0.02, 0.0),      # 15 LEFT_WRIST
    (-0.34, -0.02, 0.0),     # 16 RIGHT_WRIST
    (0.35, 0.03, 0.0),       # 17 LEFT_PINKY
    (-0.35, 0.03, 0.0),      # 18 RIGHT_PINKY
    (0.36, 0.03, 0.0),       # 19 LEFT_INDEX
    (-0.36, 0.03, 0.0),      # 20 RIGHT_INDEX
    (0.35, 0.01, 0.0),       # 21 LEFT_THUMB
    (-0.35, 0.01, 0.0),      # 22 RIGHT_THUMB
    (0.10, 0.0, 0.0),        # 23 LEFT_HIP
    (-0.10, 0.0, 0.0),       # 24 RIGHT_HIP
    (0.11, 0.42, 0.02),      # 25 LEFT_KNEE
    (-0.11, 0.42, 0.02),     # 26 RIGHT_KNEE
    (0.11, 0.82, 0.0),       # 27 LEFT_ANKLE
    (-0.11, 0.82, 0.0),      # 28 RIGHT_ANKLE
    (0.11, 0.85, -0.03),     # 29 LEFT_HEEL
    (-0.11, 0.85, -0.03),    # 30 RIGHT_HEEL
    (0.11, 0.85, 0.12),      # 31 LEFT_FOOT_INDEX
    (-0.11, 0.85, 0.12),     # 32 RIGHT_FOOT_INDEX
], dtype=np.float32)


def synthetic_pose(t):
    """The standing skeleton at time ``t`` (seconds), both arms waving. Returns
    (pose_world (33,3) metres, pose_image (33,4) normalized). MediaPipe convention."""
    p = _BASE_POSE.copy()
    import math
    # Wave: elbows and wrists swing up/down and out, mirrored across the body.
    swing = 0.28 * math.sin(2.0 * math.pi * 0.35 * t)          # vertical
    reach = 0.10 * math.sin(2.0 * math.pi * 0.5 * t + 0.7)     # forward (toward cam)
    for elbow, wrist, sign in ((LEFT_ELBOW, LEFT_WRIST, 1.0),
                               (RIGHT_ELBOW, RIGHT_WRIST, -1.0)):
        p[elbow] = p[elbow] + (0.0, -0.6 * swing, reach)
        p[wrist] = p[wrist] + (sign * 0.02, -1.0 * swing, reach + 0.05)
    # Slight torso sway so the head/shoulders move a little too.
    sway = 0.03 * math.sin(2.0 * math.pi * 0.2 * t)
    p[:, 0] = p[:, 0] + sway
    # Orthographic image projection: x_img in [0,1] from world x (flip: image x
    # grows right = subject-left negative), y_img from world y. Visibility=1.
    img = np.ones((N_POSE, 4), dtype=np.float32)
    img[:, 0] = np.clip(0.5 - p[:, 0] * 0.7, 0.0, 1.0)
    img[:, 1] = np.clip(0.5 + p[:, 1] * 0.7, 0.0, 1.0)
    img[:, 2] = 0.0
    return p, img


def synthetic_frames(n, fps=30.0):
    """Yield ``n`` synthetic frames as tuples suitable for :meth:`StreamWriter.write`:
    ``(t, pose_world, pose_image, left_hand, right_hand, w, h)`` (hands are ``None``)."""
    dt = 1.0 / float(fps)
    for i in range(n):
        t = i * dt
        pw, pi = synthetic_pose(t)
        yield (t, pw, pi, None, None, 640, 480)

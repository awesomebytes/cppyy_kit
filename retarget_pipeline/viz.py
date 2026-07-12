"""
viz -- Rerun setup for the retarget pipeline, reusing the vision demos' tested logic.

``cv_kit/demos/vision_viz.py`` already solved the hard parts of driving Rerun on
this stage machine: the live-vs-headless decision (:func:`should_spawn`), spawning
the *native* viewer binary bundled in the wheel (the ``rerun`` console script is a
broken shim in this env), and degrading to a ``.rrd`` when no window can open. Both
pipeline processes reuse it verbatim (it imports only ``rerun`` + stdlib, so it loads
in the perception env AND the pinocchio ``wbc`` env). This module adds only the
pipeline blueprints and a couple of small skeleton-logging helpers shared by both.
"""
import os
import sys

import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "cv_kit", "demos"))

import vision_viz  # noqa: E402

# Re-export the tested primitives so callers use one import.
init_rerun = vision_viz.init_rerun
announce = vision_viz.announce
should_spawn = vision_viz.should_spawn

# Single shared viewer (live ROS demo): both processes rr.init with the SAME
# recording id so their streams merge into one recording; one process spawns the
# viewer, the other connects to it over gRPC. rr.spawn serves gRPC on this port.
SHARED_RECORDING_ID = "retarget_live"
DEFAULT_VIEWER_URL = "rerun+http://127.0.0.1:9876/proxy"


def init_rerun_shared(app_id, role, blueprint, recording_id=SHARED_RECORDING_ID,
                      url=None, rrd_path=None):
    """Set up Rerun for the single-viewer live demo. ``role='spawn'`` opens the
    viewer and streams into it (perception, the camera anchor); ``role='connect'``
    attaches to that already-running viewer (retarget). Both pass the SAME
    ``recording_id`` so their entities land in one recording -> one window.

    Falls back to the normal per-process headless ``.rrd`` when no live viewer is
    wanted (no display / under pytest / RCLCPPYY_RERUN_SPAWN=0), since a shared
    window is only meaningful live. Returns a :class:`vision_viz.VizSession`.
    """
    import rerun as rr
    if not vision_viz.should_spawn():
        return vision_viz.init_rerun(app_id, rrd_path or os.path.join(
            _REPO, "build", "pipeline", "%s.rrd" % role), blueprint=blueprint)
    if role == "connect":
        rr.init(app_id, recording_id=recording_id)
        rr.connect_grpc(url or DEFAULT_VIEWER_URL, default_blueprint=blueprint)
        return vision_viz.VizSession("connect", None)
    rr.init(app_id, recording_id=recording_id, default_blueprint=blueprint)
    exe = vision_viz.native_viewer_path()
    try:
        rr.spawn(executable_path=exe) if exe else rr.spawn()
    except Exception as exc:  # pragma: no cover - environment dependent
        sys.stderr.write("[viz] shared viewer spawn failed (%s); headless .rrd.\n" % exc)
        return vision_viz.init_rerun(app_id, rrd_path or os.path.join(
            _REPO, "build", "pipeline", "%s.rrd" % role), blueprint=blueprint)
    return vision_viz.VizSession("spawn", None)


def blueprint_shared():
    """The one-window layout for the live ROS demo: camera + landmark overlay and the
    per-frame perf plot on the left; a single 3D scene on the right showing the human
    demonstration skeleton (logged by perception) and the retargeted humanoid + targets
    (logged by retarget) overlaid -- everything in one viewer."""
    import rerun.blueprint as rrb
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial2DView(origin="/camera", name="camera + landmarks"),
                rrb.TimeSeriesView(origin="/perf", name="perf (ms/frame)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[3, 1, 1],
            ),
            rrb.Spatial3DView(origin="/", name="human demonstration + retargeted robot",
                              contents=["/human/**", "/robot/**"]),
            column_shares=[3, 4],
        ),
        collapse_panels=True,
    )


def blueprint_perceive():
    """Perception layout (Process A): the live camera with the 2D landmark skeleton
    overlaid, beside the 3D world-landmark skeleton, the per-frame detect-time plot,
    and an event log."""
    import rerun.blueprint as rrb
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera", name="camera + landmarks"),
            rrb.Vertical(
                rrb.Spatial3DView(origin="/human", name="human world skeleton"),
                rrb.TimeSeriesView(origin="/perf", name="detect time (ms/frame)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[3, 1, 1],
            ),
            column_shares=[3, 3],
        ),
        collapse_panels=True,
    )


def blueprint_retarget():
    """Retarget layout (Process B): the humanoid (retargeted) beside the human world
    skeleton it is tracking, plus the per-frame solve-time plot and an event log."""
    import rerun.blueprint as rrb
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/robot", name="retargeted humanoid"),
            rrb.Vertical(
                rrb.Spatial3DView(origin="/human", name="human demonstration"),
                rrb.TimeSeriesView(origin="/perf", name="CLIK solve (ms/frame)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[3, 1, 1],
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


def log_skeleton_3d(rr, entity, points, connections, color=(120, 200, 255),
                    radius=0.012):
    """Log a 3D point skeleton as joints (points) + bones (line strips)."""
    p = np.asarray(points, dtype=np.float32)
    rr.log(entity + "/joints", rr.Points3D(p, radii=radius, colors=[color]))
    segs = [[p[a], p[b]] for (a, b) in connections
            if a < len(p) and b < len(p)]
    if segs:
        rr.log(entity + "/bones", rr.LineStrips3D(segs, colors=[color]))


def log_targets_3d(rr, entity, targets, color=(255, 140, 60), radius=0.03):
    """Log the retarget end-effector targets as fat 3D points."""
    if not targets:
        return
    p = np.asarray(list(targets.values()), dtype=np.float32)
    rr.log(entity, rr.Points3D(p, radii=radius, colors=[color]))

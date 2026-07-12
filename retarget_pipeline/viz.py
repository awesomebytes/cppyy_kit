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

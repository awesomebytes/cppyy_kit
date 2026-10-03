"""Shared Rerun setup for the vision demos.

When a display is available and the process is not running under pytest, demos open
a live viewer by default. Set `RCLCPPYY_RERUN_SPAWN=1` to force the viewer or `=0`
to write a headless `.rrd` recording. The same decision applies to each demo.

The `rerun` console command in this environment uses a Python shim that cannot
import its native bindings because of a `rerun_bindings` symbol mismatch. The demos
therefore start the native viewer binary included in the `rerun_sdk` package. If it
cannot be found or started, the demo writes a headless recording instead.

Demos use stable entity roots such as `camera/`, `perf/`, `loop/`, and `world/`.
Shared blueprint functions set the panel layout."""
import os
import sys
from collections import namedtuple

import rerun as rr

# What init_rerun returns so the caller can print the right "how to view" line.
#   mode: "spawn" (live viewer) or "headless" (.rrd on disk)
#   rrd:  the .rrd path in headless mode, else None
VizSession = namedtuple("VizSession", "mode rrd")


def under_pytest():
    """Return `True` when the process is running under pytest. Tests should not open
the live viewer."""
    return "PYTEST_CURRENT_TEST" in os.environ or "pytest" in sys.modules


def should_spawn(env=None, in_pytest=None):
    """Choose the live viewer or headless `.rrd` output.

`RCLCPPYY_RERUN_SPAWN` selects the mode when set. Otherwise, open the viewer only
when a display is available and the process is not running under pytest. The
function has no side effects; pass `env` and `in_pytest` to test the decision."""
    env = os.environ if env is None else env
    forced = env.get("RCLCPPYY_RERUN_SPAWN")
    if forced is not None and forced.strip() != "":
        return forced.strip() != "0"
    if in_pytest is None:
        in_pytest = under_pytest()
    if in_pytest:
        return False
    return bool(env.get("DISPLAY") or env.get("WAYLAND_DISPLAY"))


def native_viewer_path():
    """Return the path to the native Rerun viewer bundled with `rerun_sdk`, if found.
The `rerun` console script may fail to import the native bindings in this environment."""
    try:
        pkg = os.path.dirname(os.path.abspath(rr.__file__))          # .../rerun_sdk/rerun
        cand = os.path.join(os.path.dirname(pkg), "rerun_cli", "rerun")
        if os.path.isfile(cand) and os.access(cand, os.X_OK):
            return cand
    except Exception:
        pass
    # Fallback: search under the active env prefix.
    import glob
    prefix = os.environ.get("CONDA_PREFIX", sys.prefix)
    for cand in glob.glob(os.path.join(prefix, "lib", "python*", "site-packages",
                                       "rerun_sdk", "rerun_cli", "rerun")):
        if os.access(cand, os.X_OK):
            return cand
    return None


def init_rerun(app_id, rrd_path, blueprint=None, env=None):
    """Initialize Rerun for a demo and return a `VizSession`.

In viewer mode, start the native viewer. In headless mode, save an `.rrd` recording
at `rrd_path`."""
    env = os.environ if env is None else env
    spawn = should_spawn(env)
    rr.init(app_id, default_blueprint=blueprint)
    if spawn:
        exe = native_viewer_path()
        try:
            if exe is not None:
                rr.spawn(executable_path=exe)
            else:
                rr.spawn()
            return VizSession("spawn", None)
        except Exception as exc:  # pragma: no cover - environment dependent
            sys.stderr.write(
                "[vision_viz] could not open the live Rerun viewer (%s); "
                "falling back to a headless .rrd. Force headless with "
                "RCLCPPYY_RERUN_SPAWN=0.\n" % exc)
    os.makedirs(os.path.dirname(rrd_path), exist_ok=True)
    rr.save(rrd_path)
    return VizSession("headless", rrd_path)


def announce(session):
    """Print the selected Rerun output mode."""
    if session.mode == "spawn":
        print("Rerun: live viewer opened -- watch it stream. "
              "(headless instead: RCLCPPYY_RERUN_SPAWN=0)", flush=True)
    else:
        print("Rerun recording saved: %s  (open with: rerun %s)"
              % (session.rrd, session.rrd), flush=True)


# --- Blueprints: a comprehensible default layout per demo --------------------
# Kept here (not in the demos) because "how the viewer is arranged" is a viz
# concern, and sharing the builders keeps the entity-path vocabulary consistent.

def _rrb():
    import rerun.blueprint as rrb
    return rrb


def blueprint_camera_perf(perf_title="processing time (ms/frame)"):
    """Set the layout for the camera image and per-frame processing-time plot."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera", name="camera + keypoints"),
            rrb.TimeSeriesView(origin="/perf", name=perf_title),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )


def blueprint_loop():
    """Set the layout for the camera, loop score, matched-image pair, and event log."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera", name="live camera + ORB"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/perf", name="processing time (ms/frame)"),
                rrb.Horizontal(
                    rrb.Spatial2DView(origin="/loop/pair/query", name="loop: current"),
                    rrb.Spatial2DView(origin="/loop/pair/match", name="loop: revisited"),
                ),
                rrb.TimeSeriesView(origin="/loop/score", name="confirmed loop score"),
                rrb.TextLogView(origin="/loop/events", name="loop events"),
                row_shares=[2, 2, 1, 1],
            ),
            column_shares=[3, 3],
        ),
        collapse_panels=True,
    )


def blueprint_webcam_ab():
    """Set the layout for the webcam image, tracked features, flow vectors, pipeline timing,
CPU use, and accumulated trajectory."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="/camera",
                              name="live camera -- tracked features + flow (pipeline A)"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/perf/ms",
                                   name="processing time ms/frame (A=cppyy vs B=python)"),
                rrb.TimeSeriesView(origin="/perf/fps", name="achievable FPS (1000/ms)"),
                rrb.TimeSeriesView(origin="/perf/cpu", name="process CPU %"),
                rrb.Spatial2DView(origin="/world", name="camera trajectory (pipeline A)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[2, 1, 1, 2, 1],
            ),
            column_shares=[3, 3],
        ),
        collapse_panels=True,
    )


def blueprint_posegraph():
    """Set the layout for the ground-truth, drifted, and corrected trajectories and error
plot."""
    rrb = _rrb()
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="/world", name="trajectories + loop edges"),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="/error", name="mean position error (m)"),
                rrb.TextLogView(origin="/log", name="events"),
                row_shares=[3, 1],
            ),
            column_shares=[3, 2],
        ),
        collapse_panels=True,
    )

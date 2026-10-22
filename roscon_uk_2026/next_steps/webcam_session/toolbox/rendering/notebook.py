"""Persistent inference with independent native image and optional 3D viewers."""

import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup:
    import os
    import time
    import cv2
    import marimo as mo
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.session import Workbench
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.imgui_view import ImGuiViewer
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.viser_view import ViserViewer


@app.cell
def session():
    # A paced image source allows rehearsal without taking another session's camera.
    _path = os.environ.get('PERCEPTION_REPLAY_IMAGE')
    if _path:
        _image = cv2.imread(_path)
        if _image is None:
            raise ValueError(f'Cannot read {_path}')

        def _source():
            time.sleep(1 / 30)
            return _image.copy()

        workbench = Workbench(source=_source)
    else:
        workbench = Workbench(camera=0)
    return (workbench,)


@app.cell
def processing(workbench):
    def process(frame, state, controls):
        if controls['enabled']:
            cv2.putText(frame, f"{controls['label']} | {controls['threshold']:.2f}",
                        (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    workbench.set_processor(process)
    return


@app.cell
def native_view(workbench):
    viewer = ImGuiViewer(workbench)
    return


@app.cell
def instructions():
    mo.md("""
    # Interactive perception workbench

    The native window has raw/processed views, controls and image picking.
    Keep the session and viewer cells alive. Replace only processing or extra
    GUI callbacks. Inspect `workbench.status()` and `viewer.status()`.

    For 3D, create `scene = ViserViewer(workbench, port=8768)` in a new cell.
    Return `points` and optional `colors` alongside `image` from processing.
    Close `scene`, `viewer`, then `workbench` when finished.
    """)
    return


if __name__ == "__main__":
    app.run()

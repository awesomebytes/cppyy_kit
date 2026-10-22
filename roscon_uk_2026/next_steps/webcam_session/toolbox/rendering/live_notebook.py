"""Live-coding starting point: one camera, one processor and one viewer."""
import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup:
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.session import Workbench
    from roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.viser_view import ViserViewer


@app.cell
def capture():
    workbench = Workbench(camera=0)
    return (workbench,)


@app.cell
def processing(workbench):
    def process(frame, state, controls):
        return frame

    workbench.set_processor(process)
    return


@app.cell
def display(workbench):
    viewer = ViserViewer(workbench, port=8768)
    return (viewer,)


if __name__ == "__main__":
    app.run()

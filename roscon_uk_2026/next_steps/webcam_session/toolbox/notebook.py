"""Fresh optional-toolbox session; models belong in retained setup cells."""

import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup:
    import cv2
    import marimo as mo
    from roscon_uk_2026.next_steps.webcam_session.live import LiveCamera


@app.cell
def camera_session():
    camera = LiveCamera(camera=0, port=8767)
    return (camera,)


@app.cell
def processing(camera):
    def process(frame, state):
        output = frame.copy()
        cv2.putText(output, 'Toolbox session: ready', (16, 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return output

    camera.set_processor(process, label='toolbox starter')
    return


@app.cell
def introduction():
    mo.md("""
    # Interactive perception toolbox

    The viewer is at http://127.0.0.1:8767. Keep the camera cell running. Load models in separate retained cells and replace only the processing function. See `toolbox/README.md` for adapters.
    """)
    return


if __name__ == "__main__":
    app.run()

import marimo

__generated_with = "0.25.2"
app = marimo.App(width="medium")

with app.setup(hide_code=True):
    import os
    from pathlib import Path
    import marimo as mo
    from roscon_uk_2026.next_steps.agent_session.perception import (
        load_snapshot, process, overlay, serializable,
    )


@app.cell(hide_code=True)
def introduction():
    mo.md("""
    # Inspect recorded table objects

    This notebook retains one MCAP snapshot in memory. Change the color or region
    controls to rerun PCL clustering and OpenCV association on the same data.
    The source recording remains unchanged. Coordinates and region limits use
    `base_link`, in metres. This notebook does not command the robot.
    """)
    return


@app.cell(hide_code=True)
def recording():
    recording_path = Path("roscon_uk_2026/next_steps/agent_session/data/table_scene")
    snapshot = load_snapshot(recording_path)
    retained_state = {
        "pid": os.getpid(),
        "snapshot_id": id(snapshot),
        "points_id": id(snapshot.points),
    }
    return retained_state, snapshot


@app.cell(hide_code=True)
def controls():
    color = mo.ui.dropdown(
        options=["all", "red", "green", "blue", "yellow"],
        value="red", label="Object color",
    )
    forward = mo.ui.range_slider(
        start=0.3, stop=1.6, step=0.01, value=(0.75, 1.02),
        label="Forward region (m)",
    )
    lateral = mo.ui.range_slider(
        start=-0.6, stop=0.6, step=0.01, value=(0.1, 0.3),
        label="Lateral region (m)",
    )
    height = mo.ui.range_slider(
        start=0.7, stop=1.4, step=0.01, value=(0.9, 1.08),
        label="Height region (m)",
    )
    voxel = mo.ui.slider(
        start=0.002, stop=0.02, step=0.001, value=0.004,
        label="Voxel size (m)",
    )
    minimum_cluster = mo.ui.slider(
        start=5, stop=100, step=1, value=20,
        label="Minimum cluster points",
    )
    mo.vstack([mo.hstack([color, minimum_cluster, voxel]), forward, lateral, height])
    return color, forward, height, lateral, minimum_cluster, voxel


@app.cell(hide_code=True)
def analysis(
    color,
    forward,
    height,
    lateral,
    minimum_cluster,
    snapshot,
    voxel,
):
    result = process(
        snapshot,
        roi=(
            forward.value[0], lateral.value[0], height.value[0],
            forward.value[1], lateral.value[1], height.value[1],
        ),
        color=None if color.value == "all" else color.value,
        min_cluster=minimum_cluster.value, leaf=voxel.value,
    )
    return (result,)


@app.cell(hide_code=True)
def view(result, retained_state, snapshot):
    _selection = result["selected"]
    _summary = (
        f"{len(result['observations'])} object(s); "
        f"{result['voxel_points']} voxel points. "
        f"Selected: {_selection['color'] if _selection else 'none'}."
    )
    mo.vstack([
        mo.image(overlay(snapshot, result), width=640),
        mo.md(_summary),
        mo.md(f"Retained kernel and arrays: `{retained_state}`"),
        mo.accordion({"Perception result": mo.json(serializable(result))}),
    ])
    return


if __name__ == "__main__":
    app.run()

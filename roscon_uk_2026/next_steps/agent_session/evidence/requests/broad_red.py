"""Repeat request 2 as a durable cell edit after the dropdown update failed."""
import marimo._code_mode as cm

async with cm.get_context() as ctx:
    controls_cell = ctx.cells["controls"]
    updated_controls = controls_cell.code
    updated_controls = updated_controls.replace("value=(0.75, 1.02)", "value=(0.5, 1.3)")
    updated_controls = updated_controls.replace("value=(0.1, 0.3)", "value=(-0.5, 0.5)")
    updated_controls = updated_controls.replace("value=(0.9, 1.08)", "value=(0.85, 1.3)")
    updated_controls = updated_controls.replace("value=0.004", "value=0.008")
    updated_controls = updated_controls.replace("value=20", "value=12")
    ctx.edit_cell(controls_cell.id, updated_controls)
    ctx.run_cell(controls_cell.id)

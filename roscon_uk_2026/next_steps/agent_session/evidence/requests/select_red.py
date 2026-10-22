"""Request 2: select red while retaining the broad region and loaded snapshot."""
import marimo._code_mode as cm

async with cm.get_context() as ctx:
    ctx.set_ui_value(color, "red")

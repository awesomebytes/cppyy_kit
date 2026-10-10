"""Exercise: fill only validate_and_export. Native declarations stay unchanged."""
from pathlib import Path
from . import FilterConfig, export_config, native_namespace


def validate_and_export(values: dict, config_path: Path):
    """Return (validated FilterConfig, populated reverse_demo.Config).

    Reject invalid values before native_namespace() is called. Save defaults
    and explicit fields in the documented resolved configuration format.
    """
    raise NotImplementedError("validate values, adapt existing Config fields, export resolved settings")

"""Exercise: fill only validate_and_export. Native declarations stay unchanged."""
from pathlib import Path
from roscon_uk_2026.next_steps.reverse_core import FilterConfig, export_config, native_namespace


def validate_and_export(values: dict, config_path: Path):
    """Return (validated FilterConfig, populated reverse_demo.Config).

    Reject invalid values before native_namespace() is called. Save defaults
    and explicit fields in the documented resolved configuration format.
    """
    config = FilterConfig.model_validate(values)
    native_config = native_namespace().Config()
    native_config.time_constant_s = config.tau_s
    native_config.reset_gap_s = config.max_gap_s
    native_config.output_frame = config.frame_id
    export_config(config, config_path)
    return config, native_config

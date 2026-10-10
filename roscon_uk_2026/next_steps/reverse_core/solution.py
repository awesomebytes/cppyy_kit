"""Saved solution to the explicit configuration-adapter exercise."""
from pathlib import Path
from . import FilterConfig, export_config, native_namespace


def validate_and_export(values: dict, config_path: Path):
    resolved = FilterConfig.model_validate(values)
    native = native_namespace().Config()
    native.time_constant_s = resolved.tau_s
    native.reset_gap_s = resolved.max_gap_s
    native.output_frame = resolved.frame_id
    native.validate()
    export_config(resolved, config_path)
    return resolved, native

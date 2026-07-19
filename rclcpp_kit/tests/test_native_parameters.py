import json

from _run_helper import format_output, run_helper


REPORT_PREFIX = "NATIVE_PARAMETERS_REPORT="


def test_native_parameter_factories_node_operations_and_callbacks():
    proc = run_helper("_native_parameters_helper.py")
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    report = json.loads(lines[0][len(REPORT_PREFIX):])
    assert report == {
        "schema": "rclcpp_kit.native-parameters-proof/v1",
        "ros_distribution": "jazzy",
        "rmw": "rmw_cyclonedds_cpp",
        "native_owner": "rclcpp::Parameter",
        "factory_cases": [
            {"case": "none", "type": 0},
            {"case": "bool", "type": 1},
            {"case": "integer", "type": 2},
            {"case": "double", "type": 3},
            {"case": "string", "type": 4},
            {"case": "bytes", "type": 5},
            {"case": "bools", "type": 6},
            {"case": "integers", "type": 7},
            {"case": "doubles", "type": 8},
            {"case": "strings", "type": 9},
        ],
        "empty_array_types": [5, 6, 7, 8, 9],
        "callbacks": {
            "on_exception": {
                "calls": 1, "exceptions": 1, "rejections": 1},
            "pre_exception": {
                "calls": 1, "exceptions": 1, "rejections": 0},
            "post_exception": {
                "calls": 1, "exceptions": 1, "rejections": 0},
            "pre_replacement": {
                "preserved_when_returned": True,
                "empty_rejected": True,
            },
        },
        "helper_idempotent": True,
        "session_cycles": 2,
        "retained_after_teardown": True,
        "application_message_conversions": 0,
        "serialization_operations": 0,
    }

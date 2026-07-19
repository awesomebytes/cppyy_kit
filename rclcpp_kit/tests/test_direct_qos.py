"""Live Jazzy/Cyclone proof for strict direct-entity QoS lowering."""

import json
from pathlib import Path

from _run_helper import format_output, run_helper


PREFIX = "DIRECT_QOS_REPORT="
INFINITE_DURATION_NS = 9_223_372_036_854_775_807


def test_direct_qos_profiles_are_lowered_before_native_entity_creation():
    process = run_helper("_direct_qos_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    records = [
        line for line in process.stdout.splitlines()
        if line.startswith(PREFIX)
    ]
    assert len(records) == 1, format_output(process)
    report = json.loads(records[0][len(PREFIX):])
    assert report["schema"] == "rclcpp_kit.direct-qos-proof/v1"
    assert report["ros_distribution"] == "jazzy"
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["configuration_only"] is True
    assert report["message_conversion_calls"] == 0
    assert report["zero_depth_lowered"]["history"] == 1
    assert report["zero_depth_lowered"]["depth"] == 0
    assert report["avoid_true_lowered"] == {
        "history": 1,
        "depth": 3,
        "reliability": 1,
        "durability": 2,
        "deadline_ns": 0,
        "lifespan_ns": 0,
        "liveliness": 1,
        "liveliness_lease_duration_ns": 0,
        "avoid_ros_namespace_conventions": True,
    }
    assert report["best_available_lowered"]["reliability"] == 4
    assert [case["case_id"] for case in report["cases"]] == [
        "reliable_volatile_with_durations",
        "qos_profile_sensor_data",
        "qos_profile_system_default",
    ]

    reliable, sensor, system_default = report["cases"]
    reliable_requested = {
        "history": 1,
        "depth": 7,
        "reliability": 1,
        "durability": 2,
        "deadline_ns": 50_000_001,
        "lifespan_ns": 90_000_002,
        "liveliness": 1,
        "liveliness_lease_duration_ns": 120_000_003,
        "avoid_ros_namespace_conventions": False,
    }
    assert reliable["requested"] == reliable_requested
    assert reliable["lowered_publisher"] == reliable_requested
    assert reliable["lowered_subscription"] == reliable_requested
    assert reliable["actual_publisher"] == reliable_requested
    assert reliable["graph_publisher"] == reliable_requested
    for evidence in (
            reliable["actual_subscription"], reliable["graph_subscription"]):
        assert evidence == {
            **reliable_requested,
            "lifespan_ns": INFINITE_DURATION_NS,
        }

    sensor_requested = {
        "history": 1,
        "depth": 5,
        "reliability": 2,
        "durability": 2,
        "deadline_ns": 0,
        "lifespan_ns": 0,
        "liveliness": 0,
        "liveliness_lease_duration_ns": 0,
        "avoid_ros_namespace_conventions": False,
    }
    assert sensor["requested"] == sensor_requested
    assert sensor["lowered_publisher"] == sensor_requested
    assert sensor["lowered_subscription"] == sensor_requested
    normalized_sensor = {
        **sensor_requested,
        "deadline_ns": INFINITE_DURATION_NS,
        "lifespan_ns": INFINITE_DURATION_NS,
        "liveliness": 1,
        "liveliness_lease_duration_ns": INFINITE_DURATION_NS,
    }
    assert sensor["actual_publisher"] == normalized_sensor
    assert sensor["actual_subscription"] == normalized_sensor
    assert sensor["graph_publisher"] == normalized_sensor
    assert sensor["graph_subscription"] == normalized_sensor

    system_default_requested = {
        "history": 0,
        "depth": 0,
        "reliability": 0,
        "durability": 0,
        "deadline_ns": 0,
        "lifespan_ns": 0,
        "liveliness": 0,
        "liveliness_lease_duration_ns": 0,
        "avoid_ros_namespace_conventions": False,
    }
    assert system_default["requested"] == system_default_requested
    assert system_default["lowered_publisher"] == system_default_requested
    assert system_default["lowered_subscription"] == system_default_requested
    normalized_system_default = {
        "history": 1,
        "depth": 1,
        "reliability": 1,
        "durability": 2,
        "deadline_ns": INFINITE_DURATION_NS,
        "lifespan_ns": INFINITE_DURATION_NS,
        "liveliness": 1,
        "liveliness_lease_duration_ns": INFINITE_DURATION_NS,
        "avoid_ros_namespace_conventions": False,
    }
    assert system_default["actual_publisher"] == normalized_system_default
    assert system_default["actual_subscription"] == normalized_system_default
    assert system_default["graph_publisher"] == normalized_system_default
    assert system_default["graph_subscription"] == normalized_system_default


def test_qos_slice_changes_configuration_lowering_only():
    package = Path(__file__).resolve().parents[1] / "rclcpp_kit"
    source = (package / "direct_entities.py").read_text(encoding="utf-8")
    lowerer = source.split("def qos_from_profile", 1)[1].split(
        "def create_publisher", 1)[0]
    assert "convert_python_msg_to_cpp" not in lowerer
    assert "serialize_message" not in lowerer
    assert "publish(" not in lowerer
    assert "dispatch_callback" not in lowerer

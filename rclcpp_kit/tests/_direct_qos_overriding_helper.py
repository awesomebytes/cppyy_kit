#!/usr/bin/env python3
"""Live proof: qos_overriding declares parameters and an override is honored."""

import json
import os

from rclpy.utilities import get_rmw_implementation_identifier

from rclcpp_kit.direct_entities import create_subscription
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


PREFIX = "DIRECT_QOS_OVERRIDING_REPORT="
DEFAULT_POLICY_KINDS = ("history", "depth", "reliability")


def main():
    assert get_rmw_implementation_identifier() == "rmw_cyclonedds_cpp"
    suffix = str(os.getpid())
    overridden_topic = "/direct_qos_overriding/run_%s/overridden" % suffix
    control_topic = "/direct_qos_overriding/run_%s/control" % suffix
    override_param = "qos_overrides.%s.subscription.reliability" % overridden_topic

    with native([
        "direct-qos-overriding-proof",
        "--ros-args",
        "-p", "%s:=best_effort" % override_param,
    ]) as session:
        rclcpp = session.rclcpp
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        node = session.create_node("qos_overriding_" + suffix)
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()

        def _noop(*_args):
            pass

        overridden = create_subscription(
            node, message_type, overridden_topic, _noop, qos,
            qos_overriding=True,
        )
        declared_parameters = {
            name: bool(node.has_parameter(
                "qos_overrides.%s.subscription.%s" % (overridden_topic, name)))
            for name in DEFAULT_POLICY_KINDS
        }
        overridden_reliability = int(
            overridden.entity.get_actual_qos().get_rmw_qos_profile().reliability)

        # Control: the identical request on a different topic, no override
        # parameter set for it, keeps the QoS profile as requested (reliable).
        control = create_subscription(
            node, message_type, control_topic, _noop, qos,
            qos_overriding=True,
        )
        control_reliability = int(
            control.entity.get_actual_qos().get_rmw_qos_profile().reliability)
        control_declared_parameters = {
            name: bool(node.has_parameter(
                "qos_overrides.%s.subscription.%s" % (control_topic, name)))
            for name in DEFAULT_POLICY_KINDS
        }

        overridden.close()
        control.close()

    print(PREFIX + json.dumps({
        "schema": "rclcpp_kit.direct-qos-overriding-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "declared_parameters": declared_parameters,
        "overridden_reliability": overridden_reliability,
        "control_declared_parameters": control_declared_parameters,
        "control_reliability": control_reliability,
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()

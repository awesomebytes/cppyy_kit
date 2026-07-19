#!/usr/bin/env python3

import gc
import subprocess
import sys
import time

import cppyy
from std_msgs.msg import String as PythonString
from std_msgs.msg import UInt64 as PythonUInt64

from rclcpp_kit.bringup_rclcpp import _resolve_message_type
from rclcpp_kit.direct_subscription_lease import create_subscription_lease
from rclcpp_kit.native import native


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: helper AOT_PEER")
    retained = []
    session = native(["direct-subscription-lease-aot"])
    session.open()
    peer = None
    try:
        node = session.create_node("direct_subscription_lease_aot_receiver")
        executor = session.create_executor()
        executor.add_node(node)
        _, cpp_uint64 = _resolve_message_type(PythonUInt64)
        _, cpp_string = _resolve_message_type(PythonString)
        qos = session.rclcpp.QoS(session.rclcpp.KeepLast(10))
        uint_subscription = create_subscription_lease(
            node,
            cpp_uint64,
            "direct_lease_aot_uint64",
            retained.append,
            qos,
        )
        string_subscription = create_subscription_lease(
            node,
            cpp_string,
            "direct_lease_aot_string",
            retained.append,
            qos,
        )

        peer = subprocess.Popen(
            [sys.argv[1]],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + 15.0
        while len(retained) < 2 and time.monotonic() < deadline:
            executor.spin_some()
            time.sleep(0.005)
        assert len(retained) == 2
        stdout, stderr = peer.communicate(timeout=10)
        assert peer.returncode == 0, (stdout, stderr)
        assert "DIRECT_SUBSCRIPTION_LEASE_AOT_PUBLISHED_OK" in stdout

        uint_message = next(
            message for message in retained if type(message) is cpp_uint64)
        string_message = next(
            message for message in retained if type(message) is cpp_string)
        assert int(uint_message.data) == 18446744073709551439
        assert str(string_message.data) == "aot-to-cpp-lease"
        assert cppyy.addressof(uint_message) == (
            uint_subscription.last_message_address)
        assert cppyy.addressof(string_message) == (
            string_subscription.last_message_address)
        for subscription in (uint_subscription, string_subscription):
            assert subscription.stats().to_dict() == {
                "leases": 1,
                "message_deep_copies": 0,
                "shared_control_blocks": 1,
                "shared_owner_acquisitions": 1,
                "python_boundary_crossings": 1,
                "exceptions": 0,
            }
            subscription.close()
        print("DIRECT_SUBSCRIPTION_LEASE_AOT_OK", flush=True)
    finally:
        if peer is not None and peer.poll() is None:
            peer.kill()
            peer.wait(timeout=5)
        session.close()

    del uint_subscription, string_subscription
    gc.collect()
    assert int(uint_message.data) == 18446744073709551439
    assert str(string_message.data) == "aot-to-cpp-lease"
    assert type(uint_message) is cpp_uint64
    assert type(string_message) is cpp_string
    print("DIRECT_SUBSCRIPTION_LEASE_AOT_RETAINED_OK", flush=True)


if __name__ == "__main__":
    main()

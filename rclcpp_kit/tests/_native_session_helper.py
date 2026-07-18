#!/usr/bin/env python3

from rclcpp_kit.native import native, publisher_capabilities
from std_msgs.msg import String


def main():
    received = []
    session = native(["native-session-test", "--ros-args", "-r", "__ns:=/managed"])
    with session as ros:
        assert ros.context.is_valid()
        assert ros.capabilities.raw_rclcpp
        assert ros.capabilities.intra_process

        options = ros.rclcpp.NodeOptions()
        node = ros.create_node(
            "native_session",
            options=options,
            use_intra_process=True,
        )
        peer = ros.create_node("native_peer")
        assert node.get_fully_qualified_name() == "/managed/native_session"
        assert peer.get_fully_qualified_name() == "/managed/native_peer"

        exclusive = ros.create_callback_group(node)
        reentrant = ros.create_callback_group(node, "reentrant")
        assert exclusive is not None
        assert reentrant is not None

        publisher = node.create_publisher(String, "native_session_topic", 10)
        subscription = peer.create_subscription(
            String,
            "native_session_topic",
            lambda message: received.append(str(message.data)),
            10,
        )
        assert subscription is not None
        assert set(publisher_capabilities(publisher)) == {"loaned_messages", "reason"}

        executor = ros.create_executor("single_threaded")
        executor.add_node(node)
        executor.add_node(peer)
        for _ in range(20):
            executor.spin_some()
        publisher.publish(String(data="managed-native"))
        for _ in range(100):
            executor.spin_some()
            if received:
                break
        assert received == ["managed-native"]

        multi = ros.create_executor("multi_threaded", threads=2)
        assert multi is not None
        print("NATIVE_SESSION_OK")

    assert session.closed
    try:
        session.context
    except RuntimeError:
        print("NATIVE_TEARDOWN_OK")
    else:
        raise AssertionError("closed session exposed its context")


if __name__ == "__main__":
    main()

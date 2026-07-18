#!/usr/bin/env python3

import time

from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


def main():
    python_context = Context()
    python_context.init()
    client_node = Node("native_service_client", context=python_context)
    client_executor = SingleThreadedExecutor(context=python_context)
    client_executor.add_node(client_node)
    with native(["native-service-test"]) as ros:
        service_node = ros.create_node("native_service_server")
        service_executor = ros.create_executor()
        service_executor.add_node(service_node)
        service = ros.create_native_service(
            service_node,
            SetBool,
            "native_set_bool",
            'response->success = request->data; '
            'response->message = request->data ? "enabled" : "disabled";',
        )
        client = client_node.create_client(SetBool, "native_set_bool")
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not client.service_is_ready():
            service_executor.spin_some()
            client_executor.spin_once(timeout_sec=0.02)
        assert client.service_is_ready()

        request = SetBool.Request(data=True)
        future = client.call_async(request)
        while time.monotonic() < deadline and not future.done():
            service_executor.spin_some()
            client_executor.spin_once(timeout_sec=0.02)
        response = future.result()
        assert response.success is True
        assert response.message == "enabled"
        stats = service.stats()
        assert stats.requests == 1
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        print("NATIVE_SERVICE_OK")

    assert service.closed
    client_node.destroy_client(client)
    client_executor.remove_node(client_node)
    client_executor.shutdown(timeout_sec=1.0)
    client_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_SERVICE_TEARDOWN_OK")


if __name__ == "__main__":
    main()

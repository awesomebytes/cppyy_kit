#!/usr/bin/env python3

import time

import cppyy
import rclcpp_kit.native_client as native_client_module

from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


def main():
    python_context = Context()
    python_context.init()
    server_node = Node("native_client_server", context=python_context)
    server_executor = SingleThreadedExecutor(context=python_context)
    server_executor.add_node(server_node)

    def handle(request, response):
        response.success = request.data
        response.message = "stock-python-response"
        return response

    service = server_node.create_service(SetBool, "native_set_bool", handle)
    with native(["native-client-test"]) as ros:
        client_node = ros.create_node("native_client")
        callback_group = ros.create_callback_group(client_node, "reentrant")
        client_executor = ros.create_executor()
        client_executor.add_node(client_node)
        executor_thread = ros.start_executor(client_executor)
        client = ros.create_native_client(
            client_node,
            SetBool,
            "native_set_bool",
            callback_group=callback_group,
        )
        default_group_client = ros.create_native_client(
            client_node, SetBool, "native_set_bool")
        assert default_group_client.raw_client.get_service_name() == \
            "/native_set_bool"

        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not client.service_is_ready():
            server_executor.spin_once(timeout_sec=0.02)
        assert client.wait_for_service(0.2)
        assert client.raw_client.get_service_name() == "/native_set_bool"

        def forbidden_conversion(*_args, **_kwargs):
            raise AssertionError("Python request conversion ran")

        native_client_module.convert_python_msg_to_cpp = forbidden_conversion
        rejected_python_request = SetBool.Request(data=True)
        try:
            client.send(rejected_python_request)
        except TypeError as exc:
            assert "shared C++ request" in str(exc)
        else:
            raise AssertionError("a generated Python request was accepted")
        assert client.stats().requests_sent == 0

        request = client.make_request()
        request.data = True
        token = client.send(request)
        while time.monotonic() < deadline and not client.ready(token):
            server_executor.spin_once(timeout_sec=0.02)
        assert client.ready(token)
        response = client.take(token)
        assert response.success is True
        assert response.message == "stock-python-response"
        try:
            client.ready(token)
        except Exception as exc:
            assert "unknown or completed" in str(exc)
        else:
            raise AssertionError("a completed call token remained usable")

        cpp_value = cppyy.gbl.std_srvs.srv.SetBool.Request()
        cpp_value.data = False
        value_token = client.send_cpp_value(cpp_value)
        while time.monotonic() < deadline and not client.ready(value_token):
            server_executor.spin_once(timeout_sec=0.02)
        value_response = client.take(value_token)
        assert value_response.success is False
        assert value_response.message == "stock-python-response"

        cpp_request = client.make_request()
        cpp_request.data = False
        canceled_token = client.send(cpp_request)
        assert client.cancel(canceled_token) is True
        assert client.cancel(canceled_token) is False

        pending_request = client.make_request()
        pending_request.data = False
        pending_token = client.send(pending_request)
        assert pending_token > canceled_token
        try:
            client.take(pending_token)
        except Exception as exc:
            assert "response is not ready" in str(exc)
        else:
            raise AssertionError("an unprocessed request produced a response")
        stats = client.stats()
        assert stats.requests_sent == 4
        assert stats.responses_taken == 2
        assert stats.canceled == 1
        assert stats.exceptions == 0
        assert stats.pending_requests == 1
        assert stats.python_request_crossings == 4
        assert stats.python_response_crossings == 2
        assert stats.cpp_request_copies == 1
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1
        assert executor_thread.exceptions == 0
        print("NATIVE_CLIENT_OK")

    assert client.closed
    assert default_group_client.closed
    stats = client.stats()
    assert stats.pending_requests == 0
    assert stats.canceled == 2
    server_node.destroy_service(service)
    server_executor.remove_node(server_node)
    server_executor.shutdown(timeout_sec=1.0)
    server_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_CLIENT_TEARDOWN_OK")


if __name__ == "__main__":
    main()

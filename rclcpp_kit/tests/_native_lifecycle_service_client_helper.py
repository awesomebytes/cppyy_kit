#!/usr/bin/env python3
"""Committed proof for PLAN-lifecycle.md S4: create_lifecycle_service and
create_lifecycle_client -- concrete wrappers over
rclcpp_lifecycle::LifecycleNode::create_service<>()/create_client<>() --
round-trip between two lifecycle nodes.

Both factories go through the suite's AOT compile-cache path
(cppyy_kit.prebuild()/cppdef_cached(), via _compile_native_glue) rather than
a bare cppyy.cppdef: a plain Cling JIT of a full rclcpp_lifecycle +
service/client template body hits a known Cling
__emutls_v...std::call_once failure that is NOT lifecycle-specific --
native_service.py/native_client.py already route the plain-rclcpp::Node
service/client helpers around the identical failure the same way.

create_lifecycle_service reuses python_service.PythonService's
Invocation-pointer callback shape and facade unchanged; create_lifecycle_client
reuses native_client.NativeClient's C++-owned async-future facade unchanged --
only each factory's constructor node-parameter type and create_service/
create_client call target the lifecycle node's own template methods.
"""
import time

from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import (
    create_lifecycle_client,
    create_lifecycle_service,
)
from std_srvs.srv import SetBool


def main():
    with native(["native-lifecycle-service-client-test"]) as ros:
        service_lifecycle = ros.create_native_lifecycle_node(
            "native_lifecycle_service_host")
        client_lifecycle = ros.create_native_lifecycle_node(
            "native_lifecycle_client_host")

        def handle(request, response):
            response.success = request.data
            response.message = "lifecycle-response"
            return response

        service = create_lifecycle_service(
            service_lifecycle.raw_node, SetBool,
            "native_lifecycle_set_bool", handle)
        assert service.raw_service.get_service_name() == (
            "/native_lifecycle_set_bool")

        client = create_lifecycle_client(
            client_lifecycle.raw_node, SetBool, "native_lifecycle_set_bool")
        assert client.raw_client.get_service_name() == (
            "/native_lifecycle_set_bool")

        executor = ros.create_executor()
        service_lifecycle.attach_executor(executor)
        client_lifecycle.attach_executor(executor)
        executor_thread = ros.start_executor(executor)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not client.service_is_ready():
            time.sleep(0.02)
        assert client.service_is_ready()

        request = client.make_request()
        request.data = True
        token = client.send(request)
        while time.monotonic() < deadline and not client.ready(token):
            time.sleep(0.02)
        response = client.take(token)
        assert response.success is True
        assert str(response.message) == "lifecycle-response"

        second_request = client.make_request()
        second_request.data = False
        second_token = client.send(second_request)
        while time.monotonic() < deadline and not client.ready(second_token):
            time.sleep(0.02)
        second_response = client.take(second_token)
        assert second_response.success is False

        service_stats = service.stats()
        assert service_stats.requests == 2
        assert service_stats.exceptions == 0
        assert service_stats.python_callback_crossings == 2

        client_stats = client.stats()
        assert client_stats.requests_sent == 2
        assert client_stats.responses_taken == 2
        assert client_stats.exceptions == 0
        assert executor_thread.exceptions == 0

        service.close()
        client.close()
        assert service.closed is True
        assert client.closed is True
        try:
            service.raw_service
        except RuntimeError as exc:
            assert "closed" in str(exc)
        else:
            raise AssertionError("closed lifecycle service exposed raw_service")

        service_lifecycle.close()
        client_lifecycle.close()
        assert service_lifecycle.closed is True
        assert client_lifecycle.closed is True
        print("NATIVE_LIFECYCLE_SERVICE_CLIENT_OK")


if __name__ == "__main__":
    main()

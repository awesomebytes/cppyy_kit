#!/usr/bin/env python3

import os
from pathlib import Path
import time

import cppyy
from rclcpp_kit.native import native
from std_srvs.srv import SetBool


def spin_until(executor, predicate, description):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not predicate():
        executor.spin_some()
        time.sleep(0.005)
    assert predicate(), "timed out waiting for %s" % description


def main():
    order = os.environ.get("RCLCPP_KIT_COEXISTENCE_ORDER")
    assert order in ("service-client", "client-service")

    with native(["python-service-client-coexistence", order]) as ros:
        service_node = ros.create_node("python_service_coexistence")
        client_node = ros.create_node("python_service_client_coexistence")

        def handle(request, response):
            response.success = request.data
            response.message = "python-service"
            return response

        def create_service():
            return ros.create_python_service(
                service_node,
                SetBool,
                "/coexistence/python_service",
                handle,
            )

        def create_client():
            return ros.create_native_client(
                client_node, SetBool, "/coexistence/python_service")

        if order == "service-client":
            service = create_service()
            client = create_client()
        else:
            client = create_client()
            service = create_service()

        mappings = Path("/proc/self/maps").read_text()
        assert "/python-services/" in mappings
        assert "/native-clients/" in mappings

        executor = ros.create_executor()
        executor.add_node(service_node)
        executor.add_node(client_node)
        spin_until(executor, client.service_is_ready, "service discovery")

        request = cppyy.gbl.std_srvs.srv.SetBool.Request()
        request.data = True
        token = client.send_cpp_value(request)
        spin_until(executor, lambda: client.ready(token), "service response")
        response = client.take(token)
        assert response.success is True
        assert response.message == "python-service"
        service_stats = service.stats()
        client_stats = client.stats()
        assert service_stats.requests == 1
        assert service_stats.python_callback_crossings == 1
        assert service_stats.request_cpp_copies == 1
        assert service_stats.response_cpp_copies == 1
        assert client_stats.responses_taken == 1
        assert client_stats.cpp_request_copies == 1
        assert client_stats.compile_cache_hits == 0
        assert client_stats.compile_cache_misses == 1
        print("PYTHON_SERVICE_CLIENT_COEXISTENCE_OK order=%s" % order)

    assert service.closed
    assert client.closed
    print("PYTHON_SERVICE_CLIENT_COEXISTENCE_TEARDOWN_OK order=%s" % order)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import time

import cppyy
from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


def spin_until(native_executor, peer_executor, predicate, description):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not predicate():
        native_executor.spin_some()
        peer_executor.spin_once(timeout_sec=0.02)
    assert predicate(), "timed out waiting for %s" % description


def main():
    context = Context()
    context.init()
    client_node = Node("python_service_stock_client", context=context)
    client_executor = SingleThreadedExecutor(context=context)
    client_executor.add_node(client_node)
    retained_requests = []
    retained_responses = []
    callback_threads = []

    with native(["python-service-test"]) as ros:
        service_node = ros.create_node("python_service_server")
        service_executor = ros.create_executor()
        service_executor.add_node(service_node)

        def handle(request, response):
            assert isinstance(
                request, cppyy.gbl.std_srvs.srv.SetBool.Request)
            assert isinstance(response, response_type)
            assert not hasattr(request, "get_fields_and_field_types")
            retained_requests.append(request)
            retained_responses.append(response)
            callback_threads.append("python")
            if request.data:
                response.success = True
                response.message = "same-response"
                return response
            replacement = response_type()
            replacement.success = False
            replacement.message = "replacement-response"
            retained_responses.append(replacement)
            return replacement

        service = ros.create_python_service(
            service_node, SetBool, "python_set_bool", handle)
        response_type = cppyy.gbl.std_srvs.srv.SetBool.Response
        assert service.raw_service.get_service_name() == "/python_set_bool"
        client = client_node.create_client(SetBool, "python_set_bool")
        spin_until(
            service_executor,
            client_executor,
            client.service_is_ready,
            "Python-backed native service discovery",
        )

        responses = []
        for value in (True, False):
            future = client.call_async(SetBool.Request(data=value))
            spin_until(
                service_executor,
                client_executor,
                future.done,
                "Python-backed native service response",
            )
            responses.append(future.result())
        assert responses[0].success is True
        assert responses[0].message == "same-response"
        assert responses[1].success is False
        assert responses[1].message == "replacement-response"
        assert [request.data for request in retained_requests] == [True, False]
        assert retained_responses[0].message == "same-response"
        assert retained_responses[1].message == ""
        assert retained_responses[2].message == "replacement-response"
        assert callback_threads == ["python", "python"]

        stats = service.stats()
        assert stats.requests == 2
        assert stats.exceptions == 0
        assert stats.python_callback_crossings == 2
        assert stats.request_cpp_copies == 2
        assert stats.response_cpp_copies == 2
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1

        def fail(_request, _response):
            raise RuntimeError("expected service callback failure")

        failing_service = ros.create_python_service(
            service_node, SetBool, "python_set_bool_failure", fail)
        failing_client = client_node.create_client(
            SetBool, "python_set_bool_failure")
        spin_until(
            service_executor,
            client_executor,
            failing_client.service_is_ready,
            "failing service discovery",
        )
        failing_client.call_async(SetBool.Request(data=True))
        callback_exception = None
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and callback_exception is None:
            try:
                service_executor.spin_some()
            except Exception as exc:
                callback_exception = exc
            client_executor.spin_once(timeout_sec=0.01)
        assert callback_exception is not None, (
            "service callback exception did not escape spin")
        assert "expected service callback failure" in str(callback_exception)
        failure_stats = failing_service.stats()
        assert failure_stats.requests == 0
        assert failure_stats.exceptions == 1
        assert failure_stats.python_callback_crossings == 1
        assert failure_stats.request_cpp_copies == 1
        assert failure_stats.response_cpp_copies == 0
        print("PYTHON_SERVICE_OK")

        client_node.destroy_client(failing_client)
        client_node.destroy_client(client)

    assert service.closed
    assert failing_service.closed
    try:
        service.raw_service
    except RuntimeError as exc:
        assert "closed" in str(exc)
    else:
        raise AssertionError("closed service exposed its native entity")
    client_executor.remove_node(client_node)
    client_executor.shutdown(timeout_sec=1.0)
    client_node.destroy_node()
    context.shutdown()
    print("PYTHON_SERVICE_TEARDOWN_OK")


if __name__ == "__main__":
    main()

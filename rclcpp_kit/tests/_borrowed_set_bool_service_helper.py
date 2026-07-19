#!/usr/bin/env python3

import time

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


def capture_callback_exception(native_executor, peer_executor, predicate):
    deadline = time.monotonic() + 10.0
    callback_exception = None
    while time.monotonic() < deadline and callback_exception is None:
        try:
            native_executor.spin_some()
        except Exception as exc:
            callback_exception = exc
        peer_executor.spin_once(timeout_sec=0.01)
        if predicate() and callback_exception is None:
            time.sleep(0.002)
    assert callback_exception is not None, "callback exception did not escape spin"
    return callback_exception


def main():
    context = Context()
    context.init()
    client_node = Node("borrowed_set_bool_stock_client", context=context)
    client_executor = SingleThreadedExecutor(context=context)
    client_executor.add_node(client_node)
    retained = []

    with native(["borrowed-set-bool-service-test"]) as ros:
        service_node = ros.create_node("borrowed_set_bool_server")
        service_executor = ros.create_executor()
        service_executor.add_node(service_node)

        def handle(request, response):
            retained.extend((request, response))
            assert request.valid and response.valid
            response.success = request.data
            response.message = "borrowed:true" if request.data else "borrowed:false"
            return None

        service = ros.create_borrowed_set_bool_service(
            service_node, "/borrowed_set_bool", handle)
        assert service.raw_service.get_service_name() == "/borrowed_set_bool"
        client = client_node.create_client(SetBool, "/borrowed_set_bool")
        spin_until(
            service_executor,
            client_executor,
            client.service_is_ready,
            "borrowed SetBool service discovery",
        )

        responses = []
        for value in (True, False):
            future = client.call_async(SetBool.Request(data=value))
            spin_until(
                service_executor,
                client_executor,
                future.done,
                "borrowed SetBool response",
            )
            responses.append(future.result())
        assert responses[0].success is True
        assert responses[0].message == "borrowed:true"
        assert responses[1].success is False
        assert responses[1].message == "borrowed:false"
        assert all(not view.valid for view in retained)
        for view, field in ((retained[0], "data"), (retained[1], "message")):
            try:
                getattr(view, field)
            except RuntimeError as exc:
                assert "expired" in str(exc)
            else:
                raise AssertionError("retained borrowed view remained usable")
        stats = service.stats()
        assert stats.requests == 2
        assert stats.exceptions == 0
        assert stats.python_callback_crossings == 2
        assert stats.request_cpp_copies == 0
        assert stats.response_cpp_copies == 0
        assert stats.borrowed_request_views == 2
        assert stats.borrowed_response_views == 2
        assert stats.response_field_writes == 4
        assert stats.replacement_responses_rejected == 0
        assert stats.expired_accesses == 2
        assert stats.compile_cache_hits + stats.compile_cache_misses == 1
        print("BORROWED_SET_BOOL_STOCK_OK")

        replacement_calls = [0]

        def replace(request, response):
            replacement_calls[0] += 1
            response.success = request.data
            return response

        replacement = ros.create_borrowed_set_bool_service(
            service_node, "/borrowed_set_bool_replacement", replace)
        replacement_client = client_node.create_client(
            SetBool, "/borrowed_set_bool_replacement")
        spin_until(
            service_executor,
            client_executor,
            replacement_client.service_is_ready,
            "replacement-rejecting service discovery",
        )
        replacement_client.call_async(SetBool.Request(data=True))
        replacement_error = capture_callback_exception(
            service_executor, client_executor, lambda: replacement_calls[0] == 1)
        assert "return None" in str(replacement_error)
        replacement_stats = replacement.stats()
        assert replacement_stats.requests == 0
        assert replacement_stats.exceptions == 1
        assert replacement_stats.python_callback_crossings == 1
        assert replacement_stats.request_cpp_copies == 0
        assert replacement_stats.response_cpp_copies == 0
        assert replacement_stats.response_field_writes == 1
        assert replacement_stats.replacement_responses_rejected == 1

        failure_calls = [0]

        def fail(_request, _response):
            failure_calls[0] += 1
            raise RuntimeError("expected borrowed callback failure")

        failing = ros.create_borrowed_set_bool_service(
            service_node, "/borrowed_set_bool_failure", fail)
        failing_client = client_node.create_client(
            SetBool, "/borrowed_set_bool_failure")
        spin_until(
            service_executor,
            client_executor,
            failing_client.service_is_ready,
            "failing borrowed service discovery",
        )
        failing_client.call_async(SetBool.Request(data=False))
        failure_error = capture_callback_exception(
            service_executor, client_executor, lambda: failure_calls[0] == 1)
        assert "expected borrowed callback failure" in str(failure_error)
        failure_stats = failing.stats()
        assert failure_stats.requests == 0
        assert failure_stats.exceptions == 1
        assert failure_stats.python_callback_crossings == 1
        assert failure_stats.response_field_writes == 0
        print("BORROWED_SET_BOOL_MISUSE_OK")

        client_node.destroy_client(failing_client)
        client_node.destroy_client(replacement_client)
        client_node.destroy_client(client)

    assert service.closed
    assert replacement.closed
    assert failing.closed
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
    print("BORROWED_SET_BOOL_TEARDOWN_OK")


if __name__ == "__main__":
    main()

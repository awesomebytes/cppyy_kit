#!/usr/bin/env python3

import os
from pathlib import Path
import time

from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool


def spin_until(executor, predicate, description):
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline and not predicate():
        executor.spin_once(timeout_sec=0.02)
    assert predicate(), "timed out waiting for %s" % description


def main():
    order = os.environ.get("RCLCPP_KIT_COEXISTENCE_ORDER")
    assert order in ("service-client", "client-service")

    python_context = Context()
    python_context.init()
    peer = Node("native_service_client_coexistence_peer", context=python_context)
    peer_executor = SingleThreadedExecutor(context=python_context)
    peer_executor.add_node(peer)

    def handle(request, response):
        response.success = request.data
        response.message = "stock-server:true" if request.data else "stock-server:false"
        return response

    stock_service = peer.create_service(SetBool, "/coexistence/native_client", handle)
    stock_client = peer.create_client(SetBool, "/coexistence/native_service")

    with native(["native-service-client-coexistence", order]) as ros:
        service_node = ros.create_node("native_service_coexistence")
        client_node = ros.create_node("native_client_coexistence")

        def create_service():
            return ros.create_native_service(
                service_node,
                SetBool,
                "/coexistence/native_service",
                'response->success = request->data; '
                'response->message = request->data ? "native-service:true" '
                ': "native-service:false";',
            )

        def create_client():
            return ros.create_native_client(
                client_node, SetBool, "/coexistence/native_client")

        if order == "service-client":
            native_service = create_service()
            native_client = create_client()
        else:
            native_client = create_client()
            native_service = create_service()

        mappings = Path("/proc/self/maps").read_text()
        assert "/native-services/" in mappings
        assert "/native-clients/" in mappings

        native_executor = ros.create_executor()
        native_executor.add_node(service_node)
        native_executor.add_node(client_node)
        native_thread = ros.start_executor(native_executor)

        spin_until(peer_executor, stock_client.service_is_ready,
                   "managed native service discovery")
        spin_until(peer_executor, native_client.service_is_ready,
                   "managed native client discovery")

        stock_future = stock_client.call_async(SetBool.Request(data=True))
        spin_until(peer_executor, stock_future.done, "managed service response")
        stock_response = stock_future.result()
        assert stock_response.success is True
        assert stock_response.message == "native-service:true"

        request = native_client.make_request()
        request.data = False
        token = native_client.send(request)
        spin_until(peer_executor, lambda: native_client.ready(token),
                   "managed client response")
        native_response = native_client.take(token)
        assert native_response.success is False
        assert native_response.message == "stock-server:false"

        assert native_service.stats().requests == 1
        client_stats = native_client.stats()
        assert client_stats.responses_taken == 1
        assert client_stats.compile_cache_hits == 0
        assert client_stats.compile_cache_misses == 1
        assert native_thread.exceptions == 0
        print("NATIVE_SERVICE_CLIENT_COEXISTENCE_OK order=%s" % order)

    assert native_service.closed
    assert native_client.closed
    assert native_thread.closed

    peer.destroy_client(stock_client)
    peer.destroy_service(stock_service)
    peer_executor.remove_node(peer)
    peer_executor.shutdown(timeout_sec=1.0)
    peer.destroy_node()
    python_context.shutdown()
    print("NATIVE_SERVICE_CLIENT_COEXISTENCE_TEARDOWN_OK order=%s" % order)


if __name__ == "__main__":
    main()

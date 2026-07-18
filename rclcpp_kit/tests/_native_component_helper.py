#!/usr/bin/env python3

import time

from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterValue
from rclcpp_kit.native import native
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


def _call(executor, client, request, timeout_sec=10.0):
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not future.done():
        executor.spin_once(timeout_sec=0.02)
    assert future.done()
    response = future.result()
    assert response is not None
    return response


def _wait_for_services(executor, clients, timeout_sec=10.0):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if all(client.service_is_ready() for client in clients):
            return
        executor.spin_once(timeout_sec=0.05)
    assert all(client.service_is_ready() for client in clients)


def _assert_raw_access_is_closed(manager):
    try:
        manager.raw_manager
    except RuntimeError as exception:
        assert "closed" in str(exception)
    else:
        raise AssertionError("closed manager still exposes raw access")


def main():
    python_context = Context()
    python_context.init()
    client_node = Node("native_component_client", context=python_context)
    client_executor = SingleThreadedExecutor(context=python_context)
    client_executor.add_node(client_node)

    with native(["native-component-test"]) as ros:
        executor = ros.create_executor()
        manager = ros.create_native_component_manager(
            executor,
            name="managed_component_container",
        )
        assert manager in ros.resources
        assert str(manager.raw_manager.get_name()) == "managed_component_container"
        resources = manager.raw_manager.get_component_resources(
            "robot_state_publisher")
        assert any(
            str(resource.first) == "robot_state_publisher::RobotStatePublisher"
            for resource in resources
        )

        executor_thread = ros.start_executor(executor)
        prefix = "/managed_component_container/_container"
        load_client = client_node.create_client(LoadNode, prefix + "/load_node")
        list_client = client_node.create_client(ListNodes, prefix + "/list_nodes")
        unload_client = client_node.create_client(
            UnloadNode, prefix + "/unload_node")
        clients = (load_client, list_client, unload_client)
        _wait_for_services(client_executor, clients)

        request = LoadNode.Request()
        request.package_name = "robot_state_publisher"
        request.plugin_name = "robot_state_publisher::RobotStatePublisher"
        request.node_name = "managed_robot_state_publisher"
        request.node_namespace = "/components"
        request.parameters = [
            ParameterMsg(
                name="robot_description",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=(
                        "<robot name='native_component_test'>"
                        "<link name='base_link'/></robot>"
                    ),
                ),
            )
        ]
        loaded = _call(client_executor, load_client, request)
        assert loaded.success is True, loaded.error_message
        assert loaded.full_node_name == "/components/managed_robot_state_publisher"
        assert loaded.unique_id > 0

        listed = _call(client_executor, list_client, ListNodes.Request())
        assert list(listed.unique_ids) == [loaded.unique_id]
        assert list(listed.full_node_names) == [loaded.full_node_name]
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            names = dict(client_node.get_node_names_and_namespaces())
            if names.get("managed_robot_state_publisher") == "/components":
                break
            client_executor.spin_once(timeout_sec=0.05)
        assert dict(client_node.get_node_names_and_namespaces()).get(
            "managed_robot_state_publisher") == "/components"

        unloaded = _call(
            client_executor,
            unload_client,
            UnloadNode.Request(unique_id=loaded.unique_id),
        )
        assert unloaded.success is True, unloaded.error_message
        listed = _call(client_executor, list_client, ListNodes.Request())
        assert list(listed.unique_ids) == []
        assert list(listed.full_node_names) == []

        loaded_for_teardown = _call(client_executor, load_client, request)
        assert loaded_for_teardown.success is True, loaded_for_teardown.error_message
        assert loaded_for_teardown.unique_id > loaded.unique_id
        assert executor_thread.exceptions == 0

        for client in clients:
            client_node.destroy_client(client)
        manager.close()
        manager.close()
        assert manager.closed is True
        _assert_raw_access_is_closed(manager)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            names = dict(client_node.get_node_names_and_namespaces())
            if "managed_robot_state_publisher" not in names:
                break
            client_executor.spin_once(timeout_sec=0.05)
        assert "managed_robot_state_publisher" not in dict(
            client_node.get_node_names_and_namespaces())
        assert executor_thread.exceptions == 0
        print("NATIVE_COMPONENT_OK")

    assert manager.closed is True
    client_executor.remove_node(client_node)
    client_executor.shutdown(timeout_sec=1.0)
    client_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_COMPONENT_TEARDOWN_OK")
if __name__ == "__main__":
    main()

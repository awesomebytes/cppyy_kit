#!/usr/bin/env python3
"""Check that borrowed publication leaves Fast DDS RCL error state clear."""

import os

import rclpy
from rclpy.context import Context
from std_msgs.msg import String
from std_srvs.srv import SetBool

from rclcpp_kit import borrowed_publish


def main():
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_fastrtps_cpp"
    context = Context()
    context.init(args=[])
    node = rclpy.create_node(
        "borrowed_publish_error_hygiene_%d" % os.getpid(),
        context=context,
        enable_rosout=False,
        start_parameter_services=False,
    )

    publisher = node.create_publisher(String, "/borrowed_publish/error_hygiene", 10)
    route = borrowed_publish.prepare(String)
    route.publish(publisher, String(data="published"))

    def handle(request, response):
        response.success = request.data
        return response

    service = node.create_service(SetBool, "/borrowed_publish/error_hygiene_service", handle)
    client = node.create_client(SetBool, "/borrowed_publish/error_hygiene_service")
    assert node.destroy_client(client)
    assert node.destroy_service(service)
    assert node.destroy_publisher(publisher)
    node.destroy_node()
    context.shutdown()
    print("BORROWED_PUBLISH_ERROR_HYGIENE_OK", flush=True)


if __name__ == "__main__":
    main()

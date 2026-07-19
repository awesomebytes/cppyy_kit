#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import sys

from rclcpp_kit.native import native


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: helper.py PATH_TO_AOT_PEER")
    domain = os.environ.get("ROS_DOMAIN_ID")
    assert domain, "the parent test must assign an isolated ROS_DOMAIN_ID"
    peer = Path(sys.argv[1]).resolve()
    assert peer.is_file() and os.access(peer, os.X_OK), peer
    service_name = "/rclcpp_kit/borrowed_set_bool_aot_%s_%s" % (
        domain, os.getpid())
    retained = []

    with native(["borrowed-set-bool-aot-interop"]) as ros:
        node = ros.create_node("borrowed_set_bool_aot_server")
        executor = ros.create_executor()
        executor.add_node(node)
        executor_thread = ros.start_executor(executor)

        def handle(request, response):
            retained.extend((request, response))
            response.success = request.data
            response.message = "managed-native-service:enabled:314159"

        service = ros.create_borrowed_set_bool_service(
            node, service_name, handle)
        process = subprocess.run(
            [str(peer), "client", service_name],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
            env=os.environ.copy(),
        )
        assert process.returncode == 0, (
            "AOT client exit=%d\nstdout:\n%s\nstderr:\n%s" % (
                process.returncode, process.stdout, process.stderr))
        assert "AOT_CLIENT_OK managed-native-service:enabled:314159" in process.stdout
        assert "AOT_PEER_TEARDOWN_OK client" in process.stdout
        assert len(retained) == 2
        assert not retained[0].valid and not retained[1].valid
        stats = service.stats()
        assert stats.requests == 1
        assert stats.exceptions == 0
        assert stats.python_callback_crossings == 1
        assert stats.request_cpp_copies == 0
        assert stats.response_cpp_copies == 0
        assert stats.borrowed_request_views == 1
        assert stats.borrowed_response_views == 1
        assert stats.response_field_writes == 2
        assert executor_thread.exceptions == 0
    assert service.closed
    assert executor_thread.closed
    print("BORROWED_SET_BOOL_AOT_INTEROP_OK")


if __name__ == "__main__":
    main()

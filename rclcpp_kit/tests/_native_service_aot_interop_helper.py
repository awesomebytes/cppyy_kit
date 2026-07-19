#!/usr/bin/env python3

import os
from pathlib import Path
import select
import subprocess
import sys
import time

import cppyy
from rclcpp_kit.native import native
from std_srvs.srv import SetBool


PEER_TIMEOUT_S = 30


def run_peer(peer: Path, mode: str, service_name: str) -> subprocess.CompletedProcess:
    try:
        return subprocess.run(
            [str(peer), mode, service_name],
            capture_output=True,
            text=True,
            timeout=PEER_TIMEOUT_S,
            check=False,
            env=os.environ.copy(),
        )
    except subprocess.TimeoutExpired as exc:
        raise AssertionError(
            f"AOT {mode} exceeded {PEER_TIMEOUT_S}s; "
            f"stdout={exc.stdout!r} stderr={exc.stderr!r}"
        ) from exc


def assert_peer(proc: subprocess.CompletedProcess, marker: str) -> None:
    assert proc.returncode == 0, (
        f"peer exit={proc.returncode}\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
    )
    assert marker in proc.stdout, proc.stdout
    assert "AOT_PEER_TEARDOWN_OK" in proc.stdout, proc.stdout


def managed_service_to_aot_client(peer: Path, suffix: str) -> None:
    service_name = f"/rclcpp_kit/aot_client_{suffix}"
    with native(["native-service-aot-interop"]) as ros:
        node = ros.create_node(f"managed_service_{suffix}")
        executor = ros.create_executor()
        executor.add_node(node)
        executor_thread = ros.start_executor(executor)
        service = ros.create_native_service(
            node,
            SetBool,
            service_name,
            'response->success = request->data; '
            'response->message = request->data '
            '? "managed-native-service:enabled:314159" '
            ': "managed-native-service:disabled";',
        )
        proc = run_peer(peer, "client", service_name)
        assert_peer(proc, "AOT_CLIENT_OK managed-native-service:enabled:314159")
        stats = service.stats()
        assert stats.requests == 1
        assert stats.exceptions == 0
        assert stats.python_boundary_crossings == 0
        assert executor_thread.exceptions == 0
    assert service.closed
    assert executor_thread.closed
    print("MANAGED_SERVICE_TO_AOT_CLIENT_OK")


def python_service_to_aot_client(peer: Path, suffix: str) -> None:
    service_name = f"/rclcpp_kit/aot_python_client_{suffix}"
    retained = []
    with native(["python-service-aot-interop"]) as ros:
        node = ros.create_node(f"python_service_{suffix}")
        executor = ros.create_executor()
        executor.add_node(node)

        def handle(request, response):
            retained.extend((request, response))
            response.success = request.data
            response.message = "managed-native-service:enabled:314159"
            return response

        service = ros.create_python_service(
            node, SetBool, service_name, handle)
        process = subprocess.Popen(
            [str(peer), "client", service_name],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=os.environ.copy(),
        )
        deadline = time.monotonic() + PEER_TIMEOUT_S
        while process.poll() is None and time.monotonic() < deadline:
            executor.spin_some()
            time.sleep(0.002)
        if process.poll() is None:
            process.kill()
        stdout, stderr = process.communicate()
        proc = subprocess.CompletedProcess(
            process.args, process.returncode, stdout, stderr)
        assert_peer(proc, "AOT_CLIENT_OK managed-native-service:enabled:314159")
        stats = service.stats()
        assert stats.requests == 1
        assert stats.exceptions == 0
        assert stats.python_callback_crossings == 1
        assert stats.request_cpp_copies == 1
        assert stats.response_cpp_copies == 1
        assert retained[0].data is True
        assert retained[1].message == "managed-native-service:enabled:314159"
    assert service.closed
    print("PYTHON_SERVICE_TO_AOT_CLIENT_OK")


def managed_client_to_aot_service(peer: Path, suffix: str) -> None:
    service_name = f"/rclcpp_kit/aot_server_{suffix}"
    server = subprocess.Popen(
        [str(peer), "server", service_name],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=os.environ.copy(),
    )
    server_stdout = ""
    server_stderr = ""
    try:
        readable, _, _ = select.select([server.stdout], [], [], 10.0)
        assert readable, "AOT server did not report readiness within 10s"
        ready = server.stdout.readline().strip()
        server_stdout = ready + "\n"
        assert ready == f"AOT_SERVER_READY {service_name}", ready
        with native(["native-client-aot-interop"]) as ros:
            node = ros.create_node(f"managed_client_{suffix}")
            executor = ros.create_executor()
            executor.add_node(node)
            executor_thread = ros.start_executor(executor)
            client = ros.create_native_client(node, SetBool, service_name)
            assert client.wait_for_service(10.0)

            request = cppyy.gbl.std_srvs.srv.SetBool.Request()
            request.data = True
            token = client.send_cpp_value(request)
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline and not client.ready(token):
                time.sleep(0.002)
            assert client.ready(token), "managed native client response timed out"
            response = client.take(token)
            assert response.success is True
            assert response.message == "aot-service:enabled:271828"
            stats = client.stats()
            assert stats.requests_sent == 1
            assert stats.responses_taken == 1
            assert stats.pending_requests == 0
            assert stats.exceptions == 0
            assert stats.python_request_crossings == 1
            assert stats.python_response_crossings == 1
            assert stats.cpp_request_copies == 1
            assert executor_thread.exceptions == 0
        assert client.closed
        assert executor_thread.closed
        remaining_stdout, server_stderr = server.communicate(timeout=PEER_TIMEOUT_S)
        server_stdout += remaining_stdout
    except Exception:
        if server.poll() is None:
            server.kill()
        remaining_stdout, server_stderr = server.communicate()
        server_stdout += remaining_stdout
        raise
    assert server.returncode == 0, (
        f"server exit={server.returncode}\n"
        f"stdout:\n{server_stdout}\nstderr:\n{server_stderr}"
    )
    assert "AOT_SERVER_OK aot-service:enabled:271828" in server_stdout
    assert "AOT_PEER_TEARDOWN_OK server" in server_stdout
    print("MANAGED_CLIENT_TO_AOT_SERVICE_OK")


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(
            "usage: helper.py PATH_TO_AOT_PEER service|python-service|client")
    domain = os.environ.get("ROS_DOMAIN_ID")
    assert domain, "the parent test must assign an isolated ROS_DOMAIN_ID"
    peer = Path(sys.argv[1]).resolve()
    assert peer.is_file() and os.access(peer, os.X_OK), peer
    suffix = f"d{domain}_p{os.getpid()}"
    scenario = sys.argv[2]
    if scenario == "service":
        managed_service_to_aot_client(peer, suffix)
    elif scenario == "python-service":
        python_service_to_aot_client(peer, suffix)
    elif scenario == "client":
        managed_client_to_aot_service(peer, suffix)
    else:
        raise SystemExit(f"unknown scenario: {scenario}")
    print(f"NATIVE_AOT_INTEROP_OK scenario={scenario} domain={domain}")


if __name__ == "__main__":
    main()

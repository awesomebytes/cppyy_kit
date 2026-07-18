#!/usr/bin/env python3

import os
from pathlib import Path
import subprocess
import sys

from lifecycle_msgs.msg import State
from rclcpp_kit.native import native


PEER_TIMEOUT_S = 45
EXPECTED_TRACE = (
    "1:unconfigured,2:inactive,3:active,2:inactive,1:unconfigured"
)


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: helper.py PATH_TO_AOT_PEER")
    domain = os.environ.get("ROS_DOMAIN_ID")
    assert domain, "the parent test must assign an isolated ROS_DOMAIN_ID"
    peer = Path(sys.argv[1]).resolve()
    assert peer.is_file() and os.access(peer, os.X_OK), peer
    lifecycle_name = f"managed_lifecycle_d{domain}_p{os.getpid()}"

    with native(["native-lifecycle-aot-interop"]) as ros:
        lifecycle = ros.create_native_lifecycle_node(lifecycle_name)
        executor = ros.create_executor()
        lifecycle.attach_executor(executor)
        executor_thread = ros.start_executor(executor)

        initial = lifecycle.raw_node.get_current_state()
        assert int(initial.id()) == State.PRIMARY_STATE_UNCONFIGURED
        assert str(initial.label()) == "unconfigured"
        try:
            proc = subprocess.run(
                [str(peer), lifecycle_name],
                capture_output=True,
                text=True,
                timeout=PEER_TIMEOUT_S,
                check=False,
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired as exc:
            raise AssertionError(
                f"AOT lifecycle peer exceeded {PEER_TIMEOUT_S}s; "
                f"stdout={exc.stdout!r} stderr={exc.stderr!r}"
            ) from exc

        assert proc.returncode == 0, (
            f"peer exit={proc.returncode}\n"
            f"stdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"
        )
        assert f"AOT_LIFECYCLE_OK {EXPECTED_TRACE}" in proc.stdout
        assert "AOT_LIFECYCLE_TEARDOWN_OK" in proc.stdout
        final = lifecycle.raw_node.get_current_state()
        assert int(final.id()) == State.PRIMARY_STATE_UNCONFIGURED
        assert str(final.label()) == "unconfigured"
        assert executor_thread.exceptions == 0

        lifecycle.close()
        assert lifecycle.closed
        assert lifecycle.attached_executors == 0

    assert lifecycle.closed
    assert executor_thread.closed
    print(
        f"NATIVE_LIFECYCLE_AOT_INTEROP_OK domain={domain} trace={EXPECTED_TRACE}"
    )
    print("NATIVE_LIFECYCLE_AOT_TEARDOWN_OK")


if __name__ == "__main__":
    main()

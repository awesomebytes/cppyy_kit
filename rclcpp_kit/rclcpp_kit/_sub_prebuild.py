"""
Detached worker that compiles a subscription trampoline ``.so`` for one message type.

Spawned by ``rclcpp_kit.subscription_cache`` at interpreter exit as::

    python -m rclcpp_kit._sub_prebuild <cpp_type_str> <header>

so the next run loads the ``.so`` instead of JIT-instantiating
``create_subscription<MsgT>``. Best-effort: any failure is silent and non-fatal (the
next run just falls back to the plain path and reschedules). Detached from the run
that scheduled it, so it never blocks that run's exit.
"""
import sys


def main(argv):
    if len(argv) != 2:
        return 2
    cpp_type_str, header = argv
    try:
        from rclcpp_kit import subscription_cache
        subscription_cache.prebuild(cpp_type_str, header)
        return 0
    except Exception:
        return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

"""Slice 2.5a (PLAN-mte-unlock.md Addendum v2-completion): the C++-owned
callable lifetime for native-dispatched entities (subscriptions, timers,
shared-lease subscriptions, services). Teardown is a plain strong-ref drop
of the entity; the callable is released only afterward, never severed
eagerly while the entity may still be referenced by a native worker.
"""
import gc
import weakref

from _run_helper import format_output, run_helper
from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


class _WeakrefableCallback:
    def __call__(self, _message):
        pass


def test_managed_subscription_destroy_under_live_mte_does_not_crash():
    """The reference repro: a subscription's entity is destroyed while a
    peer subscription's callback is running on another native
    MultiThreadedExecutor worker. Pre-fix this crashed ('callable was
    deleted' / 'terminate called without an active exception'); post-fix it
    must be crash-free across every one of 60 iterations."""
    # The outer timeout must clear the helper's own WATCHDOG_SECONDS (240s)
    # with margin: on a hang, the internal faulthandler watchdog must fire
    # and flush real thread stacks to stderr before this kills the process,
    # or a genuine deadlock surfaces as a silent, undiagnosable timeout
    # instead (as a too-tight 180s outer bound once did here).
    process = run_helper(
        "_managed_subscription_destroy_under_mte_helper.py", timeout=300)
    assert process.returncode == 0, format_output(process)
    assert "MANAGED_SUBSCRIPTION_DESTROY_UNDER_MTE_OK" in process.stdout, (
        format_output(process)
    )


def test_managed_subscription_close_drops_entity_before_callback_release():
    """White-box regression guard: DirectSubscription.close() must route
    through a C++-owned ManagedSubscription (Slice 2.5a) rather than nulling
    the callable directly -- the eager-severing pattern this slice fixes.
    If ``managed`` is ever missing again (e.g. a future create_subscription
    edit drops the ``managed=`` wiring), this fails loud rather than
    silently reintroducing the UAF class.
    """
    with native(["managed-subscription-close-order"]) as session:
        node = session.create_node("managed_subscription_close_order")
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        qos = direct_entities.qos_from_depth(session.rclcpp, 10)

        subscription = direct_entities.create_subscription(
            node, cpp_type, "/managed_subscription_close_order/topic",
            lambda _message: None, qos)

        assert subscription.managed is not None, (
            "DirectSubscription must be constructed with a C++-owned "
            "ManagedSubscription (Slice 2.5a) -- got None, meaning "
            "close() would fall back to nulling the callable directly"
        )
        assert not subscription.managed.closed()
        # A live shared_ptr<Subscription> returned by value from entity() is
        # never Python None -- cppyy wraps it as a proxy either way -- but it
        # is truthy iff the underlying pointer is non-null.
        assert subscription.managed.entity()

        assert subscription.close()
        assert subscription.managed.closed()
        assert not subscription.managed.entity()
        # Idempotent: closing twice is a documented no-op, not a double-free.
        assert not subscription.close()


def test_subscription_close_drops_owner_pin_but_native_copy_retains_callback():
    """The managed proxy must not be the only callback owner after close.

    A separately held native shared_ptr keeps the rclcpp callback copy (and
    its PinnedCallable reference) alive after the Python facade closes. Once
    that native owner and the session are released, the callback should be
    collectable even though the closed managed proxy remains inspectable.
    """
    with native(["managed-subscription-owner-pin-release"]) as session:
        node = session.create_node("managed_subscription_owner_pin_release")
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        _, cpp_type, _ = direct_entities.resolve_supported_type(message_type)
        qos = direct_entities.qos_from_depth(session.rclcpp, 10)

        callback = _WeakrefableCallback()
        callback_ref = weakref.ref(callback)
        subscription = direct_entities.create_subscription(
            node, cpp_type, "/managed_subscription_owner_pin_release/topic",
            callback, qos)
        managed = subscription.managed
        assert managed is not None
        assert managed._cppyy_kit_kept_alive

        # Keep an independent native copy after DirectSubscription.close()
        # drops its entity handle and the managed wrapper's duplicate copy.
        native_subscription = subscription.entity.__smartptr__()
        assert subscription.close()
        assert subscription.managed is managed
        assert managed.closed()
        assert managed._cppyy_kit_kept_alive == []

        del callback
        gc.collect()
        assert callback_ref() is not None, (
            "the native std::function copy must retain its Python callable"
        )

        del native_subscription
        node = None
        subscription = None
        session.close()
        direct_entities.drain_callable_reaper()
        gc.collect()
        assert callback_ref() is None, (
            "the closed managed proxy must not keep the callback alive"
        )

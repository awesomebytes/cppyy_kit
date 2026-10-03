#!/usr/bin/env python3
"""Integration test that supported QoS events fire from DDS conditions.

Runs under the default RMW (rmw_cyclonedds_cpp) at ROS_DISTRO=jazzy. Drives the
publisher/subscription event callbacks added by direct_entities.py's
``event_callbacks=`` parameter. The test checks rejection of unsupported
``incompatible_type`` events, destruction ordering, and event delivery. It also
uses ``_native_qos_events_probe`` to reuse cached publisher and subscription
types, avoiding a second ~2.8s template JIT.
"""

import gc
import json
import os
import time

from rclcpp_kit.direct_entities import (
    QoSEventUnsupported,
    _publisher_options_with_events,
    _subscription_options_with_events,
    create_managed_publisher,
    create_subscription,
)
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit._native_qos_events_probe import (
    make_qos_events_publisher_probe,
    make_qos_events_subscription_probe,
)
from rclcpp_kit.native import native, qos_event_capabilities


PREFIX = "QOS_EVENTS_REPORT="
TIMEOUT_S = 15.0


def wait_for(predicate, description, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for %s" % description)


def _matched(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
        "current_count": int(info.current_count),
        "current_count_change": int(info.current_count_change),
    }


def _incompatible_qos(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
        "last_policy_kind": int(info.last_policy_kind),
    }


def _deadline(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


def _liveliness_lost(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


def _liveliness_changed(info):
    return {
        "alive_count": int(info.alive_count),
        "not_alive_count": int(info.not_alive_count),
        "alive_count_change": int(info.alive_count_change),
        "not_alive_count_change": int(info.not_alive_count_change),
    }


def _message_lost(info):
    return {
        "total_count": int(info.total_count),
        "total_count_change": int(info.total_count_change),
    }


class Counter:
    """Records fire count and the last-observed scalar fields of a QoS event."""

    def __init__(self, extractor):
        self._extractor = extractor
        self.count = 0
        self.last = None

    def __call__(self, info):
        self.count += 1
        self.last = self._extractor(info)


def main():
    suffix = str(os.getpid())
    events = {}
    incompatible_type_fail_closed = {}
    teardown_uaf_guard = {}
    cleanup = []

    session = native(["native-qos-events-proof"])
    with session as ros:
        rclcpp = ros.rclcpp
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        executor = ros.create_executor("single_threaded")

        def node(name):
            value = ros.create_node(name + "_" + suffix)
            executor.add_node(value)
            return value

        thread = ros.start_executor(executor)
        wait_for(lambda: thread.running, "native executor thread")

        # --- incompatible_qos: publisher offers BEST_EFFORT, subscriber
        # requests RELIABLE on the same topic -> both sides fire. --------------
        topic = "/native_qos_events/run_%s/incompatible_qos" % suffix
        pub_node = node("incompatible_qos_publisher")
        sub_node = node("incompatible_qos_subscriber")
        pub_qos = rclcpp.QoS(rclcpp.KeepLast(8))
        pub_qos.best_effort().durability_volatile()
        sub_qos = rclcpp.QoS(rclcpp.KeepLast(8))
        sub_qos.reliable().durability_volatile()
        pub_incompatible = Counter(_incompatible_qos)
        sub_incompatible = Counter(_incompatible_qos)
        pub_options, _pub_cpp_cbs = _publisher_options_with_events(
            pub_node, None, {"incompatible_qos": pub_incompatible})
        sub_options, _sub_cpp_cbs = _subscription_options_with_events(
            sub_node, None, {"incompatible_qos": sub_incompatible})
        publisher = make_qos_events_publisher_probe(
            pub_node, topic, pub_qos, pub_options)
        subscription = make_qos_events_subscription_probe(
            sub_node, topic, sub_qos, sub_options)
        cleanup.extend([publisher, subscription])
        wait_for(lambda: pub_incompatible.count >= 1, "publisher incompatible_qos")
        wait_for(lambda: sub_incompatible.count >= 1, "subscription incompatible_qos")
        events["publisher_incompatible_qos"] = {
            "registered": True, "fired": True, "count": pub_incompatible.count,
            "last": pub_incompatible.last, "proof_level": "fires",
        }
        events["subscription_incompatible_qos"] = {
            "registered": True, "fired": True, "count": sub_incompatible.count,
            "last": sub_incompatible.last, "proof_level": "fires",
        }

        # --- matched: create a compatible peer (0->1 on both sides), then add a
        # second compatible peer (1->2 observed on the publisher). A second peer
        # -- not destroying the first -- is the trigger here: this environment's
        # CycloneDDS endpoint-deletion propagation to a live writer's
        # matched-count took far longer than a routine test should wait on
        # (tens of seconds, confirmed empirically), while a new match is fast
        # and is equally a real DDS condition proving the event fires more than
        # once. -------------------------------------------------------------
        topic = "/native_qos_events/run_%s/matched" % suffix
        pub_node = node("matched_publisher")
        sub_node = node("matched_subscriber")
        sub_node_2 = node("matched_subscriber_two")
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()
        pub_matched = Counter(_matched)
        sub_matched = Counter(_matched)
        sub_matched_2 = Counter(_matched)
        pub_options, _pub_cpp_cbs2 = _publisher_options_with_events(
            pub_node, None, {"matched": pub_matched})
        publisher = make_qos_events_publisher_probe(pub_node, topic, qos, pub_options)
        sub_options, _sub_cpp_cbs2 = _subscription_options_with_events(
            sub_node, None, {"matched": sub_matched})
        subscription = make_qos_events_subscription_probe(
            sub_node, topic, qos, sub_options)
        cleanup.extend([publisher, subscription])
        wait_for(
            lambda: pub_matched.count >= 1 and pub_matched.last["current_count"] == 1,
            "publisher matched 0->1")
        wait_for(
            lambda: sub_matched.count >= 1 and sub_matched.last["current_count"] == 1,
            "subscription matched 0->1")
        sub_matched_count_snapshot = sub_matched.count
        sub_matched_last_snapshot = dict(sub_matched.last)

        sub_options_2, _sub_cpp_cbs2b = _subscription_options_with_events(
            sub_node_2, None, {"matched": sub_matched_2})
        subscription_2 = make_qos_events_subscription_probe(
            sub_node_2, topic, qos, sub_options_2)
        cleanup.append(subscription_2)
        wait_for(
            lambda: pub_matched.count >= 2 and pub_matched.last["current_count"] == 2,
            "publisher matched 1->2")
        wait_for(
            lambda: sub_matched_2.count >= 1 and sub_matched_2.last["current_count"] == 1,
            "second subscription matched 0->1")
        events["publisher_matched"] = {
            "registered": True, "fired": True, "count": pub_matched.count,
            "last": pub_matched.last, "proof_level": "fires",
            "transitions_observed": "0->1->2",
        }
        events["subscription_matched"] = {
            "registered": True, "fired": True, "count": sub_matched_count_snapshot,
            "last": sub_matched_last_snapshot, "proof_level": "fires",
            "transitions_observed": "0->1",
        }

        # --- deadline_missed: a short deadline, publish once, then stop. -------
        topic = "/native_qos_events/run_%s/deadline" % suffix
        pub_node = node("deadline_publisher")
        sub_node = node("deadline_subscriber")
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()
        qos.deadline(rclcpp.Duration(0, 150_000_000))
        pub_deadline = Counter(_deadline)
        sub_deadline = Counter(_deadline)
        pub_options, _pub_cpp_cbs3 = _publisher_options_with_events(
            pub_node, None, {"deadline": pub_deadline})
        publisher = make_qos_events_publisher_probe(pub_node, topic, qos, pub_options)
        sub_options, _sub_cpp_cbs3 = _subscription_options_with_events(
            sub_node, None, {"deadline": sub_deadline})
        subscription = make_qos_events_subscription_probe(
            sub_node, topic, qos, sub_options)
        cleanup.extend([publisher, subscription])
        wait_for(lambda: int(publisher.subscription_count()) == 1, "deadline discovery")
        publisher.publish(1)
        wait_for(lambda: pub_deadline.count >= 1, "publisher deadline missed", timeout=5.0)
        wait_for(lambda: sub_deadline.count >= 1, "subscription deadline missed", timeout=5.0)
        events["publisher_deadline_missed"] = {
            "registered": True, "fired": True, "count": pub_deadline.count,
            "last": pub_deadline.last, "proof_level": "fires",
        }
        events["subscription_deadline_missed"] = {
            "registered": True, "fired": True, "count": sub_deadline.count,
            "last": sub_deadline.last, "proof_level": "fires",
        }

        # --- liveliness: MANUAL_BY_TOPIC with a short lease; publish once
        # (which asserts liveliness under MANUAL_BY_TOPIC), then stop. ----------
        topic = "/native_qos_events/run_%s/liveliness" % suffix
        pub_node = node("liveliness_publisher")
        sub_node = node("liveliness_subscriber")
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()
        qos.liveliness(rclcpp.LivelinessPolicy.ManualByTopic)
        qos.liveliness_lease_duration(rclcpp.Duration(0, 150_000_000))
        pub_liveliness = Counter(_liveliness_lost)
        sub_liveliness = Counter(_liveliness_changed)
        pub_options, _pub_cpp_cbs4 = _publisher_options_with_events(
            pub_node, None, {"liveliness": pub_liveliness})
        publisher = make_qos_events_publisher_probe(pub_node, topic, qos, pub_options)
        sub_options, _sub_cpp_cbs4 = _subscription_options_with_events(
            sub_node, None, {"liveliness": sub_liveliness})
        subscription = make_qos_events_subscription_probe(
            sub_node, topic, qos, sub_options)
        cleanup.extend([publisher, subscription])
        wait_for(lambda: int(publisher.subscription_count()) == 1, "liveliness discovery")
        publisher.publish(1)
        wait_for(
            lambda: pub_liveliness.count >= 1, "publisher liveliness lost", timeout=5.0)
        wait_for(
            lambda: sub_liveliness.count >= 1, "subscription liveliness changed",
            timeout=5.0)
        events["publisher_liveliness_lost"] = {
            "registered": True, "fired": True, "count": pub_liveliness.count,
            "last": pub_liveliness.last, "proof_level": "fires",
        }
        events["subscription_liveliness_changed"] = {
            "registered": True, "fired": True, "count": sub_liveliness.count,
            "last": sub_liveliness.last, "proof_level": "fires",
        }

        # --- message_lost: registration check only. A loopback localhost run
        # cannot reliably trigger this event. ---
        topic = "/native_qos_events/run_%s/message_lost" % suffix
        pub_node = node("message_lost_publisher")
        sub_node = node("message_lost_subscriber")
        pub_qos = rclcpp.QoS(rclcpp.KeepLast(1))
        pub_qos.best_effort().durability_volatile()
        sub_qos = rclcpp.QoS(rclcpp.KeepLast(1))
        sub_qos.best_effort().durability_volatile()
        sub_message_lost = Counter(_message_lost)
        pub_options, _pub_cpp_cbs5 = _publisher_options_with_events(pub_node, None, {})
        publisher = make_qos_events_publisher_probe(pub_node, topic, pub_qos, pub_options)
        sub_options, _sub_cpp_cbs5 = _subscription_options_with_events(
            sub_node, None, {"message_lost": sub_message_lost})
        subscription = make_qos_events_subscription_probe(
            sub_node, topic, sub_qos, sub_options)
        cleanup.extend([publisher, subscription])
        wait_for(
            lambda: int(publisher.subscription_count()) == 1, "message_lost discovery")
        for value in range(2000):
            publisher.publish(value)
        time.sleep(0.5)
        events["subscription_message_lost"] = {
            "registered": True,
            "fired": sub_message_lost.count > 0,
            "count": sub_message_lost.count,
            "last": sub_message_lost.last,
            "proof_level": "registration",
        }

        # --- incompatible_type: unsupported on Cyclone -- must fail closed,
        # all-or-nothing, exercised through the real direct_entities factory. ---
        it_sub_node = node("incompatible_type_subscriber")
        it_pub_node = node("incompatible_type_publisher")
        it_sub_topic = "/native_qos_events/run_%s/incompatible_type_sub" % suffix
        it_pub_topic = "/native_qos_events/run_%s/incompatible_type_pub" % suffix
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()

        def _noop(*_args):
            pass

        sub_raised = None
        try:
            create_subscription(
                it_sub_node, message_type, it_sub_topic, _noop, qos,
                event_callbacks={"matched": _noop, "incompatible_type": _noop},
            )
        except QoSEventUnsupported as exc:
            sub_raised = str(exc)

        pub_raised = None
        try:
            create_managed_publisher(
                it_pub_node, message_type, it_pub_topic, qos,
                event_callbacks={"matched": _noop, "incompatible_type": _noop},
            )
        except QoSEventUnsupported as exc:
            pub_raised = str(exc)

        # Control: the identical request minus incompatible_type must succeed,
        # proving the failure above is specific to that one event, not the QoS
        # or the other requested event.
        control_subscription = create_subscription(
            it_sub_node, message_type, it_sub_topic, _noop, qos,
            event_callbacks={"matched": _noop},
        )
        control_ok = control_subscription.entity is not None
        control_subscription.close()

        incompatible_type_fail_closed.update({
            "subscription_raised_qos_event_unsupported": sub_raised is not None,
            "publisher_raised_qos_event_unsupported": pub_raised is not None,
            "control_subscription_without_incompatible_type_ok": bool(control_ok),
        })

        # --- teardown/UAF guard: close a subscription with a *repeating* event
        # (deadline-missed, so it would keep firing if still alive) while the
        # executor thread keeps running; assert no further callback fires and
        # the executor stays healthy (the entity-destruction-order contract). --
        topic = "/native_qos_events/run_%s/teardown_uaf" % suffix
        peer_node = node("teardown_uaf_peer")
        guarded_node = node("teardown_uaf_guarded")
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()
        qos.deadline(rclcpp.Duration(0, 100_000_000))
        peer_options, _peer_cpp_cbs = _publisher_options_with_events(
            peer_node, None, {})
        peer_publisher = make_qos_events_publisher_probe(peer_node, topic, qos, peer_options)
        cleanup.append(peer_publisher)

        teardown_deadline_count = [0]

        def on_deadline(info):
            teardown_deadline_count[0] += 1

        def on_message(_message):
            pass

        guarded_subscription = create_subscription(
            guarded_node, message_type, topic, on_message, qos,
            event_callbacks={"deadline": on_deadline},
        )
        wait_for(
            lambda: int(peer_publisher.subscription_count()) == 1,
            "teardown discovery")
        peer_publisher.publish(1)
        wait_for(
            lambda: teardown_deadline_count[0] >= 2,
            "teardown deadline missed repeats", timeout=5.0)
        fired_before_close = teardown_deadline_count[0]
        exceptions_before_close = int(thread.exceptions)
        running_before_close = bool(thread.running)
        entity_released = guarded_subscription.close()
        time.sleep(0.6)
        teardown_uaf_guard.update({
            "fired_before_close": fired_before_close,
            "fired_after_close_and_wait": teardown_deadline_count[0],
            "entity_released_on_close": bool(entity_released),
            "executor_exceptions_before_close": exceptions_before_close,
            "executor_running_before_close": running_before_close,
            "executor_exceptions_after_close": int(thread.exceptions),
            "executor_running_after_close": bool(thread.running),
        })

        capability_probe = qos_event_capabilities(["incompatible_qos", "matched"])

        assert thread.exceptions == 0
        assert thread.running

        for resource in reversed(cleanup):
            try:
                resource.close()
            except Exception:
                pass
        cleanup.clear()
        gc.collect()

    report = {
        "schema": "rclcpp_kit.native-qos-events-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "capabilities": session.capabilities.to_dict(),
        "capability_probe": capability_probe,
        "events": events,
        "incompatible_type_fail_closed": incompatible_type_fail_closed,
        "teardown_uaf_guard": teardown_uaf_guard,
    }
    print(PREFIX + json.dumps(report, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()

"""Publish C++ messages through a stock ``rclpy.Publisher`` handle.

This module preserves the Python publisher object, graph endpoint, context, and
destruction contract. It borrows the publisher's ``rcl_publisher_t`` only for the
duration of one call. The C++ message is serialized with
``rclcpp::Serialization<T>`` and sent with ``rcl_publish_serialized_message`` so
the stock publisher's C typesupport never interprets a C++ object layout. No
second node, context, or publisher is created.

The pointer is a private rclpy ABI surface. Callers must capability-gate this
module by ROS distribution and architecture and retain a stock fallback.
"""

from __future__ import annotations

import threading

import cppyy

from rclcpp_kit.bringup_rclcpp import (
    _is_msg_cpp,
    _is_msg_python,
    _resolve_message_type,
    bringup_rclcpp,
    convert_python_msg_to_cpp,
)


_INITIALIZE_LOCK = threading.RLock()
_INITIALIZED = False
_BINDING_LOCK = threading.RLock()
_BINDINGS = {}
_MAX_RETAINED_SERIALIZED_CAPACITY = 8 * 1024 * 1024

_PUBLISH_GLUE = r"""
#include <rcl/error_handling.h>
#include <rcl/publisher.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace rclcpp_kit_borrowed
{

template<class MessageT>
class PublishScratch
{
public:
  bool publish(
    uintptr_t publisher_address,
    const MessageT & message,
    size_t max_retained_capacity)
  {
    auto & raw = serialized_.get_rcl_serialized_message();
    raw.buffer_length = 0;
    if (publisher_address == 0) {
      throw std::invalid_argument("rclpy publisher handle address is null");
    }
    auto * publisher = reinterpret_cast<const rcl_publisher_t *>(publisher_address);
    const size_t capacity_before = serialized_.capacity();
    try {
      serializer_.serialize_message(&message, &serialized_);
    } catch (...) {
      raw.buffer_length = 0;
      rcl_reset_error();
      throw;
    }
    if (serialized_.capacity() != capacity_before) {
      ++reallocations_;
    }
    if (serialized_.capacity() > peak_capacity_) {
      peak_capacity_ = serialized_.capacity();
    }
    // Some type-support dispatchers probe an incompatible implementation before
    // succeeding. Do not leak that stale diagnostic into the next RCL operation.
    rcl_reset_error();
    const rcl_ret_t result = rcl_publish_serialized_message(publisher, &raw, nullptr);
    if (result != RCL_RET_OK) {
      raw.buffer_length = 0;
      const auto error_state = rcl_get_error_string();
      const char * detail = error_state.str;
      std::string error = "rcl_publish_serialized_message failed (" +
        std::to_string(result) + ")";
      if (detail != nullptr && detail[0] != '\0') {
        error += ": ";
        error += detail;
      }
      rcl_reset_error();
      throw std::runtime_error(error);
    }
    ++publishes_;
    return serialized_.capacity() > max_retained_capacity;
  }

  bool exceeds_capacity(size_t max_retained_capacity) const
  {
    return serialized_.capacity() > max_retained_capacity;
  }

  size_t capacity() const
  {
    return serialized_.capacity();
  }

  size_t peak_capacity() const
  {
    return peak_capacity_;
  }

  size_t length() const
  {
    return serialized_.size();
  }

  size_t publishes() const
  {
    return publishes_;
  }

  size_t reallocations() const
  {
    return reallocations_;
  }

private:
  rclcpp::Serialization<MessageT> serializer_;
  rclcpp::SerializedMessage serialized_;
  size_t peak_capacity_{0};
  size_t publishes_{0};
  size_t reallocations_{0};
};

}  // namespace rclcpp_kit_borrowed
"""


def _initialize():
    global _INITIALIZED
    if _INITIALIZED:
        return
    with _INITIALIZE_LOCK:
        if _INITIALIZED:
            return
        # This loads/includes rclcpp types but deliberately does not initialize
        # its default context. The stock rclpy context remains authoritative.
        bringup_rclcpp()
        cppyy.cppdef(_PUBLISH_GLUE)
        _INITIALIZED = True


class _PublisherBinding:
    """Immutable process-lifetime cppyy binding for one ROS message class."""

    __slots__ = ("cpp_type_name", "cpp_message_type", "scratch_type")

    def __init__(self, message_type):
        _initialize()
        self.cpp_type_name, self.cpp_message_type = _resolve_message_type(message_type)
        self.scratch_type = cppyy.gbl.rclcpp_kit_borrowed.PublishScratch[
            self.cpp_message_type]


def _binding_for(message_type):
    try:
        return _BINDINGS[message_type]
    except KeyError:
        pass
    with _BINDING_LOCK:
        try:
            return _BINDINGS[message_type]
        except KeyError:
            binding = _PublisherBinding(message_type)
            _BINDINGS[message_type] = binding
            return binding


class PreparedPublisher:
    """Type-resolved hot path for a stock publisher.

    The object holds message type metadata and a bound C++ template callable. It
    does not retain a publisher or native handle; callers pass the live stock
    publisher to each invocation so rclpy's handle context manager guards the
    complete borrowed-pointer lifetime.
    """

    def __init__(self, message_type):
        self.message_type = message_type
        binding = _binding_for(message_type)
        self.cpp_type_name = binding.cpp_type_name
        self.cpp_message_type = binding.cpp_message_type
        self._scratch_type = binding.scratch_type
        self._scratch_local = threading.local()
        self._max_retained_serialized_capacity = (
            _MAX_RETAINED_SERIALIZED_CAPACITY)

    def _scratch_state(self):
        try:
            return self._scratch_local.state
        except AttributeError:
            state = {
                "depth": 0,
                "slots": [],
                "created": 0,
                "reused": 0,
                "evictions": 0,
                "retired_peak_capacity": 0,
                "retired_publishes": 0,
                "retired_reallocations": 0,
            }
            self._scratch_local.state = state
            return state

    def _acquire_scratch(self):
        state = self._scratch_state()
        index = state["depth"]
        if index == len(state["slots"]):
            scratch = self._scratch_type()
            state["slots"].append(scratch)
            state["created"] += 1
        else:
            scratch = state["slots"][index]
            if scratch is None:
                scratch = self._scratch_type()
                state["slots"][index] = scratch
                state["created"] += 1
            else:
                state["reused"] += 1
        state["depth"] += 1
        return state, index, scratch

    @staticmethod
    def _retire_scratch(state, index, scratch):
        state["retired_peak_capacity"] = max(
            state["retired_peak_capacity"], int(scratch.peak_capacity()))
        state["retired_publishes"] += int(scratch.publishes())
        state["retired_reallocations"] += int(scratch.reallocations())
        state["slots"][index] = None
        state["evictions"] += 1

    def _release_scratch(self, state, index, scratch, evict):
        try:
            if evict:
                self._retire_scratch(state, index, scratch)
        finally:
            state["depth"] -= 1

    def _scratch_stats(self):
        """Return value-only diagnostics for the calling thread's scratch pool."""
        state = self._scratch_state()
        active = [scratch for scratch in state["slots"] if scratch is not None]
        return {
            "created": state["created"],
            "reused": state["reused"],
            "evictions": state["evictions"],
            "retained_slots": len(active),
            "retained_capacity": sum(int(item.capacity()) for item in active),
            "retained_length": sum(int(item.length()) for item in active),
            "peak_capacity": max(
                [state["retired_peak_capacity"]] +
                [int(item.peak_capacity()) for item in active]),
            "publishes": (
                state["retired_publishes"] +
                sum(int(item.publishes()) for item in active)),
            "reallocations": (
                state["retired_reallocations"] +
                sum(int(item.reallocations()) for item in active)),
        }

    def _to_cpp(self, message):
        if _is_msg_cpp(message):
            if not isinstance(message, self.cpp_message_type):
                raise TypeError(
                    "expected %s C++ message, got %s" % (
                        self.cpp_type_name, type(message)))
            return message
        if not _is_msg_python(message) or not isinstance(message, self.message_type):
            raise TypeError(
                "expected %s message, got %s" % (
                    getattr(self.message_type, "__name__", self.message_type),
                    type(message),
                )
            )
        return convert_python_msg_to_cpp(message, self.cpp_message_type())

    def publish(self, publisher, message):
        """Publish through ``publisher`` while its rclpy handle is locked alive."""
        try:
            handle = publisher.handle
        except AttributeError as exc:
            raise TypeError("publisher has no rclpy native handle") from exc
        cpp_message = self._to_cpp(message)
        with handle:
            state, index, scratch = self._acquire_scratch()
            evict = False
            try:
                evict = bool(scratch.publish(
                    int(handle.pointer),
                    cpp_message,
                    self._max_retained_serialized_capacity,
                ))
            except Exception:
                evict = bool(scratch.exceeds_capacity(
                    self._max_retained_serialized_capacity))
                raise
            finally:
                self._release_scratch(state, index, scratch, evict)


def prepare(message_type):
    """Resolve and JIT a reusable publisher route for ``message_type``."""
    return PreparedPublisher(message_type)


__all__ = ["PreparedPublisher", "prepare"]

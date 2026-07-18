"""Take C++ messages through a stock ``rclpy.Subscription`` handle.

The route borrows ``rcl_subscription_t`` only while rclpy's handle context is
active. It takes serialized CDR and deserializes into a distinct owning C++
message, avoiding both generated C storage and generated Python field storage.
"""

from __future__ import annotations

import threading

import cppyy

import rclcpp_kit.message_facade as message_facade
from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


_INITIALIZE_LOCK = threading.RLock()
_INITIALIZED = False
_BINDING_LOCK = threading.RLock()
_BINDINGS = {}
_UNSUPPORTED_SEQUENCE_NUMBER = 2**64 - 1

_TAKE_GLUE = r"""
#include <rcl/error_handling.h>
#include <rcl/subscription.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

namespace rclcpp_kit_borrowed
{

template<class MessageT>
class SubscriptionTakeScratch
{
public:
  std::shared_ptr<MessageT> take(uintptr_t subscription_address)
  {
    if (subscription_address == 0) {
      throw std::invalid_argument("rclpy subscription handle address is null");
    }
    auto * subscription = reinterpret_cast<const rcl_subscription_t *>(
      subscription_address);
    auto & raw = serialized_.get_rcl_serialized_message();
    raw.buffer_length = 0;
    rmw_message_info_t info{};
    rcl_reset_error();
    const rcl_ret_t result = rcl_take_serialized_message(
      subscription, &raw, &info, nullptr);
    if (result == RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
      rcl_reset_error();
      return {};
    }
    if (result != RCL_RET_OK) {
      const auto error_state = rcl_get_error_string();
      std::string error = "rcl_take_serialized_message failed (" +
        std::to_string(result) + ")";
      if (error_state.str[0] != '\0') {
        error += ": ";
        error += error_state.str;
      }
      rcl_reset_error();
      throw std::runtime_error(error);
    }
    auto output = std::make_shared<MessageT>();
    try {
      serializer_.deserialize_message(&serialized_, output.get());
    } catch (...) {
      raw.buffer_length = 0;
      rcl_reset_error();
      throw;
    }
    source_timestamp_ = info.source_timestamp;
    received_timestamp_ = info.received_timestamp;
    publication_sequence_number_ = info.publication_sequence_number;
    reception_sequence_number_ = info.reception_sequence_number;
    last_size_ = serialized_.size();
    peak_capacity_ = std::max(peak_capacity_, serialized_.capacity());
    ++takes_;
    return output;
  }

  size_t takes() const {return takes_;}
  size_t capacity() const {return serialized_.capacity();}
  size_t peak_capacity() const {return peak_capacity_;}
  size_t last_size() const {return last_size_;}
  int64_t source_timestamp() const {return source_timestamp_;}
  int64_t received_timestamp() const {return received_timestamp_;}
  uint64_t publication_sequence_number() const
  {
    return publication_sequence_number_;
  }
  uint64_t reception_sequence_number() const
  {
    return reception_sequence_number_;
  }

private:
  rclcpp::Serialization<MessageT> serializer_;
  rclcpp::SerializedMessage serialized_;
  size_t takes_{0};
  size_t peak_capacity_{0};
  size_t last_size_{0};
  int64_t source_timestamp_{0};
  int64_t received_timestamp_{0};
  uint64_t publication_sequence_number_{0};
  uint64_t reception_sequence_number_{0};
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
        bringup_rclcpp()
        cppyy.cppdef(_TAKE_GLUE)
        _INITIALIZED = True


class _SubscriptionBinding:
    __slots__ = ("facade_binding", "scratch_type")

    def __init__(self, facade_binding):
        _initialize()
        self.facade_binding = facade_binding
        self.scratch_type = cppyy.gbl.rclcpp_kit_borrowed.SubscriptionTakeScratch[
            facade_binding.cpp_message_type]


def _binding_for(facade_binding):
    original_type = facade_binding.original_type
    try:
        return _BINDINGS[original_type]
    except KeyError:
        pass
    with _BINDING_LOCK:
        try:
            return _BINDINGS[original_type]
        except KeyError:
            binding = _SubscriptionBinding(facade_binding)
            _BINDINGS[original_type] = binding
            return binding


def _optional_sequence(value):
    value = int(value)
    return None if value == _UNSUPPORTED_SEQUENCE_NUMBER else value


class PreparedSubscription:
    """One stock subscription's serialized C++ take state."""

    def __init__(self, facade_binding):
        self.facade_binding = facade_binding
        binding = _binding_for(facade_binding)
        self._scratch = binding.scratch_type()
        self._lock = threading.Lock()

    @property
    def original_type(self):
        return self.facade_binding.original_type

    @property
    def facade_type(self):
        return self.facade_binding.facade_type

    def take(self, subscription):
        """Return ``(facade, message_info)`` or ``None`` when no data is ready."""
        try:
            handle = subscription.handle
        except AttributeError as exc:
            raise TypeError("subscription has no rclpy native handle") from exc
        with handle:
            with self._lock:
                native = self._scratch.take(int(handle.pointer))
                if not native:
                    return None
                message_info = {
                    "source_timestamp": int(self._scratch.source_timestamp()),
                    "received_timestamp": int(self._scratch.received_timestamp()),
                    "publication_sequence_number": _optional_sequence(
                        self._scratch.publication_sequence_number()),
                    "reception_sequence_number": _optional_sequence(
                        self._scratch.reception_sequence_number()),
                }
        return self.facade_binding.wrap_cpp(native), message_info

    def stats(self):
        with self._lock:
            return {
                "takes": int(self._scratch.takes()),
                "capacity": int(self._scratch.capacity()),
                "peak_capacity": int(self._scratch.peak_capacity()),
                "last_size": int(self._scratch.last_size()),
            }


def prepare(message_type):
    """Prepare independent take state for a supported original or facade type."""
    if isinstance(message_type, message_facade.FacadeBinding):
        facade_binding = message_type
    else:
        facade_binding = message_facade.binding_for_type(message_type)
        if facade_binding is None:
            facade_binding = message_facade.prepare(message_type)
    return PreparedSubscription(facade_binding)


__all__ = ["PreparedSubscription", "prepare"]

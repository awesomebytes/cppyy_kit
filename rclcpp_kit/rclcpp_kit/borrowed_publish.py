"""Publish C++ messages through a stock ``rclpy.Publisher`` handle.

This module preserves the Python publisher object, graph endpoint, context, and
destruction contract. It borrows the publisher's ``rcl_publisher_t`` only for the
duration of one call and invokes ``rcl_publish`` with the ROS C++ message object.
No second node, context, or publisher is created.

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

_PUBLISH_GLUE = r"""
#include <rcl/error_handling.h>
#include <rcl/publisher.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace rclcpp_kit_borrowed
{

template<class MessageT>
void publish(uintptr_t publisher_address, const MessageT & message)
{
  if (publisher_address == 0) {
    throw std::invalid_argument("rclpy publisher handle address is null");
  }
  auto * publisher = reinterpret_cast<const rcl_publisher_t *>(publisher_address);
  const rcl_ret_t result = rcl_publish(publisher, &message, nullptr);
  if (result != RCL_RET_OK) {
    const auto error_state = rcl_get_error_string();
    const char * detail = error_state.str;
    std::string error = "rcl_publish failed (" + std::to_string(result) + ")";
    if (detail != nullptr && detail[0] != '\0') {
      error += ": ";
      error += detail;
    }
    rcl_reset_error();
    throw std::runtime_error(error);
  }
}

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


class PreparedPublisher:
    """Type-resolved hot path for a stock publisher.

    The object holds message type metadata and a bound C++ template callable. It
    does not retain a publisher or native handle; callers pass the live stock
    publisher to each invocation so rclpy's handle context manager guards the
    complete borrowed-pointer lifetime.
    """

    def __init__(self, message_type):
        _initialize()
        self.message_type = message_type
        self.cpp_type_name, self.cpp_message_type = _resolve_message_type(message_type)
        self._publish_cpp = cppyy.gbl.rclcpp_kit_borrowed.publish[
            self.cpp_message_type]

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
            self._publish_cpp(int(handle.pointer), cpp_message)


def prepare(message_type):
    """Resolve and JIT a reusable publisher route for ``message_type``."""
    return PreparedPublisher(message_type)


__all__ = ["PreparedPublisher", "prepare"]

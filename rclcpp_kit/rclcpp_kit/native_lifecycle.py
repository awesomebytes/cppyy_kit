"""Managed ownership for a real ``rclcpp_lifecycle::LifecycleNode``."""

from __future__ import annotations

import threading
from typing import Any

import cppyy
import cppyy_kit

from rclcpp_kit.bringup_rclcpp import (
    bringup_rclcpp,
    get_ros2_lib_path,
)


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False


def _install_helpers() -> None:
    """Compile the ownership operations that require C++ shared pointers."""
    global _HELPERS_READY
    if _HELPERS_READY:
        return
    with _HELPERS_LOCK:
        if _HELPERS_READY:
            return
        bringup_rclcpp()
        cppyy_kit.load_libraries(
            ["librclcpp_lifecycle.so"],
            [get_ros2_lib_path()],
        )
        cppyy.include("rclcpp_lifecycle/lifecycle_node.hpp")
        cppyy.cppdef(
            r"""
            #include <algorithm>
            #include <cstddef>
            #include <memory>
            #include <mutex>
            #include <stdexcept>
            #include <string>
            #include <vector>
            #include <rclcpp/rclcpp.hpp>
            #include <rclcpp_lifecycle/lifecycle_node.hpp>

            namespace rclcpp_kit_native_lifecycle {
            class LifecycleNodeResource {
            public:
              LifecycleNodeResource(
                  const std::string& name,
                  const std::string& namespace_,
                  const rclcpp::NodeOptions& options,
                  bool enable_communication_interface)
              : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(
                    name,
                    namespace_,
                    options,
                    enable_communication_interface))
              {}

              LifecycleNodeResource(const LifecycleNodeResource&) = delete;
              LifecycleNodeResource& operator=(const LifecycleNodeResource&) = delete;

              ~LifecycleNodeResource() noexcept
              {
                close();
              }

              std::shared_ptr<rclcpp_lifecycle::LifecycleNode> raw_node() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                return node_;
              }

              void attach_executor(std::shared_ptr<rclcpp::Executor> executor)
              {
                if (!executor) {
                  throw std::invalid_argument("executor must not be null");
                }
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                purge_expired_executors();
                const auto duplicate = std::find_if(
                  executors_.begin(),
                  executors_.end(),
                  [&executor](const std::weak_ptr<rclcpp::Executor>& candidate) {
                    const auto existing = candidate.lock();
                    return existing && existing.get() == executor.get();
                  });
                if (duplicate != executors_.end()) {
                  throw std::logic_error("executor is already attached");
                }
                executor->add_node(node_->get_node_base_interface());
                executors_.push_back(executor);
              }

              bool detach_executor(std::shared_ptr<rclcpp::Executor> executor)
              {
                if (!executor) {
                  return false;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                for (auto iterator = executors_.begin();
                     iterator != executors_.end(); ++iterator) {
                  const auto existing = iterator->lock();
                  if (existing && existing.get() == executor.get()) {
                    executor->remove_node(node_->get_node_base_interface());
                    executors_.erase(iterator);
                    return true;
                  }
                }
                purge_expired_executors();
                return false;
              }

              std::size_t attached_executors() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                return static_cast<std::size_t>(std::count_if(
                  executors_.begin(),
                  executors_.end(),
                  [](const std::weak_ptr<rclcpp::Executor>& executor) {
                    return !executor.expired();
                  }));
              }

              bool closed() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                return !node_;
              }

              void close() noexcept
              {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!node_) {
                  return;
                }
                const auto base = node_->get_node_base_interface();
                for (auto& weak_executor : executors_) {
                  if (auto executor = weak_executor.lock()) {
                    try {
                      executor->remove_node(base);
                    } catch (...) {
                    }
                  }
                }
                executors_.clear();
                node_.reset();
              }

            private:
              void require_open() const
              {
                if (!node_) {
                  throw std::runtime_error("NativeLifecycleNode is closed");
                }
              }

              void purge_expired_executors()
              {
                executors_.erase(
                  std::remove_if(
                    executors_.begin(),
                    executors_.end(),
                    [](const std::weak_ptr<rclcpp::Executor>& executor) {
                      return executor.expired();
                    }),
                  executors_.end());
              }

              mutable std::mutex mutex_;
              std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
              std::vector<std::weak_ptr<rclcpp::Executor>> executors_;
            };

            std::shared_ptr<LifecycleNodeResource> make_lifecycle_node(
                const std::string& name,
                const std::string& namespace_,
                const rclcpp::NodeOptions& options,
                bool enable_communication_interface)
            {
              return std::make_shared<LifecycleNodeResource>(
                name, namespace_, options, enable_communication_interface);
            }
            }  // namespace rclcpp_kit_native_lifecycle
            """
        )
        _HELPERS_READY = True


class NativeLifecycleNode:
    """Own a lifecycle node and its executor membership.

    The adapter does not duplicate the lifecycle API. :attr:`raw_node` is the
    original shared ``rclcpp_lifecycle::LifecycleNode`` and remains the escape
    hatch for transitions, publishers, parameters, and library integration.
    """

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    @property
    def raw_node(self) -> Any:
        """Return the original shared C++ lifecycle node."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return self._implementation.raw_node()

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    @property
    def attached_executors(self) -> int:
        return int(self._implementation.attached_executors())

    def attach_executor(self, executor: Any) -> None:
        """Add the lifecycle node's base interface to a real C++ executor."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        self._implementation.attach_executor(executor)

    def detach_executor(self, executor: Any) -> bool:
        """Remove this node from an attached executor."""
        if self.closed:
            return False
        return bool(self._implementation.detach_executor(executor))

    def close(self) -> None:
        """Detach from executors and release the owned node exactly once."""
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


def create_native_lifecycle_node(
    owner: Any,
    name: str,
    *,
    namespace: str = "",
    options: Any = None,
    enable_communication_interface: bool = True,
) -> NativeLifecycleNode:
    """Create and retain a real managed ``rclcpp_lifecycle::LifecycleNode``.

    Standard lifecycle services are enabled by default. Attach the returned
    resource to an executor with :meth:`NativeLifecycleNode.attach_executor` to
    serve those endpoints. The supplied owner must expose the ``NativeSession``
    context and resource-registration protocol.
    """
    _install_helpers()
    rclcpp = owner.rclcpp
    if options is None:
        options = rclcpp.NodeOptions()
    options.context(owner.context)
    implementation = cppyy.gbl.rclcpp_kit_native_lifecycle.make_lifecycle_node(
        str(name),
        str(namespace),
        options,
        bool(enable_communication_interface),
    )
    return owner.register_resource(NativeLifecycleNode(implementation))


__all__ = [
    "NativeLifecycleNode",
    "create_native_lifecycle_node",
]

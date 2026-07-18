"""Managed ownership for a real ``rclcpp_components::ComponentManager``.

The container loads only normal AOT C++ components registered in the ament
resource index.  It deliberately leaves the standard composition service API
unchanged and does not attempt to make Python classes loadable components.
Explicit close should not race an in-flight load or unload request; ordered
``NativeSession`` teardown stops executor threads before closing this resource.
"""

from __future__ import annotations

import threading
from typing import Any

import cppyy
import cppyy_kit

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp, get_ros2_lib_path


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False


def _install_helpers() -> None:
    """Compile the shared ownership operations needed by cppyy callers."""
    global _HELPERS_READY
    if _HELPERS_READY:
        return
    with _HELPERS_LOCK:
        if _HELPERS_READY:
            return
        bringup_rclcpp()
        cppyy_kit.load_libraries(
            ["libcomponent_manager.so"],
            [get_ros2_lib_path()],
        )
        cppyy.include("rclcpp_components/component_manager.hpp")
        cppyy.cppdef(
            r"""
            #include <memory>
            #include <mutex>
            #include <stdexcept>
            #include <string>
            #include <utility>
            #include <rclcpp/rclcpp.hpp>
            #include <rclcpp_components/component_manager.hpp>

            namespace rclcpp_kit_native_component {
            class ContextComponentManager final
              : public rclcpp_components::ComponentManager
            {
            public:
              ContextComponentManager(
                  const std::weak_ptr<rclcpp::Executor>& executor,
                  const std::string& node_name,
                  const rclcpp::NodeOptions& options,
                  std::shared_ptr<rclcpp::Context> context)
              : rclcpp_components::ComponentManager(
                    executor, node_name, options),
                context_(std::move(context))
              {}

            protected:
              rclcpp::NodeOptions create_node_options(
                  const std::shared_ptr<LoadNode::Request> request) override
              {
                auto options = ComponentManager::create_node_options(request);
                options.context(context_);
                return options;
              }

            private:
              std::shared_ptr<rclcpp::Context> context_;
            };

            class ComponentManagerResource {
            public:
              ComponentManagerResource(
                  const std::shared_ptr<rclcpp::Executor>& executor,
                  const std::string& node_name,
                  const rclcpp::NodeOptions& options)
              {
                if (!executor) {
                  throw std::invalid_argument("executor must not be null");
                }
                executor_ = executor;
                manager_ = std::make_shared<ContextComponentManager>(
                  std::weak_ptr<rclcpp::Executor>(executor),
                  node_name,
                  options,
                  options.context());
                executor->add_node(manager_->get_node_base_interface());
              }

              ComponentManagerResource(const ComponentManagerResource&) = delete;
              ComponentManagerResource& operator=(
                const ComponentManagerResource&) = delete;

              ~ComponentManagerResource() noexcept
              {
                close();
              }

              std::shared_ptr<rclcpp_components::ComponentManager>
              raw_manager() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                return manager_;
              }

              bool closed() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                return !manager_;
              }

              void close() noexcept
              {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!manager_) {
                  return;
                }
                if (auto executor = executor_.lock()) {
                  try {
                    executor->remove_node(manager_->get_node_base_interface());
                  } catch (...) {
                  }
                }
                manager_.reset();
                executor_.reset();
              }

            private:
              void require_open() const
              {
                if (!manager_) {
                  throw std::runtime_error("NativeComponentManager is closed");
                }
              }

              mutable std::mutex mutex_;
              std::shared_ptr<rclcpp_components::ComponentManager> manager_;
              std::weak_ptr<rclcpp::Executor> executor_;
            };

            std::shared_ptr<ComponentManagerResource> make_component_manager(
                const std::shared_ptr<rclcpp::Executor>& executor,
                const std::string& node_name,
                const rclcpp::NodeOptions& options)
            {
              return std::make_shared<ComponentManagerResource>(
                executor, node_name, options);
            }
            }  // namespace rclcpp_kit_native_component
            """
        )
        _HELPERS_READY = True


class NativeComponentManager:
    """Own a component manager and its membership in a C++ executor.

    Dynamic loading remains the standard ROS composition protocol.  Use stock
    ``composition_interfaces`` clients for load, list, and unload operations.
    :attr:`raw_manager` exposes the original C++ manager for resource discovery
    and advanced ``rclcpp_components`` operations.
    """

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    @property
    def raw_manager(self) -> Any:
        """Return the original shared C++ component manager."""
        if self.closed:
            raise RuntimeError("NativeComponentManager is closed")
        return self._implementation.raw_manager()

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    def close(self) -> None:
        """Detach the manager and release it and all loaded components once."""
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


def create_native_component_manager(
    owner: Any,
    executor: Any,
    *,
    name: str = "ComponentManager",
    options: Any = None,
) -> NativeComponentManager:
    """Create a managed component container on a session-owned executor.

    The manager and every component loaded through it use the owner's custom
    context, replacing any context previously set on ``options``.  The executor
    must be retained and spun by the owner for services and loaded nodes to run.
    """
    _install_helpers()
    rclcpp = owner.rclcpp
    if not any(executor is candidate for candidate in owner.executors):
        raise ValueError("executor is not owned by this NativeSession")
    if options is None:
        options = rclcpp.NodeOptions()
    options.context(owner.context)
    implementation = cppyy.gbl.rclcpp_kit_native_component.make_component_manager(
        executor,
        str(name),
        options,
    )
    return owner.register_resource(NativeComponentManager(implementation))


__all__ = [
    "NativeComponentManager",
    "create_native_component_manager",
]

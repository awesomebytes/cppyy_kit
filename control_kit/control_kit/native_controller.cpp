#include "native_controller.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace control_kit_native {
using controller_interface::CallbackReturn;
using controller_interface::return_type;
PositionServo::PositionServo() = default;
PositionServo::~PositionServo() = default;

CallbackReturn PositionServo::on_init() {
  auto_declare<std::vector<std::string>>("joints", {});
  auto_declare<double>("gain", 6.0);
  auto_declare<std::vector<double>>("speed_limits", {});
  auto_declare<std::vector<double>>("lower_limits", {});
  auto_declare<std::vector<double>>("upper_limits", {});
  return CallbackReturn::SUCCESS;
}

CallbackReturn PositionServo::on_configure(const rclcpp_lifecycle::State &) {
  subscription_.reset();
  std::lock_guard<std::mutex> lock(target_mutex_);
  configured_ = false;
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  gain_ = node->get_parameter("gain").as_double();
  speeds_ = node->get_parameter("speed_limits").as_double_array();
  lower_ = node->get_parameter("lower_limits").as_double_array();
  upper_ = node->get_parameter("upper_limits").as_double_array();
  const size_t n = joints_.size();
  if (!n || !std::isfinite(gain_) || gain_ <= 0 ||
      std::set<std::string>(joints_.begin(), joints_.end()).size() != n ||
      std::any_of(joints_.begin(), joints_.end(), [](const auto & j) { return j.empty(); })) {
    return CallbackReturn::ERROR;
  }
  if (speeds_.empty()) speeds_.assign(n, 1.0);
  if (lower_.empty()) lower_.assign(n, -std::numeric_limits<double>::infinity());
  if (upper_.empty()) upper_.assign(n, std::numeric_limits<double>::infinity());
  if (speeds_.size() != n || lower_.size() != n || upper_.size() != n) {
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(speeds_[i]) || speeds_[i] <= 0 ||
        std::isnan(lower_[i]) || std::isnan(upper_[i]) || lower_[i] >= upper_[i]) {
      return CallbackReturn::ERROR;
    }
  }
  pending_.assign(n, 0.0);
  target_.assign(n, 0.0);
  command_.assign(n, 0.0);
  state_index_.resize(n);
  command_index_.resize(n);
  configured_ = true;
  subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/commands", rclcpp::QoS(1).reliable(),
      [this](std_msgs::msg::Float64MultiArray::ConstSharedPtr msg) {
        if (!set_target(msg->data)) {
          RCLCPP_WARN(get_node()->get_logger(), "Rejected invalid position target");
        }
      });
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PositionServo::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration result;
  result.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) result.names.push_back(joint + "/position");
  return result;
}
controller_interface::InterfaceConfiguration PositionServo::state_interface_configuration() const {
  return command_interface_configuration();
}

CallbackReturn PositionServo::on_activate(const rclcpp_lifecycle::State &) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  const size_t n = joints_.size();
  if (!configured_ || state_interfaces_.size() != n || command_interfaces_.size() != n) {
    return CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < n; ++i) {
    const auto name = joints_[i] + "/position";
    auto state = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
        [&](const auto & iface) { return iface.get_name() == name; });
    auto command = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
        [&](const auto & iface) { return iface.get_name() == name; });
    if (state == state_interfaces_.end() || command == command_interfaces_.end()) {
      return CallbackReturn::ERROR;
    }
    state_index_[i] = std::distance(state_interfaces_.begin(), state);
    command_index_[i] = std::distance(command_interfaces_.begin(), command);
    const auto value = state->get_optional();
    if (!value || !std::isfinite(*value) || *value < lower_[i] || *value > upper_[i]) {
      return CallbackReturn::ERROR;
    }
    pending_[i] = target_[i] = command_[i] = *value;
  }
  active_ = true;
  return CallbackReturn::SUCCESS;
}
CallbackReturn PositionServo::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ = false;
  return CallbackReturn::SUCCESS;
}

bool PositionServo::set_target(const std::vector<double> & target) {
  std::lock_guard<std::mutex> lock(target_mutex_);
  if (!configured_ || target.size() != joints_.size()) return false;
  for (size_t i = 0; i < target.size(); ++i) {
    if (!std::isfinite(target[i]) || target[i] < lower_[i] || target[i] > upper_[i]) return false;
  }
  std::copy(target.begin(), target.end(), pending_.begin());
  return true;
}

return_type PositionServo::update(const rclcpp::Time &, const rclcpp::Duration & period) {
  const double dt = period.seconds();
  if (!active_ || !std::isfinite(dt) || dt <= 0 || dt > 0.1) return return_type::ERROR;
  // Never wait for a Python goal setter or a ROS subscription callback.
  if (target_mutex_.try_lock()) {
    std::copy(pending_.begin(), pending_.end(), target_.begin());
    target_mutex_.unlock();
  }
  // Validate the complete command before writing any joint.
  for (size_t i = 0; i < joints_.size(); ++i) {
    const auto state = state_interfaces_[state_index_[i]].get_optional();
    if (!state || !std::isfinite(*state) || *state < lower_[i] || *state > upper_[i]) {
      return return_type::ERROR;
    }
    const double step = std::clamp(std::min(gain_ * dt, 1.0) * (target_[i] - *state),
        -speeds_[i] * dt, speeds_[i] * dt);
    command_[i] = std::clamp(*state + step, lower_[i], upper_[i]);
    if (!std::isfinite(command_[i])) return return_type::ERROR;
  }
  for (size_t i = 0; i < joints_.size(); ++i) {
    if (!command_interfaces_[command_index_[i]].set_value(command_[i])) return return_type::ERROR;
  }
  return return_type::OK;
}

std::shared_ptr<PositionServo> make_position_servo() { return std::make_shared<PositionServo>(); }
}  // namespace control_kit_native

#ifdef CONTROL_KIT_PLUGINLIB
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_kit_native::PositionServo, controller_interface::ControllerInterface)
#endif

#ifndef CONTROL_KIT_NATIVE_CONTROLLER_HPP
#define CONTROL_KIT_NATIVE_CONTROLLER_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace control_kit_native {

// Position uses radians for revolute joints and metres for prismatic joints.
// Gain is s^-1; speed limits use joint units/s.
class PositionServo final : public controller_interface::ControllerInterface {
public:
  PositionServo();
  ~PositionServo() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
  // Non-real-time caller; rejects the whole goal if any component is invalid.
  bool set_target(const std::vector<double> & target);

private:
  std::vector<std::string> joints_;
  std::vector<double> speeds_, lower_, upper_, pending_, target_, command_;
  std::vector<size_t> state_index_, command_index_;
  double gain_ = 6.0;
  std::mutex target_mutex_;
  bool configured_ = false;
  bool active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};

std::shared_ptr<PositionServo> make_position_servo();
}  // namespace control_kit_native
#endif

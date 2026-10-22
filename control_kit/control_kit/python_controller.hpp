#ifndef CONTROL_KIT_PYTHON_CONTROLLER_HPP
#define CONTROL_KIT_PYTHON_CONTROLLER_HPP

#include <mutex>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

struct _object;
namespace control_kit_python {
// pluginlib and loaned-interface boundary for a Python position controller.
class PythonController final : public controller_interface::ControllerInterface {
public:
  PythonController();
  ~PythonController() override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;
  bool set_target(const std::vector<double> & target);
private:
  bool read_positions(std::vector<double> & positions) const;
  std::vector<std::string> joints_;
  std::vector<size_t> state_index_, command_index_;
  std::mutex mutex_;
  _object * controller_ = nullptr;
  bool active_ = false;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
};
}  // namespace control_kit_python
#endif

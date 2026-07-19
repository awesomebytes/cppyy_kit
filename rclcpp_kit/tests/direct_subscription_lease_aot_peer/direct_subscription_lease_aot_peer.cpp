#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

using namespace std::chrono_literals;

int main()
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(
    "direct_subscription_lease_aot_publisher");
  auto uint_publisher = node->create_publisher<std_msgs::msg::UInt64>(
    "direct_lease_aot_uint64", 10);
  auto string_publisher = node->create_publisher<std_msgs::msg::String>(
    "direct_lease_aot_string", 10);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 10s;
  while (rclcpp::ok() &&
    (uint_publisher->get_subscription_count() != 1 ||
    string_publisher->get_subscription_count() != 1) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(10ms);
  }
  if (uint_publisher->get_subscription_count() != 1 ||
    string_publisher->get_subscription_count() != 1)
  {
    std::cerr << "AOT publisher timed out waiting for lease subscriptions\n";
    rclcpp::shutdown();
    return 10;
  }

  std_msgs::msg::UInt64 uint_message;
  uint_message.data = UINT64_C(18446744073709551439);
  std_msgs::msg::String string_message;
  string_message.data = "aot-to-cpp-lease";
  uint_publisher->publish(std::move(uint_message));
  string_publisher->publish(std::move(string_message));
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(100ms);

  std::cout << "DIRECT_SUBSCRIPTION_LEASE_AOT_PUBLISHED_OK" << std::endl;
  rclcpp::shutdown();
  return 0;
}

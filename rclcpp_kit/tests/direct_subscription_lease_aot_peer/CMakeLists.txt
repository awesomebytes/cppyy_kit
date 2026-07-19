cmake_minimum_required(VERSION 3.16)
project(rclcpp_kit_direct_subscription_lease_aot_peer LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(
  direct_subscription_lease_aot_peer
  direct_subscription_lease_aot_peer.cpp)
target_compile_features(direct_subscription_lease_aot_peer PRIVATE cxx_std_17)
target_compile_options(
  direct_subscription_lease_aot_peer PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  direct_subscription_lease_aot_peer rclcpp std_msgs)

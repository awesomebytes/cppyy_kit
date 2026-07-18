cmake_minimum_required(VERSION 3.16)
project(rclcpp_kit_native_action_aot_peer LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(native_action_aot_peer native_action_aot_peer.cpp)
target_compile_features(native_action_aot_peer PRIVATE cxx_std_17)
target_compile_options(native_action_aot_peer PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  native_action_aot_peer
  rclcpp
  rclcpp_action
  tf2_msgs
)

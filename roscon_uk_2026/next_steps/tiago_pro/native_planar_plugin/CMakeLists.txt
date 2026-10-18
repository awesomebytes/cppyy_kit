cmake_minimum_required(VERSION 3.16)
project(cppyy_kit_planar_base_patch LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(cppyy_kit_gazebo_ros_planar_move SHARED "${PLANAR_SOURCE}")
target_compile_features(cppyy_kit_gazebo_ros_planar_move PRIVATE cxx_std_17)
target_include_directories(cppyy_kit_gazebo_ros_planar_move PRIVATE "$ENV{CONDA_PREFIX}/include")
ament_target_dependencies(cppyy_kit_gazebo_ros_planar_move
  gazebo_dev gazebo_ros nav_msgs tf2_geometry_msgs tf2_ros)
install(TARGETS cppyy_kit_gazebo_ros_planar_move LIBRARY DESTINATION lib)

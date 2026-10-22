#pragma once
#include "cloud.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <iomanip>
#include <sstream>
#include <cmath>
namespace pick_place {
// The first stage preserves coordinates and header. Transform before this stage
// when segmentation needs an odom/base frame instead of the native camera frame.
inline void cloud_filter(const sensor_msgs::msg::PointCloud2& input,
                         sensor_msgs::msg::PointCloud2& output, float leaf = .008f) {
  if (!(leaf > 0)) throw std::invalid_argument("leaf must be positive");
  Cloud raw, out;
  pcl::fromROSMsg(input, raw);
  Cloud::Ptr finite(new Cloud);
  for (const auto& p : raw)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite->push_back(p);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(finite); voxel.setLeafSize(leaf, leaf, leaf); voxel.filter(out);
  pcl::toROSMsg(out, output);
  output.header = input.header;
}
inline std::string cloud_json(const sensor_msgs::msg::PointCloud2& input,
                              const CloudConfig& config) {
  Cloud cloud;
  pcl::fromROSMsg(input, cloud);
  auto result = process_cloud(cloud, config);
  std::ostringstream s;
  s << std::setprecision(8) << "{\"frame_id\":" << std::quoted(input.header.frame_id)
    << ",\"stamp_sec\":" << input.header.stamp.sec
    << ",\"stamp_nanosec\":" << input.header.stamp.nanosec
    << ",\"input_points\":" << result.input_points
    << ",\"finite_points\":" << result.finite_points
    << ",\"voxel_points\":" << result.voxel_points
    << ",\"table_found\":" << (result.table_found ? "true" : "false")
    << ",\"table_height\":" << result.table_height << ",\"objects\":[";
  bool first = true;
  for (const auto& o : result.objects) {
    if (!first) s << ',';
    first = false;
    s << "{\"center\":[" << o.center[0] << ',' << o.center[1] << ',' << o.center[2]
      << "],\"size\":[" << o.size[0] << ',' << o.size[1] << ',' << o.size[2]
      << "],\"minimum\":[" << o.lower[0] << ',' << o.lower[1] << ',' << o.lower[2]
      << "],\"maximum\":[" << o.upper[0] << ',' << o.upper[1] << ',' << o.upper[2]
      << "],\"points\":" << o.points << '}';
  }
  return s.str() + "]}";
}
}

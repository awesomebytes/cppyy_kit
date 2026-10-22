#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <vector>

namespace pick_place {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct CloudConfig {
  float leaf = 0.008f, plane_distance = 0.012f;
  float cluster_distance = 0.035f, min_height = 0.018f, max_height = 0.35f;
  std::array<float, 3> lower = {0.25f, -0.8f, -1.0f};
  std::array<float, 3> upper = {2.0f, 0.8f, 0.5f};
  int min_cluster = 12, max_cluster = 100000;
};
struct Object3D {
  std::array<float, 3> center{}, size{}, lower{}, upper{};
  int points = 0;
};
struct CloudResult {
  bool table_found = false;
  float table_height = 0;
  std::array<float, 4> plane{};
  int input_points = 0, finite_points = 0, voxel_points = 0, table_points = 0;
  Cloud objects_cloud;
  std::vector<Object3D> objects;
};
// Coordinates use metres, X forward, Y left, Z up in one fixed input frame.
CloudResult process_cloud(const Cloud&, const CloudConfig&);
}

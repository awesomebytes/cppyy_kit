#include "cloud.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <cmath>
#include <stdexcept>

namespace pick_place {
CloudResult process_cloud(const Cloud& input, const CloudConfig& c) {
  if (!(c.leaf >= 0 && c.plane_distance > 0 && c.cluster_distance > 0 &&
        c.min_height > c.plane_distance && c.max_height > c.min_height &&
        c.min_cluster > 0 && c.max_cluster >= c.min_cluster))
    throw std::invalid_argument("invalid cloud filter configuration");
  for (int i = 0; i < 3; ++i)
    if (!(c.upper[i] > c.lower[i])) throw std::invalid_argument("invalid cloud ROI");
  CloudResult r;
  r.input_points = input.size();
  Cloud::Ptr roi(new Cloud);
  for (const auto& p : input) {
    if (!(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))) continue;
    ++r.finite_points;
    if (p.x >= c.lower[0] && p.x <= c.upper[0] &&
        p.y >= c.lower[1] && p.y <= c.upper[1] &&
        p.z >= c.lower[2] && p.z <= c.upper[2]) roi->push_back(p);
  }
  Cloud::Ptr down(new Cloud);
  if (c.leaf > 0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(roi);
    voxel.setLeafSize(c.leaf, c.leaf, c.leaf);
    voxel.filter(*down);
  } else {
    down = roi;
  }
  r.voxel_points = down->size();
  if (down->size() < 30) return r;
  pcl::SACSegmentation<pcl::PointXYZ> plane;
  plane.setOptimizeCoefficients(true);
  plane.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  plane.setMethodType(pcl::SAC_RANSAC);
  plane.setAxis(Eigen::Vector3f::UnitZ());
  plane.setEpsAngle(0.18);
  plane.setDistanceThreshold(c.plane_distance);
  plane.setMaxIterations(200);
  plane.setInputCloud(down);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  plane.segment(inliers, coefficients);
  if (inliers.indices.size() < 30 || coefficients.values.size() != 4) return r;
  r.table_found = true;
  r.table_points = inliers.indices.size();
  const float sign = coefficients.values[2] < 0 ? -1.f : 1.f;
  for (int i = 0; i < 4; ++i) r.plane[i] = sign * coefficients.values[i];
  r.table_height = -r.plane[3] / r.plane[2];
  Cloud::Ptr objects(new Cloud);
  for (const auto& p : *down) {
    float h = r.plane[0]*p.x + r.plane[1]*p.y + r.plane[2]*p.z + r.plane[3];
    if (h >= c.min_height && h <= c.max_height) objects->push_back(p);
  }
  r.objects_cloud = *objects;
  if (objects->empty()) return r;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> clusters;
  clusters.setInputCloud(objects);
  clusters.setSearchMethod(tree);
  clusters.setClusterTolerance(c.cluster_distance);
  clusters.setMinClusterSize(c.min_cluster);
  clusters.setMaxClusterSize(c.max_cluster);
  std::vector<pcl::PointIndices> indices;
  clusters.extract(indices);
  for (const auto& cluster : indices) {
    Eigen::Vector4f lo, hi;
    pcl::getMinMax3D(*objects, cluster.indices, lo, hi);
    Object3D o;
    o.points = cluster.indices.size();
    for (int i = 0; i < 3; ++i) {
      o.lower[i] = lo[i]; o.upper[i] = hi[i];
      o.center[i] = (lo[i] + hi[i]) / 2;
      o.size[i] = hi[i] - lo[i];
    }
    r.objects.push_back(o);
  }
  return r;
}
}

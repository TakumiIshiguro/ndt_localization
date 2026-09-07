#include "wheel_lidar_localization/map_manager.hpp"
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <stdexcept>

namespace wheel_lidar_localization
{
Cloud::Ptr voxelFilter(const Cloud::ConstPtr & cloud, double leaf)
{
  auto result = std::make_shared<Cloud>();
  if (leaf <= 0) {*result = *cloud; return result;}
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setLeafSize(leaf, leaf, leaf);
  filter.setInputCloud(cloud);
  filter.filter(*result);
  return result;
}
MapManager::MapManager(const std::string & path, double voxel_size, std::size_t min_points)
{
  auto raw = std::make_shared<Cloud>();
  if (path.empty() || pcl::io::loadPCDFile(path, *raw) < 0) {
    throw std::runtime_error("Cannot read PCD map: " + path);
  }
  raw_points_ = raw->size();
  auto finite = std::make_shared<Cloud>();
  finite->reserve(raw->size());
  for (const auto & point : *raw) {if (pcl::isFinite(point)) {finite->push_back(point);}}
  map_ = voxelFilter(finite, voxel_size);
  if (map_->size() < min_points) {
    throw std::runtime_error("Too few finite map points after filtering: " + path);
  }
  tree_.setInputCloud(map_);
}
Cloud::Ptr MapManager::crop(const Eigen::Vector3d & center, double radius) const
{
  std::vector<int> indices;
  std::vector<float> distances;
  tree_.radiusSearch(pcl::PointXYZ(center.x(), center.y(), center.z()), radius, indices, distances);
  auto result = std::make_shared<Cloud>();
  result->reserve(indices.size());
  for (int index : indices) {result->push_back((*map_)[index]);}
  return result;
}
}  // namespace wheel_lidar_localization

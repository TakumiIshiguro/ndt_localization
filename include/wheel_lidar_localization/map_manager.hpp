#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>

namespace wheel_lidar_localization
{
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
Cloud::Ptr voxelFilter(const Cloud::ConstPtr & cloud, double leaf);
class MapManager
{
public:
  MapManager(const std::string & path, double voxel_size, std::size_t min_points);
  Cloud::ConstPtr cloud() const {return map_;}
  Cloud::Ptr crop(const Eigen::Vector3d & center, double radius) const;
  std::size_t rawPoints() const {return raw_points_;}
private:
  Cloud::Ptr map_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_{false};
  std::size_t raw_points_{};
};
}  // namespace wheel_lidar_localization

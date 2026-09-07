#pragma once
#include "wheel_lidar_localization/map_manager.hpp"
#include "wheel_lidar_localization/odometry_buffer.hpp"
#include <pcl/registration/ndt.h>
#include <limits>

namespace wheel_lidar_localization
{
struct MatchResult
{
  Pose pose{Pose::Identity()};
  bool converged{false};
  double rmse{std::numeric_limits<double>::infinity()};
  double inlier_ratio{0};
  double translation_correction{0}, rotation_correction{0};
  int iterations{0};
};
class ScanMatcher
{
public:
  ScanMatcher(double resolution, double step_size, double epsilon, int iterations,
    double correspondence_distance);
  void setTarget(const Cloud::ConstPtr & target);
  MatchResult align(const Cloud::ConstPtr & source, const Pose & seed);
private:
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree_{false};
  Cloud::ConstPtr target_;
  double correspondence_distance_;
  int max_iterations_;
};
}  // namespace wheel_lidar_localization

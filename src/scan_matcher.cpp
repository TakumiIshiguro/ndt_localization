#include "wheel_lidar_localization/scan_matcher.hpp"
#include <pcl/common/point_tests.h>
#include <cmath>

namespace wheel_lidar_localization
{
ScanMatcher::ScanMatcher(double resolution, double step_size, double epsilon, int iterations,
  double correspondence_distance)
: correspondence_distance_(correspondence_distance), max_iterations_(iterations)
{
  ndt_.setResolution(resolution);
  ndt_.setStepSize(step_size);
  ndt_.setTransformationEpsilon(epsilon);
  ndt_.setMaximumIterations(iterations);
}
void ScanMatcher::setTarget(const Cloud::ConstPtr & target)
{
  target_ = target;
  if (!target_->empty()) {ndt_.setInputTarget(target_); tree_.setInputCloud(target_);}
}
MatchResult ScanMatcher::align(const Cloud::ConstPtr & source, const Pose & seed)
{
  MatchResult result;
  if (!target_ || target_->empty() || source->empty() || !validPose(seed)) {return result;}
  ndt_.setInputSource(source);
  Cloud aligned;
  ndt_.align(aligned, seed.matrix().cast<float>());
  result.pose.matrix() = ndt_.getFinalTransformation().cast<double>();
  result.iterations = ndt_.getFinalNumIteration();
  result.converged = ndt_.hasConverged() && result.iterations < max_iterations_ && validPose(result.pose);
  if (!result.converged) {return result;}
  std::vector<int> index(1);
  std::vector<float> squared_distance(1);
  double squared_sum = 0;
  std::size_t inliers = 0;
  for (const auto & point : aligned) {
    if (pcl::isFinite(point) && tree_.nearestKSearch(point, 1, index, squared_distance) > 0 &&
      squared_distance[0] <= correspondence_distance_ * correspondence_distance_)
    {
      squared_sum += squared_distance[0];
      ++inliers;
    }
  }
  result.inlier_ratio = static_cast<double>(inliers) / source->size();
  if (inliers) {result.rmse = std::sqrt(squared_sum / inliers);}
  result.translation_correction = (seed.translation() - result.pose.translation()).norm();
  result.rotation_correction = rotationDistance(seed, result.pose);
  return result;
}
}  // namespace wheel_lidar_localization

#include "wheel_lidar_localization/odometry_buffer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace wheel_lidar_localization
{
bool validPose(const Pose & pose)
{
  return pose.matrix().allFinite() &&
    (pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-4) &&
    std::abs(pose.linear().determinant() - 1.0) < 1e-4;
}
double rotationDistance(const Pose & a, const Pose & b)
{
  return Eigen::Quaterniond(a.linear()).angularDistance(Eigen::Quaterniond(b.linear()));
}
OdometryBuffer::OdometryBuffer(double duration, double max_gap, double reset_time,
  double max_speed, double max_rotation_speed)
: duration_(duration), max_gap_(max_gap), reset_time_(reset_time),
  max_speed_(max_speed), max_rotation_speed_(max_rotation_speed)
{
  if (!(duration > 0 && max_gap > 0 && reset_time > 0 && max_speed > 0 &&
    max_rotation_speed > 0)) {throw std::invalid_argument("Invalid odometry buffer limits");}
}
OdomAdmission OdometryBuffer::add(double stamp, const Pose & pose)
{
  if (!std::isfinite(stamp) || stamp < 0 || !validPose(pose)) {return OdomAdmission::Invalid;}
  bool reset = false;
  if (!samples_.empty()) {
    const auto & last = samples_.back();
    const double dt = stamp - last.stamp;
    if (dt <= 0 && dt > -reset_time_) {return OdomAdmission::OutOfOrder;}
    // A discontinuity invalidates the map/odom relation. Never silently accept it.
    reset = dt <= -reset_time_ || (dt > 0 &&
      ((pose.translation() - last.pose.translation()).norm() > max_speed_ * dt + 0.05 ||
      rotationDistance(pose, last.pose) > max_rotation_speed_ * dt + 0.05));
    if (reset) {samples_.clear();}
  }
  samples_.push_back({stamp, pose});
  while (samples_.size() > 2 && samples_[1].stamp < stamp - duration_) {samples_.pop_front();}
  return reset ? OdomAdmission::Reset : OdomAdmission::Accepted;
}
std::optional<Pose> OdometryBuffer::at(double stamp) const
{
  if (!std::isfinite(stamp) || samples_.empty() || stamp < samples_.front().stamp ||
    stamp > samples_.back().stamp) {return std::nullopt;}
  auto upper = std::lower_bound(samples_.begin(), samples_.end(), stamp,
    [](const OdomSample & s, double t) {return s.stamp < t;});
  if (upper == samples_.end()) {return std::nullopt;}
  if (upper->stamp == stamp) {return upper->pose;}
  if (upper == samples_.begin()) {return std::nullopt;}
  const auto & lower = *std::prev(upper);
  const double dt = upper->stamp - lower.stamp;
  if (dt > max_gap_) {return std::nullopt;}
  const double fraction = (stamp - lower.stamp) / dt;
  Pose result = Pose::Identity();
  result.translation() = (1 - fraction) * lower.pose.translation() + fraction * upper->pose.translation();
  result.linear() = Eigen::Quaterniond(lower.pose.linear()).slerp(
    fraction, Eigen::Quaterniond(upper->pose.linear())).normalized().toRotationMatrix();
  return result;
}
void OdometryBuffer::clear() {samples_.clear();}
}  // namespace wheel_lidar_localization

// (本パッケージ独自) 入力・出力・処理は odometry_integrator.hpp を参照。
#include "odometry_integrator.hpp"
#include "ndt_localization/utils_geometry/geometry.hpp"
#include <cmath>
#include <utility>

namespace ndt_localization::odometry_integrator
{
Pose2D integrate(const Pose2D & pose, const double vx, const double vy, const double wz, const double dt)
{
  const double yaw_mid = pose.yaw + 0.5 * wz * dt;
  Pose2D next;
  next.x = pose.x + (vx * std::cos(yaw_mid) - vy * std::sin(yaw_mid)) * dt;
  next.y = pose.y + (vx * std::sin(yaw_mid) + vy * std::cos(yaw_mid)) * dt;
  const double yaw = pose.yaw + wz * dt;
  next.yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  return next;
}

OdometryIntegrator::OdometryIntegrator(std::string odom_frame_id, std::string base_frame_id, const double max_dt)
: odom_frame_id_(std::move(odom_frame_id)), base_frame_id_(std::move(base_frame_id)), max_dt_(max_dt)
{
}

OdometryIntegrator::Status OdometryIntegrator::update(const geometry_msgs::msg::TwistWithCovarianceStamped & twist)
{
  const rclcpp::Time stamp(twist.header.stamp, RCL_ROS_TIME);
  last_twist_ = twist;
  if (!last_stamp_) {
    last_stamp_ = stamp;
    return Status::FIRST_MESSAGE;
  }
  const double dt = (stamp - *last_stamp_).seconds();
  last_stamp_ = stamp;
  if (dt < 0.0) {
    return Status::TIME_JUMPED_BACK;
  }
  if (dt > max_dt_) {
    return Status::GAP_TOO_LONG;
  }
  const auto & v = twist.twist.twist;
  pose_ = integrate(pose_, v.linear.x, v.linear.y, v.angular.z, dt);
  return Status::INTEGRATED;
}

nav_msgs::msg::Odometry OdometryIntegrator::odometry() const
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = last_twist_.header.stamp;
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position = utils_geometry::create_point(pose_.x, pose_.y, 0.0);
  odom.pose.pose.orientation = utils_geometry::create_quaternion_from_rpy(0.0, 0.0, pose_.yaw);
  odom.twist = last_twist_.twist;
  return odom;
}
}  // namespace ndt_localization::odometry_integrator

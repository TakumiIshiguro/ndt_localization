// 参照元: autoware_core (commit e0a8ab84)
//   sensing/autoware_vehicle_velocity_converter/src/vehicle_velocity_converter.cpp
#include "vehicle_velocity_converter.hpp"
#include "ndt_localization/utils_geometry/covariance.hpp"

namespace ndt_localization::vehicle_velocity_converter
{
VehicleVelocityConverter::VehicleVelocityConverter(
  const double speed_scale_factor, const double stddev_vx, const double stddev_wz)
: speed_scale_factor_(speed_scale_factor), stddev_vx_(stddev_vx), stddev_wz_(stddev_wz)
{
}

// Ref: VehicleVelocityConverter::convert（入力の型と header の作り方だけ違う。共分散は同じ）
geometry_msgs::msg::TwistWithCovarianceStamped VehicleVelocityConverter::convert(
  const nav_msgs::msg::Odometry & msg) const
{
  using COV_IDX = utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  geometry_msgs::msg::TwistWithCovarianceStamped twist_with_covariance_msg;
  twist_with_covariance_msg.header.stamp = msg.header.stamp;
  // Odometry の twist は child_frame_id（base_link）で表されている
  twist_with_covariance_msg.header.frame_id = msg.child_frame_id;
  twist_with_covariance_msg.twist.twist.linear.x = msg.twist.twist.linear.x * speed_scale_factor_;
  twist_with_covariance_msg.twist.twist.linear.y = msg.twist.twist.linear.y;
  twist_with_covariance_msg.twist.twist.angular.z = msg.twist.twist.angular.z;
  twist_with_covariance_msg.twist.covariance[COV_IDX::X_X] = stddev_vx_ * stddev_vx_;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Y_Y] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::Z_Z] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::ROLL_ROLL] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::PITCH_PITCH] = 10000.0;
  twist_with_covariance_msg.twist.covariance[COV_IDX::YAW_YAW] = stddev_wz_ * stddev_wz_;
  return twist_with_covariance_msg;
}
}  // namespace ndt_localization::vehicle_velocity_converter

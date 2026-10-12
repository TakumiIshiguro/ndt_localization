// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/src/util_func.cpp
// 各関数の入力・出力・処理はヘッダーを参照。Autoware が autoware_utils の関数を呼んでいる箇所は、
// 同じ処理を tf2 で書いている（その関数名をコメントに記した）。
#include "ndt_localization/localization_util/util_func.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace ndt_localization::localization_util
{
// 参照: calc_diff_for_radian（中身は autoware_utils_math::normalize_radian）
double calc_diff_for_radian(const double lhs_rad, const double rhs_rad)
{
  constexpr double min_value = -M_PI;
  constexpr double max_value = min_value + 2 * M_PI;
  const double value = std::fmod(lhs_rad - rhs_rad, 2 * M_PI);
  if (min_value <= value && value < max_value) {return value;}
  return value - std::copysign(2 * M_PI, value);
}

// 参照: get_rpy（中身は autoware_utils_geometry::get_rpy = tf2::Matrix3x3::getRPY）
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

// 参照: get_rpy（PoseStamped 版）
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseStamped & pose)
{
  return get_rpy(pose.pose);
}

// 参照: calc_twist
geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b)
{
  const rclcpp::Duration dt = rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp);
  const double dt_s = dt.seconds();

  if (dt_s == 0) {
    return geometry_msgs::msg::Twist();
  }

  const auto pose_a_rpy = get_rpy(pose_a);
  const auto pose_b_rpy = get_rpy(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;
  diff_rpy.x = calc_diff_for_radian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calc_diff_for_radian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calc_diff_for_radian(pose_b_rpy.z, pose_a_rpy.z);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = diff_xyz.x / dt_s;
  twist.linear.y = diff_xyz.y / dt_s;
  twist.linear.z = diff_xyz.z / dt_s;
  twist.angular.x = diff_rpy.x / dt_s;
  twist.angular.y = diff_rpy.y / dt_s;
  twist.angular.z = diff_rpy.z / dt_s;

  return twist;
}

// 参照: interpolate_pose（PoseStamped 版）
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp)
{
  const rclcpp::Time pose_a_time_stamp = pose_a.header.stamp;
  const rclcpp::Time pose_b_time_stamp = pose_b.header.stamp;
  if ((pose_a_time_stamp.seconds() == 0.0) || (pose_b_time_stamp.seconds() == 0.0) ||
    (time_stamp.seconds() == 0.0))
  {
    return geometry_msgs::msg::PoseStamped();
  }

  const auto twist = calc_twist(pose_a, pose_b);
  const double dt = (time_stamp - pose_a_time_stamp).seconds();

  const auto pose_a_rpy = get_rpy(pose_a);

  geometry_msgs::msg::Vector3 xyz;
  geometry_msgs::msg::Vector3 rpy;
  xyz.x = pose_a.pose.position.x + twist.linear.x * dt;
  xyz.y = pose_a.pose.position.y + twist.linear.y * dt;
  xyz.z = pose_a.pose.position.z + twist.linear.z * dt;
  rpy.x = pose_a_rpy.x + twist.angular.x * dt;
  rpy.y = pose_a_rpy.y + twist.angular.y * dt;
  rpy.z = pose_a_rpy.z + twist.angular.z * dt;

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp = time_stamp;
  pose.pose.position.x = xyz.x;
  pose.pose.position.y = xyz.y;
  pose.pose.position.z = xyz.z;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

// 参照: interpolate_pose（PoseWithCovarianceStamped 版）
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp)
{
  geometry_msgs::msg::PoseStamped tmp_pose_a;
  tmp_pose_a.header = pose_a.header;
  tmp_pose_a.pose = pose_a.pose.pose;

  geometry_msgs::msg::PoseStamped tmp_pose_b;
  tmp_pose_b.header = pose_b.header;
  tmp_pose_b.pose = pose_b.pose.pose;

  return interpolate_pose(tmp_pose_a, tmp_pose_b, time_stamp);
}

// 参照: pose_to_affine3d
Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose;
  tf2::fromMsg(ros_pose, eigen_pose);
  return eigen_pose;
}

// 参照: pose_to_matrix4f
Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose_affine = pose_to_affine3d(ros_pose);
  Eigen::Matrix4f eigen_pose_matrix = eigen_pose_affine.matrix().cast<float>();
  return eigen_pose_matrix;
}

// 参照: point_to_vector3d
Eigen::Vector3d point_to_vector3d(const geometry_msgs::msg::Point & ros_pos)
{
  Eigen::Vector3d eigen_pos;
  eigen_pos.x() = ros_pos.x;
  eigen_pos.y() = ros_pos.y;
  eigen_pos.z() = ros_pos.z;
  return eigen_pos;
}

// 参照: matrix4f_to_pose
geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f & eigen_pose_matrix)
{
  Eigen::Affine3d eigen_pose_affine;
  eigen_pose_affine.matrix() = eigen_pose_matrix.cast<double>();
  geometry_msgs::msg::Pose ros_pose = tf2::toMsg(eigen_pose_affine);
  return ros_pose;
}

// 参照: norm（中身は autoware_utils_geometry::calc_distance3d）
double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::hypot(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}
}  // namespace ndt_localization::localization_util

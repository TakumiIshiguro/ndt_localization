// 参照元: autoware_utils (commit 6b89a2e1)
//   autoware_utils_geometry/include/autoware_utils_geometry/geometry.hpp
//   autoware_utils_geometry/src/geometry/geometry.cpp
// ekf_localizer が使う関数だけを、同じ名前・同じ処理で移植している（ヘッダーだけで完結させた）。
#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <string>

namespace ndt_localization::utils_geometry
{
/// 点を作る。入力: x, y, z [m]。出力: geometry_msgs::msg::Point。
inline geometry_msgs::msg::Point create_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

/**
 * @brief クォータニオンを roll/pitch/yaw に変換する。
 * @param[in] quat 向き。
 * @return x=roll, y=pitch, z=yaw [rad]（tf2::Matrix3x3::getRPY と同じ）。
 */
inline geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Quaternion & quat)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

/// roll/pitch/yaw [rad] からクォータニオンを作る（tf2::Quaternion::setRPY）。
inline geometry_msgs::msg::Quaternion create_quaternion_from_rpy(const double roll, const double pitch, const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::toMsg(q);
}

/**
 * @brief 姿勢を TF に変換する。
 * @param[in] pose 親フレーム（header.frame_id）での子フレームの姿勢。
 * @param[in] child_frame_id 子フレーム名。
 * @return header は pose と同じ、translation / rotation は pose の位置と向き。
 */
inline geometry_msgs::msg::TransformStamped pose2transform(
  const geometry_msgs::msg::PoseStamped & pose, const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = pose.header;
  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;
  transform.child_frame_id = child_frame_id;
  return transform;
}
}  // namespace ndt_localization::utils_geometry

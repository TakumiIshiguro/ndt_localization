// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/include/autoware/localization_util/util_func.hpp
// このパッケージで使う関数だけを、同じ名前・同じ引数・同じ処理で移植している。
#pragma once
#include <Eigen/Geometry>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/time.hpp>

namespace ndt_localization::localization_util
{
/**
 * @brief 2 つの角度の差を [-π, π) に正規化して返す。
 * @param[in] lhs_rad 引かれる角度 [rad]。
 * @param[in] rhs_rad 引く角度 [rad]。
 * @return lhs_rad - rhs_rad を [-π, π) に収めた値 [rad]。
 */
double calc_diff_for_radian(const double lhs_rad, const double rhs_rad);

/**
 * @brief 姿勢の向き（クォータニオン）を roll/pitch/yaw に変換する。
 * @param[in] pose 変換したい姿勢。orientation だけを使う。
 * @return x=roll, y=pitch, z=yaw [rad]。tf2::Matrix3x3::getRPY と同じ ZYX 順。
 */
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose);

/// @brief PoseStamped 版の get_rpy。入力: pose。出力: x=roll, y=pitch, z=yaw [rad]。
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseStamped & pose);

/**
 * @brief 2 つの時刻付き姿勢の差から、一定とみなした速度（並進と roll/pitch/yaw の角速度）を求める。
 * @param[in] pose_a 前の姿勢（時刻 t_a）。
 * @param[in] pose_b 後の姿勢（時刻 t_b）。
 * @return (pose_b - pose_a) / (t_b - t_a)。角度の差は [-π, π) に正規化する。
 *         t_a == t_b のときは 0 の Twist。
 */
geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b);

/**
 * @brief 2 つの時刻付き姿勢から、指定時刻の姿勢を補間（範囲外なら外挿）する。
 * @details calc_twist で求めた一定速度で pose_a から time_stamp まで進める。
 * @param[in] pose_a 基準の姿勢（時刻 t_a）。
 * @param[in] pose_b もう一方の姿勢（時刻 t_b）。
 * @param[in] time_stamp 求めたい時刻。
 * @return time_stamp における姿勢。header は pose_a のもので stamp だけ time_stamp。
 *         どれかの時刻が 0 のときは空の PoseStamped。
 */
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp);

/**
 * @brief PoseWithCovarianceStamped 版の interpolate_pose。共分散は使わない。
 * @param[in] pose_a 基準の姿勢（時刻 t_a）。
 * @param[in] pose_b もう一方の姿勢（時刻 t_b）。
 * @param[in] time_stamp 求めたい時刻。
 * @return time_stamp における姿勢（共分散なし）。
 */
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp);

/**
 * @brief ROS の Pose を Eigen の同次変換（double）に変換する（tf2::fromMsg）。
 * @param[in] ros_pose 変換したい姿勢。
 * @return 回転と並進を持つ Affine3d。
 */
Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose);

/**
 * @brief ROS の Pose を 4x4 の同次変換行列（float）に変換する。NDT の初期値に使う。
 * @param[in] ros_pose 変換したい姿勢。
 * @return 4x4 行列（float）。
 */
Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose);

/**
 * @brief 4x4 の同次変換行列（float）を ROS の Pose に変換する（tf2::toMsg）。NDT の結果に使う。
 * @param[in] eigen_pose_matrix 変換したい 4x4 行列。
 * @return 対応する Pose。
 */
geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f & eigen_pose_matrix);

/**
 * @brief ROS の Point を Eigen のベクトルに変換する。
 * @param[in] ros_pos 変換したい点。
 * @return (x, y, z) のベクトル。
 */
Eigen::Vector3d point_to_vector3d(const geometry_msgs::msg::Point & ros_pos);

/**
 * @brief 2 点間の 3 次元ユークリッド距離を求める。
 * @param[in] p1 1 点目。
 * @param[in] p2 2 点目。
 * @return 距離 [m]。
 */
double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);
}  // namespace ndt_localization::localization_util

// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/include/autoware/localization_util/covariance_ellipse.hpp
// ekf_localizer の診断が使う calculate_xy_ellipse だけを、同じ名前・同じ処理で移植している。
// 移植していないもの: create_ellipse_marker（可視化用）。
#pragma once
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance.hpp>

namespace ndt_localization::localization_util
{
/// xy 平面の誤差楕円。
struct Ellipse
{
  double long_radius;             ///< 長軸の半径 [m]（scale 倍済み）
  double short_radius;            ///< 短軸の半径 [m]（scale 倍済み）
  double yaw;                     ///< 長軸の向き [rad]
  Eigen::Matrix2d P;              ///< xy の 2x2 共分散
  double size_lateral_direction;  ///< 車体の横方向の楕円の大きさ [m]（scale 倍済み）
};

/**
 * @brief 姿勢の共分散から xy の誤差楕円を求める。
 * @param[in] pose_with_covariance 姿勢と 6x6 共分散。xy の 2x2 部分と姿勢の yaw を使う。
 * @param[in] scale 半径に掛ける倍率（標準偏差の何倍の楕円か）。
 * @return 長軸・短軸の半径、長軸の向き、車体の横方向の大きさ。
 */
Ellipse calculate_xy_ellipse(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double scale);
}  // namespace ndt_localization::localization_util

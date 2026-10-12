// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_helper.hpp
// 関数の中身は同じ。移植したのは rotate_covariance と count_oscillation だけ。
#pragma once
#include <Eigen/Core>
#include <geometry_msgs/msg/pose.hpp>
#include <array>
#include <vector>

namespace ndt_localization::ndt_scan_matcher
{
/**
 * @brief 6x6 共分散の位置部分（左上 3x3）を回転させる（例: base_link 基準 → map 基準）。
 * @param[in] src_covariance 行優先の 6x6 共分散（x, y, z, roll, pitch, yaw）。
 * @param[in] rotation 掛ける回転行列 R。位置部分は R * C * R^T になる。
 * @return 位置部分だけを回転させた共分散。角度部分と位置・角度の相関はそのまま。
 */
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation);

/**
 * @brief NDT の反復で、更新の向きが連続して反転した回数の最大値を数える（局所解での振動の検出）。
 * @param[in] result_pose_msg_array NDT の各反復での姿勢の列。
 * @return 連続して「前回の更新とほぼ逆向き（cos < -0.9）」になった回数の最大値。
 *         移動量が 0 の更新は向きがないので振動として数えず、連続回数をリセットする。
 */
int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array);
}  // namespace ndt_localization::ndt_scan_matcher

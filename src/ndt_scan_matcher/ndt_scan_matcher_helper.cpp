// 参照元: autoware_core (commit e0a8ab84)
// 各関数の入力・出力・処理はヘッダーを参照。
//   localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_helper.cpp
#include "ndt_scan_matcher_helper.hpp"
#include "ndt_localization/localization_util/util_func.hpp"
#include <algorithm>
#include <cstddef>

namespace ndt_localization::ndt_scan_matcher
{
using localization_util::point_to_vector3d;

// 参照: rotate_covariance
std::array<double, 36> rotate_covariance(
  const std::array<double, 36> & src_covariance, const Eigen::Matrix3d & rotation)
{
  std::array<double, 36> ret_covariance = src_covariance;
  Eigen::Matrix3d src_cov;
  src_cov << src_covariance[0], src_covariance[1], src_covariance[2], src_covariance[6],
    src_covariance[7], src_covariance[8], src_covariance[12], src_covariance[13], src_covariance[14];
  const Eigen::Matrix3d ret_cov = rotation * src_cov * rotation.transpose();
  for (Eigen::Index i = 0; i < 3; ++i) {
    ret_covariance[i] = ret_cov(0, i);
    ret_covariance[i + 6] = ret_cov(1, i);
    ret_covariance[i + 12] = ret_cov(2, i);
  }
  return ret_covariance;
}

// 参照: count_oscillation
int count_oscillation(const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  constexpr double inversion_vector_threshold = -0.9;
  int oscillation_cnt = 0;
  int max_oscillation_cnt = 0;
  for (std::size_t i = 2; i < result_pose_msg_array.size(); ++i) {
    const Eigen::Vector3d current_pose = point_to_vector3d(result_pose_msg_array.at(i).position);
    const Eigen::Vector3d prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 1).position);
    const Eigen::Vector3d prev_prev_pose = point_to_vector3d(result_pose_msg_array.at(i - 2).position);
    const Eigen::Vector3d current_step = current_pose - prev_pose;
    const Eigen::Vector3d prev_step = prev_pose - prev_prev_pose;
    // 移動量 0 の更新は向きがないので、振動として数えない
    if (current_step.norm() == 0.0 || prev_step.norm() == 0.0) {
      oscillation_cnt = 0;
      continue;
    }
    const double cosine_value = current_step.normalized().dot(prev_step.normalized());
    oscillation_cnt = cosine_value < inversion_vector_threshold ? oscillation_cnt + 1 : 0;
    max_oscillation_cnt = std::max(max_oscillation_cnt, oscillation_cnt);
  }
  return max_oscillation_cnt;
}
}  // namespace ndt_localization::ndt_scan_matcher

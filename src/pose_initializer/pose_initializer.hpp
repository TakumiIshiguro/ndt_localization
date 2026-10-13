// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_pose_initializer/src/pose_initializer.hpp（validate_user_defined_initial_pose）
//   localization/autoware_pose_initializer/src/utils/copy_vector_to_array.hpp（get_covariance_parameter）
// 簡易版の pose_initializer が使う関数だけを、同じ名前・同じ処理で移植している。
#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <string>
#include <vector>

namespace ndt_localization::pose_initializer
{
/**
 * @brief パラメータの [x, y, z, qx, qy, qz, qw] を姿勢にする。
 * @param[in] initial_pose_array 7 要素の配列。
 * @return 姿勢（クォータニオンは正規化しない。Autoware と同じ）。
 * @throws std::invalid_argument 要素数が 7 でない場合、またはクォータニオンの 4 要素がすべてほぼ 0 の場合。
 */
geometry_msgs::msg::Pose validate_user_defined_initial_pose(const std::vector<double> & initial_pose_array);

/**
 * @brief 36 要素の共分散のパラメータを宣言して読む。
 * @param[in] node パラメータを宣言するノード。
 * @param[in] name パラメータ名。
 * @return 6x6 の共分散。
 * @throws std::invalid_argument 要素数が 36 でない場合。
 */
std::array<double, 36> get_covariance_parameter(rclcpp::Node * node, const std::string & name);
}  // namespace ndt_localization::pose_initializer

// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 参照元: autoware_universe (commit 2ee1a85)
//   sensing/autoware_imu_corrector/include/autoware/imu_corrector/imu_corrector_core.hpp
// 変更点: agnocast_wrapper を rclcpp に、autoware_utils の TransformListener をこのパッケージの utils_tf に置き換えた。
// 動的バイアスとスケールの補正（gyro_bias_input / gyro_scale_input と on_off_correction.*）は移植していない
// （推定するノード gyro_bias_estimator などがないため）。固定オフセットは常に引く（Universe の既定と同じ）。

#ifndef IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_
#define IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_

#include "ndt_localization/utils_geometry/covariance.hpp"
#include "ndt_localization/utils_tf/transform_listener.hpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <memory>
#include <string>

namespace ndt_localization::imu_corrector
{
/// \brief 3x3 の共分散（xyz）を、対角の最大値を 3 つの対角に入れた共分散にする（非対角は 0）。
std::array<double, 9> transform_covariance(const std::array<double, 9> & cov);

/// \brief ベクトル \p vec を \p transform で回す（並進は使わない）。
geometry_msgs::msg::Vector3 transform_vector3(
  const geometry_msgs::msg::Vector3 & vec, const geometry_msgs::msg::TransformStamped & transform);

/**
 * @brief IMU "input" の角速度から固定オフセットを引き、角速度と加速度の共分散をパラメータの標準偏差で上書きして、
 *        base_link に回した IMU を "output" に配信するノード。
 *        パラメータ: angular_velocity_offset_{x,y,z}、angular_velocity_stddev_{xx,yy,zz}、acceleration_stddev
 *        （Autoware と同じ名前）。出力の座標系はパラメータ "base_link"（Autoware と同じ名前。既定 "base_link"）。
 */
class ImuCorrector : public rclcpp::Node
{
  using COV_IDX = ndt_localization::utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit ImuCorrector(const rclcpp::NodeOptions & options);

private:
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr & imu_msg_ptr);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  double angular_velocity_offset_x_imu_link_;
  double angular_velocity_offset_y_imu_link_;
  double angular_velocity_offset_z_imu_link_;

  double angular_velocity_stddev_xx_imu_link_;
  double angular_velocity_stddev_yy_imu_link_;
  double angular_velocity_stddev_zz_imu_link_;

  double accel_stddev_imu_link_;

  std::shared_ptr<ndt_localization::utils_tf::TransformListener> transform_listener_;

  std::string output_frame_;
};
}  // namespace ndt_localization::imu_corrector

#endif  // IMU_CORRECTOR__IMU_CORRECTOR_CORE_HPP_

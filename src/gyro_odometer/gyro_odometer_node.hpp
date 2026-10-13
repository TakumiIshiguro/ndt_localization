// Copyright 2015-2019 Autoware Foundation
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

// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_gyro_odometer/src/gyro_odometer_node.hpp
// 変更点: agnocast_wrapper を rclcpp に、autoware_utils の TransformListener と DiagnosticsInterface を
// このパッケージの utils_tf / utils_diagnostics に置き換えた。logger_level_configure を削除した。

#ifndef GYRO_ODOMETER_NODE_HPP_
#define GYRO_ODOMETER_NODE_HPP_

#include "gyro_odometer.hpp"

#include "ndt_localization/utils_diagnostics/diagnostics_interface.hpp"
#include "ndt_localization/utils_geometry/covariance.hpp"
#include "ndt_localization/utils_tf/transform_listener.hpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>

namespace ndt_localization::gyro_odometer
{

using TransformListener = ndt_localization::utils_tf::TransformListener;

/// \brief Reduce an angular-velocity covariance (xyz layout) to an isotropic diagonal covariance.
///
/// The maximum of the three diagonal terms (X_X, Y_Y, Z_Z) is written to all three diagonal
/// terms; every off-diagonal term is zeroed. Pure function: output depends only on the input.
std::array<double, 9> transform_covariance(const std::array<double, 9> & cov);

/// \brief Express \p imu_msg's angular velocity in \p output_frame.
///
/// The transform is looked up per sample and never cached, so a frame that only becomes resolvable
/// later starts working without restarting the node.
/// \return the transformed sample, or std::nullopt if the transform is unavailable.
std::optional<sensor_msgs::msg::Imu> transform_imu(
  const sensor_msgs::msg::Imu & imu_msg, TransformListener & transform_listener,
  const std::string & output_frame);

class GyroOdometerNode : public rclcpp::Node
{
private:
  using COV_IDX = ndt_localization::utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & node_options);

private:
  void callback_vehicle_twist(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr
      vehicle_twist_msg_ptr);
  void callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr);
  void publish_data(const GyroOdometer::OutputData & output_data);
  void publish_diagnostics();

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr vehicle_twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_raw_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
  twist_with_covariance_raw_pub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
  twist_with_covariance_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<TransformListener> transform_listener_;

  std::string output_frame_;
  double message_timeout_sec_;

  /// \brief Whether the most recently arrived IMU sample could be brought into the output frame.
  bool is_succeed_transform_imu_{false};

  std::unique_ptr<ndt_localization::utils_diagnostics::DiagnosticsInterface> diagnostics_;

  GyroOdometer gyro_odometer_;
};

}  // namespace ndt_localization::gyro_odometer

#endif  // GYRO_ODOMETER_NODE_HPP_

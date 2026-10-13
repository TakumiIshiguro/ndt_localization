// 参照元: autoware_core (commit e0a8ab84)
//   sensing/autoware_vehicle_velocity_converter/src/vehicle_velocity_converter_node.hpp
// 違い: rclcpp::Node を直接使う。入力トピックが "velocity_status"（VelocityReport）ではなく "odometry"（Odometry）。
#pragma once
#include "vehicle_velocity_converter.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ndt_localization::vehicle_velocity_converter
{
/**
 * @brief 車輪オドメトリ "odometry" を受け取り、共分散付きの速度 "twist_with_covariance" を配信するノード。
 *        パラメータ: speed_scale_factor、velocity_stddev_xx、angular_velocity_stddev_zz（Autoware と同じ名前）。
 */
class VehicleVelocityConverterNode : public rclcpp::Node
{
public:
  explicit VehicleVelocityConverterNode(const rclcpp::NodeOptions & options);

private:
  /// 受け取ったオドメトリを変換してそのまま配信する。
  void callback_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_pub_;
  VehicleVelocityConverter converter_;
};
}  // namespace ndt_localization::vehicle_velocity_converter

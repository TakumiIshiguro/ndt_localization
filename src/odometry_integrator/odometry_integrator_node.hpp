// (本パッケージ独自) 入力・出力・処理は odometry_integrator.hpp を参照。
#pragma once
#include "odometry_integrator.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>

namespace ndt_localization::odometry_integrator
{
/**
 * @brief 速度 "twist_with_covariance"（gyro_odometer の出力）を積分し、オドメトリ "odometry" と
 *        TF odom->base_link を配信するノード。
 *        パラメータ: odom_frame_id、base_frame_id、publish_tf、max_dt（config/odometry_integrator.param.yaml）。
 */
class OdometryIntegratorNode : public rclcpp::Node
{
public:
  explicit OdometryIntegratorNode(const rclcpp::NodeOptions & options);

private:
  /// 速度を積分して、オドメトリと（publish_tf なら）TF を配信する。
  void callback_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  bool publish_tf_;
  OdometryIntegrator integrator_;
};
}  // namespace ndt_localization::odometry_integrator

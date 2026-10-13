// 参照元: autoware_core (commit e0a8ab84)
//   sensing/autoware_vehicle_velocity_converter/src/vehicle_velocity_converter_node.cpp
#include "vehicle_velocity_converter_node.hpp"
#include <functional>
#include <memory>

namespace ndt_localization::vehicle_velocity_converter
{
// Ref: VehicleVelocityConverterNode::VehicleVelocityConverterNode（QoS も同じ）
VehicleVelocityConverterNode::VehicleVelocityConverterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vehicle_velocity_converter", options),
  converter_(
    declare_parameter<double>("speed_scale_factor"), declare_parameter<double>("velocity_stddev_xx"),
    declare_parameter<double>("angular_velocity_stddev_zz"))
{
  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry", rclcpp::QoS{10},
    std::bind(&VehicleVelocityConverterNode::callback_odometry, this, std::placeholders::_1));
  twist_with_covariance_pub_ =
    create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("twist_with_covariance", rclcpp::QoS{10});
}

// Ref: VehicleVelocityConverterNode::callback_velocity_report
void VehicleVelocityConverterNode::callback_odometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  auto twist_with_covariance_msg = std::make_unique<geometry_msgs::msg::TwistWithCovarianceStamped>();
  *twist_with_covariance_msg = converter_.convert(*msg);
  twist_with_covariance_pub_->publish(std::move(twist_with_covariance_msg));
}
}  // namespace ndt_localization::vehicle_velocity_converter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization::vehicle_velocity_converter::VehicleVelocityConverterNode)

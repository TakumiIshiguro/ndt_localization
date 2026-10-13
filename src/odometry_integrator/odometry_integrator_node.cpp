// (本パッケージ独自) 入力・出力・処理は odometry_integrator.hpp を参照。
#include "odometry_integrator_node.hpp"
#include <functional>
#include <string>

namespace ndt_localization::odometry_integrator
{
OdometryIntegratorNode::OdometryIntegratorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("odometry_integrator", options),
  publish_tf_(declare_parameter<bool>("publish_tf")),
  integrator_(
    declare_parameter<std::string>("odom_frame_id"), declare_parameter<std::string>("base_frame_id"),
    declare_parameter<double>("max_dt"))
{
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "twist_with_covariance", rclcpp::QoS{10},
    std::bind(&OdometryIntegratorNode::callback_twist, this, std::placeholders::_1));
  odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS{10});
  if (publish_tf_) {
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
}

void OdometryIntegratorNode::callback_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr & msg)
{
  const auto status = integrator_.update(*msg);
  if (status == OdometryIntegrator::Status::TIME_JUMPED_BACK) {
    RCLCPP_WARN(get_logger(), "Twist time stamp jumped back; the odometry keeps its pose.");
  } else if (status == OdometryIntegrator::Status::GAP_TOO_LONG) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Twist input had a gap; that interval is not integrated.");
  }

  const nav_msgs::msg::Odometry odom = integrator_.odometry();
  if (msg->header.frame_id != odom.child_frame_id) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "twist frame_id is %s, but it must be %s.",
      msg->header.frame_id.c_str(), odom.child_frame_id.c_str());
  }
  odometry_pub_->publish(odom);
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_br_->sendTransform(transform);
  }
}
}  // namespace ndt_localization::odometry_integrator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization::odometry_integrator::OdometryIntegratorNode)

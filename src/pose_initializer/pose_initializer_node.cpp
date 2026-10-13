// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_pose_initializer/src/pose_initializer_node.cpp
// 入力・出力・処理と Autoware との違いは pose_initializer_node.hpp を参照。
#include "pose_initializer_node.hpp"
#include "pose_initializer.hpp"
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

namespace ndt_localization::pose_initializer
{
using std_srvs::srv::SetBool;

// Ref: PoseInitializerNode::PoseInitializerNode（user_defined_initial_pose と output_pose_covariance の部分）
PoseInitializerNode::PoseInitializerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pose_initializer", options)
{
  pub_reset_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_reset", 1);
  client_ekf_trigger_ = create_client<SetBool>("ekf_trigger_node");
  output_pose_covariance_ = get_covariance_parameter(this, "output_pose_covariance");
  sub_initial_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1, std::bind(&PoseInitializerNode::callback_initial_pose, this, std::placeholders::_1));

  if (declare_parameter<bool>("user_defined_initial_pose.enable")) {
    const geometry_msgs::msg::Pose initial_pose =
      validate_user_defined_initial_pose(declare_parameter<std::vector<double>>("user_defined_initial_pose.pose"));
    // EKF の起動を待つので、/clock に依存しない wall timer で確かめる
    user_defined_initial_pose_timer_ = create_wall_timer(std::chrono::milliseconds(500), [this, initial_pose]() {
      if (!client_ekf_trigger_->service_is_ready()) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "EKF triggering service is not available, waiting...");
        return;
      }
      user_defined_initial_pose_timer_->cancel();
      set_user_defined_initial_pose(initial_pose);
    });
  }
}

void PoseInitializerNode::callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != "map") {
    RCLCPP_WARN(get_logger(), "Initial pose frame_id is %s, but it must be map. Ignored.", msg->header.frame_id.c_str());
    return;
  }
  set_user_defined_initial_pose(msg->pose.pose);
}

// Ref: PoseInitializerNode::set_user_defined_initial_pose
void PoseInitializerNode::set_user_defined_initial_pose(const geometry_msgs::msg::Pose & initial_pose)
{
  if (is_initializing_) {
    pending_pose_ = initial_pose;
    return;
  }
  if (!client_ekf_trigger_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "Could not set user defined initial pose: EKF triggering service is not ready");
    return;
  }
  is_initializing_ = true;
  send_ekf_trigger(false, [this, initial_pose](const bool deactivated) {
    if (!deactivated) {
      RCLCPP_WARN(get_logger(), "Could not set user defined initial pose");
      finish_initialization();
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = now();
    pose.pose.pose = initial_pose;
    pose.pose.covariance = output_pose_covariance_;
    pub_reset_->publish(pose);

    send_ekf_trigger(true, [this](const bool activated) {
      if (activated) {
        RCLCPP_INFO(get_logger(), "Set user defined initial pose");
      } else {
        RCLCPP_WARN(get_logger(), "Could not set user defined initial pose");
      }
      finish_initialization();
    });
  });
}

// Ref: LocalizationTriggerModule::send_request（応答を同期で待たない）
void PoseInitializerNode::send_ekf_trigger(const bool flag, std::function<void(bool)> on_done)
{
  auto req = std::make_shared<SetBool::Request>();
  req->data = flag;
  client_ekf_trigger_->async_send_request(
    req, [this, flag, on_done = std::move(on_done)](rclcpp::Client<SetBool>::SharedFuture future) {
      const bool success = future.get()->success;
      RCLCPP_INFO(get_logger(), "EKF %s %s", flag ? "Activation" : "Deactivation", success ? "succeeded" : "failed");
      on_done(success);
    });
}

void PoseInitializerNode::finish_initialization()
{
  is_initializing_ = false;
  if (pending_pose_) {
    const geometry_msgs::msg::Pose pose = *pending_pose_;
    pending_pose_.reset();
    set_user_defined_initial_pose(pose);
  }
}
}  // namespace ndt_localization::pose_initializer

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization::pose_initializer::PoseInitializerNode)

// Copyright 2018-2019 Autoware Foundation
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
//   localization/autoware_ekf_localizer/src/ekf_localizer_node.hpp
// 変更点: agnocast_wrapper を rclcpp / tf2_ros に置き換えた。デバッグ用の estimated_yaw_bias と
// debug/processing_time_ms（autoware_internal_debug_msgs）、logger_level_configure を削除した。

#ifndef EKF_LOCALIZER_NODE_HPP_
#define EKF_LOCALIZER_NODE_HPP_

#include "ekf_localizer.hpp"
#include "utils/hyper_parameters.hpp"

#include "ndt_localization/utils_system/stop_watch.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

namespace ndt_localization::ekf_localizer
{

class EKFLocalizerNode : public rclcpp::Node
{
public:
  explicit EKFLocalizerNode(const rclcpp::NodeOptions & node_options);

  // This function is only used in static tools to know when timer callbacks are triggered.
  std::chrono::nanoseconds time_until_trigger() const
  {
    return timer_control_->time_until_trigger();
  }

private:
  //!< @brief ekf estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
  //!< @brief estimated ekf pose with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
  //!< @brief estimated ekf odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  //!< @brief ekf estimated twist publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
  //!< @brief ekf estimated twist with covariance publisher
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_biased_pose_;
  //!< @brief ekf estimated yaw bias publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_biased_pose_cov_;
  //!< @brief /diagnostics publisher (manual DiagnosticArray; same absolute topic as former
  //!< diagnostic_updater)
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;
  //!< @brief initial pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  //!< @brief measurement pose with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_with_cov_;
  //!< @brief measurement twist with covariance subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_twist_with_cov_;
  //!< @brief time for ekf calculation callback
  rclcpp::TimerBase::SharedPtr timer_control_;
  //!< @brief calls publish_diagnostics() at diagnostics_publish_period
  rclcpp::TimerBase::SharedPtr diagnostics_publish_timer_;
  //!< @brief trigger_node service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_trigger_node_;

  //!< @brief tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  //!< @brief tf buffer
  tf2_ros::Buffer tf2_buffer_;
  //!< @brief tf listener
  tf2_ros::TransformListener tf2_listener_;

  //!< @brief  extended kalman filter instance.
  std::unique_ptr<EKFLocalizer> ekf_localizer_;

  const HyperParameters params_;

  //!< @brief callback groups for parallel execution
  rclcpp::CallbackGroup::SharedPtr cb_group_pose_;
  rclcpp::CallbackGroup::SharedPtr cb_group_twist_;

  //!< @brief Merged diagnostic status from the latest EKF cycle (message/values refresh every tick)
  diagnostic_msgs::msg::DiagnosticStatus merged_diagnostic_status_;
  //!< @brief Wall time of the latest merged diagnostic level change vs. the previous EKF tick
  //!< (includes recovery to OK); published as last_level_transition_timestamp once non-zero
  rclcpp::Time merged_diagnostic_last_transition_time_;
  //!< @brief last pose callback header stamp in nanoseconds (atomic for thread safety)
  std::atomic<int64_t> last_pose_callback_time_ns_{0};
  //!< @brief last twist callback header stamp in nanoseconds (atomic for thread safety)
  std::atomic<int64_t> last_twist_callback_time_ns_{0};

  /**
   * @brief computes update & prediction of EKF for each ekf_dt_[s] time
   */
  void timer_callback();

  /**
   * @brief set pose with covariance measurement
   */
  void callback_pose_with_covariance(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief set twist with covariance measurement
   */
  void callback_twist_with_covariance(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief set initial_pose to current EKF pose
   */
  void callback_initial_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief get transform from frame_id
   */
  bool get_transform_from_tf(
    std::string parent_frame, std::string child_frame,
    geometry_msgs::msg::TransformStamped & transform);

  /**
   * @brief publish current EKF estimation result
   */
  void publish_estimate_result(const EKFUpdateResult & result);

  /**
   * @brief Overwrite merged_diagnostic_status_ from merged diagnostics each tick;
   * publish_diagnostics() when merged severity increases vs. the previous EKF tick
   * (last transition time updates on any level change).
   */
  void update_diagnostics(
    const std::vector<diagnostic_msgs::msg::DiagnosticStatus> & diag_status_array,
    const rclcpp::Time & current_time);

  /** @brief Build and publish /diagnostics (main merged + callback_pose + callback_twist) */
  void publish_diagnostics();

  /**
   * @brief trigger node
   */
  void service_trigger_node(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  ndt_localization::utils_system::StopWatch<std::chrono::milliseconds> stop_watch_;
  ndt_localization::utils_system::StopWatch<std::chrono::milliseconds> stop_watch_timer_cb_;
};

}  // namespace ndt_localization::ekf_localizer

#endif  // EKF_LOCALIZER_NODE_HPP_

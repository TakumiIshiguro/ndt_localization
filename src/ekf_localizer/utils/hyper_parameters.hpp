// Copyright 2022 Autoware Foundation
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
//   localization/autoware_ekf_localizer/src/utils/hyper_parameters.hpp
// 変更点: agnocast_wrapper::Node を rclcpp::Node に置き換えた。(本パッケージ独自) misc.odom_frame_id と misc.odom_tf_timeout_sec を追加した。

#ifndef UTILS__HYPER_PARAMETERS_HPP_
#define UTILS__HYPER_PARAMETERS_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <string>

namespace ndt_localization::ekf_localizer
{

// Plain data struct: it carries the tuned hyper-parameters and has no rclcpp dependency. Parsing
// lives in the free function load_hyper_parameters() below, so production has a single construction
// path (params_(load_hyper_parameters(this))) and a unit test can build the struct by hand.
struct HyperParameters
{
  bool show_debug_info;
  double ekf_rate;  // ekf update frequency = predict_frequency [Hz]
  double ekf_dt;    // ekf update period [s]
  double tf_rate_;
  bool enable_yaw_bias_estimation;
  size_t extend_state_step;
  std::string pose_frame_id;
  double pose_additional_delay;
  double pose_gate_dist;
  size_t pose_smoothing_steps;
  size_t max_pose_queue_size;
  double twist_additional_delay;
  double twist_gate_dist;
  size_t twist_smoothing_steps;
  size_t max_twist_queue_size;
  double proc_stddev_vx_c;   //!< @brief  vx process noise
  double proc_stddev_wz_c;   //!< @brief  wz process noise
  double proc_stddev_yaw_c;  //!< @brief  yaw process noise
  double z_filter_proc_dev;
  double roll_filter_proc_dev;
  double pitch_filter_proc_dev;
  size_t pose_no_update_count_threshold_warn;
  size_t pose_no_update_count_threshold_error;
  size_t twist_no_update_count_threshold_warn;
  size_t twist_no_update_count_threshold_error;
  double ellipse_scale;
  double error_ellipse_size;
  double warn_ellipse_size;
  double error_ellipse_size_lateral_direction;
  double warn_ellipse_size_lateral_direction;
  double diagnostics_publish_frequency;  //!< @brief diagnostics publish frequency [Hz]
  double diagnostics_publish_period;     //!< @brief diagnostics publish period [s]

  double threshold_observable_velocity_mps;
  // (this package only) Frame of the odometry. When set, the node publishes map->odom instead of
  // map->base_link; an empty string keeps autoware's map->base_link.
  std::string odom_frame_id;
  // (this package only) map->odom is not published while the latest odom->base_link is older than
  // the EKF time by more than this [s] (e.g. right after this process was paused).
  double odom_tf_timeout_sec;
};

// Declares all of the node parameters and returns a fully-populated HyperParameters. This is the
// only place that touches rclcpp, keeping HyperParameters itself a plain data struct.
HyperParameters load_hyper_parameters(rclcpp::Node * node);

}  // namespace ndt_localization::ekf_localizer

#endif  // UTILS__HYPER_PARAMETERS_HPP_

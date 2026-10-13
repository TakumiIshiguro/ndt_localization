// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/hyper_parameters.hpp
// 移植した部分は構造体の並びもパラメータ名も同じ。Autoware と同じく既定値は持たないので、
// config/ndt_scan_matcher.param.yaml ですべてを定義する必要がある。
// 移植していないもの: frame.ndt_base_frame（デバッグ用 TF を出さない）、ndt.regularization、
// initial_pose_estimation、covariance.covariance_estimation（FIXED_VALUE のみ）、dynamic_map_loading。
// 本パッケージ独自: map.path と sensor_points.voxel_size
// （Autoware では map_loader ノードと voxel_grid_downsample_filter ノードが受け持つ設定）。
#pragma once
#include "ndt_localization/ndt_scan_matcher/ndt_omp/ndt_struct.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace ndt_localization::ndt_scan_matcher
{
/// 収束判定に使うスコアの種類（score_estimation.converged_param_type）。
enum class ConvergedParamType {
  TRANSFORM_PROBABILITY = 0,
  NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD = 1
};

/// ndt_scan_matcher のパラメータ一式。コンストラクタで ROS パラメータを宣言して読み込む。
struct HyperParameters
{
  struct Frame
  {
    std::string base_frame{};
    std::string map_frame{};
  } frame{};

  // (本パッケージ独自) Autoware の map_loader ノードの代わりに、起動時に読む PCD のパス
  struct Map
  {
    std::string path{};
  } map{};

  struct SensorPoints
  {
    double timeout_sec{};
    double required_distance{};
    // (本パッケージ独自) Autoware の voxel_grid_downsample_filter ノードの代わり
    double voxel_size{};
  } sensor_points{};

  pclomp::NdtParams ndt{};

  struct Validation
  {
    double initial_pose_timeout_sec{};
    double initial_pose_distance_tolerance_m{};
    double initial_to_result_distance_tolerance_m{};
    double critical_upper_bound_exe_time_ms{};
    int64_t skipping_publish_num{};
  } validation{};

  struct ScoreEstimation
  {
    ConvergedParamType converged_param_type{};
    double converged_param_transform_probability{};
    double converged_param_nearest_voxel_transformation_likelihood{};
  } score_estimation{};

  struct Covariance
  {
    std::array<double, 36> output_pose_covariance{};
  } covariance{};

  /**
   * @brief すべてのパラメータを宣言して読み込む。値の検証はしない（Autoware と同じ）。
   * @param[in] node パラメータを宣言するノード。
   * 出力: 各メンバー変数に読み込んだ値。
   * @throws rclcpp::exceptions::ParameterUninitializedException パラメータが YAML にない場合。
   * @throws std::invalid_argument covariance.output_pose_covariance が 36 要素でない場合。
   */
  explicit HyperParameters(rclcpp::Node * node)
  {
    frame.base_frame = node->declare_parameter<std::string>("frame.base_frame");
    frame.map_frame = node->declare_parameter<std::string>("frame.map_frame");

    map.path = node->declare_parameter<std::string>("map.path");

    sensor_points.timeout_sec = node->declare_parameter<double>("sensor_points.timeout_sec");
    sensor_points.required_distance = node->declare_parameter<double>("sensor_points.required_distance");
    sensor_points.voxel_size = node->declare_parameter<double>("sensor_points.voxel_size");

    ndt.trans_epsilon = node->declare_parameter<double>("ndt.trans_epsilon");
    ndt.step_size = node->declare_parameter<double>("ndt.step_size");
    ndt.resolution = static_cast<float>(node->declare_parameter<double>("ndt.resolution"));
    ndt.max_iterations = static_cast<int>(node->declare_parameter<int64_t>("ndt.max_iterations"));
    ndt.num_threads = static_cast<int>(node->declare_parameter<int64_t>("ndt.num_threads"));
    ndt.num_threads = std::max(ndt.num_threads, 1);
    ndt.regularization_scale_factor = 0.0f;  // 正則化は移植していない

    validation.initial_pose_timeout_sec = node->declare_parameter<double>("validation.initial_pose_timeout_sec");
    validation.initial_pose_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_pose_distance_tolerance_m");
    validation.initial_to_result_distance_tolerance_m =
      node->declare_parameter<double>("validation.initial_to_result_distance_tolerance_m");
    validation.critical_upper_bound_exe_time_ms =
      node->declare_parameter<double>("validation.critical_upper_bound_exe_time_ms");
    validation.skipping_publish_num = node->declare_parameter<int64_t>("validation.skipping_publish_num");

    const int64_t converged_param_type_tmp = node->declare_parameter<int64_t>("score_estimation.converged_param_type");
    score_estimation.converged_param_type = static_cast<ConvergedParamType>(converged_param_type_tmp);
    score_estimation.converged_param_transform_probability =
      node->declare_parameter<double>("score_estimation.converged_param_transform_probability");
    score_estimation.converged_param_nearest_voxel_transformation_likelihood = node->declare_parameter<double>(
      "score_estimation.converged_param_nearest_voxel_transformation_likelihood");

    const std::vector<double> output_pose_covariance =
      node->declare_parameter<std::vector<double>>("covariance.output_pose_covariance");
    if (output_pose_covariance.size() != 36) {
      throw std::invalid_argument("covariance.output_pose_covariance needs 36 values");
    }
    std::copy(output_pose_covariance.begin(), output_pose_covariance.end(), covariance.output_pose_covariance.begin());
  }
};
}  // namespace ndt_localization::ndt_scan_matcher

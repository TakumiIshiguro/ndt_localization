// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ndt_scan_matcher/src/ndt_scan_matcher_core.cpp
// 各関数の上に対応する Autoware の関数を書いた。入力・出力・処理と違いはヘッダーを参照。
#include "ndt_localization/ndt_scan_matcher/ndt_scan_matcher_core.hpp"
#include "../map_loader/pointcloud_map_loader.hpp"
#include "../pointcloud_preprocessor/pointcloud_preprocessor.hpp"
#include "ndt_scan_matcher_helper.hpp"
#include "ndt_localization/localization_util/util_func.hpp"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <vector>

namespace ndt_localization::ndt_scan_matcher
{
using diagnostic_msgs::msg::DiagnosticStatus;
using localization_util::matrix4f_to_pose;
using localization_util::norm;
using localization_util::pose_to_matrix4f;

// 参照: NDTScanMatcher::NDTScanMatcher
NDTScanMatcher::NDTScanMatcher(const rclcpp::NodeOptions & options)
: Node("ndt_scan_matcher", options), tf2_buffer_(this->get_clock()), tf2_listener_(tf2_buffer_), param_(this)
{
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  ndt_ptr_ = std::make_unique<NormalDistributionsTransform>();
  ndt_ptr_->setParams(param_.ndt);

  // (本パッケージ独自) 固定の地図を読む。map_loader/pointcloud_map_loader.hpp を参照
  RCLCPP_INFO(get_logger(), "Loading PCD map: %s", param_.map.path.c_str());
  const auto map_points = map_loader::load_pointcloud_map(param_.map.path);
  ndt_ptr_->setInputTarget(map_points);  // ボクセルとその kd-tree をここで 1 回だけ作る
  RCLCPP_INFO(get_logger(), "NDT target ready: %zu map points, resolution %.2f m", map_points->size(),
    static_cast<double>(param_.ndt.resolution));

  initial_pose_buffer_ = std::make_unique<localization_util::SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);

  initial_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ekf_odom", 100, std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1), sensor_sub_opt);

  sensor_aligned_pose_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  // Autoware の loaded_pcd_pub_ と同じトピック名。地図は 1 回しか読まないので Transient Local で 1 回だけ配信する
  loaded_pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "debug/loaded_pointcloud_map", rclcpp::QoS(1).reliable().transient_local());
  sensor_msgs::msg::PointCloud2 loaded_pcd_msg;
  pcl::toROSMsg(*map_points, loaded_pcd_msg);
  loaded_pcd_msg.header.frame_id = param_.frame.map_frame;
  // 固定の地図はいつでも有効なので、/clock が始まる前でも表示されるよう時刻 0 にする
  loaded_pcd_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  loaded_pcd_pub_->publish(loaded_pcd_msg);

  diagnostics_scan_points_ = std::make_unique<DiagnosticsInterface>(this, "scan_matching_status");
  diagnostics_initial_pose_ = std::make_unique<DiagnosticsInterface>(this, "initial_pose_subscriber_status");
}

// 参照: NDTScanMatcher::callback_initial_pose
void NDTScanMatcher::callback_initial_pose(const nav_msgs::msg::Odometry::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();
  callback_initial_pose_main(initial_pose_msg_ptr);
  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

// 参照: NDTScanMatcher::callback_initial_pose_main
void NDTScanMatcher::callback_initial_pose_main(const nav_msgs::msg::Odometry::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // 検査: is_expected_frame_id（フレーム名が map か）
  const bool is_expected_frame_id = (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id " << initial_pose_msg_ptr->header.frame_id
            << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(DiagnosticStatus::ERROR, message.str());
    return;
  }

  auto initial_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  initial_pose->header = initial_pose_msg_ptr->header;
  initial_pose->pose = initial_pose_msg_ptr->pose;
  initial_pose_buffer_->push_back(initial_pose);
}

// 参照: NDTScanMatcher::callback_sensor_points
void NDTScanMatcher::callback_sensor_points(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  diagnostics_scan_points_->clear();

  const bool is_succeed_scan_matching = callback_sensor_points_main(sensor_points_msg_in_sensor_frame);

  // 検査: skipping_publish_num（照合に失敗した連続回数）
  static int64_t skipping_publish_num = 0;
  skipping_publish_num = is_succeed_scan_matching ? 0 : (skipping_publish_num + 1);
  diagnostics_scan_points_->add_key_value("skipping_publish_num", skipping_publish_num);
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
  }

  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

// 参照: NDTScanMatcher::callback_sensor_points_main
// 検査の内容と順番は Autoware と同じ。違い: 前処理（間引き）をここで行う、
// 正則化・out_of_map_range・地面除去スコア・マーカーとデバッグ用トピックがない、NDT を mutex で保護しない。
bool NDTScanMatcher::callback_sensor_points_main(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  // 検査: topic_time_stamp（点群の時刻）
  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;
  diagnostics_scan_points_->add_key_value("topic_time_stamp", sensor_ros_time.nanoseconds());

  // 検査: sensor_points_size（点数が 0 でないか）
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  diagnostics_scan_points_->add_key_value("sensor_points_size", sensor_points_size);
  if (sensor_points_size == 0) {
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, "Sensor points is empty.");
    return false;
  }

  // 検査: sensor_points_delay_time_sec（遅延。Autoware と同じく警告のみ）
  const double sensor_points_delay_time_sec = (this->now() - sensor_ros_time).seconds();
  diagnostics_scan_points_->add_key_value("sensor_points_delay_time_sec", sensor_points_delay_time_sec);
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency. The delay time is " << sensor_points_delay_time_sec
            << "[sec] (the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
  }

  // 入力点群の前処理
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;
  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);
  // (本パッケージ独自) ボクセルによる間引き。Autoware では手前の voxel_grid_downsample_filter ノードが
  // センサ座標系のまま行うので、同じく base_link への変換の前に行う
  sensor_points_in_sensor_frame = pointcloud_preprocessor::voxel_grid_downsample_filter(
    sensor_points_in_sensor_frame, static_cast<float>(param_.sensor_points.voxel_size));

  // 点群をセンサ座標系から base_link に変換する
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(new pcl::PointCloud<PointSource>);
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame, sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to " << param_.frame.base_frame;
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", false);
    return false;
  }
  diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", true);

  // 検査: sensor_points_max_distance（遠くまで点があるか）
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }
  diagnostics_scan_points_->add_key_value("sensor_points_max_distance", max_distance);
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3) << max_distance << " [m] < "
            << param_.sensor_points.required_distance << " [m]";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
    return false;
  }

  // 検査: is_activated（移植していないので常に有効）
  diagnostics_scan_points_->add_key_value("is_activated", true);

  // 初期値（スキャン時刻での EKF 姿勢）を求める
  const std::optional<localization_util::SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    initial_pose_buffer_->interpolate(sensor_ros_time);

  // 検査: is_succeed_interpolate_initial_pose（初期値を補間できたか）
  const bool is_succeed_interpolate_initial_pose = interpolation_result_opt != std::nullopt;
  diagnostics_scan_points_->add_key_value("is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose);
  if (!is_succeed_interpolate_initial_pose) {
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN,
      "Couldn't interpolate pose. Please verify that (1) the initial pose topic (primarily come from the EKF) is "
      "being published, and (2) the timestamps of the sensor PCD messages and pose messages are synchronized "
      "correctly.");
    return false;
  }
  initial_pose_buffer_->pop_old(sensor_ros_time);
  const localization_util::SmartPoseBuffer::InterpolateResult & interpolation_result = interpolation_result_opt.value();

  // 検査: is_set_map_points（地図が登録済みか）
  const bool is_set_map_points = ndt_ptr_->hasTarget();
  diagnostics_scan_points_->add_key_value("is_set_map_points", is_set_map_points);
  if (!is_set_map_points) {
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, "Map points is not set.");
    return false;
  }

  // NDT で照合する
  const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame);
  const pclomp::NdtResult ndt_result = ndt_ptr_->getResult();

  const geometry_msgs::msg::Pose result_pose_msg = matrix4f_to_pose(ndt_result.pose);
  std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
  for (const auto & pose_matrix : ndt_result.transformation_array) {
    transformation_msg_array.push_back(matrix4f_to_pose(pose_matrix));
  }

  // 検査: iteration_num（反復回数が上限に達していないか）
  diagnostics_scan_points_->add_key_value("iteration_num", ndt_result.iteration_num);
  const bool is_ok_iteration_num = ndt_result.iteration_num < ndt_ptr_->getMaximumIterations();
  if (!is_ok_iteration_num) {
    std::stringstream message;
    message << "The number of iterations has reached its upper limit. The number of iterations: "
            << ndt_result.iteration_num << ", Limit: " << ndt_ptr_->getMaximumIterations() << ".";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
  }

  // 検査: local_optimal_solution_oscillation_num（局所解で振動していないか）
  constexpr int oscillation_num_threshold = 10;
  const int oscillation_num = count_oscillation(transformation_msg_array);
  diagnostics_scan_points_->add_key_value("local_optimal_solution_oscillation_num", oscillation_num);
  const bool is_local_optimal_solution_oscillation = oscillation_num > oscillation_num_threshold;
  if (is_local_optimal_solution_oscillation) {
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN,
      "There is a possibility of oscillation in a local minimum");
  }

  // 検査: score（TP または NVTL が閾値を超えているか）
  diagnostics_scan_points_->add_key_value("transform_probability", ndt_result.transform_probability);
  diagnostics_scan_points_->add_key_value(
    "nearest_voxel_transformation_likelihood", ndt_result.nearest_voxel_transformation_likelihood);
  double score = 0.0;
  double score_threshold = 0.0;
  if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
    score = ndt_result.transform_probability;
    score_threshold = param_.score_estimation.converged_param_transform_probability;
  } else if (
    param_.score_estimation.converged_param_type ==
    ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD)
  {
    score = ndt_result.nearest_voxel_transformation_likelihood;
    score_threshold = param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
  } else {
    std::stringstream message;
    message << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::ERROR, message.str());
    return false;
  }
  const bool is_ok_score = score > score_threshold;
  if (!is_ok_score) {
    std::stringstream message;
    message << "Score is below the threshold. Score: " << score << ", Threshold: " << score_threshold;
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
  }

  // 検査: is_converged（収束の判定）
  const bool is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;
  diagnostics_scan_points_->add_key_value("is_converged", is_converged);

  // 共分散: FIXED_VALUE のみ（base_link 基準の固定値を map 基準に回転する）
  const Eigen::Quaterniond map_to_base_link_quat(result_pose_msg.orientation.w, result_pose_msg.orientation.x,
    result_pose_msg.orientation.y, result_pose_msg.orientation.z);
  const Eigen::Matrix3d map_to_base_link_rotation = map_to_base_link_quat.normalized().toRotationMatrix();
  const std::array<double, 36> ndt_covariance =
    rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);

  // 検査: distance_initial_to_result（初期値からの移動量。警告のみ）
  const double distance_initial_to_result =
    norm(interpolation_result.interpolated_pose.pose.pose.position, result_pose_msg.position);
  diagnostics_scan_points_->add_key_value("distance_initial_to_result", distance_initial_to_result);
  if (distance_initial_to_result > param_.validation.initial_to_result_distance_tolerance_m) {
    std::stringstream message;
    message << "distance_initial_to_result is too large (" << distance_initial_to_result << " [m]).";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
  }

  // 検査: execution_time（処理時間。警告のみ）
  const auto exe_end_time = std::chrono::system_clock::now();
  const auto duration_micro_sec =
    std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
  const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
  diagnostics_scan_points_->add_key_value("execution_time", exe_time);
  if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
    std::stringstream message;
    message << "NDT exe time is too long (took " << exe_time << " [ms]).";
    diagnostics_scan_points_->update_level_and_message(DiagnosticStatus::WARN, message.str());
  }

  // 配信
  publish_pose(sensor_ros_time, result_pose_msg, ndt_covariance, is_converged);

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(*sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
  publish_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

  return is_converged;
}

// 参照: NDTScanMatcher::transform_sensor_measurement
void NDTScanMatcher::transform_sensor_measurement(const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }
  const geometry_msgs::msg::TransformStamped transform =
    tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);

  // autoware_utils_geometry::transform2pose と同じ変換
  geometry_msgs::msg::Pose target_to_source_pose;
  target_to_source_pose.position.x = transform.transform.translation.x;
  target_to_source_pose.position.y = transform.transform.translation.y;
  target_to_source_pose.position.z = transform.transform.translation.z;
  target_to_source_pose.orientation = transform.transform.rotation;
  const Eigen::Matrix4f base_to_sensor_matrix = pose_to_matrix4f(target_to_source_pose);
  pcl::transformPointCloud(*sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

// 参照: NDTScanMatcher::publish_pose
void NDTScanMatcher::publish_pose(const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance, const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

// 参照: NDTScanMatcher::publish_point_cloud
void NDTScanMatcher::publish_point_cloud(const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_msg_in_map;
  pcl::toROSMsg(*sensor_points_in_map_ptr, sensor_points_msg_in_map);
  sensor_points_msg_in_map.header.stamp = sensor_ros_time;
  sensor_points_msg_in_map.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_msg_in_map);
}
}  // namespace ndt_localization::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_localization::ndt_scan_matcher::NDTScanMatcher)

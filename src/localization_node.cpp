#include "wheel_lidar_localization/localization_node.hpp"
#include <pcl/common/transforms.h>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace wheel_lidar_localization
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
}
std::optional<Pose> fromPose(const geometry_msgs::msg::Pose & p)
{
  Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
  if (!q.coeffs().allFinite() || q.norm() < 1e-6) {return std::nullopt;}
  Pose result = Pose::Identity();
  result.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  result.linear() = q.normalized().toRotationMatrix();
  return validPose(result) ? std::optional<Pose>(result) : std::nullopt;
}
geometry_msgs::msg::Pose toPose(const Pose & p)
{
  geometry_msgs::msg::Pose result;
  result.position.x = p.translation().x(); result.position.y = p.translation().y();
  result.position.z = p.translation().z();
  Eigen::Quaterniond q(p.linear()); q.normalize();
  result.orientation.x = q.x(); result.orientation.y = q.y();
  result.orientation.z = q.z(); result.orientation.w = q.w();
  return result;
}
// Parse XYZ explicitly: supports padded organized clouds and FLOAT32/FLOAT64,
// and rejects malformed buffers before reading. Intensity/time are not required.
Cloud::Ptr readScan(const sensor_msgs::msg::PointCloud2 & msg, double min_range, double max_range)
{
  const uint16_t endian_probe = 1;
  const bool host_big_endian = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
  if (msg.is_bigendian != host_big_endian) {throw std::runtime_error("unsupported_cloud_endianness");}
  if (msg.point_step == 0 || msg.width == 0 || msg.height == 0 ||
    static_cast<uint64_t>(msg.row_step) < static_cast<uint64_t>(msg.width) * msg.point_step ||
    static_cast<uint64_t>(msg.row_step) * msg.height > msg.data.size())
  {throw std::runtime_error("malformed_or_empty_cloud");}
  std::array<sensor_msgs::msg::PointField, 3> fields;
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (std::size_t axis = 0; axis < 3; ++axis) {
    auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
      [&](const auto & field) {return field.name == names[axis];});
    if (it == msg.fields.end() || it->count != 1 ||
      (it->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
      it->datatype != sensor_msgs::msg::PointField::FLOAT64))
    {throw std::runtime_error("missing_or_unsupported_xyz_field");}
    const uint64_t size = it->datatype == sensor_msgs::msg::PointField::FLOAT32 ? 4 : 8;
    if (static_cast<uint64_t>(it->offset) + size > msg.point_step) {
      throw std::runtime_error("invalid_xyz_field_offset");
    }
    fields[axis] = *it;
  }
  auto cloud = std::make_shared<Cloud>();
  cloud->reserve(static_cast<std::size_t>(msg.width) * msg.height);
  for (uint32_t row = 0; row < msg.height; ++row) {
    for (uint32_t col = 0; col < msg.width; ++col) {
      const auto * data = msg.data.data() + static_cast<std::size_t>(row) * msg.row_step +
        static_cast<std::size_t>(col) * msg.point_step;
      Eigen::Vector3d xyz;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        if (fields[axis].datatype == sensor_msgs::msg::PointField::FLOAT32) {
          float value; std::memcpy(&value, data + fields[axis].offset, sizeof(value)); xyz[axis] = value;
        } else {double value; std::memcpy(&value, data + fields[axis].offset, sizeof(value)); xyz[axis] = value;}
      }
      const double range_squared = xyz.squaredNorm();
      if (xyz.allFinite() && range_squared >= min_range * min_range && range_squared <= max_range * max_range) {
        cloud->push_back(pcl::PointXYZ(xyz.x(), xyz.y(), xyz.z()));
      }
    }
  }
  return cloud;
}
}  // namespace

double LocalizationNode::positiveParameter(const std::string & name, double value)
{
  const double result = declare_parameter<double>(name, value);
  if (!std::isfinite(result) || result <= 0) {throw std::invalid_argument(name + " must be finite and > 0");}
  return result;
}
LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("wheel_lidar_localization", options)
{
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  if (map_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
    map_frame_ == odom_frame_ || map_frame_ == base_frame_ || odom_frame_ == base_frame_)
  {throw std::invalid_argument("map_frame, odom_frame and base_frame must be nonempty and distinct");}
  const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/surestar_points");
  const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  const auto initial_topic = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  const auto map_topic = declare_parameter<std::string>("map_topic", "/localization/map");
  const auto pose_topic = declare_parameter<std::string>("pose_topic", "/localization/pose");
  const auto status_topic = declare_parameter<std::string>("status_topic", "/localization/status");
  const auto map_path = declare_parameter<std::string>("map_path", "");
  const double map_voxel = positiveParameter("map_voxel_size", 0.2);
  const double display_voxel = positiveParameter("map_display_voxel_size", 0.4);
  map_radius_ = positiveParameter("local_map_radius", 65.0);
  map_recenter_distance_ = positiveParameter("local_map_recenter_distance", 5.0);
  scan_voxel_ = positiveParameter("scan_voxel_size", 0.3);
  scan_min_range_ = positiveParameter("scan_min_range", 1.0);
  scan_max_range_ = positiveParameter("scan_max_range", 50.0);
  if (scan_max_range_ <= scan_min_range_ || map_radius_ <= scan_max_range_ + map_recenter_distance_) {
    throw std::invalid_argument("Require min_range < max_range and local_map_radius > max_range + recenter_distance");
  }
  min_scan_points_ = declare_parameter<int>("min_scan_points", 100);
  min_map_points_ = declare_parameter<int>("min_map_points", 500);
  const int max_iterations = declare_parameter<int>("ndt_max_iterations", 40);
  const int failures_to_lost = declare_parameter<int>("failures_to_lost", 10);
  if (min_scan_points_ < 6 || min_map_points_ < 6 || max_iterations < 2 || failures_to_lost < 1) {
    throw std::invalid_argument("Invalid point count, iteration count, or failure count");
  }
  const double resolution = positiveParameter("ndt_resolution", 1.0);
  const double step = positiveParameter("ndt_step_size", 0.1);
  const double epsilon = positiveParameter("ndt_transformation_epsilon", 0.01);
  const double distance = positiveParameter("correspondence_distance", 1.0);
  tracking_limits_ = {positiveParameter("max_rmse", 0.5), positiveParameter("min_inlier_ratio", 0.5),
    positiveParameter("max_translation_correction", 1.5), positiveParameter("max_rotation_correction", 0.5)};
  if (tracking_limits_.min_inlier_ratio > 1) {throw std::invalid_argument("min_inlier_ratio must be <= 1");}
  initial_limits_ = tracking_limits_;
  initial_limits_.max_translation = positiveParameter("initial_max_translation_correction", 5.0);
  initial_limits_.max_rotation = positiveParameter("initial_max_rotation_correction", 1.0);
  const double duration = positiveParameter("odom_buffer_duration", 10.0);
  const double gap = positiveParameter("odom_max_interpolation_gap", 0.2);
  const double reset_time = positiveParameter("odom_reset_time_jump", 1.0);
  const double max_speed = positiveParameter("odom_max_speed", 5.0);
  const double max_rotation_speed = positiveParameter("odom_max_rotation_speed", 5.0);
  odom_wait_ = positiveParameter("odom_wait_timeout", 0.3);
  transform_wait_ = positiveParameter("transform_wait_timeout", 0.1);
  max_scan_age_ = positiveParameter("max_scan_age", 2.0);
  stale_timeout_ = positiveParameter("stale_timeout", 3.0);
  const double tf_frequency = positiveParameter("tf_publish_frequency", 20.0);
  tf_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  scan_time_offset_ = declare_parameter<double>("scan_time_offset", 0.0);
  initial_z_ = declare_parameter<double>("initial_z", 0.0);
  initial_roll_ = declare_parameter<double>("initial_roll", 0.0);
  initial_pitch_ = declare_parameter<double>("initial_pitch", 0.0);
  initial_use_2d_ = declare_parameter<bool>("initial_pose_use_2d", true);
  // Startup pose; /initialpose (RViz 2D Pose Estimate) can still override it.
  const double initial_x = declare_parameter<double>("initial_x", 0.0);
  const double initial_y = declare_parameter<double>("initial_y", 0.0);
  const double initial_yaw = declare_parameter<double>("initial_yaw", 0.0);
  position_stddev_ = positiveParameter("pose_position_stddev", 0.3);
  angle_stddev_ = positiveParameter("pose_angle_stddev", 0.1);
  for (double v : {tf_tolerance_, scan_time_offset_, initial_z_, initial_roll_, initial_pitch_,
      initial_x, initial_y, initial_yaw})
  {
    if (!std::isfinite(v)) {throw std::invalid_argument("Non-finite time offset or initial pose setting");}
  }
  if (tf_tolerance_ < 0) {throw std::invalid_argument("transform_tolerance must be >= 0");}
  odom_ = std::make_unique<OdometryBuffer>(duration, gap, reset_time, max_speed, max_rotation_speed);
  state_ = std::make_unique<LocalizationState>(failures_to_lost);
  matcher_ = std::make_unique<ScanMatcher>(resolution, step, epsilon, max_iterations, distance);
  RCLCPP_INFO(get_logger(), "Loading PCD map: %s", map_path.c_str());
  map_ = std::make_unique<MapManager>(map_path, map_voxel, min_map_points_);
  RCLCPP_INFO(get_logger(), "Map ready: %zu raw -> %zu filtered points", map_->rawPoints(), map_->cloud()->size());
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, rclcpp::QoS(1).reliable().transient_local());
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, rclcpp::QoS(10));
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic, rclcpp::QoS(1).reliable().transient_local());
  sensor_msgs::msg::PointCloud2 map_msg;
  pcl::toROSMsg(*voxelFilter(map_->cloud(), display_voxel), map_msg);
  map_msg.header.frame_id = map_frame_;
  // A fixed map is valid at all times, including before /clock starts.
  map_msg.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  map_pub_->publish(map_msg);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SensorDataQoS().keep_last(200),
    std::bind(&LocalizationNode::odometryCallback, this, std::placeholders::_1));
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&LocalizationNode::cloudCallback, this, std::placeholders::_1));
  initial_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(initial_topic, rclcpp::QoS(10),
    std::bind(&LocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / tf_frequency),
    std::bind(&LocalizationNode::publishStatusAndTf, this));
  RCLCPP_INFO(get_logger(), "Cloud: %s; odometry: %s; frames: %s -> %s -> %s",
    cloud_topic.c_str(), odom_topic.c_str(), map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "Map published on %s. Initial pose can be overridden via %s.",
    map_topic.c_str(), initial_topic.c_str());
  Pose pose = Pose::Identity();
  pose.translation() = Eigen::Vector3d(initial_x, initial_y, initial_z_);
  pose.linear() = (Eigen::AngleAxisd(initial_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(initial_pitch_, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(initial_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
  applyInitialPose(pose, 0.0);
  worker_ = std::thread(&LocalizationNode::workerLoop, this);
}
LocalizationNode::~LocalizationNode()
{
  {std::lock_guard<std::mutex> lock(mutex_); stopping_ = true;}
  cv_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}
void LocalizationNode::resetForDiscontinuity(const std::string & reason)
{
  state_->invalidate();
  ++generation_;
  pending_scan_.reset();
  last_scan_stamp_ = -1;
  reason_ = reason;
  RCLCPP_ERROR(get_logger(), "%s; send a new initial pose", reason.c_str());
}
void LocalizationNode::odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->header.frame_id != odom_frame_ || msg->child_frame_id != base_frame_) {
    ++rejected_odometry_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "Odometry must describe %s -> %s (received %s -> %s)", odom_frame_.c_str(), base_frame_.c_str(),
      msg->header.frame_id.c_str(), msg->child_frame_id.c_str());
    return;
  }
  auto pose = fromPose(msg->pose.pose);
  if (!pose) {++rejected_odometry_; return;}
  const auto admission = odom_->add(seconds(msg->header.stamp), *pose);
  if (admission == OdomAdmission::Reset) {resetForDiscontinuity("odometry_discontinuity");}
  if (admission == OdomAdmission::Invalid || admission == OdomAdmission::OutOfOrder) {
    ++rejected_odometry_; return;
  }
  last_odom_arrival_ = now().seconds();
  cv_.notify_all();
}
void LocalizationNode::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_cloud_arrival_ = now().seconds();
  if (state_->state() == TrackingState::WaitingInitialPose || state_->state() == TrackingState::Lost) {return;}
  if (pending_scan_) {++dropped_scans_;}
  pending_scan_ = msg;
  cv_.notify_all();
}
void LocalizationNode::initialPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  if (msg->header.frame_id != map_frame_) {
    RCLCPP_ERROR(get_logger(), "Initial pose frame must be '%s'", map_frame_.c_str()); return;
  }
  auto pose = fromPose(msg->pose.pose);
  if (!pose) {RCLCPP_ERROR(get_logger(), "Invalid initial pose position or quaternion"); return;}
  if (initial_use_2d_) {
    const double yaw = std::atan2(pose->linear()(1, 0), pose->linear()(0, 0));
    pose->translation().z() = initial_z_;
    pose->linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(initial_pitch_, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(initial_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  applyInitialPose(*pose, seconds(msg->header.stamp));
}
void LocalizationNode::applyInitialPose(const Pose & pose, double request_stamp)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    state_->initialize(pose); ++generation_; pending_scan_.reset();
    initial_request_stamp_ = request_stamp; last_scan_stamp_ = -1;
    reason_ = "waiting_scan_and_odometry";
  }
  cv_.notify_all();
  RCLCPP_INFO(get_logger(), "Initial pose accepted: [%.3f, %.3f, %.3f]; waiting for first valid alignment",
    pose.translation().x(), pose.translation().y(), pose.translation().z());
}
void LocalizationNode::workerLoop()
{
  while (true) {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan;
    std::uint64_t generation;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [&] {return stopping_ || pending_scan_;});
      if (stopping_) {return;}
      scan = std::move(pending_scan_); generation = generation_;
    }
    const auto started = std::chrono::steady_clock::now();
    try {processScan(scan, generation);} catch (const std::exception & e) {rejectScan(generation, e.what());}
    std::lock_guard<std::mutex> lock(mutex_);
    last_processing_ms_ = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
  }
}
void LocalizationNode::rejectScan(std::uint64_t generation, const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_) {return;}
  reason_ = reason; state_->reject();
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s: %s", stateName(state_->state()), reason.c_str());
}
void LocalizationNode::processScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  std::uint64_t generation)
{
  const double stamp = seconds(msg->header.stamp) + scan_time_offset_;
  Pose odom, seed;
  MatchLimits limits;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (generation != generation_ || state_->state() == TrackingState::Lost) {return;}
    const double age = now().seconds() - stamp;
    if (stamp < 0 || stamp <= last_scan_stamp_ || stamp < initial_request_stamp_) {++dropped_scans_; return;}
    if (age > max_scan_age_ || age < -max_scan_age_) {
      lock.unlock(); rejectScan(generation, "scan_clock_mismatch_or_stale"); return;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(odom_wait_);
    cv_.wait_until(lock, deadline, [&] {return stopping_ || generation != generation_ || odom_->at(stamp).has_value();});
    if (stopping_ || generation != generation_) {return;}
    auto odom_pose = odom_->at(stamp);
    if (!odom_pose) {lock.unlock(); rejectScan(generation, "no_odometry_bracketing_scan"); return;}
    odom = *odom_pose; seed = state_->seed(odom);
    limits = state_->hasCorrection() ? tracking_limits_ : initial_limits_;
    last_scan_stamp_ = stamp;
  }
  auto cloud = readScan(*msg, scan_min_range_, scan_max_range_);
  if (msg->header.frame_id.empty()) {rejectScan(generation, "empty_cloud_frame"); return;}
  if (msg->header.frame_id != base_frame_) {
    const auto transform = tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
      rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME),
      rclcpp::Duration::from_seconds(transform_wait_));
    geometry_msgs::msg::Pose extrinsic;
    extrinsic.position.x = transform.transform.translation.x;
    extrinsic.position.y = transform.transform.translation.y;
    extrinsic.position.z = transform.transform.translation.z;
    extrinsic.orientation = transform.transform.rotation;
    const auto pose = fromPose(extrinsic);
    if (!pose) {rejectScan(generation, "invalid_lidar_transform"); return;}
    pcl::transformPointCloud(*cloud, *cloud, pose->matrix().cast<float>());
  }
  cloud = voxelFilter(cloud, scan_voxel_);
  if (cloud->size() < static_cast<std::size_t>(min_scan_points_)) {rejectScan(generation, "too_few_scan_points"); return;}
  if (target_generation_ != generation || (seed.translation() - target_center_).norm() > map_recenter_distance_) {
    auto target = map_->crop(seed.translation(), map_radius_);
    if (target->size() < static_cast<std::size_t>(min_map_points_)) {rejectScan(generation, "too_few_local_map_points"); return;}
    matcher_->setTarget(target); target_center_ = seed.translation(); target_generation_ = generation;
  }
  const auto result = matcher_->align(cloud, seed);
  const auto rejection = rejectionReason(result, limits);
  std::lock_guard<std::mutex> lock(mutex_);
  if (generation != generation_ || stopping_ || state_->state() == TrackingState::Lost) {return;}
  last_match_ = result;
  if (now().seconds() - stamp > max_scan_age_) {
    state_->reject(); reason_ = "alignment_result_too_old"; return;
  }
  if (!rejection.empty()) {
    state_->reject(); reason_ = rejection;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s: %s, rmse=%.3f, inliers=%.2f",
      stateName(state_->state()), reason_.c_str(), result.rmse, result.inlier_ratio); return;
  }
  const bool first_alignment = !state_->hasCorrection();
  state_->accept(result.pose, odom, stamp); reason_ = "alignment_accepted";
  geometry_msgs::msg::PoseWithCovarianceStamped output;
  output.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
  output.header.frame_id = map_frame_; output.pose.pose = toPose(result.pose);
  // Configured uncertainty, not an NDT covariance estimate; documented in README.
  for (int i = 0; i < 3; ++i) {
    output.pose.covariance[i * 6 + i] = position_stddev_ * position_stddev_;
    output.pose.covariance[(i + 3) * 6 + i + 3] = angle_stddev_ * angle_stddev_;
  }
  pose_pub_->publish(output);
  if (first_alignment) {RCLCPP_INFO(get_logger(), "Tracking started: rmse=%.3f, inliers=%.2f", result.rmse, result.inlier_ratio);}
}
void LocalizationNode::publishStatusAndTf()
{
  const auto time = now(); const double clock = time.seconds();
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_clock_ >= 0 && clock < last_clock_ - 1e-6) {
    odom_->clear(); resetForDiscontinuity("ros_clock_moved_backwards");
    last_odom_arrival_ = last_cloud_arrival_ = -1;
  }
  last_clock_ = clock;
  if (state_->hasCorrection() && state_->state() != TrackingState::Lost &&
    clock - state_->acceptedStamp() > stale_timeout_)
  {state_->markLost(); reason_ = "no_recent_accepted_alignment"; ++generation_; pending_scan_.reset();}
  if (state_->hasCorrection()) {
    // Re-publish a held map->odom correction, never re-label a stale map->base pose.
    const auto pose = toPose(state_->correction());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = time + rclcpp::Duration::from_seconds(tf_tolerance_);
    transform.header.frame_id = map_frame_; transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
  diagnostic_msgs::msg::DiagnosticArray array; array.header.stamp = time;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "wheel_lidar_localization"; status.hardware_id = "wheel_lidar";
  status.level = state_->state() == TrackingState::Tracking ? 0 : (state_->state() == TrackingState::Lost ? 2 : 1);
  status.message = std::string(stateName(state_->state())) + ": " + reason_;
  const auto add = [&](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue item; item.key = key; item.value = value; status.values.push_back(item);
    };
  add("state", stateName(state_->state())); add("reason", reason_);
  add("map_points", std::to_string(map_->cloud()->size()));
  add("rmse_m", std::to_string(last_match_.rmse)); add("inlier_ratio", std::to_string(last_match_.inlier_ratio));
  add("ndt_iterations", std::to_string(last_match_.iterations));
  add("processing_ms", std::to_string(last_processing_ms_));
  add("dropped_scans", std::to_string(dropped_scans_));
  add("rejected_odometry", std::to_string(rejected_odometry_));
  add("consecutive_failures", std::to_string(state_->failures()));
  add("seconds_since_alignment", state_->hasCorrection() ? std::to_string(clock - state_->acceptedStamp()) : "none");
  add("seconds_since_odometry", last_odom_arrival_ < 0 ? "none" : std::to_string(clock - last_odom_arrival_));
  add("seconds_since_cloud", last_cloud_arrival_ < 0 ? "none" : std::to_string(clock - last_cloud_arrival_));
  add("covariance_model", "configured_diagonal_not_estimated");
  array.status.push_back(status); status_pub_->publish(array);
}
}  // namespace wheel_lidar_localization

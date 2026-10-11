#pragma once
#include "wheel_lidar_localization/localization_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace wheel_lidar_localization
{
class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LocalizationNode() override;
private:
  void odometryCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void initialPoseCallback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
  void applyInitialPose(const Pose & pose, double request_stamp);
  void workerLoop();
  void processScan(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    std::uint64_t generation);
  void rejectScan(std::uint64_t generation, const std::string & reason);
  void publishStatusAndTf();
  double positiveParameter(const std::string & name, double value);
  void resetForDiscontinuity(const std::string & reason);

  std::string map_frame_, odom_frame_, base_frame_;
  std::unique_ptr<MapManager> map_;
  std::unique_ptr<ScanMatcher> matcher_;
  std::unique_ptr<OdometryBuffer> odom_;
  std::unique_ptr<LocalizationState> state_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopping_{false};
  std::uint64_t generation_{0}, dropped_scans_{0}, rejected_odometry_{0};
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_scan_;
  std::string reason_{"waiting_initial_pose"};
  MatchResult last_match_;
  double last_processing_ms_{0};
  double last_scan_stamp_{-1}, initial_request_stamp_{0};
  double last_clock_{-1}, last_odom_arrival_{-1}, last_cloud_arrival_{-1};
  double map_radius_, map_recenter_distance_, scan_voxel_, scan_min_range_, scan_max_range_;
  double odom_wait_, transform_wait_, scan_time_offset_, max_scan_age_, stale_timeout_;
  double tf_tolerance_, initial_z_, initial_roll_, initial_pitch_;
  double position_stddev_, angle_stddev_;
  bool initial_use_2d_;
  int min_scan_points_, min_map_points_;
  MatchLimits tracking_limits_, initial_limits_;
  // Worker-only cache, replaced whenever the initial pose / odom epoch changes.
  std::uint64_t target_generation_{std::numeric_limits<std::uint64_t>::max()};
  Eigen::Vector3d target_center_{Eigen::Vector3d::Zero()};
};
}  // namespace wheel_lidar_localization

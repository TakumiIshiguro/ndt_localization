// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ndt_scan_matcher/include/autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp
//
//   points_raw（センサ座標系）─▶ 前処理 ─▶ NDT（multigrid ndt_omp）─▶ ndt_pose(_with_covariance)
//   ekf_odom（map → base_link）─▶ SmartPoseBuffer ─▶ スキャン時刻の初期値 ─┘
//
// 移植したもの: callback_initial_pose(_main)、callback_sensor_points(_main)、
// transform_sensor_measurement、publish_pose、publish_point_cloud、診断
// （scan_matching_status / initial_pose_subscriber_status）。名前と採否判定は Autoware と同じ。
// 違い:
//  - 初期値の入力が PoseWithCovarianceStamped の "ekf_pose_with_covariance" ではなく、
//    nav_msgs/Odometry の "ekf_odom"（robot_localization の出力）。
//  - map_update_module / pcd_loader クライアントがない。PCD を 1 つコンストラクタで読み
//    （map_loader/pointcloud_map_loader）、debug/loaded_pointcloud_map に配信する。
//  - 点群の間引きをノード内で行う（pointcloud_preprocessor）。
//  - 初期値は nav_msgs/Odometry で受け取る（robot_localization が PoseWithCovarianceStamped を出さないため）。
//  - 移植していないもの: trigger_node / ndt_align サービス（常に有効扱い）、正則化、
//    共分散推定（FIXED_VALUE のみ）、publish_tf / publish_marker とデバッグ用トピック。
#pragma once
#include "ndt_localization/localization_util/smart_pose_buffer.hpp"
#include "ndt_localization/ndt_scan_matcher/hyper_parameters.hpp"
#include "ndt_localization/ndt_scan_matcher/ndt_omp/multigrid_ndt_omp.h"
#include "ndt_localization/utils_diagnostics/diagnostics_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <array>
#include <memory>
#include <string>

namespace ndt_localization::ndt_scan_matcher
{
using DiagnosticsInterface = utils_diagnostics::DiagnosticsInterface;

/**
 * @brief 固定の 3D 点群地図に対して LiDAR 点群を NDT で照合し、map 座標系での姿勢を出すノード。
 *
 * 購読: points_raw（PointCloud2、センサ座標系）、ekf_odom（Odometry、map → base_link）
 * 配信: ndt_pose（PoseStamped）、ndt_pose_with_covariance（PoseWithCovarianceStamped）、
 *       points_aligned（照合後の点群）、debug/loaded_pointcloud_map（表示用の地図、Transient Local）、
 *       /diagnostics
 */
class NDTScanMatcher : public rclcpp::Node
{
  using PointSource = pcl::PointXYZ;
  using PointTarget = pcl::PointXYZ;
  using NormalDistributionsTransform = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;

public:
  /**
   * @brief パラメータを読み、地図を NDT に登録し、購読・配信・診断を作る。
   * @details Autoware との違い: MapUpdateModule / pcd_loader クライアントの代わりにここで PCD を読む。
   *          タイマー、正則化用の購読、サービスはない。
   *          購読はコールバックグループを 2 つに分け（EKF 姿勢 / 点群）、
   *          照合中でも EKF 姿勢の受信が止まらないようにしている。
   * @param[in] options ノードのオプション（コンポーネントとして読み込むときに渡される）。
   * @throws std::runtime_error 地図が読めない場合。
   * @throws rclcpp の例外 パラメータが欠けている場合。
   */
  explicit NDTScanMatcher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief EKF 姿勢のコールバック。診断を初期化し、本体を呼び、診断を配信する。
   * @param[in] initial_pose_msg_ptr robot_localization の出力（map → base_link）。
   * 出力: /diagnostics の initial_pose_subscriber_status。
   */
  void callback_initial_pose(const nav_msgs::msg::Odometry::ConstSharedPtr initial_pose_msg_ptr);

  /**
   * @brief EKF 姿勢を検証し、問題なければ SmartPoseBuffer に追加する。
   * @details Autoware との違い: Odometry を受け取る、有効化の確認がない、
   *          map_update_module 用の latest_ekf_position_ を持たない。
   * @param[in] initial_pose_msg_ptr robot_localization の出力。header.frame_id が map_frame であること。
   * 出力: initial_pose_buffer_ への追加。不正なら診断を ERROR にして何もしない。
   */
  void callback_initial_pose_main(const nav_msgs::msg::Odometry::ConstSharedPtr initial_pose_msg_ptr);

  /**
   * @brief 点群のコールバック。照合を行い、失敗の連続回数を数えて診断を配信する。
   * @param[in] sensor_points_msg_in_sensor_frame LiDAR 点群（センサ座標系）。
   * 出力: /diagnostics の scan_matching_status。失敗が skipping_publish_num 回続くと WARN。
   */
  void callback_sensor_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);

  /**
   * @brief 点群 1 回分の処理本体。前処理、初期値の補間、NDT 照合、採否判定、配信を行う。
   * @details Autoware と同じ順で検査する:
   *          点数 → 遅延（警告のみ）→ 間引き（本パッケージ独自）→ base_link への変換 → 最大距離 → 初期値の補間 → 地図の有無 → 照合 → 反復回数・振動・スコア →
   *          共分散 → 初期値からの移動量（警告のみ）→ 処理時間（警告のみ）→ 配信。
   *          収束の条件は「(反復回数 < 上限 または 局所解で振動) かつ スコア > 閾値」。
   * @param[in] sensor_points_msg_in_sensor_frame LiDAR 点群（センサ座標系）。
   * @return 収束して姿勢を配信したら true。途中で中断した場合や収束しなかった場合は false。
   * 出力: 収束時は ndt_pose / ndt_pose_with_covariance、常に points_aligned、
   *       各検査の結果を diagnostics_scan_points_ に記録する。
   */
  bool callback_sensor_points_main(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame);

  /**
   * @brief 点群を TF で別の座標系に変換する（センサ → base_link）。
   * @details Autoware と同じく、LiDAR の取り付けは固定とみなして最新の TF を使う。
   * @param[in] source_frame 入力点群の座標系。
   * @param[in] target_frame 変換先の座標系。
   * @param[in] sensor_points_input_ptr 入力点群。
   * @param[out] sensor_points_output_ptr 変換後の点群。座標系が同じなら入力をそのまま指す。
   * @throws tf2::TransformException TF が取得できない場合。
   */
  void transform_sensor_measurement(const std::string & source_frame, const std::string & target_frame,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
    pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr);

  /**
   * @brief NDT の結果の姿勢を配信する（収束したときだけ）。
   * @param[in] sensor_ros_time 点群の時刻。出力の stamp になる。
   * @param[in] result_pose_msg NDT の結果（map → base_link）。
   * @param[in] ndt_covariance 出力する共分散（map 座標系）。
   * @param[in] is_converged false なら何も配信しない。
   * 出力: ndt_pose と ndt_pose_with_covariance（frame_id は map_frame）。
   */
  void publish_pose(const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
    const std::array<double, 36> & ndt_covariance, const bool is_converged);

  /**
   * @brief 照合後の点群を points_aligned に配信する（RViz での確認用）。
   * @param[in] sensor_ros_time 点群の時刻。
   * @param[in] frame_id 点群の座標系（map_frame）。
   * @param[in] sensor_points_in_map_ptr NDT の結果で map 座標系に変換した点群。
   */
  void publish_point_cloud(const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
    const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr ndt_pose_with_covariance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loaded_pcd_pub_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // Autoware は地図更新に備えて Guarded<> で保護している。本パッケージでは地図の登録は
  // コンストラクタの 1 回だけで、以後は点群のコールバックグループからしか使わないので、ロックは不要。
  std::unique_ptr<NormalDistributionsTransform> ndt_ptr_;
  std::unique_ptr<localization_util::SmartPoseBuffer> initial_pose_buffer_;

  std::unique_ptr<DiagnosticsInterface> diagnostics_scan_points_;
  std::unique_ptr<DiagnosticsInterface> diagnostics_initial_pose_;

  HyperParameters param_;
};
}  // namespace ndt_localization::ndt_scan_matcher

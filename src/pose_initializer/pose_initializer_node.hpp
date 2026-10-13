// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_pose_initializer/src/pose_initializer_node.hpp
// 簡易版。Autoware の pose_initializer のうち、user_defined_initial_pose（起動時の姿勢）と、
// EKF の trigger_node を止める -> pose_reset を配信する -> 動かす手順（set_user_defined_initial_pose）だけを移植した。
// (本パッケージ独自) RViz の /initialpose（"initialpose"）も同じ手順で扱う
// （Universe の initial_pose_adaptor の役割。map_height_fitter による z の補正はしない）。
// 移植していないもの: AD API の初期化サービスと状態の配信、GNSS、NDT の ndt_align と trigger_node、停止の確認、
// 誤差の確認。サービスは同期で待たず、応答のコールバックで次の手順に進む。
#pragma once
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <array>
#include <functional>
#include <optional>

namespace ndt_localization::pose_initializer
{
/**
 * @brief 初期姿勢で EKF をリセットして動かすノード。
 *        入力: "initialpose"（RViz の 2D Pose Estimate）、パラメータ user_defined_initial_pose.{enable,pose}。
 *        出力: "pose_reset"（EKF の initialpose へ）、サービス "ekf_trigger_node"（EKF の trigger_node へ）。
 */
class PoseInitializerNode : public rclcpp::Node
{
public:
  explicit PoseInitializerNode(const rclcpp::NodeOptions & options);

private:
  /// RViz の姿勢を受け取って初期化する（共分散は output_pose_covariance に置き換える）。
  void callback_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);

  /**
   * @brief EKF を止める -> pose_reset を配信する -> EKF を動かす。
   *        Ref: PoseInitializerNode::set_user_defined_initial_pose。処理中に呼ばれたら最後の姿勢だけを覚えておき、
   *        終わってから続けて処理する。
   * @param[in] initial_pose map 座標系の姿勢。
   */
  void set_user_defined_initial_pose(const geometry_msgs::msg::Pose & initial_pose);

  /**
   * @brief EKF の trigger_node を非同期で呼ぶ。Ref: LocalizationTriggerModule::send_request。
   * @param[in] flag true で動かす、false で止める。
   * @param[in] on_done 応答を受け取ったら success を渡して呼ぶ。
   */
  void send_ekf_trigger(bool flag, std::function<void(bool)> on_done);

  /// 1 回分の初期化が終わった（または失敗した）ときに呼び、待っている姿勢があれば続けて処理する。
  void finish_initialization();

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_reset_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_ekf_trigger_;
  /// 起動時の姿勢を、EKF のサービスが使えるようになってから一度だけ設定するためのタイマー。
  rclcpp::TimerBase::SharedPtr user_defined_initial_pose_timer_;
  std::array<double, 36> output_pose_covariance_{};
  bool is_initializing_{false};
  std::optional<geometry_msgs::msg::Pose> pending_pose_;
};
}  // namespace ndt_localization::pose_initializer

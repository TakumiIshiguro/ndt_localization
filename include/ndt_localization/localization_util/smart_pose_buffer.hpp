// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/include/autoware/localization_util/smart_pose_buffer.hpp
// インターフェースも動作も同じ。
#pragma once
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <deque>
#include <mutex>
#include <optional>

namespace ndt_localization::localization_util
{
/**
 * @brief EKF の姿勢を時刻順に溜め、スキャン時刻の姿勢を補間して NDT の初期値を作るバッファ。
 *        内部で mutex を持つので、複数スレッドから呼んでよい。
 */
class SmartPoseBuffer
{
private:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

public:
  /// interpolate() の結果。
  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;           ///< 目標時刻以前で最も新しい姿勢
    PoseWithCovarianceStamped new_pose;           ///< 目標時刻より後で最も古い姿勢（なければ最後の姿勢）
    PoseWithCovarianceStamped interpolated_pose;  ///< 目標時刻での補間結果（共分散は old_pose のもの）
  };

  SmartPoseBuffer() = delete;

  /**
   * @brief バッファを作る。
   * @param[in] logger 検証に失敗したときの警告を出すロガー。
   * @param[in] pose_timeout_sec 補間に使う姿勢と目標時刻の差の上限 [s]。
   * @param[in] pose_distance_tolerance_meters 補間に使う 2 つの姿勢の距離の上限 [m]。
   */
  SmartPoseBuffer(const rclcpp::Logger & logger, const double & pose_timeout_sec,
    const double & pose_distance_tolerance_meters);

  /**
   * @brief 目標時刻を前後から挟む 2 つの姿勢を探し、その時刻の姿勢を補間する。
   * @details 目標時刻が最後の姿勢より新しい場合は、最後の姿勢をそのまま使う（タイムアウト内に限る）。
   * @param[in] target_ros_time 目標時刻（通常はスキャンの時刻）。
   * @return 補間結果。次の場合は std::nullopt:
   *         姿勢が 2 つ未満 / 目標時刻がバッファの先頭より古い /
   *         挟む姿勢のどちらかが目標時刻から pose_timeout_sec 以上離れている /
   *         挟む 2 つの姿勢が pose_distance_tolerance_meters 以上離れている（再初期化の直後など）。
   */
  std::optional<InterpolateResult> interpolate(const rclcpp::Time & target_ros_time);

  /**
   * @brief 姿勢を末尾に追加する。
   * @param[in] pose_msg_ptr 追加する姿勢。時刻が末尾より古い場合（rosbag の巻き戻しなど）は、
   *            先にバッファを空にしてから追加する。
   */
  void push_back(const PoseWithCovarianceStamped::ConstSharedPtr & pose_msg_ptr);

  /**
   * @brief 目標時刻より古い姿勢を先頭から捨てる。
   * @param[in] target_ros_time この時刻より前の姿勢を削除する。
   */
  void pop_old(const rclcpp::Time & target_ros_time);

  /// @brief バッファを空にする。
  void clear();

private:
  rclcpp::Logger logger_;
  std::deque<PoseWithCovarianceStamped::ConstSharedPtr> pose_buffer_;
  std::mutex mutex_;  // pose_buffer_ を守る
  const double pose_timeout_sec_;
  const double pose_distance_tolerance_meters_;

  /**
   * @brief 2 つの時刻の差が許容範囲内かを調べ、範囲外なら警告を出す。
   * @param[in] target_time 調べる時刻。
   * @param[in] reference_time 基準の時刻。
   * @param[in] time_tolerance_sec 許容する差 [s]。
   * @return |target_time - reference_time| < time_tolerance_sec なら true。
   */
  [[nodiscard]] bool validate_time_stamp_difference(const rclcpp::Time & target_time,
    const rclcpp::Time & reference_time, const double time_tolerance_sec) const;

  /**
   * @brief 2 点の距離が許容範囲内かを調べ、範囲外なら警告を出す。
   * @param[in] target_point 調べる点。
   * @param[in] reference_point 基準の点。
   * @param[in] distance_tolerance_m_ 許容する距離 [m]。
   * @return 距離 < distance_tolerance_m_ なら true。
   */
  [[nodiscard]] bool validate_position_difference(const geometry_msgs::msg::Point & target_point,
    const geometry_msgs::msg::Point & reference_point, const double distance_tolerance_m_) const;
};
}  // namespace ndt_localization::localization_util

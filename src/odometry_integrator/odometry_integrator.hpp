// (本パッケージ独自) Autoware Core には odom フレームがないので、対応するものはない。
// orne-box の Nav2 が odom フレーム（odom->base_link）と /odometry/filtered を使うので、gyro_odometer が出す
// 速度（車輪の vx と IMU の wz）を 2D で積分してオドメトリを作る。robot_localization の odom 用 EKF
// （two_d_mode、車輪の vx/vy と IMU の vyaw）の代わり。
#pragma once
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <optional>
#include <string>

namespace ndt_localization::odometry_integrator
{
/// odom 座標系での 2D 姿勢。
struct Pose2D
{
  double x{0.0};    ///< [m]
  double y{0.0};    ///< [m]
  double yaw{0.0};  ///< [rad]、[-π, π]
};

/**
 * @brief 速度を dt だけ積分した姿勢を返す（2 次のルンゲ・クッタ。向きは区間の中点の値を使う）。
 * @param[in] pose 積分前の姿勢。
 * @param[in] vx 前後方向の速度 [m/s]（base_link）。
 * @param[in] vy 横方向の速度 [m/s]（base_link）。
 * @param[in] wz yaw 角速度 [rad/s]。
 * @param[in] dt 積分する時間 [s]。
 * @return 積分後の姿勢。yaw は [-π, π] に正規化する。
 */
Pose2D integrate(const Pose2D & pose, double vx, double vy, double wz, double dt);

/**
 * @brief 速度を受け取るたびに、前回からの時間だけ積分してオドメトリを作る。
 *        受け取った速度は「前回の時刻から今回の時刻まで」の速度として扱う（gyro_odometer の出力はこの区間の平均）。
 */
class OdometryIntegrator
{
public:
  /**
   * @param[in] odom_frame_id 出力の header.frame_id。
   * @param[in] base_frame_id 出力の child_frame_id。入力の速度もこの座標系である必要がある。
   * @param[in] max_dt これより長い間隔は積分しない [s]。これ以下なら、途切れた後に届いた速度でその区間も積分する
   *                   （robot_localization の odom 用 EKF が途切れの間も速度で予測を続けるのと同じ扱い）。
   */
  OdometryIntegrator(std::string odom_frame_id, std::string base_frame_id, double max_dt);

  /// 処理の結果。
  enum class Status {
    INTEGRATED,         ///< 積分した
    FIRST_MESSAGE,      ///< 最初の速度なので時刻だけを覚えた（姿勢は変えない）
    TIME_JUMPED_BACK,   ///< 時刻が前回より前（bag の巻き戻しなど）。時刻だけを覚え直した
    GAP_TOO_LONG,       ///< 間隔が max_dt を超えた。時刻だけを覚え直した
  };

  /**
   * @brief 速度を 1 つ受け取って姿勢を進める。
   * @param[in] twist base_link 座標系の速度（linear.x、linear.y、angular.z を使う）。
   * @return 処理の結果。どの場合でも odometry() は今の姿勢と受け取った速度を返す。
   */
  Status update(const geometry_msgs::msg::TwistWithCovarianceStamped & twist);

  /**
   * @brief 今の姿勢と最後に受け取った速度をオドメトリにする。
   * @return header.stamp は最後の速度の時刻、frame_id は odom、child_frame_id は base_link。
   *         pose.covariance は推定しないので 0。twist は受け取った速度と共分散のまま。
   */
  nav_msgs::msg::Odometry odometry() const;

  /// 今の姿勢を返す。
  const Pose2D & pose() const {return pose_;}

private:
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double max_dt_;
  Pose2D pose_{};
  std::optional<rclcpp::Time> last_stamp_;
  geometry_msgs::msg::TwistWithCovarianceStamped last_twist_{};
};
}  // namespace ndt_localization::odometry_integrator

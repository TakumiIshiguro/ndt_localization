// 参照元: autoware_core (commit e0a8ab84)
//   sensing/autoware_vehicle_velocity_converter/src/vehicle_velocity_converter.hpp
// 違い: 入力が autoware_vehicle_msgs/VelocityReport ではなく nav_msgs/Odometry（車輪オドメトリ。
// 速度と共分散を持たない icart の /odom を想定）。出力の frame_id は Odometry の child_frame_id（twist の座標系）。
#pragma once
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace ndt_localization::vehicle_velocity_converter
{
/// 車輪オドメトリの速度に、固定の共分散を付けて TwistWithCovarianceStamped にする。
class VehicleVelocityConverter
{
public:
  /**
   * @param[in] speed_scale_factor 前後方向の速度に掛ける倍率。
   * @param[in] stddev_vx 前後方向の速度の標準偏差 [m/s]（分散は stddev_vx^2）。
   * @param[in] stddev_wz yaw 角速度の標準偏差 [rad/s]（分散は stddev_wz^2）。
   */
  VehicleVelocityConverter(double speed_scale_factor, double stddev_vx, double stddev_wz);

  /**
   * @brief 速度を変換する。
   * @param[in] msg 車輪オドメトリ。twist の linear.x / linear.y / angular.z だけを使う。
   * @return stamp は msg と同じ、frame_id は msg.child_frame_id。linear.x は倍率を掛けた値、
   *         linear.y と angular.z はそのまま。共分散は対角だけで、vx と wz 以外は 10000（観測しない）。
   */
  geometry_msgs::msg::TwistWithCovarianceStamped convert(const nav_msgs::msg::Odometry & msg) const;

private:
  double speed_scale_factor_;
  double stddev_vx_;
  double stddev_wz_;
};
}  // namespace ndt_localization::vehicle_velocity_converter

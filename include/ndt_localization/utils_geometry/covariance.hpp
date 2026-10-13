// 参照元: autoware_utils (commit 6b89a2e1)
//   autoware_utils_geometry/include/autoware_utils_geometry/msg/covariance.hpp
// このパッケージで使う xyz_covariance_index と xyzrpy_covariance_index だけを、同じ名前・同じ値で移植している。
#pragma once

namespace ndt_localization::utils_geometry
{
namespace xyz_covariance_index
{
/// x-y-z の 3x3 共分散の添字。sensor_msgs/msg/Imu の angular_velocity_covariance などで使う。
enum XYZ_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  Y_X = 3,
  Y_Y = 4,
  Y_Z = 5,
  Z_X = 6,
  Z_Y = 7,
  Z_Z = 8,
};
}  // namespace xyz_covariance_index

namespace xyzrpy_covariance_index
{
/// 6 自由度の 6x6 共分散の添字。PoseWithCovariance / TwistWithCovariance の covariance で使う。
enum XYZRPY_COV_IDX {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35
};
}  // namespace xyzrpy_covariance_index
}  // namespace ndt_localization::utils_geometry

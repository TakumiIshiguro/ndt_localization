// 参照元: autoware_core (commit e0a8ab84)
//   sensing/autoware_vehicle_velocity_converter/test/test_vehicle_velocity_converter.cpp（テスト名を合わせている）
// 違い: 入力を VelocityReport ではなく Odometry にした。HeaderCopiedVerbatim は、frame_id が child_frame_id になる
// ことを確かめる HeaderFromOdometry に置き換えた。
#include "vehicle_velocity_converter.hpp"
#include "ndt_localization/utils_geometry/covariance.hpp"
#include <gtest/gtest.h>
#include <set>
#include <string>

using ndt_localization::vehicle_velocity_converter::VehicleVelocityConverter;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using COV_IDX = ndt_localization::utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

namespace
{
/**
 * @brief テスト用の車輪オドメトリを作る。
 * @param[in] vx 前後方向の速度 [m/s]。
 * @param[in] vy 横方向の速度 [m/s]。
 * @param[in] wz yaw 角速度 [rad/s]。
 * @param[in] child_frame_id twist の座標系。
 * @return header.frame_id が odom、stamp が 123.000000456 の Odometry。
 */
Odometry make_odometry(
  const double vx, const double vy, const double wz, const std::string & child_frame_id = "base_link")
{
  Odometry msg;
  msg.header.frame_id = "odom";
  msg.header.stamp.sec = 123;
  msg.header.stamp.nanosec = 456;
  msg.child_frame_id = child_frame_id;
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.linear.y = vy;
  msg.twist.twist.angular.z = wz;
  return msg;
}

/// 6 つの対角成分以外の共分散がすべて 0 であることを確かめる。
void expect_off_diagonal_zero(const TwistWithCovarianceStamped & twist)
{
  const std::set<size_t> diagonal = {COV_IDX::X_X,       COV_IDX::Y_Y,         COV_IDX::Z_Z,
                                     COV_IDX::ROLL_ROLL, COV_IDX::PITCH_PITCH, COV_IDX::YAW_YAW};
  for (size_t i = 0; i < twist.twist.covariance.size(); ++i) {
    if (diagonal.count(i) == 0U) {
      EXPECT_EQ(twist.twist.covariance[i], 0.0) << "covariance[" << i << "] expected to be zero";
    }
  }
}
}  // namespace

// 前後方向の速度に倍率が掛かり、横方向と yaw はそのまま写されること
TEST(VehicleVelocityConverterConvert, AxisMappingAndScale)
{
  const auto msg = make_odometry(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_DOUBLE_EQ(twist.twist.twist.linear.x, 2.0 * 1.5);
  EXPECT_DOUBLE_EQ(twist.twist.twist.linear.y, 0.1);
  EXPECT_DOUBLE_EQ(twist.twist.twist.angular.z, 0.3);
  EXPECT_EQ(twist.twist.twist.linear.z, 0.0);
  EXPECT_EQ(twist.twist.twist.angular.x, 0.0);
  EXPECT_EQ(twist.twist.twist.angular.y, 0.0);
}

// 対角成分が vx/wz は標準偏差の 2 乗、それ以外は 10000 で、非対角はすべて 0 であること
TEST(VehicleVelocityConverterConvert, FullCovarianceLayout)
{
  const auto msg = make_odometry(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::Y_Y], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::Z_Z], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::ROLL_ROLL], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::PITCH_PITCH], 10000.0);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);
  expect_off_diagonal_zero(twist);
}

// stamp は入力と同じで、frame_id は header.frame_id（odom）ではなく child_frame_id になること
TEST(VehicleVelocityConverterConvert, HeaderFromOdometry)
{
  const auto msg = make_odometry(2.0, 0.1, 0.3, "custom_frame");
  const VehicleVelocityConverter converter(1.0, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_EQ(twist.header.frame_id, "custom_frame");
  EXPECT_EQ(twist.header.stamp.sec, msg.header.stamp.sec);
  EXPECT_EQ(twist.header.stamp.nanosec, msg.header.stamp.nanosec);
}

// 倍率 0 なら前後方向の速度だけが 0 になること
TEST(VehicleVelocityConverterConvert, ZeroScaleFactor)
{
  const auto msg = make_odometry(2.0, 0.1, 0.3);
  const VehicleVelocityConverter converter(0.0, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_EQ(twist.twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.twist.twist.linear.y, 0.1);
  EXPECT_DOUBLE_EQ(twist.twist.twist.angular.z, 0.3);
}

// 負の速度と負の倍率でも符号どおりに計算され、分散は負にならないこと
TEST(VehicleVelocityConverterConvert, NegativeVelocityAndScale)
{
  const auto msg = make_odometry(-2.0, -0.1, -0.3);
  const VehicleVelocityConverter converter(-1.5, 0.2, 0.1);

  const auto twist = converter.convert(msg);

  EXPECT_DOUBLE_EQ(twist.twist.twist.linear.x, -2.0 * -1.5);
  EXPECT_DOUBLE_EQ(twist.twist.twist.linear.y, -0.1);
  EXPECT_DOUBLE_EQ(twist.twist.twist.angular.z, -0.3);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::X_X], 0.2 * 0.2);
  EXPECT_DOUBLE_EQ(twist.twist.covariance[COV_IDX::YAW_YAW], 0.1 * 0.1);
}

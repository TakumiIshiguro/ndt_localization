// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/test/test_util_func.cpp（テスト名を合わせている）
#include "ndt_localization/localization_util/util_func.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <gtest/gtest.h>
#include <cmath>
using namespace ndt_localization::localization_util;
namespace
{
/**
 * @brief テスト用の時刻付き姿勢を作る。
 * @param[in] stamp 時刻 [s]。
 * @param[in] x x 座標 [m]（y, z は 0）。
 * @param[in] yaw yaw 角 [rad]（roll, pitch は 0）。
 * @return frame_id が map の PoseStamped。
 */
geometry_msgs::msg::PoseStamped pose_stamped(double stamp, double x, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
  p.header.frame_id = "map";
  p.pose.position.x = x;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  p.pose.orientation = tf2::toMsg(q);
  return p;
}
/// 秒（double）を ROS 時刻に変換する。入力: stamp [s]。出力: RCL_ROS_TIME の rclcpp::Time。
rclcpp::Time t(double stamp) {return rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);}
}  // namespace

// 2 つの角度の差が [-π, π) に正規化されること
TEST(UtilFuncTest, CalcDiffForRadian)
{
  EXPECT_NEAR(calc_diff_for_radian(0.3, 0.1), 0.2, 1e-12);
  EXPECT_NEAR(calc_diff_for_radian(-3.1, 3.1), 2 * M_PI - 6.2, 1e-12);
  EXPECT_NEAR(calc_diff_for_radian(3.1, -3.1), 6.2 - 2 * M_PI, 1e-12);
}

// rpy → クォータニオン → rpy で元の角度に戻ること
TEST(UtilFuncTest, GetRPY)
{
  geometry_msgs::msg::Pose pose;
  tf2::Quaternion q;
  q.setRPY(0.1, -0.2, 0.3);
  pose.orientation = tf2::toMsg(q);
  const auto rpy = get_rpy(pose);
  EXPECT_NEAR(rpy.x, 0.1, 1e-9);
  EXPECT_NEAR(rpy.y, -0.2, 1e-9);
  EXPECT_NEAR(rpy.z, 0.3, 1e-9);
}

// 2 姿勢の中間時刻で位置と yaw が線形補間され、範囲外の時刻では同じ速さで外挿されること。
// 時刻が 0 の姿勢を渡すと空の姿勢が返ること
TEST(UtilFuncTest, InterpolatePose)
{
  const auto a = pose_stamped(10.0, 0, 0.2), b = pose_stamped(11.0, 2, 0.6);
  const auto mid = interpolate_pose(a, b, t(10.5));
  EXPECT_EQ(rclcpp::Time(mid.header.stamp), t(10.5));
  EXPECT_NEAR(mid.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(get_rpy(mid.pose).z, 0.4, 1e-9);
  const auto extrapolated = interpolate_pose(a, b, t(11.5));
  EXPECT_NEAR(extrapolated.pose.position.x, 3.0, 1e-9);
  EXPECT_NEAR(get_rpy(extrapolated.pose).z, 0.8, 1e-9);
  const auto empty = interpolate_pose(pose_stamped(0.0, 1, 0), b, t(10.5));
  EXPECT_DOUBLE_EQ(empty.pose.position.x, 0.0);
  EXPECT_TRUE(empty.header.frame_id.empty());
}

// yaw が ±π をまたいでも、短いほうの回転で補間されること（中間が π 付近になる）
TEST(UtilFuncTest, InterpolatePoseAcrossYawWrap)
{
  const auto mid = interpolate_pose(pose_stamped(1, 0, 3.1), pose_stamped(2, 0, -3.1), t(1.5));
  EXPECT_NEAR(std::abs(get_rpy(mid.pose).z), M_PI, 1e-6);
}

// Pose → 4x4 行列 → Pose で元に戻ること、Affine3d 版とも一致すること
TEST(UtilFuncTest, PoseConversion)
{
  const auto p = pose_stamped(0, 1.5, -0.7).pose;
  const auto back = matrix4f_to_pose(pose_to_matrix4f(p));
  EXPECT_NEAR(back.position.x, 1.5, 1e-6);
  EXPECT_NEAR(get_rpy(back).z, -0.7, 1e-6);
  EXPECT_TRUE(pose_to_affine3d(p).matrix().isApprox(pose_to_matrix4f(p).cast<double>(), 1e-6));
}

// 2 点間の距離が正しいこと（3-4-5 の直角三角形）
TEST(UtilFuncTest, NormCalculation)
{
  geometry_msgs::msg::Point a, b;
  b.x = 3; b.y = 4;
  EXPECT_DOUBLE_EQ(norm(a, b), 5.0);
}

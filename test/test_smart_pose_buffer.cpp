// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/test/test_smart_pose_buffer.cpp（テスト名を合わせている）
#include "ndt_localization/localization_util/smart_pose_buffer.hpp"
#include <gtest/gtest.h>
using ndt_localization::localization_util::SmartPoseBuffer;
using geometry_msgs::msg::PoseWithCovarianceStamped;
namespace
{
/**
 * @brief テスト用の EKF 姿勢を作る。
 * @param[in] stamp 時刻 [s]。
 * @param[in] x x 座標 [m]。どの姿勢から来たか分かるよう、共分散の [0] にも同じ値を入れる。
 * @return frame_id が map の PoseWithCovarianceStamped。
 */
PoseWithCovarianceStamped::ConstSharedPtr pose(double stamp, double x)
{
  auto p = std::make_shared<PoseWithCovarianceStamped>();
  p->header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
  p->header.frame_id = "map";
  p->pose.pose.position.x = x;
  p->pose.covariance[0] = x;  // marks which pose the covariance came from
  return p;
}
/// 秒（double）を ROS 時刻に変換する。入力: stamp [s]。出力: RCL_ROS_TIME の rclcpp::Time。
rclcpp::Time t(double stamp) {return rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);}
const rclcpp::Logger logger = rclcpp::get_logger("test_smart_pose_buffer");
}  // namespace

// 挟む 2 姿勢の間で位置が補間され、時刻は目標時刻、共分散は old_pose のものになること
TEST(TestSmartPoseBuffer, interpolate_pose)
{
  SmartPoseBuffer buffer(logger, 1.0, 10.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 1.0));
  buffer.push_back(pose(10.2, 2.0));
  const auto result = buffer.interpolate(t(10.15));
  ASSERT_TRUE(result);
  EXPECT_NEAR(result->interpolated_pose.pose.pose.position.x, 1.5, 1e-9);
  EXPECT_EQ(rclcpp::Time(result->interpolated_pose.header.stamp), t(10.15));
  EXPECT_DOUBLE_EQ(result->interpolated_pose.pose.covariance[0], 1.0);  // old_pose's covariance
}

// 姿勢が 0 個・1 個のときは補間できないこと
TEST(TestSmartPoseBuffer, empty_buffer)
{
  SmartPoseBuffer buffer(logger, 1.0, 10.0);
  EXPECT_FALSE(buffer.interpolate(t(10.0)));
  buffer.push_back(pose(10.0, 0.0));
  EXPECT_FALSE(buffer.interpolate(t(10.0)));  // needs two poses
}

// 最後の姿勢から目標時刻までがタイムアウトを超えたら補間できないこと
TEST(TestSmartPoseBuffer, timeout_validation)
{
  SmartPoseBuffer buffer(logger, 0.5, 10.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 1.0));
  EXPECT_FALSE(buffer.interpolate(t(10.7)));
}

// 挟む 2 姿勢が許容距離以上離れている（再初期化の直後など）と補間できないこと
TEST(TestSmartPoseBuffer, position_tolerance_validation)
{
  SmartPoseBuffer buffer(logger, 1.0, 2.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 5.0));  // e.g. EKF re-initialized
  EXPECT_FALSE(buffer.interpolate(t(10.05)));
}

// pop_old で目標時刻より古い姿勢だけが消え、clear で空になること
TEST(TestSmartPoseBuffer, buffer_operations)
{
  SmartPoseBuffer buffer(logger, 1.0, 10.0);
  for (int i = 0; i < 5; ++i) {buffer.push_back(pose(10.0 + 0.1 * i, i));}
  buffer.pop_old(t(10.25));
  EXPECT_FALSE(buffer.interpolate(t(10.15)));  // 10.3 より前の姿勢は消えている
  EXPECT_TRUE(buffer.interpolate(t(10.35)));   // 10.3 と 10.4 は残っている
  buffer.clear();
  EXPECT_FALSE(buffer.interpolate(t(10.35)));
}

// 時刻が逆行した姿勢を追加すると、バッファが空になってからその姿勢だけが入ること
TEST(TestSmartPoseBuffer, non_chronological_timestamps)
{
  SmartPoseBuffer buffer(logger, 1.0, 10.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 0.0));
  buffer.push_back(pose(3.0, 0.0));  // 時刻が逆行（rosbag の巻き戻しなど）
  EXPECT_FALSE(buffer.interpolate(t(3.0)));  // 空にされたので姿勢は 1 つだけ
  buffer.push_back(pose(3.1, 1.0));
  EXPECT_TRUE(buffer.interpolate(t(3.05)));
}

// 目標時刻がバッファの先頭より古いと補間できないこと
TEST(TestSmartPoseBuffer, target_time_before_first_pose)
{
  SmartPoseBuffer buffer(logger, 1.0, 10.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 1.0));
  EXPECT_FALSE(buffer.interpolate(t(9.9)));
}

// 目標時刻が最後の姿勢より新しくても、タイムアウト内なら最後の姿勢をそのまま使うこと
TEST(TestSmartPoseBuffer, target_time_after_last_pose)
{
  SmartPoseBuffer buffer(logger, 0.5, 10.0);
  buffer.push_back(pose(10.0, 0.0));
  buffer.push_back(pose(10.1, 1.0));
  const auto held = buffer.interpolate(t(10.3));  // holds the last pose within the timeout
  ASSERT_TRUE(held);
  EXPECT_NEAR(held->interpolated_pose.pose.pose.position.x, 1.0, 1e-9);
}

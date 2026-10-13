// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_pose_initializer/src/pose_initializer.cpp（validate_user_defined_initial_pose）
// Autoware には validate_user_defined_initial_pose の単体テストがないので、このパッケージで追加した。
#include "pose_initializer/pose_initializer.hpp"
#include <gtest/gtest.h>
#include <stdexcept>
#include <vector>

using ndt_localization::pose_initializer::validate_user_defined_initial_pose;

// 7 要素の配列が [x, y, z, qx, qy, qz, qw] の順に姿勢へ入ること
TEST(ValidateUserDefinedInitialPose, CopiesSevenValues)
{
  const auto pose = validate_user_defined_initial_pose({1.0, 2.0, 3.0, 0.0, 0.0, 0.6, 0.8});
  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.z, 0.6);
  EXPECT_DOUBLE_EQ(pose.orientation.w, 0.8);
}

// 要素数が 7 でなければ例外になること
TEST(ValidateUserDefinedInitialPose, RejectsWrongSize)
{
  EXPECT_THROW(validate_user_defined_initial_pose({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
  EXPECT_THROW(validate_user_defined_initial_pose(std::vector<double>(8, 1.0)), std::invalid_argument);
}

// クォータニオンがすべて 0 なら例外になること
TEST(ValidateUserDefinedInitialPose, RejectsZeroQuaternion)
{
  EXPECT_THROW(validate_user_defined_initial_pose({1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0}), std::invalid_argument);
}

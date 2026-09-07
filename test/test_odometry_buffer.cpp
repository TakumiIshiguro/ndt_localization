#include "wheel_lidar_localization/odometry_buffer.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace wheel_lidar_localization;
namespace
{
Pose pose(double x, double yaw = 0)
{
  Pose p = Pose::Identity(); p.translation().x() = x;
  p.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix(); return p;
}
}
TEST(Odometry, FirstSampleAndInterpolation)
{
  OdometryBuffer b(10, 2, 1, 100, 100);
  EXPECT_FALSE(b.at(0));
  EXPECT_EQ(b.add(10, pose(2, 0)), OdomAdmission::Accepted);
  EXPECT_TRUE(b.at(10)); EXPECT_FALSE(b.at(10.1)); EXPECT_FALSE(b.at(9));
  b.add(11, pose(4, 1)); auto p = b.at(10.5); ASSERT_TRUE(p);
  EXPECT_NEAR(p->translation().x(), 3, 1e-9);
  EXPECT_NEAR(rotationDistance(*p, pose(3, 0.5)), 0, 1e-9);
}
TEST(Odometry, RotationWrapAndMissingInterval)
{
  OdometryBuffer b(10, 0.2, 1, 100, 100);
  b.add(0, pose(0, 3.1)); b.add(0.1, pose(0, -3.1));
  ASSERT_TRUE(b.at(0.05));
  EXPECT_NEAR(rotationDistance(*b.at(0.05), pose(0, 3.141592653589793)), 0, 1e-8);
  b.add(1, pose(0)); EXPECT_FALSE(b.at(0.5));
}
TEST(Odometry, InvalidAndDiscontinuity)
{
  OdometryBuffer b(10, 0.2, 1, 5, 5);
  auto invalid = pose(0); invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(b.add(10, invalid), OdomAdmission::Invalid);
  b.add(10, pose(0));
  EXPECT_EQ(b.add(9.9, pose(0)), OdomAdmission::OutOfOrder);
  EXPECT_EQ(b.add(10.1, pose(100)), OdomAdmission::Reset);
  EXPECT_FALSE(b.at(10));
  EXPECT_EQ(b.add(1, pose(0)), OdomAdmission::Reset);
  b.clear(); EXPECT_FALSE(b.at(1));
}

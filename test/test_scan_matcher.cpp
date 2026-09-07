#include "wheel_lidar_localization/scan_matcher.hpp"
#include <pcl/common/transforms.h>
#include <gtest/gtest.h>
#include <random>
using namespace wheel_lidar_localization;
TEST(ScanMatcher, RecoversKnownRigidTransform)
{
  auto map = std::make_shared<Cloud>();
  std::mt19937 generator(42);
  std::uniform_real_distribution<float> center(-8, 8);
  std::normal_distribution<float> noise(0, 0.18);
  for (int cluster = 0; cluster < 140; ++cluster) {
    const Eigen::Vector3f c(center(generator), center(generator), center(generator) * 0.3f);
    for (int point = 0; point < 80; ++point) {
      map->push_back(pcl::PointXYZ(c.x() + noise(generator), c.y() + noise(generator), c.z() + noise(generator)));
    }
  }
  Pose truth = Pose::Identity(); truth.translation() << 0.2, -0.12, 0.05;
  truth.linear() = Eigen::AngleAxisd(0.035, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto source = std::make_shared<Cloud>();
  pcl::transformPointCloud(*map, *source, truth.inverse().matrix().cast<float>());
  ScanMatcher matcher(1.0, 0.1, 0.0001, 80, 0.5); matcher.setTarget(map);
  const auto result = matcher.align(source, Pose::Identity());
  ASSERT_TRUE(result.converged);
  EXPECT_LT((result.pose.translation() - truth.translation()).norm(), 0.06);
  EXPECT_LT(rotationDistance(result.pose, truth), 0.02);
  EXPECT_GT(result.inlier_ratio, 0.95);
  EXPECT_LT(result.rmse, 0.08);
}
TEST(ScanMatcher, EmptyInputRejected)
{
  ScanMatcher matcher(1, 0.1, 0.01, 40, 1);
  auto empty = std::make_shared<Cloud>(); matcher.setTarget(empty);
  EXPECT_FALSE(matcher.align(empty, Pose::Identity()).converged);
}

// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ndt_scan_matcher/test/test_ndt_scan_matcher_helper.cpp（テスト名を合わせている）
#include "ndt_scan_matcher/ndt_scan_matcher_helper.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <cmath>
using namespace ndt_localization::ndt_scan_matcher;
namespace
{
/// x 軸上の姿勢を作る。入力: x [m]。出力: 位置 (x, 0, 0)、向きは単位クォータニオンの Pose。
geometry_msgs::msg::Pose pose_at(double x)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  return p;
}
}  // namespace

// z 軸まわりに 90° 回すと x と y の分散が入れ替わり、z と角度部分は変わらないこと
TEST(NdtScanMatcherHelper, RotateCovarianceSwapsDiagonalUnderQuarterTurn)
{
  std::array<double, 36> cov{};
  cov[0] = 4.0; cov[7] = 1.0; cov[14] = 0.5; cov[35] = 0.1;
  const auto r = rotate_covariance(cov, Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()).toRotationMatrix());
  EXPECT_NEAR(r[0], 1.0, 1e-9);
  EXPECT_NEAR(r[7], 4.0, 1e-9);
  EXPECT_NEAR(r[14], 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(r[35], 0.1);  // angular block untouched
}

// 0,1,0,1,... と往復する 8 姿勢では、連続した反転が 6 回と数えられること
TEST(NdtScanMatcherHelper, CountOscillationCountsConsecutiveInversions)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (int i = 0; i < 8; ++i) {poses.push_back(pose_at(i % 2));}
  EXPECT_EQ(count_oscillation(poses), 6);
}

// 一直線に進む場合は振動が 0 回であること
TEST(NdtScanMatcherHelper, CountOscillationStraightLineIsZero)
{
  std::vector<geometry_msgs::msg::Pose> poses;
  for (int i = 0; i < 8; ++i) {poses.push_back(pose_at(i));}
  EXPECT_EQ(count_oscillation(poses), 0);
}

// 同じ位置が続く（移動量 0）場合は振動として数えないこと
TEST(NdtScanMatcherHelper, CountOscillationZeroLengthStepsAreNotOscillations)
{
  std::vector<geometry_msgs::msg::Pose> poses(5, pose_at(1.0));
  EXPECT_EQ(count_oscillation(poses), 0);
}

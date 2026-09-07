#include "wheel_lidar_localization/localization_state.hpp"
#include <gtest/gtest.h>
using namespace wheel_lidar_localization;
TEST(State, CorrectionCompositionAndPrediction)
{
  LocalizationState s(3);
  Pose map_base = Pose::Identity(), odom_base = Pose::Identity();
  map_base.translation() << 7, -2, 0.3;
  map_base.linear() = Eigen::AngleAxisd(0.9, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  odom_base.translation() << 1, 2, 0;
  odom_base.linear() = Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  s.initialize(map_base); s.accept(map_base, odom_base, 10);
  EXPECT_TRUE((s.correction() * odom_base).matrix().isApprox(map_base.matrix()));
  Pose delta = Pose::Identity(); delta.translation().x() = 1;
  EXPECT_TRUE(s.seed(odom_base * delta).matrix().isApprox((map_base * delta).matrix()));
}
TEST(State, FailedMatchesHoldCorrectionAndRequireReinitialization)
{
  LocalizationState s(2); s.initialize(Pose::Identity());
  s.accept(Pose::Identity(), Pose::Identity(), 5);
  s.reject(); EXPECT_EQ(s.state(), TrackingState::Degraded); EXPECT_TRUE(s.hasCorrection());
  s.reject(); EXPECT_EQ(s.state(), TrackingState::Lost);
  Pose wrong = Pose::Identity(); wrong.translation().x() = 20;
  s.accept(wrong, Pose::Identity(), 6); EXPECT_TRUE(s.correction().matrix().isIdentity());
  s.initialize(wrong); EXPECT_FALSE(s.hasCorrection()); EXPECT_EQ(s.state(), TrackingState::Initializing);
  s.accept(wrong, Pose::Identity(), 7); EXPECT_EQ(s.state(), TrackingState::Tracking);
  s.invalidate(); EXPECT_FALSE(s.hasCorrection()); EXPECT_EQ(s.state(), TrackingState::Lost);
}
TEST(State, MeasurementGates)
{
  MatchResult r; r.converged = true; r.rmse = 0.1; r.inlier_ratio = 0.9;
  MatchLimits limits{0.5, 0.5, 1.5, 0.5};
  EXPECT_TRUE(rejectionReason(r, limits).empty());
  r.inlier_ratio = 0.1; EXPECT_EQ(rejectionReason(r, limits), "insufficient_overlap");
  r.inlier_ratio = 0.9; r.rotation_correction = 1;
  EXPECT_EQ(rejectionReason(r, limits), "rotation_correction_too_large");
  r.rotation_correction = 0; r.translation_correction = 10;
  EXPECT_EQ(rejectionReason(r, limits), "translation_correction_too_large");
  r.translation_correction = 0; r.rmse = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(rejectionReason(r, limits), "rmse_too_large");
}

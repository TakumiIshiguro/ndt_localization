// (本パッケージ独自) Autoware に対応するテストはない。
#include "odometry_integrator.hpp"
#include <gtest/gtest.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using ndt_localization::odometry_integrator::integrate;
using ndt_localization::odometry_integrator::OdometryIntegrator;
using ndt_localization::odometry_integrator::Pose2D;
using geometry_msgs::msg::TwistWithCovarianceStamped;

namespace
{
/**
 * @brief テスト用の速度を作る。
 * @param[in] stamp 時刻 [s]。
 * @param[in] vx 前後方向の速度 [m/s]。
 * @param[in] wz yaw 角速度 [rad/s]。
 * @return frame_id が base_link の速度。共分散の [0] に目印として 0.04 を入れる。
 */
TwistWithCovarianceStamped twist(const double stamp, const double vx, const double wz)
{
  TwistWithCovarianceStamped t;
  t.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp * 1e9), RCL_ROS_TIME);
  t.header.frame_id = "base_link";
  t.twist.twist.linear.x = vx;
  t.twist.twist.angular.z = wz;
  t.twist.covariance[0] = 0.04;
  return t;
}
}  // namespace

// 直進なら向きの方向に vx * dt だけ進むこと
TEST(OdometryIntegrator, IntegrateStraight)
{
  const Pose2D p = integrate(Pose2D{1.0, 2.0, M_PI_2}, 0.5, 0.0, 0.0, 2.0);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 3.0, 1e-9);
  EXPECT_NEAR(p.yaw, M_PI_2, 1e-9);
}

// 横方向の速度は向きに対して左向きに進むこと
TEST(OdometryIntegrator, IntegrateLateral)
{
  const Pose2D p = integrate(Pose2D{0.0, 0.0, 0.0}, 0.0, 1.0, 0.0, 1.0);
  EXPECT_NEAR(p.x, 0.0, 1e-9);
  EXPECT_NEAR(p.y, 1.0, 1e-9);
}

// 一定の vx と wz で細かく積分すると円弧に乗り、yaw は [-π, π] に収まること
TEST(OdometryIntegrator, IntegrateArc)
{
  Pose2D p;
  const double vx = 1.0;
  const double wz = 0.5;  // radius 2 m
  for (int i = 0; i < 1000; ++i) {
    p = integrate(p, vx, 0.0, wz, 2.0 * M_PI / wz / 1000.0 / 2.0);  // half a turn in total
  }
  EXPECT_NEAR(p.x, 0.0, 1e-3);
  EXPECT_NEAR(p.y, 4.0, 1e-3);
  EXPECT_NEAR(std::abs(p.yaw), M_PI, 1e-6);
}

// 最初の速度では時刻だけを覚え、次の速度から前回との時間差で積分すること
TEST(OdometryIntegrator, UpdateIntegratesFromSecondMessage)
{
  OdometryIntegrator integrator("odom", "base_link", 1.0);
  EXPECT_EQ(integrator.update(twist(10.0, 1.0, 0.0)), OdometryIntegrator::Status::FIRST_MESSAGE);
  EXPECT_DOUBLE_EQ(integrator.pose().x, 0.0);
  EXPECT_EQ(integrator.update(twist(10.1, 1.0, 0.0)), OdometryIntegrator::Status::INTEGRATED);
  EXPECT_NEAR(integrator.pose().x, 0.1, 1e-9);
  EXPECT_EQ(integrator.update(twist(10.3, 2.0, 0.0)), OdometryIntegrator::Status::INTEGRATED);
  EXPECT_NEAR(integrator.pose().x, 0.5, 1e-9);  // the new twist is applied to the past 0.2 s
}

// 時刻が戻ったときと間隔が max_dt を超えたときは積分せず、その後は新しい時刻から積分すること
TEST(OdometryIntegrator, UpdateSkipsJumpsAndGaps)
{
  OdometryIntegrator integrator("odom", "base_link", 0.5);
  integrator.update(twist(10.0, 1.0, 0.0));
  EXPECT_EQ(integrator.update(twist(9.0, 1.0, 0.0)), OdometryIntegrator::Status::TIME_JUMPED_BACK);
  EXPECT_EQ(integrator.update(twist(10.0, 1.0, 0.0)), OdometryIntegrator::Status::GAP_TOO_LONG);
  EXPECT_DOUBLE_EQ(integrator.pose().x, 0.0);
  EXPECT_EQ(integrator.update(twist(10.2, 1.0, 0.0)), OdometryIntegrator::Status::INTEGRATED);
  EXPECT_NEAR(integrator.pose().x, 0.2, 1e-9);
}

// オドメトリのフレーム、時刻、姿勢、速度と共分散が正しく入ること
TEST(OdometryIntegrator, OdometryMessage)
{
  OdometryIntegrator integrator("odom", "base_link", 1.0);
  integrator.update(twist(10.0, 1.0, 0.0));
  integrator.update(twist(10.5, 1.0, 0.2));
  const auto odom = integrator.odometry();
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(odom.header.stamp).nanoseconds(), 10'500'000'000);
  EXPECT_NEAR(tf2::getYaw(odom.pose.pose.orientation), 0.1, 1e-9);
  EXPECT_DOUBLE_EQ(odom.twist.twist.angular.z, 0.2);
  EXPECT_DOUBLE_EQ(odom.twist.covariance[0], 0.04);
  EXPECT_DOUBLE_EQ(odom.pose.pose.position.z, 0.0);
}

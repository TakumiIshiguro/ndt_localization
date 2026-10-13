#include "imu_corrector_core.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <thread>

namespace ndt_localization::imu_corrector
{
namespace
{
using sensor_msgs::msg::Imu;
using namespace std::chrono_literals;

geometry_msgs::msg::TransformStamped make_transform(
  const std::string & parent, const std::string & child, double yaw)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = parent;
  tf.child_frame_id = child;
  tf.transform.translation.x = 100.0;
  tf.transform.translation.y = -200.0;
  tf.transform.translation.z = 300.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  tf.transform.rotation = tf2::toMsg(q);
  return tf;
}

void expect_vector(const geometry_msgs::msg::Vector3 & v, double x, double y, double z)
{
  EXPECT_NEAR(v.x, x, 1e-9);
  EXPECT_NEAR(v.y, y, 1e-9);
  EXPECT_NEAR(v.z, z, 1e-9);
}

void expect_covariance(const std::array<double, 9> & cov, double variance)
{
  for (size_t i = 0; i < cov.size(); ++i) {
    EXPECT_DOUBLE_EQ(cov[i], i % 4 == 0 ? variance : 0.0) << "index " << i;
  }
}

TEST(ImuCorrectorHelpers, CovarianceUsesLargestDiagonalAndClearsOffDiagonal)
{
  expect_covariance(transform_covariance({0.01, 10, 20, 10, 0.09, 30, 20, 30, 0.04}), 0.09);
}

TEST(ImuCorrectorHelpers, ZeroCovarianceRemainsZero)
{
  expect_covariance(transform_covariance({}), 0.0);
}

TEST(ImuCorrectorHelpers, RotatesVectorWithoutTranslation)
{
  geometry_msgs::msg::Vector3 vec;
  vec.x = 1.0;
  vec.y = 2.0;
  vec.z = 3.0;
  expect_vector(transform_vector3(vec, make_transform("base_link", "imu_link", M_PI_2)), -2, 1, 3);
}

class ImuCorrectorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    driver_ = std::make_shared<rclcpp::Node>("imu_corrector_test_driver");
    publisher_ = driver_->create_publisher<Imu>("input", rclcpp::QoS(10));
    subscriber_ = driver_->create_subscription<Imu>(
      "output", rclcpp::QoS(10), [this](Imu::ConstSharedPtr msg) { output_ = *msg; });
    broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver_);
    executor_->add_node(driver_);
  }

  void TearDown() override
  {
    if (corrector_) {
      executor_->remove_node(corrector_);
      corrector_.reset();
    }
    executor_->remove_node(driver_);
    broadcaster_.reset();
    subscriber_.reset();
    publisher_.reset();
    driver_.reset();
    executor_.reset();
    rclcpp::shutdown();
  }

  bool pump_until(const std::function<bool()> & predicate, std::chrono::milliseconds budget = 2s)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    do {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(2ms);
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }

  void start(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    corrector_ = std::make_shared<ImuCorrector>(options);
    executor_->add_node(corrector_);
    ASSERT_TRUE(pump_until([this]() {
      return publisher_->get_subscription_count() == 1 &&
             subscriber_->get_publisher_count() == 1;
    }));
  }

  Imu input(const std::string & frame = "base_link")
  {
    Imu msg;
    msg.header.frame_id = frame;
    msg.header.stamp.sec = 123;
    msg.header.stamp.nanosec = 456789;
    msg.angular_velocity.x = 1;
    msg.angular_velocity.y = 2;
    msg.angular_velocity.z = 3;
    msg.linear_acceleration.x = 4;
    msg.linear_acceleration.y = 5;
    msg.linear_acceleration.z = 6;
    msg.angular_velocity_covariance.fill(42);
    msg.linear_acceleration_covariance.fill(43);
    return msg;
  }

  bool deliver(const Imu & msg)
  {
    output_.reset();
    publisher_->publish(msg);
    return pump_until([this]() { return output_.has_value(); });
  }

  // A separate listener confirms that the static TF has actually reached a listener before input.
  void send_transform(const geometry_msgs::msg::TransformStamped & tf)
  {
    utils_tf::TransformListener probe(driver_.get());
    broadcaster_->sendTransform(tf);
    ASSERT_TRUE(pump_until([&]() {
      return bool(probe.get_latest_transform(tf.header.frame_id, tf.child_frame_id));
    }));
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<rclcpp::Node> driver_;
  std::shared_ptr<ImuCorrector> corrector_;
  rclcpp::Publisher<Imu>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr subscriber_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::optional<Imu> output_;
};

TEST_F(ImuCorrectorNodeTest, DefaultsOverwriteCovarianceAndPreserveStamp)
{
  start();
  const auto msg = input();
  ASSERT_TRUE(deliver(msg));
  EXPECT_EQ(output_->header.frame_id, "base_link");
  EXPECT_EQ(output_->header.stamp, msg.header.stamp);
  expect_vector(output_->angular_velocity, 1, 2, 3);
  expect_vector(output_->linear_acceleration, 4, 5, 6);
  expect_covariance(output_->angular_velocity_covariance, 0.03 * 0.03);
  expect_covariance(output_->linear_acceleration_covariance, 1e8);
}

TEST_F(ImuCorrectorNodeTest, AppliesOffsetsAndCustomStandardDeviations)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"angular_velocity_offset_x", 0.5}, {"angular_velocity_offset_y", -0.25},
    {"angular_velocity_offset_z", 3.0}, {"angular_velocity_stddev_xx", 0.1},
    {"angular_velocity_stddev_yy", 0.3}, {"angular_velocity_stddev_zz", 0.2},
    {"acceleration_stddev", 2.0}});
  start(options);
  ASSERT_TRUE(deliver(input()));
  expect_vector(output_->angular_velocity, 0.5, 2.25, 0);
  expect_covariance(output_->angular_velocity_covariance, 0.3 * 0.3);
  expect_covariance(output_->linear_acceleration_covariance, 4.0);
}

TEST_F(ImuCorrectorNodeTest, RotatesFromSensorToBaseAfterSubtractingSensorOffsets)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("angular_velocity_offset_x", 0.5);
  start(options);
  send_transform(make_transform("base_link", "imu_link", M_PI_2));
  ASSERT_TRUE(deliver(input("imu_link")));
  expect_vector(output_->angular_velocity, -2, 0.5, 3);
  expect_vector(output_->linear_acceleration, -5, 4, 6);
  EXPECT_EQ(output_->header.frame_id, "base_link");
}

TEST_F(ImuCorrectorNodeTest, UsesConfiguredOutputFrame)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("base_link", "robot_body");
  start(options);
  send_transform(make_transform("robot_body", "imu_link", -M_PI_2));
  ASSERT_TRUE(deliver(input("imu_link")));
  EXPECT_EQ(output_->header.frame_id, "robot_body");
  expect_vector(output_->angular_velocity, 2, -1, 3);
  expect_vector(output_->linear_acceleration, 5, -4, 6);
}

TEST_F(ImuCorrectorNodeTest, MissingTransformDropsInputAndRecoversWhenTransformArrives)
{
  start();
  publisher_->publish(input("late_imu"));
  EXPECT_FALSE(pump_until([this]() { return output_.has_value(); }, 200ms));
  send_transform(make_transform("base_link", "late_imu", 0));
  ASSERT_TRUE(deliver(input("late_imu")));
  expect_vector(output_->angular_velocity, 1, 2, 3);
}

TEST_F(ImuCorrectorNodeTest, EmptyInputFrameDropsInputWithoutCrashing)
{
  start();
  publisher_->publish(input(""));
  EXPECT_FALSE(pump_until([this]() { return output_.has_value(); }, 200ms));
  ASSERT_TRUE(deliver(input()));
}
}  // namespace
}  // namespace ndt_localization::imu_corrector

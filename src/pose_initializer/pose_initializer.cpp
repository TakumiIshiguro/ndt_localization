// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_pose_initializer/src/pose_initializer.cpp
//   localization/autoware_pose_initializer/src/utils/copy_vector_to_array.hpp
#include "pose_initializer.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ndt_localization::pose_initializer
{
constexpr float near_tol = 1e-6F;

// Ref: PoseInitializer::validate_user_defined_initial_pose（処理は同じ）
geometry_msgs::msg::Pose validate_user_defined_initial_pose(const std::vector<double> & initial_pose_array)
{
  if (initial_pose_array.size() != 7) {
    throw std::invalid_argument(
      "Could not set user defined initial pose. The size of initial_pose is " +
      std::to_string(initial_pose_array.size()) + ". It must be 7.");
  }
  if (
    std::abs(initial_pose_array[3]) < near_tol && std::abs(initial_pose_array[4]) < near_tol &&
    std::abs(initial_pose_array[5]) < near_tol && std::abs(initial_pose_array[6]) < near_tol)
  {
    throw std::invalid_argument("Input quaternion is invalid. All elements are close to zero.");
  }

  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position.x = initial_pose_array[0];
  initial_pose.position.y = initial_pose_array[1];
  initial_pose.position.z = initial_pose_array[2];
  initial_pose.orientation.x = initial_pose_array[3];
  initial_pose.orientation.y = initial_pose_array[4];
  initial_pose.orientation.z = initial_pose_array[5];
  initial_pose.orientation.w = initial_pose_array[6];

  return initial_pose;
}

// Ref: get_covariance_parameter / copy_vector_to_array（処理は同じ）
std::array<double, 36> get_covariance_parameter(rclcpp::Node * node, const std::string & name)
{
  const auto parameter = node->declare_parameter<std::vector<double>>(name);
  std::array<double, 36> covariance{};
  if (parameter.size() != covariance.size()) {
    // throws the error to prevent causing an anonymous bug
    // such as only partial array is initialized
    throw std::invalid_argument(
      "Vector size (which is " + std::to_string(parameter.size()) + ") is different from the copy size (which is " +
      std::to_string(covariance.size()) + ")");
  }
  std::copy_n(parameter.begin(), covariance.size(), covariance.begin());
  return covariance;
}
}  // namespace ndt_localization::pose_initializer

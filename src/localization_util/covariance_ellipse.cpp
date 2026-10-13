// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_localization_util/src/covariance_ellipse.cpp
#include "ndt_localization/localization_util/covariance_ellipse.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace ndt_localization::localization_util
{
// Ref: calculate_xy_ellipse（処理は同じ）
Ellipse calculate_xy_ellipse(
  const geometry_msgs::msg::PoseWithCovariance & pose_with_covariance, const double scale)
{
  // input geometry_msgs::PoseWithCovariance contain 6x6 matrix
  Eigen::Matrix2d xy_covariance;
  const auto cov = pose_with_covariance.covariance;
  xy_covariance(0, 0) = cov[0 * 6 + 0];
  xy_covariance(0, 1) = cov[0 * 6 + 1];
  xy_covariance(1, 0) = cov[1 * 6 + 0];
  xy_covariance(1, 1) = cov[1 * 6 + 1];

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(xy_covariance);

  Ellipse ellipse;

  // eigen values and vectors are sorted in ascending order
  ellipse.long_radius = scale * std::sqrt(eigensolver.eigenvalues()(1));
  ellipse.short_radius = scale * std::sqrt(eigensolver.eigenvalues()(0));

  // principal component vector
  const Eigen::Vector2d pc_vector = eigensolver.eigenvectors().col(1);
  ellipse.yaw = std::atan2(pc_vector.y(), pc_vector.x());

  // ellipse size along lateral direction (body-frame)
  ellipse.P = xy_covariance;
  const double yaw_vehicle = tf2::getYaw(pose_with_covariance.pose.orientation);
  const Eigen::Matrix2d & p_inv = ellipse.P.inverse();
  Eigen::MatrixXd e(2, 1);
  e(0, 0) = std::cos(yaw_vehicle);
  e(1, 0) = std::sin(yaw_vehicle);
  const double d = std::sqrt((e.transpose() * p_inv * e)(0, 0) / p_inv.determinant());
  ellipse.size_lateral_direction = scale * d;

  return ellipse;
}
}  // namespace ndt_localization::localization_util

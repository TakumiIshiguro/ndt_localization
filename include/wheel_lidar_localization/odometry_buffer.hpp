#pragma once
#include <Eigen/Geometry>
#include <deque>
#include <optional>

namespace wheel_lidar_localization
{
using Pose = Eigen::Isometry3d;
bool validPose(const Pose & pose);
double rotationDistance(const Pose & a, const Pose & b);

struct OdomSample {double stamp; Pose pose;};
enum class OdomAdmission {Accepted, Invalid, OutOfOrder, Reset};

// Owner supplies synchronization. Interpolation never extrapolates.
class OdometryBuffer
{
public:
  OdometryBuffer(double duration, double max_gap, double reset_time,
    double max_speed, double max_rotation_speed);
  OdomAdmission add(double stamp, const Pose & pose);
  std::optional<Pose> at(double stamp) const;
  void clear();
private:
  std::deque<OdomSample> samples_;
  double duration_, max_gap_, reset_time_, max_speed_, max_rotation_speed_;
};
}  // namespace wheel_lidar_localization

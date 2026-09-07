#pragma once
#include "wheel_lidar_localization/scan_matcher.hpp"
#include <string>

namespace wheel_lidar_localization
{
enum class TrackingState {WaitingInitialPose, Initializing, Tracking, Degraded, Lost};
const char * stateName(TrackingState state);
struct MatchLimits
{
  double max_rmse, min_inlier_ratio, max_translation, max_rotation;
};
std::string rejectionReason(const MatchResult & result, const MatchLimits & limits);

// Accessed under the node mutex. Only accepted measurements update map_to_odom.
class LocalizationState
{
public:
  explicit LocalizationState(int failures_to_lost);
  void initialize(const Pose & pose);
  void invalidate();
  Pose seed(const Pose & odom) const;
  void accept(const Pose & pose, const Pose & odom, double stamp);
  void reject();
  void markLost();
  TrackingState state() const {return state_;}
  bool hasCorrection() const {return correction_valid_;}
  const Pose & correction() const {return map_to_odom_;}
  double acceptedStamp() const {return accepted_stamp_;}
  int failures() const {return failures_;}
private:
  TrackingState state_{TrackingState::WaitingInitialPose};
  Pose initial_pose_{Pose::Identity()}, map_to_odom_{Pose::Identity()};
  bool correction_valid_{false};
  int failures_{0}, failures_to_lost_;
  double accepted_stamp_{0};
};
}  // namespace wheel_lidar_localization

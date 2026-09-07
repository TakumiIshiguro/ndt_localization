#include "wheel_lidar_localization/localization_state.hpp"
#include <cmath>

namespace wheel_lidar_localization
{
const char * stateName(TrackingState state)
{
  switch (state) {
    case TrackingState::WaitingInitialPose: return "WAITING_INITIAL_POSE";
    case TrackingState::Initializing: return "INITIALIZING";
    case TrackingState::Tracking: return "TRACKING";
    case TrackingState::Degraded: return "DEGRADED";
    case TrackingState::Lost: return "LOST";
  }
  return "UNKNOWN";
}
std::string rejectionReason(const MatchResult & r, const MatchLimits & limits)
{
  if (!r.converged || !validPose(r.pose)) {return "ndt_not_converged_or_invalid_pose";}
  if (!std::isfinite(r.rmse) || r.rmse > limits.max_rmse) {return "rmse_too_large";}
  if (!std::isfinite(r.inlier_ratio) || r.inlier_ratio < limits.min_inlier_ratio) {
    return "insufficient_overlap";
  }
  if (!std::isfinite(r.translation_correction) || r.translation_correction > limits.max_translation) {
    return "translation_correction_too_large";
  }
  if (!std::isfinite(r.rotation_correction) || r.rotation_correction > limits.max_rotation) {
    return "rotation_correction_too_large";
  }
  return "";
}
LocalizationState::LocalizationState(int failures_to_lost) : failures_to_lost_(failures_to_lost) {}
void LocalizationState::initialize(const Pose & pose)
{
  initial_pose_ = pose;
  correction_valid_ = false;
  failures_ = 0;
  accepted_stamp_ = 0;
  state_ = TrackingState::Initializing;
}
void LocalizationState::invalidate()
{
  correction_valid_ = false;
  state_ = TrackingState::Lost;
}
Pose LocalizationState::seed(const Pose & odom) const
{
  return correction_valid_ ? map_to_odom_ * odom : initial_pose_;
}
void LocalizationState::accept(const Pose & pose, const Pose & odom, double stamp)
{
  if (state_ == TrackingState::Lost || state_ == TrackingState::WaitingInitialPose) {return;}
  map_to_odom_ = pose * odom.inverse();
  correction_valid_ = true;
  accepted_stamp_ = stamp;
  failures_ = 0;
  state_ = TrackingState::Tracking;
}
void LocalizationState::reject()
{
  if (state_ == TrackingState::Lost || state_ == TrackingState::WaitingInitialPose) {return;}
  ++failures_;
  state_ = failures_ >= failures_to_lost_ ? TrackingState::Lost :
    (correction_valid_ ? TrackingState::Degraded : TrackingState::Initializing);
}
void LocalizationState::markLost() {state_ = TrackingState::Lost;}
}  // namespace wheel_lidar_localization

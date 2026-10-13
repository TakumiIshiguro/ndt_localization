// Copyright 2022 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 参照元: autoware_core (commit e0a8ab84)
//   localization/autoware_ekf_localizer/src/utils/state_transition.hpp
// 変更点: 名前空間と include のパスだけ（処理は同じ）。

#ifndef UTILS__STATE_TRANSITION_HPP_
#define UTILS__STATE_TRANSITION_HPP_

#include "matrix_types.hpp"

namespace ndt_localization::ekf_localizer
{

double normalize_yaw(const double & yaw);
Vector6d predict_next_state(const Vector6d & X_curr, const double dt);
Matrix6d create_state_transition_matrix(const Vector6d & X_curr, const double dt);
Matrix6d process_noise_covariance(
  const double proc_cov_yaw_d, const double proc_cov_vx_d, const double proc_cov_wz_d);

}  // namespace ndt_localization::ekf_localizer

#endif  // UTILS__STATE_TRANSITION_HPP_

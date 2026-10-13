// Copyright 2023 Autoware Foundation
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
//   localization/autoware_ekf_localizer/src/utils/warning_message.hpp
// 変更点: 名前空間と include のパスだけ（処理は同じ）。

#ifndef UTILS__WARNING_MESSAGE_HPP_
#define UTILS__WARNING_MESSAGE_HPP_

#include <string>

namespace ndt_localization::ekf_localizer
{

std::string pose_delay_step_warning_message(
  const double delay_time, const double delay_time_threshold);
std::string twist_delay_step_warning_message(
  const double delay_time, const double delay_time_threshold);
std::string pose_delay_time_warning_message(const double delay_time);
std::string twist_delay_time_warning_message(const double delay_time);
std::string mahalanobis_warning_message(const double distance, const double max_distance);
std::string large_ekf_dt_waring_message(const double ekf_dt);
std::string too_slow_ekf_dt_waring_message(const double ekf_dt);

}  // namespace ndt_localization::ekf_localizer

#endif  // UTILS__WARNING_MESSAGE_HPP_

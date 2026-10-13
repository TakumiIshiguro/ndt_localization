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
//   localization/autoware_ekf_localizer/src/utils/numeric.hpp
// 変更点: 名前空間と include のパスだけ（処理は同じ）。

#ifndef UTILS__NUMERIC_HPP_
#define UTILS__NUMERIC_HPP_

#include <Eigen/Core>

#include <cmath>

namespace ndt_localization::ekf_localizer
{

inline bool has_inf(const Eigen::MatrixXd & v)
{
  return v.array().isInf().any();
}

inline bool has_nan(const Eigen::MatrixXd & v)
{
  return v.array().isNaN().any();
}

}  // namespace ndt_localization::ekf_localizer

#endif  // UTILS__NUMERIC_HPP_

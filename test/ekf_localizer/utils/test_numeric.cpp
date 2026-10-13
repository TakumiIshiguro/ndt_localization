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
//   localization/autoware_ekf_localizer/test/utils/test_numeric.cpp（テスト名を合わせている）
// 変更点: 名前空間と include のパスだけ（テストの中身は同じ）。

#include "utils/numeric.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <limits>

namespace ndt_localization::ekf_localizer
{

TEST(Numeric, has_nan)
{
  const Eigen::VectorXd empty(0);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::nan("");

  EXPECT_FALSE(has_nan(empty));
  EXPECT_FALSE(has_nan(Eigen::Vector3d(0., 0., 1.)));
  EXPECT_FALSE(has_nan(Eigen::Vector3d(1e16, 0., 1.)));
  EXPECT_FALSE(has_nan(Eigen::Vector3d(0., 1., inf)));

  EXPECT_TRUE(has_nan(Eigen::Vector3d(nan, 1., 0.)));
}

TEST(Numeric, has_inf)
{
  const Eigen::VectorXd empty(0);
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::nan("");

  EXPECT_FALSE(has_inf(empty));
  EXPECT_FALSE(has_inf(Eigen::Vector3d(0., 0., 1.)));
  EXPECT_FALSE(has_inf(Eigen::Vector3d(1e16, 0., 1.)));
  EXPECT_FALSE(has_inf(Eigen::Vector3d(nan, 1., 0.)));

  EXPECT_TRUE(has_inf(Eigen::Vector3d(0., 1., inf)));
}

}  // namespace ndt_localization::ekf_localizer

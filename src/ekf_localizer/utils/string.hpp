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
//   localization/autoware_ekf_localizer/src/utils/string.hpp
// 変更点: 名前空間と include のパスだけ（処理は同じ）。

#ifndef UTILS__STRING_HPP_
#define UTILS__STRING_HPP_

#include <string>

namespace ndt_localization::ekf_localizer
{

inline std::string erase_leading_slash(const std::string & s)
{
  std::string a = s;
  if (a.front() == '/') {
    a.erase(0, 1);
  }
  return a;
}

}  // namespace ndt_localization::ekf_localizer

#endif  // UTILS__STRING_HPP_

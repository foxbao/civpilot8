/******************************************************************************
 * Copyright 2022 The CIV Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once
#include <iterator>
namespace civ {
namespace drivers {
namespace common {
class Parser {
 public:
  Parser();
  ~Parser();
  void UpdateDataPtr(const std::uint8_t *data, std::size_t dataLen);
  std::size_t RemainSize() { return std::distance(data_, data_end_); }

 protected:
  const std::uint8_t *data_;
  const std::uint8_t *data_end_;
};
}  // namespace common
}  // namespace drivers
}  // namespace civ

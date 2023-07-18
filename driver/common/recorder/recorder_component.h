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
#include "cyber/cyber.h"
#include "cyber/tools/cyber_recorder/recorder.h"
namespace civ {
namespace drivers {
namespace common {
using apollo::cyber::Component;
class RecorderComponent {
 public:
  RecorderComponent() = default;
  ~RecorderComponent() = default;

 private:
};
}  // namespace common
}  // namespace drivers
}  // namespace civ

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
#include "modules/loc/fuse/fuser_base.hpp"

namespace civ {
namespace civloc {
class GnssFuser : public FuserBase {
 public:
  GnssFuser();
  bool IsSatReduceFast();
  bool IsSameFrame();
  // bool IsSatTooLess();
  bool ConfirmFrameValid(crsp_cZFrame frame) override;
  void CalcuteFixedCount();
  void ConfirmSolutionStatus(spState state) override;

 private:
  sp_cZGnss last_frame_ = nullptr;
  sp_cZGnss frame_ = nullptr;
  size_t continous_fixed_count_ = 0;
};
}  // namespace civloc
}  // namespace civ

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
#include <map>
#include <memory>
#include "modules/common/inner_types.hpp"
#include "modules/loc/fuse/measurement_base.hpp"
namespace civ {
namespace civloc {
class FuserBase {
 public:
  sp_cState DoFuse(sp_cState pre_state, crsp_cZFrame frame);
  std::map<MeasurementType, std::shared_ptr<MeasurementBase>>
      measurements_;  // 按量测类型进行排序
  virtual bool ConfirmFrameValid(crsp_cZFrame frame) { return true; }
  virtual void ConfirmSolutionStatus(spState state) {}

 private:
};
}  // namespace civloc
}  // namespace civ

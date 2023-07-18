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

#include "modules/loc/fuse/fuser_base.hpp"
#include <memory>
namespace civ {
namespace civloc {
sp_cState FuserBase::DoFuse(sp_cState pre_state, crsp_cZFrame frame) {
  auto fused_state = std::make_shared<State>(*pre_state);
  fused_state->fs_.reset();
  if (!this->ConfirmFrameValid(frame)) {
    return nullptr;
  }

  for (auto &one_measuement : measurements_) {
    auto &mt_type = one_measuement.first;
    auto &measurement = one_measuement.second;

    auto result = measurement->DoMeasurement(pre_state, frame, fused_state);
    // AINFO << UnixTimeString(frame->t0_) << " " << mt_type;
    if (result) {
      // 1.

      // auto aaa=static_cast<int>(mt_type);
      fused_state->fs_.set_state(mt_type);  // 追加融合标志
      // 2.
      this->ConfirmSolutionStatus(fused_state);  // 追加传感器状态
      // 3.
      pre_state =
          fused_state;  // 如果融合成功，上一个融合结果为为下一个融合的初值
    }
  }

  return fused_state;
}
}  // namespace civloc
}  // namespace civ

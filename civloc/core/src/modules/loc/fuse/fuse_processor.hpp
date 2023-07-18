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
#include <memory>
#include <unordered_map>
#include "modules/common/inner_types.hpp"
#include "modules/loc/fuse/fuser_base.hpp"
namespace civ {
namespace civloc {
class FuseProcessor {
 public:
  FuseProcessor();
  // 确定滤波器初始状态
  void ConfirmInitialState(spState state);
  // 对外接口
  sp_cState PredictAndDoFuse(crsp_cZFrame frame);
  // 获得最新的滤波器状态
  sp_cState get_lastest_fused_state() const { return lastest_fused_state_; }
  sp_cState get_lastest_predicted_state() const {
    return lastest_predicted_state_;
  }

  spState PredictStateMecanization(sp_cZImu imu);

 private:
  // 根据传感器消息与预测的状态，获得融合状态; 当无融合结果时，返回nullptr
  sp_cState DoFuse(crsp_cState pre_state, crsp_cZFrame frame);
  // 根据最新的预测状态，将预测状态更新到frame时刻, 预测间隔0.1ms
  sp_cState PredictStateAndCovariance(crsp_cZFrame frame);
  // 获得与当前预测状态最接近的IMU, 需要进行误差补偿
  sp_cZImu FetchNearestCompensatedImu(crsp_cState state);

 private:
  sp_cState lastest_predicted_state_ =
      nullptr;  // 最新的预测状态，保证状态与方差的更新频率为IMU的频率
  sp_cState lastest_fused_state_ = nullptr;  // 最新的融合状态
                                             // 同意管理各融合方法
  std::unordered_map<ZFrameType, std::shared_ptr<FuserBase>> fusers_;
  // sp_cState lastest_fused_state_bao_ = nullptr;     // 最新的融合状态
};
}  // namespace civloc
}  // namespace civ

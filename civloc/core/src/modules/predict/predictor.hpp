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
#include <vector>
#include "modules/common/inner_types.hpp"
namespace civ {
namespace civloc {
class Predictor {
 public:
  // spState DoPredict(spState pre_state, sp_cZCnState cgi610);  //
  // 根据最新的refine_state,获取当前帧产生时刻的状态 spState DoPredict(double
  // time_s);              // 根据时间预测最新的状态
  spState DoPredict(
      crsp_cZFrame frame);  // 根据最新的refine_state,获取当前帧产生时刻的状态
  spState
  // 根据最新的refine_state,获取最新主传感器状态
  DoPredictUntilLastestMainSensor();  
  // spState PredictStateMecanization(spState filter_state, sp_cZImu imu);
};
}  // namespace civloc
}  // namespace civ

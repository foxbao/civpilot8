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

#include "modules/common/inner_types.hpp"
namespace civ {
namespace civloc {

class Detector {
 public:
  sp_cZVehicleModel Detect(
      crsp_cZImu frame);  // 根据最新的IMU以及cacher数据,进行车辆状态检测
 private:
  bool StaticCheck();  // 静止检测,返回当次静止检测结果
  void AnalyzeDetectResult(
      sp_cZVehicleModel vm_frame);  // 分析各个检测指标零速检测结果
 private:
  spZVehicleModel vm_ = nullptr;  // 车辆状态
  int static_index = 0;           // 连续静止计数
  double gyr_stat[3] = {0, 0, 0};  // 陀螺仪相关检测指标{w_mean, w_std, ARE}
  double acc_stat[3] = {0, 0, 0};  // 加速度相关检测指标{MAG, MV, GLRT}
};

}  // namespace civloc
}  // namespace civ

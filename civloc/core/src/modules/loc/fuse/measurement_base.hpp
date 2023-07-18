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
#include "modules/loc/fuse/filter.hpp"

namespace civ {
namespace civloc {
class MeasurementBase {
 protected:
  virtual bool ConfirmValidity() = 0;            //  确认数据的有效性
  virtual void AdaptNoise() {}                   //  调整量测噪声
  virtual void ComputeJacobian() = 0;            //  计算雅克比
  virtual bool ChiSquareTest() { return true; }  //
  virtual bool DeltaXTest() { return true; }  //  根据修正量是否接收该量测

 private:
  void ComputeChiSquareValue();  //  计算卡方值
  void ComputeDeltaX();          //  计算修正量
  void Update();                 //  更新状态和方差

 public:
  spState DoMeasurement(crsp_cState pre_state, crsp_cZFrame frame,
                        spState fused_state);

 protected:
  sp_cState pre_state_ = nullptr;  // 当前预测状态
  spState fused_state_ = nullptr;  // 融合状态

 private:
  sp_cZFrame base_frame_ = nullptr;  // 传感器数据帧

 protected:
  // 滤波相关状态
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::VectorXd Z_;
  Eigen::VectorXd dx_;

  double chi_square_value_ = 0;
};
}  // namespace civloc
}  // namespace civ

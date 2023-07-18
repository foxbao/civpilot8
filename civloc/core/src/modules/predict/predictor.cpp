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

#include "modules/predict/predictor.hpp"
#include <iomanip>
#include <iostream>
#include <memory>
// #include "modules/common/units.hpp"
#include "common/coordinate_transform/earth.hpp"
#include "modules/common/error_model.hpp"
#include "modules/common/time_system.hpp"

#include "modules/common/utils.hpp"
#include "modules/message_cache/cacher.hpp"
#include "modules/state_server/state_server.hpp"
namespace civ {
namespace civloc {
// 根据最新的refine_state,获取当前帧产生时刻的状态
spState Predictor::DoPredict(crsp_cZFrame frame) {
  spState predicted_state = std::make_shared<State>();

  predicted_state->t0_ = frame->t0_;
  return predicted_state;
}

spState Predictor::DoPredictUntilLastestMainSensor() {
  //
  using namespace civ::common::coord_transform;
  auto const &lastest_fused_state = g_state_server->get_lastest_state();
  if (!lastest_fused_state) {
    AERROR << "Predictor g_state_server->get_lastest_state() is empty";
    return nullptr;
  }

  auto const &imus =
      g_cacher->GetRemainingMainChannelFrames(lastest_fused_state->t0_);
  //
  if (imus.empty()) {
    AERROR << "Predictor g_cacher->GetRemainingMainChannelFrames() is empty";
    return nullptr;
  }

  spState predict_state = std::make_shared<State>(*lastest_fused_state);
  double current_t0 = imus.back()->t0_;
  // 补偿误差
  ZImu mean_imu_raw = {};
  for (auto const &imu : imus) {
    auto const &one_imu = std::dynamic_pointer_cast<ZImu const>(imu);
    mean_imu_raw.acc_ += one_imu->acc_, mean_imu_raw.gyr_ += one_imu->gyr_;
  }
  mean_imu_raw.acc_ /= imus.size(), mean_imu_raw.gyr_ /= imus.size();
  spZImu mean_imu =
      Compenstate(mean_imu_raw, lastest_fused_state->imu_in_para_);
  // dt
  double dt = current_t0 - lastest_fused_state->t0_;
  // 机械编排
  Eigen::Vector3d ang_inc = mean_imu->gyr_ * dt;
  // Vector3d vel_inc = mean_imu->acc_ * gl_g0 * dt;
  Eigen::Vector3d vel_inc =
      mean_imu->acc_ * dt;  // pay attention here, no need to multiply by gl_g0,
                            // as the data is m/s2
  Eigen::Vector3d wnie = Earth::GetWnie(lastest_fused_state->pos_),
                  wnen = Earth::GetWnen(lastest_fused_state->pos_,
                                        lastest_fused_state->vel_);

  // 速度更新
  Eigen::Vector3d wnin = wnie + wnen;
  Eigen::Quaterniond q_nn1 = rv2q(-wnin * dt);
  Eigen::Vector3d dvn = q_nn1 * lastest_fused_state->qua_ * vel_inc;
  Eigen::Vector3d dv_gcor = (-(wnie + wnin).cross(lastest_fused_state->vel_) +
                             Earth::GetGn(lastest_fused_state->pos_)) *
                            dt;  // 重力及哥氏加速度
  Eigen::Vector3d vel_last = lastest_fused_state->vel_;
  predict_state->vel_ = lastest_fused_state->vel_ + dvn + dv_gcor;

  // 位置更新
  Eigen::Matrix3d cen = Earth::Pos2Cne(lastest_fused_state->pos_).transpose();
  predict_state->pos_ =
      Earth::ECEF2LLH(Earth::LLH2ECEF(lastest_fused_state->pos_) +
                      cen * (vel_last + predict_state->vel_) / 2 * dt);

  // 姿态更新
  predict_state->qua_ = q_nn1 * lastest_fused_state->qua_ * rv2q(ang_inc);

  // 时间更新
  predict_state->t0_ = current_t0;
  predict_state->t1_ = current_t0;

  return predict_state;
}


}  // namespace civloc
}  // namespace civ

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
#include "modules/loc/fuse/fuse_processor.hpp"
#include <memory>
#include "common/coordinate_transform/earth.hpp"
#include "common/coordinate_transform/units.hpp"
#include "modules/common/error_model.hpp"

#include "modules/message_cache/cacher.hpp"
#include "modules/state_server/state_server.hpp"
//
#include "common/coordinate_transform/coordinate_transform_core.h"
#include "modules/common/utils.hpp"
#include "modules/loc/fuse/filter.hpp"
#include "modules/loc/fuse/gnss/fuser.hpp"

namespace civ {
namespace civloc {
FuseProcessor::FuseProcessor() {
  if (g_config->has_fuse_cfg() &&
      g_config->fuse_cfg().mode() == FuseConfig_FuseMode_ESKF) {
    g_eskf = std::make_shared<EKF>();
    fusers_[ZFrameType::GNSS_ALL] = std::make_shared<GnssFuser>();
  }
}

sp_cState FuseProcessor::PredictAndDoFuse(crsp_cZFrame frame) {
  sp_cState predict_state = PredictStateAndCovariance(frame);
  sp_cState fused_result = DoFuse(predict_state, frame);
  // sp_cState fused_result = predict_state;
  // std::cout<<"pos in fuser"<<fused_result->pos_.transpose()<<std::endl;
  return fused_result;
}

sp_cState FuseProcessor::DoFuse(crsp_cState pre_state, crsp_cZFrame frame) {
  // 处理该类数据
  sp_cState fused_result = nullptr;
  if (fusers_.find(frame->type_) != fusers_.end()) {
    fused_result = fusers_[frame->type_]->DoFuse(pre_state, frame);
  }

  // 未完成融合
  if (!fused_result) {
    return nullptr;
  }
  // ! 融合完成，更新最新的融合状态
  lastest_predicted_state_ = lastest_fused_state_ =
      fused_result;  // 存在新的融合结果，则更新最新的lvbo
  return fused_result;
  // return pre_state;
}

sp_cState FuseProcessor::PredictStateAndCovariance(crsp_cZFrame frame) {
  using namespace civ::common::coord_transform;
  // double dt = frame->t0_ - lastest_predicted_state_->t0_;
  // g_eskf->PredictStateAndCovarianceByIMU(lastest_predicted_state_, frame,
  // imu_compensated, predicted_state); 从最新的预测状态开始

  // sp_cZImu* derived_ptr = static_cast<sp_cZImu*>(frame);
  Eigen::Vector3d latetest_enu = get_enu_pos(lastest_predicted_state_->pos_);

  // std::cout << "lastest_predicted_state_ time:" <<
  // lastest_predicted_state_->t0_ << std::endl; std::cout << latetest_enu(0) <<
  // " " << std::endl; std::cout << latetest_enu(1) << " " << std::endl;
  // std::cout << latetest_enu(2) << " " << std::endl;

  spState predicted_state = std::make_shared<State>(*lastest_predicted_state_);
  sp_cZImu imu_compensated =
      FetchNearestCompensatedImu(lastest_predicted_state_);
  g_eskf->PredictStateAndCovarianceByIMU(lastest_predicted_state_, frame,
                                         imu_compensated, predicted_state);
  // AINFO << "PredictStateAndCovarianceByIMU done";

  lastest_predicted_state_ = predicted_state;  // 记录本次预测状态
  // std::cout << "predict_state time:" << predicted_state->t0_ << std::endl;
  // Eigen::Vector3d predicted_enu=get_enu_pos(predicted_state->pos_);
  // std::cout << predicted_enu(0) << " " << std::endl;
  // std::cout << predicted_enu(1) << " " << std::endl;
  // std::cout << predicted_enu(2) << " " << std::endl;
  // AWARN << frame->type_ << " EKF::PredictStateByIMU dt_ms = " << s2ms(dt);
  return lastest_predicted_state_;
}

sp_cZImu FuseProcessor::FetchNearestCompensatedImu(crsp_cState state) {
  sp_cZImu imu = std::dynamic_pointer_cast<ZImu const>(
      g_cacher->GetNearestMainChannelFrames(state->t0_));
  // std::cout<<"state time before compensate"<<state->t0_<<std::endl;
  // std::cout<<"gyro time before compensate"<<imu->t0_<<std::endl;
  if (!imu) {
    AERROR << " g_cacher not contain imu";
    exit(0);  // 严重错误
    return nullptr;
  }
  sp_cZImu imu_compensated = Compenstate(*imu, state->imu_in_para_);
  return imu_compensated;
}

void FuseProcessor::ConfirmInitialState(spState state) {
  // 滤波器确定初始方差
  g_eskf->ConfirmInitialCovariance(state);
  // 确定最新融合状态
  lastest_predicted_state_ = lastest_fused_state_ = state;
}

spState FuseProcessor::PredictStateMecanization(sp_cZImu imu) {
  using namespace civ::common::coord_transform;
  auto filter_state = lastest_fused_state_;
  spState predicted_state = std::make_shared<State>();
  double dt = imu->t0_ - filter_state->t0_;

  Eigen::Vector3d wnie = Earth::GetWnie(filter_state->pos_),
                  wnen = Earth::GetWnen(filter_state->pos_, filter_state->vel_);
  // Vector3d const vel_inc = imu->acc_ * gl_g0 * dt; // 可以使用差值结果
  Eigen::Vector3d const vel_inc = imu->acc_ * dt;  // 可以使用差值结果

  Eigen::Vector3d const ang_inc =
      imu->gyr_ * dt;  // 可以使用差值结果  //z轴结果进行了调整
  // 速度更新
  Eigen::Vector3d wnin = wnie + wnen;
  Eigen::Quaterniond q_nn1 = rv2q(-wnin * dt);
  Eigen::Vector3d dvn = q_nn1 * filter_state->qua_ * vel_inc;
  Eigen::Vector3d dv_gcor = (-(wnie + wnin).cross(filter_state->vel_) +
                             Earth::GetGn(filter_state->pos_)) *
                            dt;  // 重力及哥氏加速度
  Eigen::Vector3d vel_last = filter_state->vel_;
  predicted_state->vel_ = filter_state->vel_ + (dvn + dv_gcor);

  // 位置更新
  Eigen::Matrix3d cen = Earth::Pos2Cne(filter_state->pos_).transpose();
  predicted_state->pos_ =
      Earth::ECEF2LLH(Earth::LLH2ECEF(filter_state->pos_) +
                      cen * (vel_last + predicted_state->vel_) / 2 * dt);

  // 姿态更新

  predicted_state->qua_ = q_nn1 * filter_state->qua_ * rv2q(ang_inc);
  predicted_state->qua_.normalize();

  // 时间更新
  predicted_state->t0_ = imu->t0_;
  lastest_fused_state_ = predicted_state;
  return predicted_state;
}
}  // namespace civloc
}  // namespace civ

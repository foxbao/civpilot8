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

#include "modules/init/initializer.hpp"
#include <memory>
#include "modules/common/converter.hpp"

#include "modules/common/time_system.hpp"
#include "modules/common/utils.hpp"
#include "common/coordinate_transform/earth.hpp"
#include "common/coordinate_transform/LocalCartesian_util.h"
namespace civ {
namespace civloc {
Initializer::Initializer() { align_state_ = std::make_shared<State>(); }

bool Initializer::DoAlign(crsp_cZFrame named_frame) {
  // 只有CGI610出固定解的情况下对准
  if (status_ == AlignStatus::Aligned) {
    return true;
  }
  if (named_frame->type_ != ZFrameType::CGI610) {
    return false;
  }

  // 具体对准操作

  auto const &cgi610_state =
      std::dynamic_pointer_cast<ZCnState const>(named_frame);
  if (cgi610_state->sat_status_ !=
      civ::civloc::CHC_SATELLITE_STATUS::RTK_FIXED) {
    return false;
  }
  
  ConfirmInitialState(cgi610_state);

  // // 给定初始状态噪声
  status_ = AlignStatus::Aligned;

  AINFO << "Initializer::DoAlign done. t0 = "
        << UnixTimeString(align_state_->t0_);
  return (status_ == AlignStatus::Aligned);
}

// frame 为车体系位姿
void Initializer::ConfirmInitialState(crsp_cZCnState frame) {
  using namespace civ::common::coord_transform;
  // a v p
  align_state_ = Convert(frame);
  // 确定初始外参数
  // 所有外参
  if (g_sensor_config->imu().size() > 0) {
    auto const &imu_ex = g_sensor_config->imu(0).ex_para();
    align_state_->ex_para_["imu"] = Expara2Isometry3d(imu_ex);
  }
  if (g_sensor_config->gnss().size() > 0) {
    auto const &gnss_ex = g_sensor_config->gnss(0).ex_para();
    align_state_->ex_para_["gnss"] = Expara2Isometry3d(gnss_ex);
  }
  // optional
  if (g_sensor_config->perc().size() > 0) {
    auto const &perc_ex = g_sensor_config->perc(0).cam_para().ex_para();
    align_state_->ex_para_["perc"] = Expara2Isometry3d(perc_ex);
  }
  // optional
  if (g_sensor_config->has_lidar()) {
    auto const &lidar_ex = g_sensor_config->lidar().ex_para();
    align_state_->ex_para_["lidar"] = Expara2Isometry3d(lidar_ex);
  }

  // 将车体系的位姿转换至imu中心处
  AWARN << "Initializer::ConfirmInitialState align_state_->pos_ = "
        << align_state_->pos_.transpose();
  Eigen::Vector3d l_vi_n =
      align_state_->qua_ * align_state_->ex_para_["imu"].translation();
  align_state_->pos_ = Earth::PlusDeltaEnuAtPos(align_state_->pos_, l_vi_n);
  AWARN << "Initializer::ConfirmInitialState align_state_->pos_ done = "
        << align_state_->pos_.transpose();
  // ba
  align_state_->imu_in_para_.ba_ = {0, 0, 0};
  // bg
  align_state_->imu_in_para_.bg_ = {0, 0, 0};
  // skew
  align_state_->imu_in_para_.gyr_skew_ =
      Eigen::Map<Eigen::Matrix3d const>(
          g_sensor_config->imu(0).gyr().skew().data())
          .transpose();
  align_state_->imu_in_para_.acc_skew_ =
      Eigen::Map<Eigen::Matrix3d const>(
          g_sensor_config->imu(0).acc().skew().data())
          .transpose();
  // odo ko
  if (g_sensor_config->wheel().ko().size() > 0) {
    align_state_->odo_in_para_ =
        Eigen::Map<Eigen::Vector4d const>(g_sensor_config->wheel().ko().data());
  } else {
    align_state_->odo_in_para_ = {1, 1, 1, 1};
  }
  // AERROR << "Initializer::ConfirmInitialState vel = " <<
  // align_state_->vel_.transpose();
}

}  // namespace civloc
}  // namespace civ

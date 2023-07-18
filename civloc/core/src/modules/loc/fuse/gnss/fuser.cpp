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

#include <memory>
#include "modules/loc/fuse/gnss/fuser.hpp"
#include "modules/loc/fuse/gnss/gnss_position_measurement.hpp"
#include "modules/message_cache/cacher.hpp"

namespace civ {
namespace civloc {
GnssFuser::GnssFuser() {
  //
  // measurements_[MeasurementType::GNSS_VEL] =
  // std::make_shared<GnssVelocityMeasurement>(frame_);
  measurements_[MeasurementType::GNSS_POS] =
      std::make_shared<GnssPositionMeasurement>(frame_);

  // measurements_[MeasurementType::GNSS_DUAL_ANTENNA] =
  // std::make_shared<GnssDualAntennaMeasurement>(frame_);

  // std::make_shared<GnssPositionMeasurement>();
  // GnssPositionMeasurement aaa(frame_);
}

bool GnssFuser::IsSatReduceFast() {
  // auto gnss_cacher = g_cacher->GetNamedBuff2()["/zhito/rawcgi610/bestpos"];
  auto gnss_cacher = g_cacher->GetNamedBuff2()["/GNGGA"];
  // AERROR << "gnss_cacher size: " << gnss_cacher.size();
  auto const &gnss_cfg = g_config->fuse_cfg().gnss_filter();
  int min_sat_num = frame_->sat_num_;
  int max_sat_num = frame_->sat_num_;
  // 判断1.5s以内的数据是否存在卫星数目急速下降
  for (auto const &one_frame : gnss_cacher) {
    std::shared_ptr<ZGnss const> temp_gnss_frame =
        std::dynamic_pointer_cast<ZGnss const>(one_frame.second);
    if ((frame_->t0_ - temp_gnss_frame->t0_) > gnss_cfg.farme_judge_time_s()) {
      continue;
    }
    if (min_sat_num > temp_gnss_frame->sat_num_) {
      min_sat_num = temp_gnss_frame->sat_num_;
    }

    if (max_sat_num < temp_gnss_frame->sat_num_) {
      max_sat_num = temp_gnss_frame->sat_num_;
    }
  }
  float sat_change_ratio = 1;
  if (min_sat_num == 0) {
    // 如果当前的卫星数目为0则直接拒绝当前帧
    AERROR << "Min_sat_num = 0! " << UnixTimeString(frame_->t0_);
    return true;
  }

  // 最大卫星数为0就直接剔除了
  // if (max_sat_num != 0) {
  sat_change_ratio = (static_cast<float>(max_sat_num - min_sat_num)) /
                     static_cast<float>(min_sat_num);
  // }
  bool is_reduce_fast = sat_change_ratio > gnss_cfg.max_change_ratio();
  if (!is_reduce_fast) {
    AINFO << "max_sat_num: " << max_sat_num << " min_sat_num: " << min_sat_num
          << " start_time: " << UnixTimeString(gnss_cacher.begin()->second->t0_)
          << " end_time: " << UnixTimeString(gnss_cacher.rbegin()->second->t0_)
          << " Current Time: " << UnixTimeString(frame_->t0_)
          << " sat_num_change_ratio:" << sat_change_ratio * 100 << "%";
    return false;
  }
  AINFO << "max_sat_num: " << max_sat_num << " min_sat_num: " << min_sat_num
        << " start_time: " << UnixTimeString(gnss_cacher.begin()->second->t0_)
        << " end_time: " << UnixTimeString(gnss_cacher.rbegin()->second->t0_)
        << " Current Time: " << UnixTimeString(frame_->t0_)
        << " sat_num_change_ratio:" << sat_change_ratio * 100 << "%";
  return true;
}

bool GnssFuser::IsSameFrame() {
  using namespace civ::common::coord_transform;
  bool is_same_pos = (frame_->pos_ - last_frame_->pos_).norm() < (1e-9);
  bool is_move = frame_->vel_.norm() > 0.1_mps;
  last_frame_ = frame_;
  bool is_repeated = is_same_pos && is_move;
  // if (is_repeated) {
  //   AERROR << "找到连续帧 " << UnixTimeString(frame_->t0_);
  // }

  if (!is_repeated) {
    return false;
  }
  return true;
}

void GnssFuser::CalcuteFixedCount() {
  if (frame_->status_ == 4) {
    continous_fixed_count_++;
  } else {
    continous_fixed_count_ = 0;
  }
}

bool GnssFuser::ConfirmFrameValid(crsp_cZFrame frame) {
  frame_ = std::dynamic_pointer_cast<ZGnss const>(frame);
  if (!last_frame_) {
    last_frame_ = std::dynamic_pointer_cast<ZGnss const>(frame);
    return true;
  }

  // !去除status=0的数据
  if (frame_->status_ == 0) {
    return false;
  }
  // 计算连续固定帧
  this->CalcuteFixedCount();

  // todo 丢帧数据如何控制
  // 判断距离现在1.5秒的数据内卫星数是否骤变
  bool is_reduce_fast = IsSatReduceFast();
  if (is_reduce_fast) {
    return false;
  }
  // 判断是否为重复帧
  bool is_repeated = IsSameFrame();
  if (is_repeated) {
    AERROR << UnixTimeString(frame->t0_) << " 当前帧为重复帧。当前帧判断无效 "
           << continous_fixed_count_;
    return false;
  }
  return true;
}

void GnssFuser::ConfirmSolutionStatus(spState state) {
  // gnss 解状态
  state->status_ = frame_->status_;
}

}  // namespace civloc
}  // namespace civ

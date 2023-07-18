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
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "modules/common/inner_types.hpp"
#include "modules/common/time_system.hpp"
#include "modules/common/utils.hpp"

namespace civ {
namespace civloc {

/**
 * 传感器数据缓存模块
 * 主要功能: 1, 数据缓存与排序
 * 主要功能: 2, 各类数据丢帧与时间戳检查
 * */
class Cacher {
 public:
  Cacher();
  template <typename T>
  std::deque<sp_cZFrame> push_back(crsp_c(T) msg);  // 最终加入buff
  std::deque<sp_cZFrame> GetRemainingMainChannelFrames(
      double const start_t0_s);  // 获得 (start_t0_s, 最新]的主传感器输数据
  sp_cZFrame GetNearestMainChannelFrames(
      double const t0_s);  // 获得距离最近t0_s的主传感器数据
  std::deque<sp_cZImu> getMainImuInRange(
      double const t0_s, double const dt_s);  // 获得 [t0_s - dts,t0_s - dts]
  sp_cZFrame GetNearestFrame(
      double const start_t0_s,
      const std::string
          &frame_name);  // 获得离start_t0_s 最近的指定传感器的数据帧
  // 或者差值后的组合导航结果
  sp_cZCnState GetInterpolatedCnState(
      double const t0_s,
      crString frame_name);  // 获得差值到start_t0_s的frame_name通道的数据

 private:
  std::deque<sp_cZFrame> GetSampleTimeOrderedFrames();
  std::deque<sp_cZFrame> GetSampleTimeOrderedFramesBao();

 private:
  std::map<std::string, std::map<size_t, sp_cZFrame>>
      named_buff_;  // 缓存duration_s_的所有数据
  std::mutex mtx_named_buff_;
  double last_recieve_time_s_ = -1;
  double current_recieve_time_s_ = -1;
  // config
 private:
  std::string main_channel_name_ = "";  // 延时最低的主传感器
  double duration_s_ = 1.5;
  double max_delay_ms_ = 1000;

 public:
  std::map<std::string, std::deque<sp_cZFrame>> GetNamedBuff();
  std::map<std::string, std::map<size_t, sp_cZFrame>> GetNamedBuff2();
  double GetDurationS() { return duration_s_; }
};
DEFINE_EXTEND_TYPE(Cacher);

template <typename T>
std::deque<sp_cZFrame> Cacher::push_back(crsp_c(T) msg) {
  double dt = 0;
  // 加锁
  {
    std::lock_guard<std::mutex> lock(mtx_named_buff_);

    current_recieve_time_s_ = msg->t1_;
    size_t t0_ms = s2ms(msg->t0_);

    // null check
    std::map<size_t, sp_cZFrame> &single_buff = named_buff_[msg->channel_name_];
    single_buff[t0_ms] = msg;

    //! 缓存1.5秒的数据
    dt = single_buff.rbegin()->second->t0_ -
         single_buff.begin()->second->t0_;  // 当前buffer缓存
    if (dt >= duration_s_) {
      size_t remove_ms = s2ms(single_buff.rbegin()->second->t0_ - duration_s_);
      single_buff.erase(single_buff.begin(),
                        single_buff.lower_bound(remove_ms));
    }
  }

  // to be deleted
  // return GetSampleTimeOrderedFrames();
  // return GetSampleTimeOrderedFramesBao();

  //

  // 等待缓存大于max_delay_ms_的数据，只有此时才
  if (main_channel_name_ == msg->channel_name_ && (dt * 1000 > max_delay_ms_)) {
    return GetSampleTimeOrderedFrames();
  } else {
    return std::deque<sp_cZFrame>();
  }
}

}  // namespace civloc
}  // namespace civ

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

#include "modules/message_cache/cacher.hpp"
#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include "modules/common/utils.hpp"
namespace civ {
namespace civloc {
Cacher::Cacher() {
  auto const &cfg = g_config->cacher_cfg();
  duration_s_ = cfg.duration_s();
  max_delay_ms_ = cfg.max_delay_ms();
  main_channel_name_ = cfg.main_channel_name();

  std::cout << "Cacher init done. duration_s = " << duration_s_
            << " max_delay_ms = " << max_delay_ms_
            << " main_channel_name_ = " << main_channel_name_ << std::endl;
}

std::deque<sp_cZFrame> Cacher::GetSampleTimeOrderedFramesBao() {
  std::deque<sp_cZFrame> result;

  if (!named_buff_.count(main_channel_name_)) {
    return result;
  }

  std::map<std::string, std::map<size_t, sp_cZFrame>> buffs = {};
  {
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    buffs = named_buff_;
  }

  if (buffs.empty()) {
    return result;
  }

  int empty_n = 0;
  for (auto buffer : buffs) {
    if (buffer.second.empty()) {
      empty_n += 1;
    }
  }
  if (empty_n == buffs.size()) {
    return result;
  }

  // 从大到小排序, top为最小值
  std::priority_queue<sp_cZFrame, std::vector<sp_cZFrame>,
                      std::greater<sp_cZFrame>>
      ordered_frame;
  // 第一次每个buff个取一个元素
  for (auto &one_chanel : buffs) {
    auto &single_buff = one_chanel.second;
    if (!single_buff.empty()) {
      ordered_frame.push(single_buff.begin()->second);
      single_buff.erase(single_buff.begin());
    }
  }
  // 最小元素所在通道
  while (!ordered_frame.empty()) {
    // 每个单独buff的第一个元素加入到priority_queue
    result.push_back(ordered_frame.top());
    ordered_frame.pop();

    // 追加下一个元素
    auto &poped_buff = buffs[result.back()->channel_name_];
    if (!poped_buff.empty()) {
      ordered_frame.push(poped_buff.begin()->second);
      poped_buff.erase(poped_buff.begin());
    }
  }

  return result;
}

std::deque<sp_cZFrame> Cacher::GetSampleTimeOrderedFrames() {
  std::deque<sp_cZFrame> result;
  double current_sampling_time =
      named_buff_[main_channel_name_].rbegin()->second->t0_ -
      max_delay_ms_ * 0.001;                          // 最新的主传感器
  double last_recieve_time_s = last_recieve_time_s_;  // 上一次的主传感器
  size_t current_sampling_time_ms = s2ms(current_sampling_time);
  size_t last_recieve_time_ms = s2ms(last_recieve_time_s);
  // 获得buff拷贝
  std::map<std::string, std::map<size_t, sp_cZFrame>> buffs = {};
  {
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    buffs = named_buff_;
  }
  // buffs 空时不排序
  if (buffs.empty()) {
    return result;
  }
  // buffs中删除不满足条件元素
  for (auto &one_chanel : buffs) {
    auto &single_buff = one_chanel.second;

    // 待被删除的迭代器,只保留 (last_recieve_time_s, current_sampling_time]
    if (last_recieve_time_s_ != -1) {
      single_buff.erase(single_buff.begin(),
                        single_buff.upper_bound(last_recieve_time_ms));
    }
    single_buff.erase(single_buff.upper_bound(current_sampling_time_ms),
                      single_buff.end());
  }
  // buffs全空退出

  // int empty_n = std::count_if(buffs.begin(), buffs.end(), [](std::map<size_t,
  // sp_cZFrame> const &buff)
  //                             { return buff.second.empty(); });
  int empty_n = 0;
  for (auto buffer : buffs) {
    if (buffer.second.empty()) {
      empty_n += 1;
    }
  }
  if (empty_n == buffs.size()) {
    return result;
  }
  // 从大到小排序, top为最小值
  std::priority_queue<sp_cZFrame, std::vector<sp_cZFrame>,
                      std::greater<sp_cZFrame>>
      ordered_frame;
  // 第一次每个buff个取一个元素
  for (auto &one_chanel : buffs) {
    auto &single_buff = one_chanel.second;
    if (!single_buff.empty()) {
      ordered_frame.push(single_buff.begin()->second);
      single_buff.erase(single_buff.begin());
    }
  }
  // 最小元素所在通道
  while (!ordered_frame.empty()) {
    // 每个单独buff的第一个元素加入到priority_queue
    result.push_back(ordered_frame.top());
    ordered_frame.pop();

    // 追加下一个元素
    auto &poped_buff = buffs[result.back()->channel_name_];
    if (!poped_buff.empty()) {
      ordered_frame.push(poped_buff.begin()->second);
      poped_buff.erase(poped_buff.begin());
    }
  }

  // for (auto const& frame : result) {
  //   AINFO<<"ordered frame " << frame;
  // }
  // AERROR << "!!!!!!!!!!!! GetSampleTimeOrderedFrames2 " << result.size();
  // AERROR << "!!!!!!!!!!!! GetSampleTimeOrderedFrames2 begin " <<
  // UnixTimeString(current_sampling_time); AERROR << "!!!!!!!!!!!!
  // GetSampleTimeOrderedFrames2 end " << UnixTimeString(last_recieve_time_s);
  // 记录上一次抛出的t0
  last_recieve_time_s_ = current_sampling_time;

  return result;
}

std::deque<sp_cZFrame> Cacher::GetRemainingMainChannelFrames(
    double const start_t0_s) {
  std::lock_guard<std::mutex> lock(mtx_named_buff_);
  //
  std::deque<sp_cZFrame> result;
  auto &buff = named_buff_[main_channel_name_];
  size_t t0_ms = s2ms(start_t0_s);
  for (auto it = buff.upper_bound(t0_ms); it != buff.end(); ++it) {
    result.push_back(it->second);
  }
  return result;
}
sp_cZFrame Cacher::GetNearestMainChannelFrames(double const start_t0_s) {
  std::lock_guard<std::mutex> lock(mtx_named_buff_);
  auto &buff = named_buff_[main_channel_name_];
  size_t t0_ms = s2ms(start_t0_s);
  auto it = buff.lower_bound(t0_ms);
  // return main_buff.empty() ? nullptr : main_buff.front();
  AINFO_IF(it != buff.end() && fabs(s2ms(it->second->t0_ - start_t0_s)) > 10)
      << " request " << UnixTimeString(start_t0_s) << " got "
      << UnixTimeString(it->second->t0_)
      << " delta = " << s2ms(it->second->t0_ - start_t0_s);
  return it == buff.end() ? nullptr : it->second;
}

// 获得 [t0_s - dts,t0_s - dts]
std::deque<sp_cZImu> Cacher::getMainImuInRange(double const t0_s,
                                               double const dt_s) {
  std::deque<sp_cZImu> result = {};
  std::lock_guard<std::mutex> lock(mtx_named_buff_);
  auto &buff = named_buff_[main_channel_name_];
  size_t t0_ms = s2ms(t0_s);
  size_t dt_ms = s2ms(dt_s);
  auto it_start = buff.lower_bound(t0_ms - dt_ms);
  auto it_end = buff.lower_bound(t0_ms + dt_ms);
  // bad
  if (it_start == buff.end()) {
    AERROR << "Cacher::getMainImuInRange failed "
           << UnixTimeString(t0_s - dt_s);
    return result;
  }
  // normal
  for (auto it = it_start; it != it_end; ++it) {
    result.push_back(std::dynamic_pointer_cast<ZImu const>(it->second));
  }
  return result;
}

std::map<std::string, std::deque<sp_cZFrame>> Cacher::GetNamedBuff() {
  std::lock_guard<std::mutex> lock(mtx_named_buff_);

  std::map<std::string, std::deque<sp_cZFrame>> result;

  for (auto const &one_buff : named_buff_) {
    for (auto const &frame : one_buff.second) {
      result[one_buff.first].push_back(frame.second);
    }
  }

  return result;
}

std::map<std::string, std::map<size_t, sp_cZFrame>> Cacher::GetNamedBuff2() {
  std::lock_guard<std::mutex> lock(mtx_named_buff_);

  return named_buff_;
}

sp_cZFrame Cacher::GetNearestFrame(double const start_t0_s,
                                   const std::string &frame_name) {
  std::lock_guard<std::mutex> lock(mtx_named_buff_);

  auto &buff = named_buff_[frame_name];
  if (buff.size() == 0) return nullptr;
  size_t t0_ms = s2ms(start_t0_s);
  auto it = buff.lower_bound(t0_ms);
  return it == buff.end() ? std::prev(it)->second : it->second;
}

sp_cZCnState Cacher::GetInterpolatedCnState(double const t0_s,
                                            crString frame_name) {
  std::map<size_t, sp_cZFrame> buff;
  {
    std::lock_guard<std::mutex> lock(mtx_named_buff_);
    buff = named_buff_[frame_name];
  }
  if (buff.empty()) {
    return nullptr;
  }
  // 查找差值时刻
  size_t t0_ms = s2ms(t0_s);

  auto it = buff.lower_bound(t0_ms);
  // 没找到元素,检查最后一个是否满足
  if (it == buff.end()) {
    if (labs(std::prev(it)->first - t0_ms) >= 15) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<ZCnState const>(std::prev(it)->second);
  }
  // 第一个元素
  if (it == buff.begin()) {
    if (labs(it->first - t0_ms) >= 15) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<ZCnState const>(it->second);
  }

  // 最后一个元素
  if (it == std::prev(buff.end())) {
    if (labs(it->first - t0_ms) >= 15) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<ZCnState const>(it->second);
  }

  // 不是第一个也不是最后一个
  auto f1 = std::dynamic_pointer_cast<ZCnState const>(std::prev(it)->second);
  auto f2 = std::dynamic_pointer_cast<ZCnState const>(it->second);

  return Interpolate(f1, f2, t0_s);
}
}  // namespace civloc
}  // namespace civ

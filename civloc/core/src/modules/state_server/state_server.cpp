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

#include "modules/state_server/state_server.hpp"
#include "modules/message_cache/cacher.hpp"  // SP_CNF的定义位置
namespace civ {
namespace civloc {

StateServer::StateServer() {}

// 线程安全
// void StateServer::AddRefineState(crsp_cZFrame msg, crsp_cState state) {
//   // 时间差小于0.1ms，取后者
//   size_t t0 = static_cast<size_t>(msg->t0_ * 1e4);
//   {
//     std::lock_guard<std::mutex> lg(mtx_states_);
//     states_[t0] = state;
//     // 只保留1秒的状态, 10000需要参数化
//     auto it = states_.lower_bound(t0 - 10000);
//     if (it != states_.end()) {
//       states_.erase(states_.begin(), it);
//     }
//   }
// }

sp_cState StateServer::AddRefineState(sp_cState state) {
  if (!state) {
    return nullptr;
  }
  size_t t0_ms = s2ms(state->t0_);
  all_states_.insert({t0_ms, state});
  // 保留1s的所有融合状态
  auto it = all_states_.lower_bound(t0_ms - 1000);
  if (it != all_states_.end()) {
    all_states_.erase(all_states_.begin(), it);
  }

  sp_cState main_state = nullptr;
  // prepare_main_state_ 上一时刻确定待输出
  // t0_ms > main_state_time_ms 当前时刻大于待输出时刻
  // main_state_time_ms != 0 待输出时刻不为0
  if (prepare_main_state_ && (t0_ms > main_state_time_ms) &&
      (main_state_time_ms != 0)) {
    main_state = ConfirmMainState(main_state_time_ms);
  }

  // 确认当前时刻是否为待输出时刻
  if (t0_ms % 100 == 0) {
    main_state_time_ms = t0_ms;
    prepare_main_state_ = true;
  } else {
    prepare_main_state_ = false;
  }

  return main_state;
}

/** KeyState
 * */
sp_cState StateServer::ConfirmMainState(size_t t0_ms) {
  auto range = all_states_.equal_range(t0_ms);

  spState state = std::make_shared<State>(*range.first->second);

  for (auto it = range.first; it != range.second; ++it) {
    state->fs_ |= (it->second)->fs_;  // 融合状态相或
  }

  // 主状态缓存
  {
    std::lock_guard<std::mutex> lg(mtx_states_);
    main_states_[t0_ms] = state;
  }

  return state;
}

sp_cState StateServer::get_lastest_state() {
  std::lock_guard<std::mutex> lg(mtx_states_);

  return all_states_.empty() ? nullptr : all_states_.rbegin()->second;
}
}  // namespace civloc
}  // namespace civ

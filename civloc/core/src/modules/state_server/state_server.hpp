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
#include "modules/common/inner_types.hpp"
namespace civ {
namespace civloc {

// 维持一个时间段的状态
class StateServer {
 public:
  StateServer();
  bool empty() const { return main_states_.empty(); }
  int size() const { return main_states_.size(); }

  // void AddRefineState(crsp_cZFrame msg, crsp_cState state);  //
  // state状态不可再改变
  sp_cState AddRefineState(sp_cState state);  // state状态不可再改变,保存所有的传感器融合后的状态,
                                              // 返回值非空时，表明为主状态
  sp_cState get_lastest_state();

 private:
  sp_cState ConfirmMainState(
      size_t t0_ms);  // 确认主要融合状态,以10hz的整点状态为主要状态
  std::multimap<size_t, sp_cState>
      all_states_;  // ! 暂时不加锁,以1ms 为刻度, 所有传感器融合后的状态
  std::map<size_t, sp_cState>
      main_states_;  // ! 暂时不加锁,以1ms 为刻度, 整点时刻滤波器状态
  std::mutex mtx_states_;
  size_t main_state_time_ms = 0;  // 当前待输出时刻
  bool prepare_main_state_ = false;
};

}  // namespace civloc
}  // namespace civ

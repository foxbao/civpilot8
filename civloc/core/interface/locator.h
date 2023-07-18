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
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <future>
#include <memory>
#include <mutex>
#include <map>
#include "middle_ware/cyber/tool/imgprocessor.hpp"
#include "modules/common/inner_types.hpp"
#include "modules/loc/fuse/filter.hpp"

namespace civ {
namespace civloc {
class Initializer;
class Predictor;
class FuseProcessor;
class Detector;
class Locator {
 public:
  Locator(crString locator_cfg_pth, crString sensors_setting_cfg_pth);
  ~Locator();

 public:
  sp_cState ProcessCGI610Frame(crsp_cZCnState frame);
  // void PredictByIMU();
  // void ReadKittiData(crString data_folder,crString timestamps_file);
  // 对外接口4,IMU数据回调
  sp_cState ProcessIMU(crsp_cZImu frame);
  // 对外接口6,gnss 数据回调
  void ProcessGnss(crsp_cZGnss frame);
  // // 对外接口7,gnss 地图回调
  // void ProcessMap(crsp_cZSemanticMap frame);
  void FuseTaskSingle();
  // spState PredictStateMecanization(spState current_state,sp_cZImu imu);
  public:
    void set_input_data_dir(std::string const& path);

 private:
  // 以imu数据帧触发车辆模型约束, 默认10hz,整点触发
  void GenerateVehicleModelFrame(crsp_cZImu frame);

 private:
  std::shared_ptr<Initializer> initializer_;       // 初始化
  std::shared_ptr<Predictor> predictor_;           // 预测器
  std::shared_ptr<FuseProcessor> fuse_processor_;  // 融合处理
  std::shared_ptr<Detector> detector_;             // 车辆状态检测

  spState fused_state_;  //当前状态

 private:
  bool inited_ = false;
  std::atomic_bool finished_ = {false};
  std::deque<sp_cZFrame> frame_tobe_processed_;  // 排好序将被处理的数据
  std::mutex mtx_frame_tobe_processed_;
  std::mutex mtx_tobe_send_;
  std::condition_variable cv_frame_tobe_processed_;
  void FuseTask();  // 融合线程

  std::future<void> future_fuse_task_;
  std::deque<sp_cZFrame> frame_tobe_send_;
  // std::ofstream output_data_;
  std::shared_ptr<IMGPROCESSOR> sp_imgprocessor_;

 public:
  std::map<std::string, std::ofstream> files_;
};

}  // namespace civloc
}  // namespace civ

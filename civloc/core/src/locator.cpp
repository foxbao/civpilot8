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

#include "interface/locator.h"
#include <iostream>
#include <memory>
#include "../common/interface/file/file.h"
#include "common/util/util.h"
// #include "gflags/gflags.h"
#include "common/coordinate_transform/earth.hpp"
#include "modules/common/time_system.hpp"
#include "modules/common/utils.hpp"
#include "modules/detect/detector.hpp"
#include "modules/message_cache/cacher.hpp"

#include "modules/init/initializer.hpp"
#include "modules/loc/fuse/fuse_processor.hpp"
#include "modules/predict/predictor.hpp"
#include "modules/state_server/state_server.hpp"

namespace civ {
namespace civloc {
// 全局变量定义与初始化
std::shared_ptr<LocatorConfig> g_config = std::make_shared<LocatorConfig>();
std::shared_ptr<SensorsSetting> g_sensor_config =
    std::make_shared<SensorsSetting>();  // 传感器配置
std::shared_ptr<Cacher> g_cacher = nullptr;
std::shared_ptr<StateServer> g_state_server = nullptr;

Locator::Locator(crString locator_cfg_pth, crString sensors_setting_cfg_pth) {
  sp_imgprocessor_ = std::make_shared<IMGPROCESSOR>();
  using civ::common::file::GetProtoFromFile;
  common::coord_transform::Earth::SetOrigin(Eigen::Vector3d(civ::common::util::g_ori_pos_deg[0],
                                   civ::common::util::g_ori_pos_deg[1],
                                   civ::common::util::g_ori_pos_deg[2]),
                   true);
  // 全局传感器初始化
  ACHECK(GetProtoFromFile(sensors_setting_cfg_pth, g_sensor_config.get()))
      << "load from file failed, path = " << sensors_setting_cfg_pth;

  // 全局传感器初始化
  ACHECK(GetProtoFromFile(locator_cfg_pth, g_config.get()))
      << "load from file failed, path = " << locator_cfg_pth;

  // GetProtoFromFile(locator_cfg_pth, g_config.get());

  g_state_server = std::make_shared<StateServer>();
  ACHECK(g_state_server) << "ZLocator\t (g_state_server) created failed.";

  g_cacher = std::make_shared<Cacher>();
  ACHECK(g_cacher) << "ZLocator\t (g_cacher) created failed.";

  initializer_ = std::make_shared<Initializer>();
  ACHECK(initializer_) << "ZLocator submodule\t (initializer_) created failed.";

  predictor_ = std::make_shared<Predictor>();
  ACHECK(predictor_) << "ZLocator submodule\t (predictor_) created failed.";

  fuse_processor_ = std::make_shared<FuseProcessor>();
  ACHECK(fuse_processor_)
      << "ZLocator submodule\t (fuse_processor_) created failed.";

  detector_ = std::make_shared<Detector>();
  ACHECK(detector_) << "ZLocator submodule\t (detector_) created failed.";

  fused_state_ = std::make_shared<State>();

  // 开启fuse线程
  // future_fuse_task_ =
  //     std::async(std::launch::async, [this] { this->FuseTask(); });

  AINFO << "zlocator init done!";

  // 开启fuse线程
}

Locator::~Locator() {
  // output_data_.close();
}

void Locator::set_input_data_dir(std::string const &path) {
  //
  *g_config->mutable_io_cfg()->mutable_data_dir() = path;
}

sp_cState Locator::ProcessCGI610Frame(crsp_cZCnState frame) {
  // spState result;
  // AnalysizeCgi610(frame);
  // AINFO << "Locator::ProcessCGI610Frame " << UnixTimeString(frame->t0_);
  g_cacher->push_back<ZCnState>(frame);
  // AINFO << "Locator::ProcessCGI610Frame done" << UnixTimeString(frame->t0_);

  return nullptr;
}

sp_cState Locator::ProcessIMU(crsp_cZImu frame) {
  GenerateVehicleModelFrame(frame);  // 虚拟帧优先
  auto ordered_frame = g_cacher->push_back<ZImu>(frame);
  // initialize with the first gnss signal with fixed solution
  // after initialization, store the frames to be processed
  if (!ordered_frame.empty()) {
    for (auto const &frame : ordered_frame) {
      if (!inited_) {
        inited_ = initializer_->DoAlign(frame);
        if (inited_) {  // 对准
          spState aligned_state = initializer_->get_aligned_state();
          fuse_processor_->ConfirmInitialState(aligned_state);
          g_state_server->AddRefineState(aligned_state);
        }
      } else {
        std::unique_lock<std::mutex> lg(mtx_frame_tobe_processed_);
        frame_tobe_processed_.emplace_back(frame);
        cv_frame_tobe_processed_.notify_one();
      }
    }
  }

  // todo 控制输出频率,给出预测结果
  if (!inited_) {
    AINFO_EVERY(g_sensor_config->imu(0).rate_hz()) << ("Locator initialing");
    return nullptr;
  }

  // 机械编排预测
  spState result = predictor_->DoPredictUntilLastestMainSensor();

  return result;
}

void Locator::ProcessGnss(crsp_cZGnss frame) {
  if (frame->status_ == 0) {
    return;
  }
  // g_cacher->push_back<ZGnss>(frame);
  g_cacher->push_back<ZGnss>(frame);
}

void Locator::GenerateVehicleModelFrame(crsp_cZImu frame) {
  auto vm_frame = detector_->Detect(frame);
  if (vm_frame) {
    g_cacher->push_back<ZVehicleModel>(vm_frame);
  }
}

void Locator::FuseTaskSingle() {
  // AINFO << "fuse one batch data";
  std::deque<sp_cZFrame> frame_tobe_processing;
  frame_tobe_processing = std::move(frame_tobe_processed_);  // 拷贝待处理数据

  for (auto const &one_frame : frame_tobe_processing) {
    // std::cout << "one frame time to fuse:" << one_frame->t0_ << std::endl;
    std::cout << "one_frame->type_:" << one_frame->type_ << std::endl;
    if (one_frame->type_ == ZFrameType::GNSS_ALL) {
      sp_cZGnss gnss = std::dynamic_pointer_cast<ZGnss const>(one_frame);
      sp_imgprocessor_->PlotRawGNSS(gnss);
    }

    sp_cState fused_state = fuse_processor_->PredictAndDoFuse(one_frame);

    // auto lastest_fused_state=fuse_processor_->get_lastest_fused_state();
    if (fused_state) {
      std::cout << "fused state time:" << fused_state->t0_ << std::endl;
      sp_imgprocessor_->PlotSaveStateLLH(fused_state, "_fused_filter",
                                         cv::Scalar(0, 0, 255));
    }

    // sp_cState fused_state = fuse_processor_->PredictAndDoFuse(one_frame);
    if (!fused_state) {
      continue;
    }

    // // 整点时刻滤波器结果
    auto main_state = g_state_server->AddRefineState(fused_state);
    if (main_state) {
      // AppendViewData(main_state);
      // AnalysizeFusedstate(main_state);
    }
  }
}

void Locator::FuseTask() {
  while (!finished_) {
    std::cout << "not finished" << std::endl;
    std::deque<sp_cZFrame> frame_tobe_processing;
    {
      std::unique_lock<std::mutex> lg(mtx_frame_tobe_processed_);
      cv_frame_tobe_processed_.wait(lg, [this] {
        return this->finished_ || !this->frame_tobe_processed_.empty();
      });
      frame_tobe_processing =
          std::move(frame_tobe_processed_);  // 拷贝待处理数据
    }

    for (auto const &one_frame : frame_tobe_processing) {
      // 结果数据帧单独处理
      // AppendViewData(one_frame);
      std::cout << "fusing" << std::endl;
      sp_cState fused_state = fuse_processor_->PredictAndDoFuse(one_frame);
      if (!fused_state) {
        continue;
      }
      // 整点时刻滤波器结果
      auto main_state = g_state_server->AddRefineState(fused_state);
      if (main_state) {
        // AppendViewData(main_state);
        // AnalysizeFusedstate(main_state);
      }
    }
  }
}
}  // namespace civloc
}  // namespace civ

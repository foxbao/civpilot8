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

#include <memory>
#include <string>
#include "civloc/proto/civloc.pb.h"
#include "core/interface/locator.h"
#include "cyber/component/component.h"
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "middle_ware/cyber/message_convert.hpp"
#include "middle_ware/cyber/tool/imgprocessor.hpp"

#include "core/imu_gps_localizer/imu_gps_localizer.h"

namespace civ {
namespace civloc {
class Locator;
using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using ImuGpsLocalization::ImuGpsLocalizer;

class CivLocComponent : public Component<> {
 public:
  CivLocComponent() {}
  CivLocComponent(std::string const &dag_file_path);
  CivLocComponent(std::string const &config_file_path,
                  std::string const &locator_setting_path,
                  std::string const &sensors_setting_path);
  ~CivLocComponent();
  bool Init() override;
  bool InitCore();

  void ReadKittiData(crString data_folder, crString timestamps_file);
  spState PredictStateMecanization(spState current_state, sp_cZImu imu);

 private:
  bool InitIO();

 public:
  void OnGetRawImuFrame(crsp_cIMUProto msg);
  void OnGetGNGGAFrame(crsp_cGNSSProto msg);
  void set_input_data_dir(std::string const &path);
  std::shared_ptr<CivLocConfig> get_config() const { return cfg_; }

 private:
  std::shared_ptr<Writer<LocalizationEstimate>> msfr_writer_ = nullptr;

 private:
  std::shared_ptr<CivLocConfig> cfg_;
  std::shared_ptr<Locator> locator_;  // 定位模块
  std::shared_ptr<ImuGpsLocalizer> imu_gps_localizer_ptr_;
  std::shared_ptr<Reader<IMUProto>> imu_listener_;
  std::string imu_topic_ = "";
  std::shared_ptr<Reader<GNSSProto>> gnss_listener_;
  std::string gnss_topic_ = "";
  bool first_send_flag_ = true;
  std::string fused_output_data_path_;
  std::ofstream fused_output_data_;
  std::string fused_output_enu_data_path_;
  std::ofstream fused_output_enu_data_;
  std::string gnss_output_data_path_;
  std::ofstream gnss_output_data_;

  std::shared_ptr<IMGPROCESSOR> sp_imgprocessor_;
  bool gngga_test_sent = false;
};
}  // namespace civloc
}  // namespace civ

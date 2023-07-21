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

#include <gflags/gflags.h>
#include <iostream>
#include <memory>
#include <string>
#include "core/interface/locator.h"
#include "core/src/modules/common/inner_types.hpp"
#include "middle_ware/cyber/civloc_component.hpp"
#include "middle_ware/cyber/message_convert.hpp"
DEFINE_string(data_folder, "data",
              "the data folder that contains kitti imu data");
DEFINE_string(timestamps_file, "time",
              "the timestamp file that contains kitti imu data time");
DEFINE_string(sensors_setting_pth, "time",
              "the timestamp file that contains kitti imu data time");
DEFINE_string(locator_setting_pth, "time",
              "the timestamp file that contains kitti imu data time");
DEFINE_string(
    config_file_path,
    "path as /home/baojiali/Downloads/civpilot/civloc/conf/input_output.pb.txt",
    "civloc input output setting");
DEFINE_string(output_dir,
              "path as /home/baojiali/Downloads/civpilot/loc/output",
              "segment record dir");

DEFINE_int32(bao, 0, "delay time_ms for every process imu loop");

#include "common/interface/logger/log_interface.h"

using civ::civloc::IMUProto;

void MessageCallback(const std::shared_ptr<IMUProto>& msg) {
  std::cout<<msg->angular_velocity().x()<<std::endl;
  // std::cout << "Received message seq-> " << msg->seq() << std::endl;
  // std::cout << "msgcontent-> " << msg->content() << std::endl;
}

int main(int argc, char **argv) {
  apollo::cyber::Init(argv[0]);
  // google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);


  apollo::cyber::WaitForShutdown();

  using civ::civloc::CivLocComponent;
  std::shared_ptr<CivLocComponent> civloc = std::make_shared<CivLocComponent>(
      FLAGS_config_file_path, FLAGS_locator_setting_pth,
      FLAGS_sensors_setting_pth);

  // civloc->ReadKittiData(FLAGS_data_folder, FLAGS_timestamps_file);
  // int aaa = 1;
}

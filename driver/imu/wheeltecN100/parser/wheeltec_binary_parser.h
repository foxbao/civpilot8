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
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "common/parser/parser.h"
#include "imu/wheeltecN100/messages/Wheeltec_N100_messages.h"
#include "message/drivers/imu/proto/imu.pb.h"
namespace civ {
namespace drivers {
namespace wheeltec {
using civ::drivers::imu::sp_cIMUData_Packet_t;
using civ::drivers::imu::spIMUData_Packet_t;

using civ::drivers::common::Parser;
class WheeltecBinaryParser:public Parser {
 public:
  WheeltecBinaryParser();
  ~WheeltecBinaryParser();

  void ParseData(void);

  bool InitWriters(std::unique_ptr<apollo::cyber::Node> &node);

 private:
  bool ParseData(const uint8_t *buff, const uint32_t &buff_len,
                      std::size_t *shift, spIMUData_Packet_t imuMsg);
  void IMUData2PC(spIMUData_Packet_t imuMsg);
  void PublishIMU(sp_cIMUData_Packet_t imumsg, const double &timestamp);

 private:
  std::shared_ptr<apollo::cyber::Writer<civ::drivers::imu::CorrectedImu>>
      imu_writer_;
};
}  // namespace wheeltec
}  // namespace drivers
}  // namespace civ
